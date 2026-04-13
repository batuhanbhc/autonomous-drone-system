#include "drone_pipeline/h264_encoder.hpp"

#include <stdexcept>

extern "C" {
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <turbojpeg.h>
}

namespace drone_pipeline
{

void H264Encoder::open(int width, int height, int fps, int gop_size)
{
  width_  = width;
  height_ = height;

  // ── libjpeg-turbo decompress handle ───────────────────────────────────────
  tj_decompress_ = tjInitDecompress();
  if (!tj_decompress_)
    throw std::runtime_error("H264Encoder: tjInitDecompress failed");

  // ── YUV422P staging buffer ────────────────────────────────────────────────
  // tjDecompressToYUV2 writes padded planar YUV.  We allocate via
  // tjBufSizeYUV2 so the padding libjpeg-turbo expects is always present.
  const unsigned long buf_size =
    tjBufSizeYUV2(width, /*pad=*/1, height, TJSAMP_422);
  if (buf_size == 0)
    throw std::runtime_error("H264Encoder: tjBufSizeYUV2 returned 0");
  yuv422_staging_.resize(buf_size);

  // ── Output packet ─────────────────────────────────────────────────────────
  pkt_out_ = av_packet_alloc();
  if (!pkt_out_)
    throw std::runtime_error("H264Encoder: av_packet_alloc failed");

  // ── YUV422P frame (encoder input) ─────────────────────────────────────────
  yuv_frame_ = av_frame_alloc();
  if (!yuv_frame_)
    throw std::runtime_error("H264Encoder: av_frame_alloc failed");

  yuv_frame_->format = AV_PIX_FMT_YUV422P;
  yuv_frame_->width  = width;
  yuv_frame_->height = height;

  // We will point yuv_frame_->data[] directly at yuv422_staging_ in encode(),
  // so we don't allocate a separate frame buffer here.

  // ── H.264 encoder ─────────────────────────────────────────────────────────
  const AVCodec * h264_enc = avcodec_find_encoder_by_name("libx264");
  if (!h264_enc) h264_enc  = avcodec_find_encoder(AV_CODEC_ID_H264);
  if (!h264_enc)
    throw std::runtime_error("H264Encoder: libx264 not found");

  codec_ctx_ = avcodec_alloc_context3(h264_enc);
  if (!codec_ctx_)
    throw std::runtime_error("H264Encoder: cannot alloc encoder context");

  codec_ctx_->width        = width;
  codec_ctx_->height       = height;
  codec_ctx_->pix_fmt      = AV_PIX_FMT_YUV422P;
  codec_ctx_->time_base    = AVRational{1, fps};
  codec_ctx_->framerate    = AVRational{fps, 1};
  codec_ctx_->gop_size     = gop_size;
  codec_ctx_->max_b_frames = 0;

  av_opt_set(codec_ctx_->priv_data, "preset",       "ultrafast",   0);
  av_opt_set(codec_ctx_->priv_data, "tune",         "zerolatency", 0);
  av_opt_set(codec_ctx_->priv_data, "crf",          "18",          0);
  av_opt_set(codec_ctx_->priv_data, "cabac",        "0",           0);
  av_opt_set_int(codec_ctx_->priv_data, "rc-lookahead", 0,         0);

  codec_ctx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  av_opt_set(codec_ctx_->priv_data, "x264opts", "repeat-headers=1", 0);

  if (avcodec_open2(codec_ctx_, h264_enc, nullptr) < 0)
    throw std::runtime_error("H264Encoder: cannot open libx264");

  pts_ = 0;
}

void H264Encoder::close()
{
  if (codec_ctx_) {
    avcodec_send_frame(codec_ctx_, nullptr);
    while (pkt_out_ && avcodec_receive_packet(codec_ctx_, pkt_out_) == 0)
      av_packet_unref(pkt_out_);
    if (pkt_out_) av_packet_unref(pkt_out_);
    avcodec_free_context(&codec_ctx_);
  }
  if (yuv_frame_)     av_frame_free(&yuv_frame_);
  if (pkt_out_)       av_packet_free(&pkt_out_);
  if (tj_decompress_) { tjDestroy(tj_decompress_); tj_decompress_ = nullptr; }
  yuv422_staging_.clear();
  pts_ = 0;
}

H264Encoder::EncodeResult H264Encoder::encode(const uint8_t * jpeg_data, size_t jpeg_size)
{
  if (!codec_ctx_ || !tj_decompress_) return {};

  // ── Step 1: JPEG → planar YUV422P ─────────────────────────────────────────
  // pad=1 matches the tjBufSizeYUV2 call in open(), so plane offsets agree.
  const int ret = tjDecompressToYUV2(
    tj_decompress_,
    jpeg_data,
    static_cast<unsigned long>(jpeg_size),
    yuv422_staging_.data(),
    width_,
    /*pad=*/1,
    height_,
    TJFLAG_FASTDCT);

  if (ret != 0) return {};

  // ── Step 2: point AVFrame planes at the staging buffer ────────────────────
  // tjDecompressToYUV2 with pad=1 lays out planes contiguously with no row
  // padding, so the plane offsets are simply:
  //   Y : offset 0,                     stride = width
  //   U : offset width*height,          stride = width/2
  //   V : offset width*height + (width/2)*height, stride = width/2
  const int y_size  = width_ * height_;
  const int uv_size = (width_ / 2) * height_;

  yuv_frame_->data[0]     = yuv422_staging_.data();
  yuv_frame_->data[1]     = yuv422_staging_.data() + y_size;
  yuv_frame_->data[2]     = yuv422_staging_.data() + y_size + uv_size;
  yuv_frame_->linesize[0] = width_;
  yuv_frame_->linesize[1] = width_ / 2;
  yuv_frame_->linesize[2] = width_ / 2;
  yuv_frame_->pts         = pts_++;

  // ── Step 3: H.264 encode ──────────────────────────────────────────────────
  if (avcodec_send_frame(codec_ctx_, yuv_frame_) < 0) return {};

  EncodeResult result;
  av_packet_unref(pkt_out_);

  while (avcodec_receive_packet(codec_ctx_, pkt_out_) == 0) {
    if (pkt_out_->flags & AV_PKT_FLAG_KEY)
      result.is_key_frame = true;
    result.data.insert(result.data.end(),
                       pkt_out_->data,
                       pkt_out_->data + pkt_out_->size);
    av_packet_unref(pkt_out_);
  }

  result.pts = yuv_frame_->pts;
  return result;
}

}  // namespace drone_pipeline