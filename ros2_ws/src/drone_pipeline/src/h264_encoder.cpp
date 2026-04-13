#include "drone_pipeline/h264_encoder.hpp"

#include <stdexcept>
#include <cstring>

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

  // ── RGB staging buffer ────────────────────────────────────────────────────
  rgb_staging_.resize(width * height * 3);

  // ── Output packet ─────────────────────────────────────────────────────────
  pkt_out_ = av_packet_alloc();
  if (!pkt_out_)
    throw std::runtime_error("H264Encoder: av_packet_alloc failed");

  // ── YUV420P frame (encoder input) ─────────────────────────────────────────
  yuv_frame_ = av_frame_alloc();
  if (!yuv_frame_)
    throw std::runtime_error("H264Encoder: av_frame_alloc failed");

  yuv_frame_->format = AV_PIX_FMT_YUV420P;
  yuv_frame_->width  = width;
  yuv_frame_->height = height;

  if (av_frame_get_buffer(yuv_frame_, 32) < 0)
    throw std::runtime_error("H264Encoder: cannot allocate YUV frame buffer");

  // ── swscale: RGB24 → YUV420P ──────────────────────────────────────────────
  sws_ctx_ = sws_getContext(
    width, height, AV_PIX_FMT_RGB24,
    width, height, AV_PIX_FMT_YUV420P,
    SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

  if (!sws_ctx_)
    throw std::runtime_error("H264Encoder: sws_getContext failed");

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
  codec_ctx_->pix_fmt      = AV_PIX_FMT_YUV420P;
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
  if (sws_ctx_)       { sws_freeContext(sws_ctx_); sws_ctx_ = nullptr; }
  if (yuv_frame_)     av_frame_free(&yuv_frame_);
  if (pkt_out_)       av_packet_free(&pkt_out_);
  if (tj_decompress_) { tjDestroy(tj_decompress_); tj_decompress_ = nullptr; }
  rgb_staging_.clear();
  pts_ = 0;
}

H264Encoder::EncodeResult H264Encoder::encode(const uint8_t * jpeg_data, size_t jpeg_size)
{
  if (!codec_ctx_ || !tj_decompress_ || !sws_ctx_) return {};

  if (av_frame_make_writable(yuv_frame_) < 0) return {};

  // ── Step 1: JPEG → RGB24 ──────────────────────────────────────────────────
  const int ret = tjDecompress2(
    tj_decompress_,
    jpeg_data,
    static_cast<unsigned long>(jpeg_size),
    rgb_staging_.data(),
    width_, 0, height_,
    TJPF_RGB, TJFLAG_FASTDCT);

  if (ret != 0) return {};

  // ── Step 2: RGB24 → YUV420P via swscale ───────────────────────────────────
  const uint8_t * src_planes[1] = { rgb_staging_.data() };
  int             src_stride[1] = { width_ * 3 };

  sws_scale(sws_ctx_,
            src_planes, src_stride, 0, height_,
            yuv_frame_->data, yuv_frame_->linesize);

  yuv_frame_->pts = pts_++;

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