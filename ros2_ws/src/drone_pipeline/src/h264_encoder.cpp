#include "drone_pipeline/h264_encoder.hpp"

#include <stdexcept>

extern "C" {
#include <libavutil/opt.h>
#include <turbojpeg.h>
}

namespace drone_pipeline
{

void H264Encoder::open(int width, int height, int fps, int gop_size)
{
  width_ = width;
  height_ = height;

  tj_decompress_ = tjInitDecompress();
  if (!tj_decompress_) {
    throw std::runtime_error("H264Encoder: tjInitDecompress failed");
  }

  rgb_staging_.resize(static_cast<size_t>(width_) * static_cast<size_t>(height_) * 3U);

  pkt_out_ = av_packet_alloc();
  if (!pkt_out_) {
    throw std::runtime_error("H264Encoder: av_packet_alloc failed");
  }

  yuv_frame_ = av_frame_alloc();
  if (!yuv_frame_) {
    throw std::runtime_error("H264Encoder: av_frame_alloc failed");
  }
  yuv_frame_->format = AV_PIX_FMT_YUV420P;
  yuv_frame_->width = width_;
  yuv_frame_->height = height_;
  if (av_frame_get_buffer(yuv_frame_, 32) < 0) {
    throw std::runtime_error("H264Encoder: av_frame_get_buffer failed");
  }

  const AVCodec * h264_enc = avcodec_find_encoder_by_name("libx264");
  if (!h264_enc) {
    h264_enc = avcodec_find_encoder(AV_CODEC_ID_H264);
  }
  if (!h264_enc) {
    throw std::runtime_error("H264Encoder: libx264 not found");
  }

  codec_ctx_ = avcodec_alloc_context3(h264_enc);
  if (!codec_ctx_) {
    throw std::runtime_error("H264Encoder: cannot alloc encoder context");
  }

  codec_ctx_->width = width_;
  codec_ctx_->height = height_;
  codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
  codec_ctx_->time_base = AVRational{1, fps};
  codec_ctx_->framerate = AVRational{fps, 1};
  codec_ctx_->gop_size = gop_size;
  codec_ctx_->max_b_frames = 0;

  av_opt_set(codec_ctx_->priv_data, "preset", "veryfast", 0);
  av_opt_set(codec_ctx_->priv_data, "tune", "zerolatency", 0);
  av_opt_set(codec_ctx_->priv_data, "crf", "25", 0);
  av_opt_set_int(codec_ctx_->priv_data, "rc-lookahead", 0, 0);
  av_opt_set(codec_ctx_->priv_data, "maxrate", "2500k", 0);
  av_opt_set(codec_ctx_->priv_data, "bufsize", "5000k", 0);

  codec_ctx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  av_opt_set(codec_ctx_->priv_data, "x264opts", "repeat-headers=1", 0);

  if (avcodec_open2(codec_ctx_, h264_enc, nullptr) < 0) {
    throw std::runtime_error("H264Encoder: cannot open libx264");
  }

  pts_ = 0;
}

void H264Encoder::close()
{
  if (codec_ctx_) {
    avcodec_send_frame(codec_ctx_, nullptr);
    while (pkt_out_ && avcodec_receive_packet(codec_ctx_, pkt_out_) == 0) {
      av_packet_unref(pkt_out_);
    }
    if (pkt_out_) {
      av_packet_unref(pkt_out_);
    }
    avcodec_free_context(&codec_ctx_);
  }
  if (yuv_frame_) {
    av_frame_free(&yuv_frame_);
  }
  if (pkt_out_) {
    av_packet_free(&pkt_out_);
  }
  if (sws_ctx_) {
    sws_freeContext(sws_ctx_);
    sws_ctx_ = nullptr;
  }
  if (tj_decompress_) {
    tjDestroy(tj_decompress_);
    tj_decompress_ = nullptr;
  }
  rgb_staging_.clear();
  pts_ = 0;
}

H264Encoder::EncodeResult H264Encoder::encode(const uint8_t * jpeg_data, size_t jpeg_size)
{
  if (!codec_ctx_ || !tj_decompress_) {
    return {};
  }

  const int ret = tjDecompress2(
    tj_decompress_,
    jpeg_data,
    static_cast<unsigned long>(jpeg_size),
    rgb_staging_.data(),
    width_,
    0,
    height_,
    TJPF_RGB,
    TJFLAG_FASTDCT);
  if (ret != 0) {
    return {};
  }

  if (av_frame_make_writable(yuv_frame_) < 0) {
    return {};
  }

  sws_ctx_ = sws_getCachedContext(
    sws_ctx_,
    width_,
    height_,
    AV_PIX_FMT_RGB24,
    width_,
    height_,
    AV_PIX_FMT_YUV420P,
    SWS_FAST_BILINEAR,
    nullptr,
    nullptr,
    nullptr);
  if (!sws_ctx_) {
    return {};
  }

  const uint8_t * src_data[4] = {rgb_staging_.data(), nullptr, nullptr, nullptr};
  const int src_linesize[4] = {width_ * 3, 0, 0, 0};
  sws_scale(
    sws_ctx_,
    src_data,
    src_linesize,
    0,
    height_,
    yuv_frame_->data,
    yuv_frame_->linesize);

  yuv_frame_->pts = pts_++;

  if (avcodec_send_frame(codec_ctx_, yuv_frame_) < 0) {
    return {};
  }

  EncodeResult result;
  av_packet_unref(pkt_out_);

  while (avcodec_receive_packet(codec_ctx_, pkt_out_) == 0) {
    if (pkt_out_->flags & AV_PKT_FLAG_KEY) {
      result.is_key_frame = true;
    }
    result.data.insert(result.data.end(), pkt_out_->data, pkt_out_->data + pkt_out_->size);
    av_packet_unref(pkt_out_);
  }

  result.pts = yuv_frame_->pts;
  return result;
}

bool H264Encoder::copyCodecParameters(AVCodecParameters * out) const
{
  if (!codec_ctx_ || !out) {
    return false;
  }
  return avcodec_parameters_from_context(out, codec_ctx_) >= 0;
}

}  // namespace drone_pipeline
