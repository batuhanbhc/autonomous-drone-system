#include "drone_pipeline/h264_encoder.hpp"

#include <cstring>
#include <stdexcept>

extern "C" {
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}

namespace drone_pipeline
{

void H264Encoder::open(int width, int height, int fps, int gop_size)
{
  width_  = width;
  height_ = height;

  // ── Allocate reusable packets / frames ───────────────────────────────────
  pkt_in_    = av_packet_alloc();
  pkt_out_   = av_packet_alloc();
  raw_frame_ = av_frame_alloc();
  yuv_frame_ = av_frame_alloc();

  if (!pkt_in_ || !pkt_out_ || !raw_frame_ || !yuv_frame_)
    throw std::runtime_error("H264Encoder: av_alloc failed");

  // ── JPEG decoder ─────────────────────────────────────────────────────────
  const AVCodec * jpeg_dec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
  if (!jpeg_dec)
    throw std::runtime_error("H264Encoder: MJPEG decoder not found");

  jpeg_dec_ctx_ = avcodec_alloc_context3(jpeg_dec);
  if (!jpeg_dec_ctx_)
    throw std::runtime_error("H264Encoder: cannot alloc MJPEG decoder context");

  jpeg_dec_ctx_->width  = width;
  jpeg_dec_ctx_->height = height;

  if (avcodec_open2(jpeg_dec_ctx_, jpeg_dec, nullptr) < 0)
    throw std::runtime_error("H264Encoder: cannot open MJPEG decoder");

  // ── YUV420P output frame (encoder input) ─────────────────────────────────
  yuv_frame_->format = AV_PIX_FMT_YUV420P;
  yuv_frame_->width  = width;
  yuv_frame_->height = height;
  if (av_frame_get_buffer(yuv_frame_, 32) < 0)
    throw std::runtime_error("H264Encoder: cannot allocate YUV frame buffer");

  // ── H.264 encoder ────────────────────────────────────────────────────────
  const AVCodec * h264_enc = avcodec_find_encoder_by_name("libx264");
  if (!h264_enc) h264_enc  = avcodec_find_encoder(AV_CODEC_ID_H264);
  if (!h264_enc)
    throw std::runtime_error("H264Encoder: libx264 not found — "
                             "rebuild FFmpeg with --enable-libx264");

  codec_ctx_ = avcodec_alloc_context3(h264_enc);
  if (!codec_ctx_)
    throw std::runtime_error("H264Encoder: cannot alloc encoder context");

  codec_ctx_->width        = width;
  codec_ctx_->height       = height;
  codec_ctx_->pix_fmt      = AV_PIX_FMT_YUV420P;
  codec_ctx_->color_range = AVCOL_RANGE_JPEG;
  codec_ctx_->time_base    = AVRational{1, fps};
  codec_ctx_->framerate    = AVRational{fps, 1};
  codec_ctx_->gop_size     = gop_size;   // I-frame every gop_size frames
  codec_ctx_->max_b_frames = 0;          // no B-frames → minimal latency

  av_opt_set(codec_ctx_->priv_data, "preset",      "ultrafast",                  0);
  av_opt_set(codec_ctx_->priv_data, "tune",        "zerolatency",                0);
  av_opt_set(codec_ctx_->priv_data, "crf",         "18",                         0);
  av_opt_set(codec_ctx_->priv_data, "cabac",       "0",                          0);
  // Inline SPS/PPS before every keyframe — essential for Annex-B pipe/stream
  // consumers that may join mid-stream (ffplay, WebRTC, etc.)
  av_opt_set(codec_ctx_->priv_data, "x264-params",  "annexb=1", 0);
  av_opt_set_int(codec_ctx_->priv_data, "repeat_headers", 1, 0);

  if (avcodec_open2(codec_ctx_, h264_enc, nullptr) < 0)
    throw std::runtime_error("H264Encoder: cannot open libx264 encoder");

  pts_          = 0;
  last_src_fmt_ = -1;
}

void H264Encoder::close()
{
  if (codec_ctx_) {
    // Flush encoder and discard remaining packets
    avcodec_send_frame(codec_ctx_, nullptr);
    while (pkt_out_ && avcodec_receive_packet(codec_ctx_, pkt_out_) == 0)
      av_packet_unref(pkt_out_);
    avcodec_free_context(&codec_ctx_);
  }
  if (jpeg_dec_ctx_) avcodec_free_context(&jpeg_dec_ctx_);
  if (sws_ctx_)      { sws_freeContext(sws_ctx_); sws_ctx_ = nullptr; }
  if (raw_frame_)    av_frame_free(&raw_frame_);
  if (yuv_frame_)    av_frame_free(&yuv_frame_);
  if (pkt_in_)       av_packet_free(&pkt_in_);
  if (pkt_out_)      av_packet_free(&pkt_out_);
  last_src_fmt_ = -1;
  pts_          = 0;
}

H264Encoder::EncodeResult H264Encoder::encode(const std::vector<uint8_t> & jpeg_data)
{
  // ── Step 1: decode JPEG → raw_frame_ ────────────────────────────────────
  av_packet_unref(pkt_in_);
  if (av_new_packet(pkt_in_, static_cast<int>(jpeg_data.size())) < 0)
    return {};
  std::memcpy(pkt_in_->data, jpeg_data.data(), jpeg_data.size());

  if (avcodec_send_packet(jpeg_dec_ctx_, pkt_in_) < 0) return {};

  av_frame_unref(raw_frame_);
  if (avcodec_receive_frame(jpeg_dec_ctx_, raw_frame_) < 0) return {};

  // raw_frame_->format is only valid after receive_frame succeeds.
  // JPEG decoder typically outputs YUVJ420P (full-range variant of YUV420P).
  const int src_fmt = raw_frame_->format;
  if (src_fmt == AV_PIX_FMT_NONE) return {};

  // ── Step 2: colour-space conversion → YUV420P ───────────────────────────
  // Re-initialise swscale when pixel format changes (constant in practice).
  if (!sws_ctx_ || src_fmt != last_src_fmt_) {
      if (sws_ctx_) { sws_freeContext(sws_ctx_); sws_ctx_ = nullptr; }  // ← free old one first
      
      sws_ctx_ = sws_getContext(
          raw_frame_->width, raw_frame_->height,
          static_cast<AVPixelFormat>(src_fmt),  // ← actual format from decoder, YUVJ420P
          width_, height_,
          AV_PIX_FMT_YUV420P,
          SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

      if (!sws_ctx_) return {};

      // This call suppresses the warning AND correctly handles the full→limited conversion
      sws_setColorspaceDetails(sws_ctx_,
          sws_getCoefficients(SWS_CS_DEFAULT), 1,  // src full range
          sws_getCoefficients(SWS_CS_DEFAULT), 1,  // dst full range
          0, 1<<16, 1<<16);
      
      last_src_fmt_ = src_fmt;
  }


  if (av_frame_make_writable(yuv_frame_) < 0) return {};

  sws_scale(sws_ctx_,
            raw_frame_->data, raw_frame_->linesize, 0, raw_frame_->height,
            yuv_frame_->data, yuv_frame_->linesize);
  yuv_frame_->pts = pts_++;

  // ── Step 3: H.264 encode ─────────────────────────────────────────────────
  if (avcodec_send_frame(codec_ctx_, yuv_frame_) < 0) return {};

  EncodeResult result;
  av_packet_unref(pkt_out_);

  // ultrafast/zerolatency normally produces exactly one packet per frame;
  // drain the loop anyway to be correct in all configurations.
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