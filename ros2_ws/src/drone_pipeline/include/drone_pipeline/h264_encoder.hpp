#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace drone_pipeline
{

// ─────────────────────────────────────────────────────────────────────────────
//  H264Encoder
//
//  Accepts raw JPEG bytes and produces Annex-B H.264 NAL units.
//
//  Internal pipeline per frame:
//    1. MJPEG decoder  →  raw AVFrame  (any pixel format)
//    2. libswscale     →  YUV420P      (required by libx264)
//    3. libx264        →  AVPacket     (Annex-B NAL units)
//
//  Tuned for low-latency embedded use (ultrafast, zerolatency, no B-frames).
//  I-frame interval is set via gop_size (default: fps → one keyframe/second).
//
//  RPi5 note: no stable V4L2 M2M H.264 path exists across kernel versions.
//  libx264 ultrafast handles 640×480 @ 30 fps on a single A76 core (~15% CPU).
//  To switch to hardware encoding later, replace "libx264" with "h264_v4l2m2m"
//  and remove the av_opt_set calls.
// ─────────────────────────────────────────────────────────────────────────────

class H264Encoder
{
public:
  struct EncodeResult
  {
    std::vector<uint8_t> data;          // Annex-B NAL bytes; empty = buffering
    bool is_key_frame{false};
    int64_t pts = 0;
  };

  H264Encoder() = default;
  ~H264Encoder() { close(); }

  H264Encoder(const H264Encoder &) = delete;
  H264Encoder & operator=(const H264Encoder &) = delete;

  /// Initialise decoder + swscale + encoder.  Throws on failure.
  void open(int width, int height, int fps, int gop_size = 30);

  /// Flush and release all FFmpeg resources.
  void close();

  bool isOpen() const { return codec_ctx_ != nullptr; }

  /// Decode one JPEG buffer and encode it as H.264.
  /// result.data is empty when the encoder is still buffering.
  EncodeResult encode(const std::vector<uint8_t> & jpeg_data);

private:
  // JPEG decoder
  AVCodecContext * jpeg_dec_ctx_{nullptr};
  AVFrame        * raw_frame_{nullptr};    // decoder output (any pixel fmt)
  AVPacket       * pkt_in_{nullptr};

  // Colour-space conversion
  AVFrame        * yuv_frame_{nullptr};    // YUV420P input to encoder
  SwsContext     * sws_ctx_{nullptr};
  int              last_src_fmt_{-1};      // detects pixel-format changes

  // H.264 encoder
  AVCodecContext * codec_ctx_{nullptr};
  AVPacket       * pkt_out_{nullptr};

  int     width_{0}, height_{0};
  int64_t pts_{0};
};

}  // namespace drone_pipeline