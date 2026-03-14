#pragma once

#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

namespace drone_pipeline
{

// ─────────────────────────────────────────────────────────────────────────────
//  MjpegWriter
//
//  Wraps raw JPEG frames in an AVI/MJPEG container using libavformat as a
//  pure muxer.  No pixel data is decoded or re-encoded — each JPEG buffer is
//  handed directly to av_interleaved_write_frame() as an AVPacket.
// ─────────────────────────────────────────────────────────────────────────────

class MjpegWriter
{
public:
  MjpegWriter() = default;
  ~MjpegWriter() { close(); }

  MjpegWriter(const MjpegWriter &) = delete;
  MjpegWriter & operator=(const MjpegWriter &) = delete;

  /// Open a new AVI/MJPEG file.  Throws std::runtime_error on failure.
  void open(const std::string & path, int width, int height, int fps);

  /// Write one raw JPEG frame.  jpeg_data must be a valid JPEG bitstream.
  void writeFrame(const std::vector<uint8_t> & jpeg_data);

  /// Flush trailer and close the file.
  void close();

  bool isOpen() const { return fmt_ctx_ != nullptr; }

private:
  AVFormatContext * fmt_ctx_{nullptr};
  AVStream        * video_stream_{nullptr};
  int64_t           pts_{0};
  int               fps_{30};
};

}  // namespace drone_pipeline