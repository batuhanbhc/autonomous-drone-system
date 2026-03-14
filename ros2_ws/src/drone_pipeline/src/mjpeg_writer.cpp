#include "drone_pipeline/mjpeg_writer.hpp"

#include <cstring>
#include <stdexcept>

extern "C" {
#include <libavutil/avutil.h>
}

namespace drone_pipeline
{

void MjpegWriter::open(const std::string & path, int width, int height, int fps)
{
  fps_ = fps;

  if (avformat_alloc_output_context2(&fmt_ctx_, nullptr, "avi", path.c_str()) < 0)
    throw std::runtime_error("MjpegWriter: cannot allocate output context for " + path);

  // We only need the codec descriptor to fill the stream parameters —
  // no AVCodecContext is opened; we are purely muxing, not encoding.
  const AVCodec * codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
  if (!codec)
    throw std::runtime_error("MjpegWriter: MJPEG codec not found in FFmpeg build");

  video_stream_ = avformat_new_stream(fmt_ctx_, codec);
  if (!video_stream_)
    throw std::runtime_error("MjpegWriter: cannot allocate video stream");

  AVCodecParameters * par = video_stream_->codecpar;
  par->codec_type = AVMEDIA_TYPE_VIDEO;
  par->codec_id   = AV_CODEC_ID_MJPEG;
  par->width      = width;
  par->height     = height;
  par->format     = AV_PIX_FMT_YUVJ420P;   // standard MJPEG chroma format
  par->bit_rate   = 0;

  video_stream_->time_base      = AVRational{1, fps};
  video_stream_->avg_frame_rate = AVRational{fps, 1};

  if (!(fmt_ctx_->oformat->flags & AVFMT_NOFILE)) {
    if (avio_open(&fmt_ctx_->pb, path.c_str(), AVIO_FLAG_WRITE) < 0)
      throw std::runtime_error("MjpegWriter: cannot open file " + path);
  }

  if (avformat_write_header(fmt_ctx_, nullptr) < 0)
    throw std::runtime_error("MjpegWriter: avformat_write_header failed");

  pts_ = 0;
}

void MjpegWriter::writeFrame(const std::vector<uint8_t> & jpeg_data)
{
  if (!fmt_ctx_) return;

  AVPacket * pkt = av_packet_alloc();
  if (!pkt) return;

  // av_packet_from_data takes ownership of a malloc'd buffer.
  uint8_t * buf = static_cast<uint8_t *>(av_malloc(jpeg_data.size()));
  if (!buf) { av_packet_free(&pkt); return; }
  std::memcpy(buf, jpeg_data.data(), jpeg_data.size());

  av_packet_from_data(pkt, buf, static_cast<int>(jpeg_data.size()));
  pkt->stream_index = video_stream_->index;
  pkt->pts          = pts_;
  pkt->dts          = pts_;
  pkt->duration     = 1;
  pkt->flags       |= AV_PKT_FLAG_KEY;   // every MJPEG frame is a keyframe
  ++pts_;

  av_interleaved_write_frame(fmt_ctx_, pkt);
  av_packet_free(&pkt);
}

void MjpegWriter::close()
{
  if (!fmt_ctx_) return;

  av_write_trailer(fmt_ctx_);

  if (!(fmt_ctx_->oformat->flags & AVFMT_NOFILE))
    avio_closep(&fmt_ctx_->pb);

  avformat_free_context(fmt_ctx_);
  fmt_ctx_      = nullptr;
  video_stream_ = nullptr;
  pts_          = 0;
}

}  // namespace drone_pipeline