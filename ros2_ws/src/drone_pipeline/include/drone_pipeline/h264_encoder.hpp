#ifndef DRONE_PIPELINE__H264_ENCODER_HPP_
#define DRONE_PIPELINE__H264_ENCODER_HPP_

#include <cstdint>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <turbojpeg.h>
}

namespace drone_pipeline
{

class H264Encoder
{
public:
  struct EncodeResult
  {
    std::vector<uint8_t> data;
    int64_t              pts{0};
    bool                 is_key_frame{false};
  };

  void open(int width, int height, int fps, int gop_size);
  void close();

  // Input: raw MJPEG bytes (4:2:2 subsampled).
  // Decompresses to planar YUV422P via libjpeg-turbo (no RGB, no swscale),
  // then encodes directly with libx264 in YUV422P.
  EncodeResult encode(const uint8_t * jpeg_data, size_t jpeg_size);

  bool isOpen() const { return codec_ctx_ != nullptr; }
  bool copyCodecParameters(AVCodecParameters * out) const;

  const uint8_t* extradata()      const { return codec_ctx_ ? codec_ctx_->extradata : nullptr; }
  int            extradata_size() const { return codec_ctx_ ? codec_ctx_->extradata_size : 0; }

private:
  AVCodecContext * codec_ctx_{nullptr};
  AVFrame        * yuv_frame_{nullptr};
  AVPacket       * pkt_out_{nullptr};
  SwsContext     * sws_ctx_{nullptr};
  tjhandle         tj_decompress_{nullptr};

  std::vector<uint8_t> rgb_staging_;

  int     width_{0};
  int     height_{0};
  int64_t pts_{0};
};

}  // namespace drone_pipeline
#endif  // DRONE_PIPELINE__H264_ENCODER_HPP_
