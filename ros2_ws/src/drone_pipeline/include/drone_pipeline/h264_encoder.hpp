#ifndef DRONE_PIPELINE__H264_ENCODER_HPP_
#define DRONE_PIPELINE__H264_ENCODER_HPP_

#include <cstdint>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
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

  // Input: packed RGB24, width*height*3 bytes
  EncodeResult encode(const uint8_t * rgb_data, int width, int height);

  bool isOpen() const { return codec_ctx_ != nullptr; }

  const uint8_t* extradata()      const { return codec_ctx_ ? codec_ctx_->extradata: nullptr;}
  int extradata_size() const { return codec_ctx_ ? codec_ctx_->extradata_size : 0;}
private:
  AVCodecContext * codec_ctx_{nullptr};
  SwsContext     * sws_ctx_{nullptr};
  AVFrame        * yuv_frame_{nullptr};
  AVPacket       * pkt_out_{nullptr};

  int     width_{0};
  int     height_{0};
  int64_t pts_{0};
};

}  // namespace drone_pipeline
#endif  // DRONE_PIPELINE__H264_ENCODER_HPP_