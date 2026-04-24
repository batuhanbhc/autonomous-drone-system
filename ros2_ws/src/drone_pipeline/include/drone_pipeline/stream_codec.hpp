#ifndef DRONE_PIPELINE__STREAM_CODEC_HPP_
#define DRONE_PIPELINE__STREAM_CODEC_HPP_

#include <algorithm>
#include <cctype>
#include <string>

namespace drone_pipeline
{

enum class StreamCodec
{
  kH264,
  kMjpeg,
};

inline std::string normalizeStreamCodec(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

inline bool parseStreamCodec(const std::string & raw_value, StreamCodec & codec)
{
  const std::string value = normalizeStreamCodec(raw_value);
  if (value == "h264") {
    codec = StreamCodec::kH264;
    return true;
  }
  if (value == "mjpeg" || value == "jpeg") {
    codec = StreamCodec::kMjpeg;
    return true;
  }
  return false;
}

inline const char * streamCodecName(StreamCodec codec)
{
  switch (codec) {
    case StreamCodec::kH264:
      return "h264";
    case StreamCodec::kMjpeg:
      return "mjpeg";
  }
  return "unknown";
}

}  // namespace drone_pipeline

#endif  // DRONE_PIPELINE__STREAM_CODEC_HPP_
