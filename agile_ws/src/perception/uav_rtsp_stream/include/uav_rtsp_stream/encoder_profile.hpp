#ifndef UAV_RTSP_STREAM__ENCODER_PROFILE_HPP_
#define UAV_RTSP_STREAM__ENCODER_PROFILE_HPP_

#include <string>

namespace uav_rtsp_stream
{

struct EncoderProfile
{
  std::string name;
  std::string element;
  // GStreamer pipeline fragment between raw video and h264parse.
  std::string pipeline_mid;
};

// Detect the best available H.264 encoder at runtime.
EncoderProfile detect_encoder(const std::string & preferred = "auto");

// Build encoder fragment with low-latency tuning.
std::string build_encoder_fragment(
  const EncoderProfile & profile,
  int bitrate_kbps,
  int gop_size);

// Detect GPU JPEG decoder (nvjpegdec) when available.
std::string detect_jpeg_decoder(const std::string & preferred = "auto");

// nvh264enc/x264enc accept I420 from jpegdec without videoconvert.
bool encoder_skips_videoconvert(const std::string & element);

}  // namespace uav_rtsp_stream

#endif  // UAV_RTSP_STREAM__ENCODER_PROFILE_HPP_
