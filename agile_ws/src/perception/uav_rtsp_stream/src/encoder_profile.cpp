#include "uav_rtsp_stream/encoder_profile.hpp"

#include <gst/gst.h>
#include <iostream>
#include <stdexcept>

namespace uav_rtsp_stream
{

namespace
{

std::string gop_property(const std::string & element, int gop_size)
{
  if (element == "x264enc") {
    return "key-int-max=" + std::to_string(gop_size);
  }
  if (element == "vaapih264enc") {
    return "keyframe-period=" + std::to_string(gop_size);
  }
  return "gop-size=" + std::to_string(gop_size);
}

const EncoderProfile kProfiles[] = {
  {
    "nvh264enc",
    "nvh264enc",
    "! video/x-raw,format=I420 ! nvh264enc preset=low-latency-hp rc-mode=cbr "
  },
  {
    "vaapih264enc",
    "vaapih264enc",
    "! video/x-raw,format=NV12 ! vaapih264enc rate-control=2 "
  },
  {
    "x264enc",
    "x264enc",
    "! video/x-raw,format=I420 ! x264enc tune=zerolatency speed-preset=ultrafast "
  },
};

bool probe_encoder(const EncoderProfile & profile)
{
  if (!gst_element_factory_find(profile.element.c_str())) {
    return false;
  }

  const std::string pipeline_str =
    "videotestsrc num-buffers=1 ! video/x-raw,width=640,height=480,framerate=10/1 "
    "! videoconvert " + profile.pipeline_mid +
    "bitrate=2000 " + gop_property(profile.element, 15) + " ! h264parse ! fakesink";

  GError * error = nullptr;
  GstElement * pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
  if (!pipeline) {
    if (error) {
      std::cerr << "[uav_rtsp_stream] probe failed for " << profile.name
                << ": " << error->message << std::endl;
      g_error_free(error);
    }
    return false;
  }

  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  GstState state = GST_STATE_NULL;
  const GstStateChangeReturn ret =
    gst_element_get_state(pipeline, &state, nullptr, 5 * GST_SECOND);
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
  return ret != GST_STATE_CHANGE_FAILURE && state == GST_STATE_PLAYING;
}

}  // namespace

EncoderProfile detect_encoder(const std::string & preferred)
{
  gst_init(nullptr, nullptr);

  if (preferred != "auto") {
    for (const auto & profile : kProfiles) {
      if (profile.element == preferred && probe_encoder(profile)) {
        std::cout << "[uav_rtsp_stream] using requested encoder: "
                  << profile.name << std::endl;
        return profile;
      }
    }
    std::cerr << "[uav_rtsp_stream] requested encoder '" << preferred
              << "' unavailable, falling back to auto-detect" << std::endl;
  }

  for (const auto & profile : kProfiles) {
    if (probe_encoder(profile)) {
      std::cout << "[uav_rtsp_stream] detected encoder: "
                << profile.name << std::endl;
      return profile;
    }
  }

  throw std::runtime_error("[uav_rtsp_stream] no H.264 encoder available");
}

std::string build_encoder_fragment(
  const EncoderProfile & profile,
  int bitrate_kbps,
  int gop_size)
{
  return profile.pipeline_mid +
         "bitrate=" + std::to_string(bitrate_kbps) + " " +
         gop_property(profile.element, gop_size) + " ";
}

std::string detect_jpeg_decoder(const std::string & preferred)
{
  if (preferred == "auto") {
    if (gst_element_factory_find("nvjpegdec")) {
      std::cout << "[uav_rtsp_stream] using GPU JPEG decoder: nvjpegdec" << std::endl;
      return "nvjpegdec";
    }
  } else if (preferred == "nvjpegdec") {
    if (gst_element_factory_find("nvjpegdec")) {
      std::cout << "[uav_rtsp_stream] using GPU JPEG decoder: nvjpegdec" << std::endl;
      return "nvjpegdec";
    }
    std::cerr << "[uav_rtsp_stream] nvjpegdec unavailable, falling back to jpegdec"
              << std::endl;
  }

  // USB UVC cameras over usbipd/WSL often fail with nvjpegdec; jpegdec is more reliable.
  std::cout << "[uav_rtsp_stream] using CPU JPEG decoder: jpegdec" << std::endl;
  return "jpegdec";
}

bool encoder_skips_videoconvert(const std::string & element)
{
  return element == "nvh264enc" || element == "x264enc";
}

}  // namespace uav_rtsp_stream
