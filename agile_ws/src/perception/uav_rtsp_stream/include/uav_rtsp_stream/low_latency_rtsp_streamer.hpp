#ifndef UAV_RTSP_STREAM__LOW_LATENCY_RTSP_STREAMER_HPP_
#define UAV_RTSP_STREAM__LOW_LATENCY_RTSP_STREAMER_HPP_

#include <atomic>
#include <functional>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <thread>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/rtsp-server/rtsp-server.h>

#include "uav_rtsp_stream/encoder_profile.hpp"

namespace uav_rtsp_stream
{

struct StreamerConfig
{
  std::string source_type = "camera";  // "camera" | "ros"
  std::string device = "/dev/video0";
  int width = 640;
  int height = 480;
  int fps = 30;
  // Normalized: "raw" | "mjpg" | "h264"
  std::string input_format = "mjpg";
  bool publish_rtsp_stream = false;
  bool publish_ros_image = true;
  int rtsp_port = 8554;
  std::string stream_uri = "/uav1/live";
  int bitrate_kbps = 2500;
  int gop_size = 1;
  std::string encoder = "auto";
  std::string jpeg_decoder = "jpegdec";
  int io_mode = 2;
};

using RosFrameCallback = std::function<void(const cv::Mat & frame)>;

// Thread-safe single-slot frame buffer: always keeps the latest frame.
class LatestFrameBuffer
{
public:
  void publish(const cv::Mat & frame);
  bool take(cv::Mat & out);

private:
  std::mutex mutex_;
  cv::Mat latest_;
  bool has_frame_{false};
};

class LowLatencyRtspStreamer
{
public:
  explicit LowLatencyRtspStreamer(const StreamerConfig & config);
  ~LowLatencyRtspStreamer();

  LowLatencyRtspStreamer(const LowLatencyRtspStreamer &) = delete;
  LowLatencyRtspStreamer & operator=(const LowLatencyRtspStreamer &) = delete;

  bool start(RosFrameCallback ros_cb = nullptr);
  void stop();
  void push_frame(const cv::Mat & frame);

  std::string rtsp_url() const;
  bool publish_rtsp() const { return config_.publish_rtsp_stream; }
  bool publish_ros() const { return config_.publish_ros_image; }

private:
  bool start_camera();
  bool start_ros_to_rtsp();
  bool start_camera_rtsp_only();
  bool start_camera_ros_only();
  bool start_camera_shared();

  bool setup_rtsp_server_launch(const std::string & launch_string, bool use_appsrc);
  bool start_owned_pipeline(const std::string & pipeline_str, bool connect_ros_sink);
  bool preconstruct_shared_media();
  static void on_media_configure_static(
    GstRTSPMediaFactory * factory, GstRTSPMedia * media, gpointer user_data);
  void on_media_configure(GstRTSPMedia * media);
  bool attach_rtsp_server();
  void run_main_loop();
  void ensure_encoder();
  void connect_ros_appsink(GstElement * pipeline);

  std::string camera_src_head() const;
  std::string caps_str() const;
  std::string rtsp_encode_branch() const;
  std::string rtsp_encode_branch_from_raw() const;
  std::string rtsp_h264_passthrough_branch() const;
  std::string ros_appsink_branch() const;
  std::string ros_appsink_branch_from_raw() const;
  std::string build_shared_camera_pipeline() const;

  void on_need_data(GstElement * appsrc);
  static void on_need_data_static(GstElement * appsrc, guint, gpointer user_data);
  static GstFlowReturn on_new_sample_static(GstAppSink * sink, gpointer user_data);
  GstFlowReturn on_new_sample(GstAppSink * sink);

  StreamerConfig config_;
  EncoderProfile encoder_;
  bool encoder_ready_{false};
  RosFrameCallback ros_cb_;

  GstRTSPServer * server_{nullptr};
  GstRTSPMediaFactory * factory_{nullptr};
  GstElement * owned_pipeline_{nullptr};
  GMainLoop * loop_{nullptr};
  std::thread loop_thread_;
  std::atomic<bool> running_{false};

  LatestFrameBuffer frame_buffer_;
  GstElement * appsrc_{nullptr};
  bool ros_sink_connected_{false};

  guint64 frame_index_{0};
  GstClockTime frame_duration_{0};
};

std::string normalize_input_format(const std::string & value);

}  // namespace uav_rtsp_stream

#endif  // UAV_RTSP_STREAM__LOW_LATENCY_RTSP_STREAMER_HPP_
