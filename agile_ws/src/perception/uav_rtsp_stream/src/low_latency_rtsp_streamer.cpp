#include "uav_rtsp_stream/low_latency_rtsp_streamer.hpp"

#include <gst/rtsp/gstrtspurl.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace uav_rtsp_stream
{

namespace
{

std::string to_lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

}  // namespace

std::string normalize_input_format(const std::string & value)
{
  std::string fmt = to_lower(value);
  if (fmt == "yuyv" || fmt == "uyvy" || fmt == "nv12") {
    return "raw";
  }
  if (fmt == "mjpeg" || fmt == "jpeg" || fmt == "mjpg") {
    return "mjpg";
  }
  if (fmt == "h264") {
    return "h264";
  }
  if (fmt == "raw" || fmt == "mjpg") {
    return fmt;
  }
  throw std::runtime_error(
    "[uav_rtsp_stream] unsupported input_format '" + value +
    "', choose from: raw, mjpg, h264");
}

void LatestFrameBuffer::publish(const cv::Mat & frame)
{
  if (frame.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = frame;
  has_frame_ = true;
}

bool LatestFrameBuffer::take(cv::Mat & out)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_frame_ || latest_.empty()) {
    return false;
  }
  out = latest_;
  return true;
}

LowLatencyRtspStreamer::LowLatencyRtspStreamer(const StreamerConfig & config)
: config_(config)
{
  config_.input_format = normalize_input_format(config_.input_format);
  if (config_.fps <= 0) {
    throw std::runtime_error("[uav_rtsp_stream] fps must be > 0");
  }
  frame_duration_ = gst_util_uint64_scale(1, GST_SECOND, config_.fps);
}

LowLatencyRtspStreamer::~LowLatencyRtspStreamer()
{
  stop();
}

std::string LowLatencyRtspStreamer::rtsp_url() const
{
  return "rtsp://127.0.0.1:" + std::to_string(config_.rtsp_port) + config_.stream_uri;
}

void LowLatencyRtspStreamer::ensure_encoder()
{
  if (encoder_ready_) {
    return;
  }
  encoder_ = detect_encoder(config_.encoder);
  encoder_ready_ = true;
}

bool LowLatencyRtspStreamer::start(RosFrameCallback ros_cb)
{
  if (running_) {
    return true;
  }

  ros_cb_ = std::move(ros_cb);

  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }

  if (config_.source_type == "ros") {
    config_.publish_rtsp_stream = true;
    config_.publish_ros_image = false;
    return start_ros_to_rtsp();
  }
  if (config_.source_type == "camera") {
    return start_camera();
  }

  std::cerr << "[uav_rtsp_stream] unknown source_type: "
            << config_.source_type << std::endl;
  return false;
}

bool LowLatencyRtspStreamer::start_camera()
{
  if (!config_.publish_rtsp_stream && !config_.publish_ros_image) {
    std::cerr << "[uav_rtsp_stream] at least one of publish_rtsp_stream / "
                 "publish_ros_image must be true"
              << std::endl;
    return false;
  }

  if (config_.publish_rtsp_stream && config_.input_format != "h264") {
    ensure_encoder();
  }

  if (config_.publish_rtsp_stream && config_.publish_ros_image) {
    return start_camera_shared();
  }
  if (config_.publish_rtsp_stream) {
    return start_camera_rtsp_only();
  }
  return start_camera_ros_only();
}

std::string LowLatencyRtspStreamer::caps_str() const
{
  return "width=" + std::to_string(config_.width) +
         ",height=" + std::to_string(config_.height) +
         ",framerate=" + std::to_string(config_.fps) + "/1";
}

std::string LowLatencyRtspStreamer::camera_src_head() const
{
  std::ostringstream ss;
  ss << "v4l2src device=" << config_.device
     << " io-mode=" << config_.io_mode << " do-timestamp=true ! ";
  if (config_.input_format == "mjpg") {
    ss << "image/jpeg," << caps_str() << " ! ";
  } else if (config_.input_format == "h264") {
    ss << "video/x-h264," << caps_str() << " ! ";
  } else {
    ss << "video/x-raw," << caps_str() << " ! ";
  }
  return ss.str();
}

std::string LowLatencyRtspStreamer::rtsp_encode_branch() const
{
  const std::string encoder = build_encoder_fragment(
    encoder_, config_.bitrate_kbps, config_.gop_size);
  const std::string jpeg_decoder = detect_jpeg_decoder(config_.jpeg_decoder);

  std::ostringstream ss;
  if (config_.input_format == "mjpg") {
    ss << jpeg_decoder;
    if (!encoder_skips_videoconvert(encoder_.element)) {
      ss << " ! videoconvert ";
    } else {
      ss << " ";
    }
  } else {
    // raw
    ss << "videoconvert ";
  }
  ss << encoder
     << " ! h264parse config-interval=1 "
     << " ! rtph264pay name=pay0 pt=96 config-interval=1";
  return ss.str();
}

std::string LowLatencyRtspStreamer::rtsp_h264_passthrough_branch() const
{
  return "h264parse config-interval=1 ! "
         "rtph264pay name=pay0 pt=96 config-interval=1";
}

std::string LowLatencyRtspStreamer::rtsp_encode_branch_from_raw() const
{
  const std::string encoder = build_encoder_fragment(
    encoder_, config_.bitrate_kbps, config_.gop_size);

  // encoder fragment starts with "! video/x-raw,format=... ! x264enc ...";
  // prefix with "videoconvert " (space only) to avoid "queue ! ! video/x-raw".
  std::ostringstream ss;
  ss << "videoconvert " << encoder
     << " ! h264parse config-interval=1 "
     << " ! rtph264pay name=pay0 pt=96 config-interval=1";
  return ss.str();
}

std::string LowLatencyRtspStreamer::ros_appsink_branch_from_raw() const
{
  const std::string sink =
    "appsink name=ros_sink emit-signals=true sync=false "
    "max-buffers=2 drop=true";
  const std::string queue = "queue leaky=downstream max-size-buffers=2 ! ";
  const std::string to_bgr = "videoconvert ! video/x-raw,format=BGR ! ";
  return queue + to_bgr + sink;
}

std::string LowLatencyRtspStreamer::build_shared_camera_pipeline() const
{
  const std::string rtsp_queue =
    "queue max-size-buffers=1 leaky=downstream ! ";

  std::ostringstream pipeline;
  pipeline << "( " << camera_src_head();

  if (config_.input_format == "mjpg") {
    const std::string jpeg_decoder = detect_jpeg_decoder(config_.jpeg_decoder);
    pipeline << jpeg_decoder << " ! video/x-raw ! tee name=t "
             << "t. ! " << rtsp_queue << rtsp_encode_branch_from_raw()
             << " t. ! " << ros_appsink_branch_from_raw();
  } else if (config_.input_format == "h264") {
    // H.264 passthrough for RTSP; ROS branch still needs its own decode.
    pipeline << "tee name=t "
             << "t. ! " << rtsp_queue << rtsp_h264_passthrough_branch()
             << " t. ! " << ros_appsink_branch();
  } else {
    pipeline << "tee name=t "
             << "t. ! " << rtsp_queue << rtsp_encode_branch_from_raw()
             << " t. ! " << ros_appsink_branch_from_raw();
  }

  pipeline << " )";
  return pipeline.str();
}

std::string LowLatencyRtspStreamer::ros_appsink_branch() const
{
  const std::string sink =
    "appsink name=ros_sink emit-signals=true sync=false "
    "max-buffers=2 drop=true";
  const std::string queue = "queue leaky=downstream max-size-buffers=2 ! ";
  const std::string to_bgr = "videoconvert ! video/x-raw,format=BGR ! ";

  if (config_.input_format == "mjpg") {
    const std::string jpeg_decoder = detect_jpeg_decoder(config_.jpeg_decoder);
    return queue + jpeg_decoder + " ! " + to_bgr + sink;
  }
  if (config_.input_format == "h264") {
    return queue + "h264parse ! avdec_h264 ! " + to_bgr + sink;
  }
  return queue + to_bgr + sink;
}

bool LowLatencyRtspStreamer::start_camera_rtsp_only()
{
  std::ostringstream pipeline;
  pipeline << "( " << camera_src_head()
           << "queue max-size-buffers=1 leaky=downstream ! ";
  if (config_.input_format == "h264") {
    pipeline << rtsp_h264_passthrough_branch();
  } else {
    pipeline << rtsp_encode_branch();
  }
  pipeline << " )";
  return setup_rtsp_server_launch(pipeline.str(), false);
}

bool LowLatencyRtspStreamer::start_camera_ros_only()
{
  if (!ros_cb_) {
    std::cerr << "[uav_rtsp_stream] publish_ros_image requires a callback"
              << std::endl;
    return false;
  }
  const std::string pipeline = camera_src_head() + ros_appsink_branch();
  return start_owned_pipeline(pipeline, true);
}

bool LowLatencyRtspStreamer::start_camera_shared()
{
  if (!ros_cb_) {
    std::cerr << "[uav_rtsp_stream] publish_ros_image requires a callback"
              << std::endl;
    return false;
  }

  const std::string pipeline = build_shared_camera_pipeline();

  factory_ = gst_rtsp_media_factory_new();
  gst_rtsp_media_factory_set_launch(factory_, pipeline.c_str());
  gst_rtsp_media_factory_set_shared(factory_, TRUE);
  gst_rtsp_media_factory_set_suspend_mode(factory_, GST_RTSP_SUSPEND_MODE_NONE);
  g_signal_connect(
    factory_, "media-configure",
    G_CALLBACK(LowLatencyRtspStreamer::on_media_configure_static), this);

  std::cout << "[uav_rtsp_stream] pipeline (single-decode tee): "
            << pipeline << std::endl;
  if (!attach_rtsp_server()) {
    return false;
  }
  return preconstruct_shared_media();
}

bool LowLatencyRtspStreamer::start_ros_to_rtsp()
{
  ensure_encoder();
  const std::string encoder = build_encoder_fragment(
    encoder_, config_.bitrate_kbps, config_.gop_size);

  std::ostringstream pipeline;
  pipeline
    << "( appsrc name=source is-live=true format=time block=false max-buffers=1 "
    << "caps=video/x-raw,format=BGR,width=" << config_.width
    << ",height=" << config_.height
    << ",framerate=" << config_.fps << "/1 "
    << " ! videoconvert "
    << encoder
    << " ! h264parse config-interval=1 "
    << " ! rtph264pay name=pay0 pt=96 config-interval=1 )";

  return setup_rtsp_server_launch(pipeline.str(), true);
}

bool LowLatencyRtspStreamer::start_owned_pipeline(
  const std::string & pipeline_str, bool connect_ros_sink)
{
  GError * error = nullptr;
  owned_pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
  if (!owned_pipeline_) {
    std::cerr << "[uav_rtsp_stream] failed to parse pipeline: "
              << (error ? error->message : "unknown") << std::endl;
    if (error) {
      g_error_free(error);
    }
    return false;
  }
  if (error) {
    g_error_free(error);
  }

  std::cout << "[uav_rtsp_stream] pipeline: " << pipeline_str << std::endl;

  if (connect_ros_sink) {
    connect_ros_appsink(owned_pipeline_);
  }

  const GstStateChangeReturn ret =
    gst_element_set_state(owned_pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    std::cerr << "[uav_rtsp_stream] failed to set pipeline PLAYING" << std::endl;
    gst_object_unref(owned_pipeline_);
    owned_pipeline_ = nullptr;
    return false;
  }

  if (!loop_) {
    loop_ = g_main_loop_new(nullptr, FALSE);
    running_ = true;
    loop_thread_ = std::thread(&LowLatencyRtspStreamer::run_main_loop, this);
  } else {
    running_ = true;
  }
  return true;
}

void LowLatencyRtspStreamer::connect_ros_appsink(GstElement * pipeline)
{
  GstElement * sink = gst_bin_get_by_name(GST_BIN(pipeline), "ros_sink");
  if (!sink) {
    throw std::runtime_error("[uav_rtsp_stream] ros_sink not found in pipeline");
  }
  GstAppSinkCallbacks callbacks = {};
  callbacks.new_sample = on_new_sample_static;
  gst_app_sink_set_callbacks(GST_APP_SINK(sink), &callbacks, this, nullptr);
  gst_object_unref(sink);
}

bool LowLatencyRtspStreamer::setup_rtsp_server_launch(
  const std::string & launch_string, bool use_appsrc)
{
  factory_ = gst_rtsp_media_factory_new();
  gst_rtsp_media_factory_set_launch(factory_, launch_string.c_str());
  gst_rtsp_media_factory_set_shared(factory_, TRUE);

  if (use_appsrc) {
    g_signal_connect(
      factory_, "media-configure",
      G_CALLBACK(+[](GstRTSPMediaFactory * /*factory*/, GstRTSPMedia * media,
                     gpointer user_data) {
        auto * self = static_cast<LowLatencyRtspStreamer *>(user_data);
        GstElement * element = gst_rtsp_media_get_element(media);
        self->appsrc_ =
          GST_ELEMENT(gst_bin_get_by_name(GST_BIN(element), "source"));
        if (self->appsrc_) {
          g_signal_connect(
            self->appsrc_, "need-data",
            G_CALLBACK(LowLatencyRtspStreamer::on_need_data_static), self);
        }
        gst_object_unref(element);
      }),
      this);
  }

  std::cout << "[uav_rtsp_stream] pipeline: " << launch_string << std::endl;
  return attach_rtsp_server();
}

void LowLatencyRtspStreamer::on_media_configure_static(
  GstRTSPMediaFactory * /*factory*/, GstRTSPMedia * media, gpointer user_data)
{
  static_cast<LowLatencyRtspStreamer *>(user_data)->on_media_configure(media);
}

void LowLatencyRtspStreamer::on_media_configure(GstRTSPMedia * media)
{
  if (ros_sink_connected_ || !ros_cb_) {
    return;
  }

  GstElement * element = gst_rtsp_media_get_element(media);
  if (!element) {
    return;
  }
  connect_ros_appsink(element);
  ros_sink_connected_ = true;
  gst_object_unref(element);
}

bool LowLatencyRtspStreamer::preconstruct_shared_media()
{
  GstRTSPUrl * url = nullptr;
  const std::string full_url = rtsp_url();
  if (gst_rtsp_url_parse(full_url.c_str(), &url) != GST_RTSP_OK || !url) {
    std::cerr << "[uav_rtsp_stream] failed to parse RTSP url: "
              << full_url << std::endl;
    return false;
  }

  GstRTSPMedia * media = gst_rtsp_media_factory_construct(factory_, url);
  gst_rtsp_url_free(url);
  if (!media) {
    std::cerr << "[uav_rtsp_stream] failed to construct shared RTSP media"
              << std::endl;
    return false;
  }

  gst_rtsp_media_set_suspend_mode(media, GST_RTSP_SUSPEND_MODE_NONE);
  const gboolean prepared = gst_rtsp_media_prepare(media, nullptr);
  gst_object_unref(media);
  if (!prepared) {
    std::cerr << "[uav_rtsp_stream] failed to prepare shared RTSP media"
              << std::endl;
    return false;
  }
  return true;
}

bool LowLatencyRtspStreamer::attach_rtsp_server()
{
  server_ = gst_rtsp_server_new();
  const std::string port_str = std::to_string(config_.rtsp_port);
  g_object_set(
    server_, "service", port_str.c_str(), "address", "0.0.0.0", nullptr);

  GstRTSPMountPoints * mounts = gst_rtsp_server_get_mount_points(server_);
  gst_rtsp_mount_points_add_factory(mounts, config_.stream_uri.c_str(), factory_);
  gst_object_unref(mounts);

  const guint attach_id = gst_rtsp_server_attach(server_, nullptr);
  if (attach_id == 0) {
    std::cerr << "[uav_rtsp_stream] RTSP attach failed on port "
              << config_.rtsp_port
              << " (port in use? stop stale rtsp_server_node first)" << std::endl;
    if (server_) {
      gst_object_unref(server_);
      server_ = nullptr;
    }
    factory_ = nullptr;
    return false;
  }

  if (!loop_) {
    loop_ = g_main_loop_new(nullptr, FALSE);
    running_ = true;
    loop_thread_ = std::thread(&LowLatencyRtspStreamer::run_main_loop, this);
  } else {
    running_ = true;
  }

  std::cout << "[uav_rtsp_stream] RTSP ready at " << rtsp_url() << std::endl;
  return true;
}

void LowLatencyRtspStreamer::run_main_loop()
{
  g_main_loop_run(loop_);
}

void LowLatencyRtspStreamer::stop()
{
  running_ = false;
  if (loop_) {
    g_main_loop_quit(loop_);
  }
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }
  if (owned_pipeline_) {
    gst_element_set_state(owned_pipeline_, GST_STATE_NULL);
    gst_object_unref(owned_pipeline_);
    owned_pipeline_ = nullptr;
  }
  if (factory_) {
    gst_object_unref(factory_);
    factory_ = nullptr;
  }
  if (server_) {
    gst_object_unref(server_);
    server_ = nullptr;
  }
  if (loop_) {
    g_main_loop_unref(loop_);
    loop_ = nullptr;
  }
  appsrc_ = nullptr;
  ros_sink_connected_ = false;
}

void LowLatencyRtspStreamer::push_frame(const cv::Mat & frame)
{
  if (!running_ || config_.source_type != "ros") {
    return;
  }
  frame_buffer_.publish(frame);
}

void LowLatencyRtspStreamer::on_need_data_static(
  GstElement * appsrc, guint, gpointer user_data)
{
  static_cast<LowLatencyRtspStreamer *>(user_data)->on_need_data(appsrc);
}

void LowLatencyRtspStreamer::on_need_data(GstElement * appsrc)
{
  if (!running_) {
    return;
  }

  cv::Mat frame;
  if (!frame_buffer_.take(frame)) {
    return;
  }

  if (frame.cols != config_.width || frame.rows != config_.height) {
    cv::resize(frame, frame, cv::Size(config_.width, config_.height), 0, 0,
               cv::INTER_LINEAR);
  }

  if (!frame.isContinuous()) {
    frame = frame.clone();
  }

  const gsize size = static_cast<gsize>(frame.total() * frame.elemSize());
  GstBuffer * buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
  if (!buffer) {
    return;
  }

  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    gst_buffer_unref(buffer);
    return;
  }
  std::memcpy(map.data, frame.data, size);
  gst_buffer_unmap(buffer, &map);

  GST_BUFFER_PTS(buffer) = frame_index_ * frame_duration_;
  GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
  GST_BUFFER_DURATION(buffer) = frame_duration_;
  ++frame_index_;

  GstFlowReturn ret = GST_FLOW_OK;
  g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
  if (ret != GST_FLOW_OK) {
    std::cerr << "[uav_rtsp_stream] push-buffer failed: " << ret << std::endl;
    gst_buffer_unref(buffer);
  }
}

GstFlowReturn LowLatencyRtspStreamer::on_new_sample_static(
  GstAppSink * sink, gpointer user_data)
{
  return static_cast<LowLatencyRtspStreamer *>(user_data)->on_new_sample(sink);
}

GstFlowReturn LowLatencyRtspStreamer::on_new_sample(GstAppSink * sink)
{
  if (!ros_cb_) {
    return GST_FLOW_OK;
  }

  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (!sample) {
    return GST_FLOW_ERROR;
  }

  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstCaps * caps = gst_sample_get_caps(sample);
  if (!buffer || !caps) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  GstStructure * structure = gst_caps_get_structure(caps, 0);
  int width = 0;
  int height = 0;
  gst_structure_get_int(structure, "width", &width);
  gst_structure_get_int(structure, "height", &height);
  if (width <= 0 || height <= 0) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  cv::Mat frame(height, width, CV_8UC3, const_cast<guint8 *>(map.data));
  ros_cb_(frame.clone());
  gst_buffer_unmap(buffer, &map);
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

}  // namespace uav_rtsp_stream
