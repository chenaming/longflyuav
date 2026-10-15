#include "uav_rtsp_stream/low_latency_rtsp_streamer.hpp"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>
#include <string>

namespace uav_rtsp_stream
{

class RtspServerNode
{
public:
  explicit RtspServerNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  {
    load_config(pnh);

    RosFrameCallback ros_cb;
    if (config_.source_type == "camera" && config_.publish_ros_image) {
      image_pub_ = nh.advertise<sensor_msgs::Image>(image_topic_, 1);
      const std::string frame_id = frame_id_;
      ros_cb = [this, frame_id](const cv::Mat & frame) {
        publish_ros_image(frame, frame_id);
      };
    }

    streamer_ = std::make_unique<LowLatencyRtspStreamer>(config_);
    if (!streamer_->start(ros_cb)) {
      throw std::runtime_error("failed to start streamer");
    }

    ROS_INFO(
      "uav_rtsp_stream ready: source=%s format=%s %dx%d@%d rtsp=%s ros_image=%s",
      config_.source_type.c_str(),
      config_.input_format.c_str(),
      config_.width, config_.height, config_.fps,
      config_.publish_rtsp_stream ? "on" : "off",
      config_.publish_ros_image ? "on" : "off");

    if (config_.publish_rtsp_stream) {
      ROS_INFO("RTSP url: %s", streamer_->rtsp_url().c_str());
    }
    if (config_.publish_ros_image) {
      ROS_INFO("publishing ROS image on %s", image_topic_.c_str());
    }

    if (config_.source_type == "ros") {
      sub_ = nh.subscribe(
        subscribe_image_topic_, 1, &RtspServerNode::on_image, this,
        ros::TransportHints().tcpNoDelay());
      ROS_INFO("subscribed to %s", subscribe_image_topic_.c_str());
    }
  }

  ~RtspServerNode()
  {
    if (streamer_) {
      streamer_->stop();
    }
  }

private:
  void load_config(ros::NodeHandle & pnh)
  {
    int uav_id = 1;
    int base_port = 8554;
    pnh.param<int>("uav_id", uav_id, 1);
    pnh.param<std::string>("source_type", config_.source_type, "camera");
    pnh.param<std::string>("device", config_.device, "/dev/video0");
    pnh.param<int>("width", config_.width, 640);
    pnh.param<int>("height", config_.height, 480);
    pnh.param<int>("fps", config_.fps, 30);
    pnh.param<std::string>("input_format", config_.input_format, "mjpg");
    pnh.param<bool>("publish_rtsp_stream", config_.publish_rtsp_stream, false);
    pnh.param<bool>("publish_ros_image", config_.publish_ros_image, true);
    pnh.param<int>("bitrate", config_.bitrate_kbps, 2500);
    pnh.param<int>("gop_size", config_.gop_size, 1);
    pnh.param<std::string>("encoder", config_.encoder, "auto");
    pnh.param<std::string>("jpeg_decoder", config_.jpeg_decoder, "jpegdec");
    pnh.param<int>("io_mode", config_.io_mode, 2);
    pnh.param<int>("base_port", base_port, 8554);
    pnh.param<std::string>("image_topic", image_topic_, "camera/image_raw");
    pnh.param<std::string>("frame_id", frame_id_, "camera_link");
    pnh.param<std::string>(
      "subscribe_image_topic", subscribe_image_topic_, "/image_raw");

    if (uav_id < 1) {
      throw std::runtime_error("uav_id must be >= 1");
    }

    config_.rtsp_port = base_port + uav_id - 1;
    config_.stream_uri = "/uav" + std::to_string(uav_id) + "/live";
    config_.input_format = normalize_input_format(config_.input_format);

    if (config_.source_type == "ros") {
      config_.publish_rtsp_stream = true;
      config_.publish_ros_image = false;
    }
  }

  void publish_ros_image(const cv::Mat & frame, const std::string & frame_id)
  {
    if (!image_pub_) {
      return;
    }
    try {
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = frame_id;
      const auto msg =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame)
          .toImageMsg();
      image_pub_.publish(msg);
    } catch (const cv_bridge::Exception & ex) {
      ROS_WARN_THROTTLE(2.0, "cv_bridge publish error: %s", ex.what());
    }
  }

  void on_image(const sensor_msgs::ImageConstPtr & msg)
  {
    if (!streamer_) {
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
      } else {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    } catch (const cv_bridge::Exception & ex) {
      ROS_WARN_THROTTLE(2.0, "cv_bridge error: %s", ex.what());
      return;
    }

    cv::Mat frame;
    if (cv_ptr->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(cv_ptr->image, frame, cv::COLOR_RGB2BGR);
    } else {
      frame = cv_ptr->image;
    }
    streamer_->push_frame(frame);
  }

  StreamerConfig config_;
  std::string image_topic_;
  std::string frame_id_;
  std::string subscribe_image_topic_;
  std::unique_ptr<LowLatencyRtspStreamer> streamer_;
  ros::Publisher image_pub_;
  ros::Subscriber sub_;
};

}  // namespace uav_rtsp_stream

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "rtsp_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    uav_rtsp_stream::RtspServerNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception & ex) {
    ROS_ERROR("%s", ex.what());
    return 1;
  }
  return 0;
}
