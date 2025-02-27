#include <rclcpp/rclcpp.hpp>
#include <libfreenect2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class Kinect2Node : public rclcpp::Node
{
public:
  Kinect2Node() : Node("kinect2_node")
  {
    // Initialize the Kinect device
    if (!freenect2_.enumerateDevices()) {
      RCLCPP_ERROR(this->get_logger(), "No Kinect v2 device found.");
      return;
    }

    std::string serial = freenect2_.getDeviceSerialNumber(0);
    device_ = freenect2_.openDevice(serial);

    if (!device_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open Kinect device.");
      return;
    }

    // Initialize the publisher for RGB and depth data
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("kinect/rgb_image", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("kinect/depth_image", 10);

    // Start the RGB and depth streams
    listener_ = std::make_shared<libfreenect2::SyncMultiFrameListener>(
        libfreenect2::Frame::Color | libfreenect2::Frame::Depth);

    device_->setColorFrameListener(listener_);
    device_->setDepthFrameListener(listener_);
    device_->start();
  }

  void start()
  {
    rclcpp::Rate loop_rate(30);  // 30 Hz
    while (rclcpp::ok()) {
      // Wait for new frames
      listener_->waitForNewFrame(frames_);

      // Convert RGB image to ROS message and publish
      sensor_msgs::msg::Image::SharedPtr rgb_msg =
          cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frames_[0]).toImageMsg();
      rgb_pub_->publish(*rgb_msg);

      // Convert depth image to ROS message and publish
      sensor_msgs::msg::Image::SharedPtr depth_msg =
          cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", frames_[1]).toImageMsg();
      depth_pub_->publish(*depth_msg);

      listener_->releaseFrame(frames_);

      rclcpp::spin_some(this->get_node_base_interface());
      loop_rate.sleep();
    }

    device_->stop();
    device_->close();
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  libfreenect2::Freenect2 freenect2_;
  libfreenect2::Freenect2Device* device_;
  libfreenect2::FrameMap frames_;
  std::shared_ptr<libfreenect2::SyncMultiFrameListener> listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Kinect2Node>());
  rclcpp::shutdown();
  return 0;
}
