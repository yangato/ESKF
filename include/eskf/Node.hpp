/**
Needs modifications to allow for the use with ros2 jazzy!

modifications should be done
*/
#ifndef ESKF_NODE_HPP_
#define ESKF_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
//#include <message_filters/subscriber.h>
#include <eskf/ESKF.hpp>
#include<cstdint>

namespace eskf {

  class eskf_node : public rclcpp::Node {
  public:
    static constexpr int default_publish_rate_ = 100;
    static constexpr int default_fusion_mask_ = MASK_EV; // ensure MASK_EV is defined

    explicit eskf_node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  private:

    // publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubPose_;

    //  subsribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subVisionPose_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGpsPose_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr subMagPose_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subRangeFinderPose_;
  // Topic names (populated from parameters)
    std::string imu_in_;
    std::string vision_in_;
    std::string gps_in_;
    std::string mag_in_;
    std::string range_in_;
    // implementation
    eskf::ESKF eskf_;
    rclcpp::Time prevStampImu_;
    rclcpp::Time prevStampVisionPose_;
    rclcpp::Time prevStampGpsPose_;
    rclcpp::Time prevStampMagPose_;
    rclcpp::Time prevStampRangeFinderPose_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    bool init_{false};
    //  callbacks
    void inputCallback(const sensor_msgs::msg::Imu::ConstSharedPtr&);
    void visionCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr&);
    void gpsCallback(const nav_msgs::msg::Odometry::ConstSharedPtr&);
    void magCallback(const sensor_msgs::msg::MagneticField::ConstSharedPtr&);
    void rangeFinderCallback(const sensor_msgs::msg::Range::ConstSharedPtr&);
    void publishState();
    };
} //  namespace eskf

#endif // ESKF_NODE_HPP_
