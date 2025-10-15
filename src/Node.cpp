/****
Adam Srbenjak Yang Updated ESKF for ros2 jazzy
we will eliminate the mavros stuff for now, this is not necessary for basic
the goal is to fuse imu data with unscaled pose estimates 
look at imu and look at vision
vision represents a monocular odometry estimate


update from ros1 to ros2 jazzy!

merge in the main here





*/
#include <eskf/Node.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include<cstdint>

using sensor_msgs::msg::Imu;
using sensor_msgs::msg::MagneticField;
using sensor_msgs::msg::Range;
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::PoseWithCovarianceStamped;

namespace eskf
{

static inline rclcpp::Time to_time(const builtin_interfaces::msg::Time & t) {
  return rclcpp::Time(t);
}

eskf_node::eskf_node(const rclcpp::NodeOptions& options) : rclcpp::Node("eskf_node", options), init_(false) {
  //Ros2 needs to receive from the parameter server!
  imu_in_     = declare_parameter<std::string>("imu_in",    "/livox/imu");
  vision_in_     = declare_parameter<std::string>("vision_in",    "/orbslam3/pose");
  gps_in_     = declare_parameter<std::string>("gps_in",    "/gps/odom");
  mag_in_     = declare_parameter<std::string>("mag_in",    "/mag");
  range_in_     = declare_parameter<std::string>("range_in",    "/range");

  int fusion_mask = default_fusion_mask_;
  this->declare_parameter<int>("fusion_mask", fusion_mask);
  this->get_parameter("fusion_mask", fusion_mask);

  int publish_rate = default_publish_rate_;
  this->declare_parameter<int>("publish_rate", publish_rate);
  this->get_parameter("publish_rate", publish_rate);

  eskf_.setFusionMask(fusion_mask);


  
  // --- QoS profiles ---
  auto sensor_qos = rclcpp::ServicesQoS();//rclcpp::SensorDataQoS(); // for IMU, mag, range (best-effort, small history)
  auto default_qos = rclcpp::QoS(10);        // for VO/GPS/pose out

  // SUBSCRIBERS
  RCLCPP_INFO(get_logger(), "Subscribing to imu"); // REPLACES ROS_INFO RCLCPP_INFO(get_logger(),"HEELO WORLD");
  subImu_ = this->create_subscription<Imu>(
      imu_in_, sensor_qos,
      std::bind(&eskf_node::inputCallback, this, std::placeholders::_1));
  

  //NEXT
  if ((fusion_mask & MASK_EV_POS) || (fusion_mask & MASK_EV_YAW) || (fusion_mask & MASK_EV_HGT)) {
    RCLCPP_INFO(get_logger(), "Subscribing to vision");
    subVisionPose_ = this->create_subscription<PoseWithCovarianceStamped>(
        vision_in_, default_qos,
        std::bind(&eskf_node::visionCallback, this, std::placeholders::_1));
  }


  if ((fusion_mask & MASK_GPS_POS) || (fusion_mask & MASK_GPS_VEL) || (fusion_mask & MASK_GPS_HGT)) {
    RCLCPP_INFO(get_logger(), "Subscribing to gps");
    subGpsPose_ = this->create_subscription<Odometry>(
        gps_in_, default_qos,
        std::bind(&eskf_node::gpsCallback, this, std::placeholders::_1));
  }

  if (fusion_mask & MASK_MAG_HEADING) {
    RCLCPP_INFO(get_logger(), "Subscribing to mag");
    subMagPose_ = this->create_subscription<MagneticField>(
        mag_in_, sensor_qos,
        std::bind(&eskf_node::magCallback, this, std::placeholders::_1));
  }

  if (fusion_mask & MASK_RANGEFINDER) {
    RCLCPP_INFO(get_logger(), "Subscribing to rangefinder");
    subRangeFinderPose_ = this->create_subscription<Range>(
        range_in_, sensor_qos,
        std::bind(&eskf_node::rangeFinderCallback, this, std::placeholders::_1));
  }

  pubPose_ = this->create_publisher<Odometry>("pose", default_qos);
// --- Timer (publish fused odometry at publish_rate) ---
  using namespace std::chrono_literals;
  const auto period = std::chrono::milliseconds( (publish_rate > 0) ? (1000 / publish_rate) : 10 );
  pub_timer_ = this->create_wall_timer(period, std::bind(&eskf_node::publishState, this));

  RCLCPP_INFO(get_logger(), "eskf_node initialized. fusion_mask=0x%X, publish_rate=%d Hz", fusion_mask, publish_rate);

}
//////////////////////////
void eskf_node::inputCallback(const Imu::ConstSharedPtr& imuMsg) {

  vec3 wm = vec3(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z); //  measured angular rate
  vec3 am = vec3(imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z); //  measured linear acceleration
  const rclcpp::Time t = to_time(imuMsg->header.stamp);

  if (prevStampImu_.nanoseconds() != 0) {
    const double delta = (t - prevStampImu_).seconds();

    if (!init_) {
      init_ = true;
      RCLCPP_INFO(get_logger(), "Initialized ESKF");
        }
    const uint64_t usec = static_cast<uint64_t>(t.seconds() * 1e6);
    //  run kalman filter
    eskf_.run(wm, am, usec, delta);
  }
  prevStampImu_ = t;
}


  //////////////////////////////////////
void eskf_node::visionCallback(const PoseWithCovarianceStamped::ConstSharedPtr& poseMsg) {

    const rclcpp::Time t = to_time(poseMsg->header.stamp);
  if(prevStampVisionPose_.nanoseconds() != 0) {
    const double delta = (t - prevStampVisionPose_).seconds();
    // get measurements
    quat z_q = quat(poseMsg->pose.pose.orientation.w, poseMsg->pose.pose.orientation.x, poseMsg->pose.pose.orientation.y, poseMsg->pose.pose.orientation.z);
    vec3 z_p = vec3(poseMsg->pose.pose.position.x, poseMsg->pose.pose.position.y, poseMsg->pose.pose.position.z);
    // update vision
    const uint64_t usec = static_cast<uint64_t>(t.seconds() * 1e6);

    eskf_.updateVision(z_q, z_p, usec, delta);
  }
  prevStampVisionPose_ = t;
}

void eskf_node::gpsCallback(const Odometry::ConstSharedPtr& odomMsg) {

      const rclcpp::Time t = to_time(odomMsg->header.stamp);

  if (prevStampGpsPose_.nanoseconds() != 0) {
    const double delta = (t - prevStampGpsPose_).seconds();
    // get gps measurements
    vec3 z_v = vec3(odomMsg->twist.twist.linear.x, odomMsg->twist.twist.linear.y, odomMsg->twist.twist.linear.z);
    vec3 z_p = vec3(odomMsg->pose.pose.position.x, odomMsg->pose.pose.position.y, odomMsg->pose.pose.position.z);
    // update gps
        const uint64_t usec = static_cast<uint64_t>(t.seconds() * 1e6);

    eskf_.updateGps(z_v, z_p,usec, delta);
  }
  prevStampGpsPose_ = t;
}



void eskf_node::magCallback(const MagneticField::ConstSharedPtr& magMsg) {
        const rclcpp::Time t = to_time(magMsg->header.stamp);

  if (prevStampMagPose_.nanoseconds() != 0) {
    const double delta = (t - prevStampMagPose_).seconds();
    // get mag measurements
    vec3 m = vec3(magMsg->magnetic_field.x * 1e4f, magMsg->magnetic_field.y * 1e4f, magMsg->magnetic_field.z * 1e4f);
    const uint64_t usec = static_cast<uint64_t>(t.seconds() * 1e6);

    eskf_.updateMagnetometer(m, usec, delta);
  }
  prevStampMagPose_ = t;
}

void eskf_node::rangeFinderCallback(const Range::ConstSharedPtr& rangeMsg) {

          const rclcpp::Time t = to_time(rangeMsg->header.stamp);

  if (prevStampRangeFinderPose_.nanoseconds() != 0) {
    const double delta = (t - prevStampRangeFinderPose_).seconds();

        const uint64_t usec = static_cast<uint64_t>(t.seconds() * 1e6);
    // get rangefinder measurements
    eskf_.updateRangeFinder(rangeMsg->range,usec, delta);
  }
  prevStampRangeFinderPose_ =t;
}



void eskf_node::publishState() {

  // get kalman filter result
  const quat e2g = eskf_.getQuat();
  const vec3 position = eskf_.getPosition();
  const vec3 velocity = eskf_.getVelocity();

  static size_t trace_id_ = 0;
  std_msgs::msg::Header header;

  //FRAME DEFAULTS
  std::string frame_id = "map";
  std::string child_frame_id = "base_link";
  this->get_parameter("frame_id", frame_id);
  this->get_parameter("child_frame_id", child_frame_id);

  //
  header.frame_id = frame_id;
  //  header.child_frame_id = child_frame_id;

  //header.seq = trace_id_++;
  header.stamp = this->now();

  nav_msgs::msg::Odometry odom;
  odom.header = header;
  odom.pose.pose.position.x = position[0];
  odom.pose.pose.position.y = position[1];
  odom.pose.pose.position.z = position[2];
  odom.twist.twist.linear.x = velocity[0];
  odom.twist.twist.linear.y = velocity[1];
  odom.twist.twist.linear.z = velocity[2];
  odom.pose.pose.orientation.w = e2g.w();
  odom.pose.pose.orientation.x = e2g.x();
  odom.pose.pose.orientation.y = e2g.y();
  odom.pose.pose.orientation.z = e2g.z();

  pubPose_->publish(odom);
}

}