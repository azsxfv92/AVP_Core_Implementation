#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class AebNode : public rclcpp::Node
{
public:
    AebNode() : Node("aeb_node")
    {
        brake_dist_     = declare_parameter<double>("break_distance", 12.0);
        lane_half_      = declare_parameter<double>("lane_half_width", 1.5);
        min_exist_      = declare_parameter<double>("min_existence", 0.4);
        brake_accel_    = declare_parameter<double>("brake_accel", -5.0);
        release_hold_time_ = declare_parameter<double>("release_hold_time", 0.5);
        target_speed_   = declare_parameter<double>("target_speed", 8.0);
        timeout_sec_    = declare_parameter<double>("input_timeout", 1.0);
        vehicle_frame_  = declare_parameter<std::string>("vehicle_frame", "base_link");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

        sub_track_ = create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
            "/perception/object_recognition/tracking/objects",
            qos, std::bind(&AebNode::onTracked, this, std::placeholders::_1)
        );

        detection_topic_ = declare_parameter<std::string>(
            "detection_topic", "/perception/object_recognition/detection/objects");
        sub_detection_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
            detection_topic_, qos,
            [this](const autoware_perception_msgs::msg::DetectedObjects::SharedPtr) {
                last_detection_time_ = now();
                has_detection_input_ = true;
            });

        sub_speed_ = create_subscription<std_msgs::msg::Float32>(
            "/avp/vehicle/speed", 10,
            [this](const std_msgs::msg::Float32::SharedPtr m){current_speed_ = m->data;}
        );
        
        pub_accel_ = create_publisher<std_msgs::msg::Float32>("/avp/vehicle/target_accel", 10);
        pub_status_ = create_publisher<std_msgs::msg::Bool>("/avp/aeb/status", 10);

        timer_ = create_wall_timer(50ms, std::bind(&AebNode::onControlTick, this));

        RCLCPP_INFO(get_logger(), "aeb_node up | brake_dist=%.1fm lane_half=%.1fm", brake_dist_, lane_half_);
    }

private:
    void onTracked(const autoware_perception_msgs::msg::TrackedObjects::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf;
        try{
            tf = tf_buffer_->lookupTransform(
                vehicle_frame_, msg->header.frame_id, tf2::TimePointZero);
        } catch(const tf2::TransformException & ex){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF 실패: %s", ex.what());
            return;
        }

        bool raw_hazard = false;
        double nearest_x = 0.0, nearest_y = 0.0, nearest_p = 0.0;

        for(const auto & obj : msg->objects){
            if(obj.existence_probability < min_exist_) continue;

            geometry_msgs::msg::PoseStamped in, out;
            in.header = msg->header;
            in.pose = obj.kinematics.pose_with_covariance.pose;

            // convert the coordinate
            tf2::doTransform(in, out, tf);

            const double bx = out.pose.position.x;
            const double by = out.pose.position.y;

            if(bx > 0.0 && bx < brake_dist_ && std::abs(by) < lane_half_){
                raw_hazard = true;
                nearest_x = bx;
                nearest_y = by;
                nearest_p = obj.existence_probability;
                break;
            }
        }

        if(raw_hazard){
            last_hazard_seen_time_ = now();
            have_seen_hazard_ = true;
        }

        const bool held = have_seen_hazard_ &&
            (now() - last_hazard_seen_time_).seconds() < release_hold_time_;
        const bool new_hazard = raw_hazard || held;

        if(new_hazard && !hazard_){
            RCLCPP_WARN(get_logger(), "AEB ENGAGED :: x=%.1fm y=%.1fm p=%.2f",
                nearest_x, nearest_y, nearest_p);
        } else if(!new_hazard && hazard_){
            RCLCPP_WARN(get_logger(), "AEB RELEASED");
        }

        hazard_ = new_hazard;
        last_valid_time_ = now();
        has_input_ = true;
    }

    void onControlTick()
    {
        bool tracking_timed_out = true;
        if(has_input_){
            tracking_timed_out = (now() - last_valid_time_).seconds() > timeout_sec_;
        }

        bool detection_timed_out = true;
        if(has_detection_input_){
            detection_timed_out = (now() - last_detection_time_).seconds() > timeout_sec_;
        }

        const bool timed_out = tracking_timed_out || detection_timed_out;

        double accel;
        if(hazard_ || timed_out){
            accel = brake_accel_;
        }
        else{
            const double err = target_speed_ - current_speed_;
            accel = std::clamp(err * 0.5, -1.0, 2.0);
        }

        std_msgs::msg::Float32 a;
        a.data = static_cast<float>(accel);
        pub_accel_->publish(a);

        std_msgs::msg::Bool s;
        s.data = (hazard_ || timed_out);
        pub_status_->publish(s);

        if(++heartbeat_tick_ >= 100){
            heartbeat_tick_ = 0;
            RCLCPP_INFO(get_logger(),
                "alive | hazard=%d timed_out=%d(track=%d,detect=%d) speed=%.1f accel=%.2f",
                hazard_, timed_out, tracking_timed_out, detection_timed_out,
                current_speed_, accel);
        }
    }
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr sub_track_;
    rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr sub_detection_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_speed_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_accel_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_;
    rclcpp::TimerBase::SharedPtr timer_;

    double brake_dist_, lane_half_, min_exist_, brake_accel_, target_speed_, timeout_sec_;
    double release_hold_time_;
    std::string vehicle_frame_;
    std::string detection_topic_;
    double current_speed_{0.0};
    bool hazard_{false}, has_input_{false};
    rclcpp::Time last_valid_time_;
    bool has_detection_input_{false};
    rclcpp::Time last_detection_time_;
    int heartbeat_tick_{0};
    bool have_seen_hazard_{false};
    rclcpp::Time last_hazard_seen_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AebNode>());
  rclcpp::shutdown();
  return 0;
}