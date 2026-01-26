#include <memory>
#include <string>
#include <deque>
#include <numeric>
#include "rclcpp/rclcpp.hpp"
#include "avp_core_implementation/msg/vehicle_state.hpp"
#include "avp_core_implementation/msg/parking_slot.hpp"

using std::placeholders::_1;

// register a name of ROS node and connect to ROS network
class AVPController : public rclcpp::Node
{
public:
    AVPController() : Node("avp_controller_mode")
    {
        sub_vehicleState = this->create_subscription<avp_core_implementation::msg::VehicleState>(
            "/avp/vehicle_state", 10, std::bind(&AVPController::vehicle_callback, this, _1)
        );

        sub_parkingSlot = this->create_subscription<avp_core_implementation::msg::ParkingSlot>(
            "/avp/parking_slot", 10, std::bind(&AVPController::parking_slot_callback, this, _1)
        );

        // recent 5 windows
        filter_window_size_ = 5;
    
        RCLCPP_INFO(this->get_logger(), "AVP Controller Node Started.");
    }

private:
  // vehicle state process callback function
  void vehicle_callback(const avp_core_implementation::msg::VehicleState::SharedPtr msg)
  {
    float raw_vel = msg->velocity;
    float filtered_vel = 0.0;
    //add recent velocity to velocity_buffer
    velocity_buffer_.push_back(raw_vel);
    // check the velocity_buffer size 
    if(velocity_buffer_.size() > filter_window_size_){
        velocity_buffer_.pop_front(); // pop first one
    }
    // sum of entire data in velocity_buffer_ and devide by the size of velocity_buffer_
    filtered_vel = std::accumulate(velocity_buffer_.begin(), velocity_buffer_.end(), 0.0) / velocity_buffer_.size();
    

    // is the vehicle mode auto?
    if(msg->mode == avp_core_implementation::msg::VehicleState::MODE_AUTO){
        // check remain battery
        if(msg->battery < 10){
            RCLCPP_WARN(this->get_logger(), "Battery is low.");
        }
    }
    RCLCPP_INFO(this->get_logger(), 
                "Recv Vehicle: [Mode: %d] [Vel: %.1f] [Bat: %.1f] [Raw: %.2f] [Filtered: %.2f]", 
                msg->mode, msg->velocity, msg->battery, raw_vel, filtered_vel);
  }

  // parking slot process callback function
  void parking_slot_callback(const avp_core_implementation::msg::ParkingSlot::SharedPtr msg)
  {
    // is parking slot empty? is the width at least 2.3m?
    if(!msg->is_occupied && msg->width >= 2.3){
        // print empty parking slot id and width
        RCLCPP_INFO(this->get_logger(), "Parking Available! [ID: %d] Width: %.2fm", msg->id, msg->width);
    }    
  }

  // Declare member variables
  rclcpp::Subscription<avp_core_implementation::msg::VehicleState>::SharedPtr sub_vehicleState;
  rclcpp::Subscription<avp_core_implementation::msg::ParkingSlot>::SharedPtr sub_parkingSlot;

  std::deque<float> velocity_buffer_;
  int filter_window_size_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AVPController>());
  rclcpp::shutdown();
  return 0;
}

