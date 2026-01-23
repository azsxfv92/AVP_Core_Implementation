#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "avp_core_implementation/msg/vehicle_state.hpp"
#include "avp_core_implementation/msg/parking_slot.hpp"

using std::placeholders::_1;

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
    
        RCLCPP_INFO(this->get_logger(), "AVP Controller Node Started.");
    }

private:
  // vehicle state process callback function
  void vehicle_callback(const avp_core_implementation::msg::VehicleState::SharedPtr msg)
  {
    // is the vehicle mode auto?
    if(msg->mode == avp_core_implementation::msg::VehicleState::MODE_AUTO){
        // check remain battery
        if(msg->battery < 10){
            RCLCPP_WARN(this->get_logger(), "Battery is low.");
        }
    }
    RCLCPP_INFO(this->get_logger(), "Recv Vehicle: [Mode: %d] [Vel: %.1f] [Bat: %.1f]", msg->mode, msg->velocity, msg->battery);
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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AVPController>());
  rclcpp::shutdown();
  return 0;
}

