#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "avp_core_implementation/msg/vehicle_state.hpp"
#include "avp_core_implementation/msg/parking_slot.hpp"

#define TIMER_PERIOD 500
using namespace std::chrono_literals;


class AVPMainNode : public rclcpp::Node{
public:
    AVPMainNode() : Node("avp_main_node"){
        pub_vehecleState = this->create_publisher<avp_core_implementation::msg::VehicleState>(
            "/avp/vehicle_state", 10
        );

        pub_parkingSlot = this->create_publisher<avp_core_implementation::msg::ParkingSlot>(
            "avp/parking_slot", 10
        );

        timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_PERIOD), std::bind(&AVPMainNode::timer_callback, this));
    }

private:
    void timer_callback(){
        auto current_time = this->now();
        // create VehicleState message and send it
        auto vehicle_msg = avp_core_implementation::msg::VehicleState();
        vehicle_msg.header.stamp = current_time;
        vehicle_msg.header.frame_id = "base_link";
        vehicle_msg.mode = avp_core_implementation::msg::VehicleState::MODE_AUTO;
        vehicle_msg.velocity = 15.5;
        vehicle_msg.battery = 88.0;

        RCLCPP_INFO(this->get_logger(), 
                    "Current time: %.2f, velocity : %.1f, battery : %.1f", 
                    current_time.seconds(),   
                    vehicle_msg.velocity, 
                    vehicle_msg.battery);
        pub_vehecleState->publish(vehicle_msg);

        auto slot_msg = avp_core_implementation::msg::ParkingSlot();
        slot_msg.header.stamp = current_time;
        slot_msg.width = 2.5;
        slot_msg.is_occupied = false;
        slot_msg.id = 101;
        
        RCLCPP_INFO(this->get_logger(), 
                    "Current time: %.2f, Width: %.1f, Is_occupied: %d, ID: %d",
                    current_time.seconds(), 
                    slot_msg.width, 
                    slot_msg.is_occupied, 
                    slot_msg.id);
        pub_parkingSlot->publish(slot_msg);

        RCLCPP_INFO(this->get_logger(), "Publishing Data...");
    }   
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<avp_core_implementation::msg::VehicleState>::SharedPtr pub_vehecleState;
    rclcpp::Publisher<avp_core_implementation::msg::ParkingSlot>::SharedPtr pub_parkingSlot;   
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AVPMainNode>());
    rclcpp::shutdown();
    return 0;
}






