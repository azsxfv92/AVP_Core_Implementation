#include <chrono>
#include <memory>
#include <string>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "avp_core_implementation/msg/vehicle_state.hpp"
#include "avp_core_implementation/msg/parking_slot.hpp"

// #define TIMER_PERIOD 500
using namespace std::chrono_literals;


class AVPMainNode : public rclcpp::Node{
public:
    AVPMainNode() : Node("avp_main_node"){
        pub_vehecleState = this->create_publisher<avp_core_implementation::msg::VehicleState>(
            "/avp/vehicle_state", 10
        );

        pub_parkingSlot = this->create_publisher<avp_core_implementation::msg::ParkingSlot>(
            "/avp/parking_slot", 10
        );

        // init seed
        std::random_device rd;
        // initialize an random number generator engine using seed
        gen_ = std::mt19937(rd());
        dist_ = std::normal_distribution<float>(15.0, 2.0);

        prob_dist_ = std::uniform_real_distribution<float>(0.0, 1.0);

        // timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_PERIOD), std::bind(&AVPMainNode::timer_callback, this));

        vs_period_ms_ = this->declare_parameter<int>("vs_period_ms", 500);
        ps_period_ms_ = this->declare_parameter<int>("ps_period_ms", 500);
        ps_jitter_ms_ = this->declare_parameter<int>("ps_jitter_ms", 0);
        enable_jitter_ = this->declare_parameter<bool>("enable_jitter", false);

        timer_vehicle_ = this->create_wall_timer(std::chrono::milliseconds(vs_period_ms_), std::bind(&AVPMainNode::publish_vehicle_state, this));
        timer_parking_ = this->create_wall_timer(std::chrono::milliseconds(ps_period_ms_), std::bind(&AVPMainNode::publish_parking_slot, this));
    }

private:
    void publish_vehicle_state(){
        auto current_time = this->now();
        auto vehicle_msg = avp_core_implementation::msg::VehicleState();

        vehicle_msg.header.stamp = current_time;
        vehicle_msg.header.frame_id = "base_link";
        vehicle_msg.mode = avp_core_implementation::msg::VehicleState::MODE_AUTO;
        vehicle_msg.velocity = dist_(gen_);
        vehicle_msg.battery = 88.0;

        RCLCPP_INFO(this->get_logger(), 
                    "Current time: %.2f, velocity : %.1f, battery : %.1f", 
                    current_time.seconds(),   
                    vehicle_msg.velocity, 
                    vehicle_msg.battery);

        pub_vehecleState->publish(vehicle_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing Vehicle State Data...");
    }
    void publish_parking_slot(){
        auto slot_msg = avp_core_implementation::msg::ParkingSlot();

        if(enable_jitter_ && ps_jitter_ms_ > 0){
            std::uniform_int_distribution<int> dist(0, ps_jitter_ms_);
            const int delay = dist(gen_);
            rclcpp::sleep_for(std::chrono::milliseconds(delay));
        }

        auto current_time = this->now();
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
        RCLCPP_INFO(this->get_logger(), "Publishing Parking Slot Data...");
    }

    rclcpp::TimerBase::SharedPtr timer_vehicle_;
    rclcpp::TimerBase::SharedPtr timer_parking_;

    int vs_period_ms_;
    int ps_period_ms_;
    int ps_jitter_ms_;
    bool enable_jitter_;

    rclcpp::Publisher<avp_core_implementation::msg::VehicleState>::SharedPtr pub_vehecleState;
    rclcpp::Publisher<avp_core_implementation::msg::ParkingSlot>::SharedPtr pub_parkingSlot;   

    std::mt19937 gen_;
    std::normal_distribution<float> dist_;
    std::uniform_real_distribution<float> prob_dist_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AVPMainNode>());
    rclcpp::shutdown();
    return 0;
}






