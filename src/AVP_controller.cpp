#include <memory>
#include <string>
#include <deque>
#include <numeric>
#include <random>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "avp_core_implementation/msg/vehicle_state.hpp"
#include "avp_core_implementation/msg/parking_slot.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using std::placeholders::_1;

// register a name of ROS node and connect to ROS network
class AVPController : public rclcpp::Node
{
public:
    AVPController() : Node("avp_controller_node"), running_(true)
    {
        // create subscription for vehicle state information message
        sub_vehicleState = this->create_subscription<avp_core_implementation::msg::VehicleState>(
            "/avp/vehicle_state", 10, std::bind(&AVPController::vehicle_callback, this, _1)
        );
        // create subscription for parking slot information message
        sub_parkingSlot = this->create_subscription<avp_core_implementation::msg::ParkingSlot>(
            "/avp/parking_slot", 10, std::bind(&AVPController::parking_slot_callback, this, _1)
        );
        // create publisher for filtered velocity message
        pub_filtered_vel = this->create_publisher<std_msgs::msg::Float32>(
            "/avp/velocity_filtered", 10
        );

        // notice the used parameters to ROS2
        declare_parameter<int>("filter_window_size_", 5);
        declare_parameter<int>("median_window_size_", 5);
        // get the declared parameters as member variables
        get_parameter("filter_window_size_", filter_window_size_);
        get_parameter("median_window_size_", median_window_size_);
        // request alarm to notice the parameters change
        params_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&AVPController::param_callback, this, std::placeholders::_1)
        );

        vehicle_queue_max_size = 5;
        vehicle_drop_count_ = 0;
        // add worker thread
        vehicle_worker_ = std::thread(&AVPController::vehicle_worker_loop, this);

        RCLCPP_INFO(this->get_logger(), "AVP Controller Node Started.");
        RCLCPP_INFO(this->get_logger(), "Worker Thread Started");
    }

    ~AVPController(){
        // finish worker thread
        running_ = false;
        // to wake up the worker thread
        vehicle_cv_.notify_all();
        
        if(vehicle_worker_.joinable()){
            // wait for worker thread
            vehicle_worker_.join();
        }
    }

private:
  // vehicle state process callback function
  void vehicle_callback(const avp_core_implementation::msg::VehicleState::SharedPtr msg)
  {
    {
        std::lock_guard<std::mutex> lock(vehicle_queue_mtx_);
        // check whether the queue is full 
        if(vehicle_msg_queue_.size() > vehicle_queue_max_size){
            // drop the oldest msg
            vehicle_msg_queue_.pop_front();
            vehicle_drop_count_++;
            RCLCPP_INFO(this->get_logger(), "======== The oldest msg is dropped, drop count is %d ========", vehicle_drop_count_ );
        }
        RCLCPP_INFO(this->get_logger(), "========PUSH======");
        vehicle_msg_queue_.push_back(msg);
        RCLCPP_INFO(this->get_logger(), "msg is pushed in vehicle_callback funcion");
    }
    vehicle_cv_.notify_one();
  }

  void vehicle_worker_loop()
  {
    while(running_){
        avp_core_implementation::msg::VehicleState::SharedPtr msg;
        {
            std::unique_lock<std::mutex> lock(vehicle_queue_mtx_);

            // wait for finish signal or message 
            vehicle_cv_.wait(lock, [this](){
                return !vehicle_msg_queue_.empty() || !running_;

            });
            
            if(!running_){
                break;
            }

            // pop msg from queue
            msg = vehicle_msg_queue_.front();
            vehicle_msg_queue_.pop_front();
            RCLCPP_INFO(this->get_logger(), "msg is poped in vehicle_worker_loop funcion");
        }
        process_vehicle_message(msg);
    }
  }

  void process_vehicle_message(const avp_core_implementation::msg::VehicleState::SharedPtr msg)
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

    // publish the filtered sensor data value
    auto filtered_msg = std_msgs::msg::Float32();
    filtered_msg.data = filtered_vel;
    pub_filtered_vel->publish(filtered_msg);
    

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

    // for message drop test
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // parking slot process callback function
  void parking_slot_callback(const avp_core_implementation::msg::ParkingSlot::SharedPtr msg)
  {
    float raw_width = msg->width;
    float filtered_width = 0.0;

    // push the current data and buffer size check
    width_buffer_.push_back(raw_width);
    if(width_buffer_.size() > median_window_size_){
        width_buffer_.pop_front();
    }
    
    // process to adopt medium filter
    std::vector<float>  temp_vector(width_buffer_.begin(), width_buffer_.end());
    // sort the temp_vector and adopt the middle of index 
    std::sort(temp_vector.begin(), temp_vector.end());
    int mid = temp_vector.size()/2;
    filtered_width = temp_vector[mid];

    // is parking slot empty? is the width at least 2.3m?
    if(!msg->is_occupied && msg->width >= 2.3){
        // print empty parking slot id and width
        RCLCPP_INFO(this->get_logger(), 
                    "Parking Available! [ID: %d] Width: %.2fm, [Raw: %.2f] -> [Filtered_width: %.2f]: ", 
                    msg->id, msg->width, raw_width, filtered_width);
    }    
  }

  // detect parameter change and adopt it
  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> &parameters){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        // check target parameters
        for(const auto &param : parameters){
            if(param.get_name() == "median_window_size_"){
                int filterSize = param.as_int();
                if(filterSize < 1){
                    result.successful = false;
                    result.reason = "Size mismatch";
                }
                else{
                    // update window size
                    median_window_size_ = filterSize;
                    RCLCPP_INFO(this->get_logger(), "Median Window Size Changed: %d", median_window_size_);
                }
            }
            else if(param.get_name() == "filter_window_size_"){
                int filterSize = param.as_int();
                if(filterSize < 1){
                    result.successful = false;
                    result.reason  = "Size Mismatch";
                }
                else{
                    // update window size
                    filter_window_size_ = filterSize;
                    RCLCPP_INFO(this->get_logger(), "Filtered Window Size Changed: %d", filter_window_size_);
                }
            }
        }
        return result;
    }

  // Declare member variables
  rclcpp::Subscription<avp_core_implementation::msg::VehicleState>::SharedPtr sub_vehicleState;
  rclcpp::Subscription<avp_core_implementation::msg::ParkingSlot>::SharedPtr sub_parkingSlot;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_filtered_vel;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  std::deque<float> velocity_buffer_;
  std::deque<float> width_buffer_;
  int filter_window_size_;
  int median_window_size_;
  int vehicle_queue_max_size;
  int vehicle_drop_count_;
  std::deque<avp_core_implementation::msg::VehicleState::SharedPtr> vehicle_msg_queue_;
  std::mutex vehicle_queue_mtx_;
  std::condition_variable vehicle_cv_;
  std::thread vehicle_worker_;
  std::atomic<bool> running_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AVPController>());
  rclcpp::shutdown();
  return 0;
}

