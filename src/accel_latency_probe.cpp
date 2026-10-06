#include <atomic>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "shm_channel.hpp"

class AccelLatencyProbe : public rclcpp::Node
{
public:
    AccelLatencyProbe() : Node("accel_latency_probe")
    {
        setvbuf(stdout, nullptr, _IOLBF, 0);   

        ch_ = rtmw::open_accel_channel(false);  
        if (!ch_.valid()) {
            RCLCPP_FATAL(get_logger(), "shm channel not found - start aeb_node first");
            throw std::runtime_error("shm channel not found");
        }

        std::printf("# probe start name=%s clock=CLOCK_MONOTONIC\n", rtmw::accel_shm_name());
        std::printf("path,seq,age_ns\n");

        sub_ = create_subscription<std_msgs::msg::Float32>(
            "/avp/vehicle/target_accel", 10,
            [this](std_msgs::msg::Float32::ConstSharedPtr) {
                const uint64_t t1 = rtmw::mono_ns();

                rtmw::AccelSample s;
                if (!rtmw::read_accel(ch_.slot, &s) || s.seq == 0) return;

                if (s.seq == last_ros_seq_) return;
                last_ros_seq_ = s.seq;

                std::printf("ros,%lu,%lu\n",
                            (unsigned long)s.seq, (unsigned long)(t1 - s.stamp_ns));
            });

        running_ = true;
        shm_thread_ = std::thread([this]() {
            uint64_t last = 0;
            {   
                rtmw::AccelSample s0;
                if (rtmw::read_accel(ch_.slot, &s0)) last = s0.seq;
            }
            while (running_.load(std::memory_order_relaxed)) {
                rtmw::AccelSample s;
                if (rtmw::read_accel(ch_.slot, &s) && s.seq != 0 && s.seq != last) {
                    last = s.seq;
                    const uint64_t t1 = rtmw::mono_ns();
                    std::printf("shm,%lu,%lu\n",
                                (unsigned long)s.seq, (unsigned long)(t1 - s.stamp_ns));
                }
            }
        });
    }

    ~AccelLatencyProbe() override
    {
        running_ = false;
        if (shm_thread_.joinable()) shm_thread_.join();
    }

private:
    rtmw::AccelChannel ch_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    std::thread shm_thread_;
    std::atomic<bool> running_{false};
    uint64_t last_ros_seq_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AccelLatencyProbe>());
    rclcpp::shutdown();
    return 0;
}
