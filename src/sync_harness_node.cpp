#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "avp_core_implementation/msg/parking_slot.hpp"
#include "avp_core_implementation/msg/vehicle_state.hpp"
// #include "avp_core_implementation/avp_core_implementation/msg/parking_slot.hpp"
// #include "avp_core_implementation/avp_core_implementation/msg/vehicle_state.hpp"

using VehicleState = avp_core_implementation::msg::VehicleState;
using ParkingSlot = avp_core_implementation::msg::ParkingSlot;

namespace{
    inline int64_t to_ns(const builtin_interfaces::msg::Time& t){
        return (int64_t)t.sec * 1000000000LL + (int64_t)t.nanosec;
    }
}

class SyncHarnessNode : public rclcpp::Node{
public:
    SyncHarnessNode(): Node("sync_harness_node"){
        policy_     = this->declare_parameter<std::string>("policy", "approx");
        queue_size_ = this->declare_parameter<int>("queue_size", 10);
        slop_ms_    = this->declare_parameter<int>("slop_ms", 100);
        report_sec_ = this->declare_parameter<double>("report_sec", 2.0);

        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(queue_size_)).reliable().durability_volatile();
        auto rmw_qos = qos.get_rmw_qos_profile();
        sub_vs_.subscribe(this, "/avp/vehicle_state", rmw_qos);
        sub_ps_.subscribe(this, "/avp/parking_slot", rmw_qos);

        if(policy_ == "exact"){
            using ExactPolicy = message_filters::sync_policies::ExactTime<VehicleState, ParkingSlot>;
            exact_sync_ = std::make_shared<message_filters::Synchronizer<ExactPolicy>>(ExactPolicy(queue_size_), sub_vs_, sub_ps_);
            exact_sync_->registerCallback(std::bind(&SyncHarnessNode::onSync, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "Sync policy = ExactTime (queue=%d)", queue_size_);
        }
        else{
            using ApproxPolicy = message_filters::sync_policies::ApproximateTime<VehicleState, ParkingSlot>;
            approx_sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy>>(ApproxPolicy(queue_size_), sub_vs_, sub_ps_);
            approx_sync_->setMaxIntervalDuration(rclcpp::Duration(0, (int64_t)slop_ms_*1000000LL));
            approx_sync_->registerCallback(std::bind(&SyncHarnessNode::onSync, this, std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "Sync policy = ApproximateTIme (queue=%d, slop_ms=%d)", queue_size_, slop_ms_);
        }

        report_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(report_sec_)),
            std::bind(&SyncHarnessNode::report, this)
        );
    }
private:
    void onSync(const VehicleState::ConstSharedPtr& vs, const ParkingSlot::ConstSharedPtr& ps){
        auto t0 = std::chrono::steady_clock::now();
        const auto vs_ns = (int64_t)vs->header.stamp.sec * 1000000000LL + (int64_t)vs->header.stamp.nanosec;
        const auto ps_ns = (int64_t)ps->header.stamp.sec * 1000000000LL + (int64_t)ps->header.stamp.nanosec;
        const double skew_ms = std::abs((double)(vs_ns - ps_ns)) / 1e6;

        sync_count_++;
        skew_samples_ms_.push_back(skew_ms);

        auto t1 = std::chrono::steady_clock::now();
        const auto cb_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        cb_latency_us_.push_back((double)cb_us);

        if((sync_count_ % 100) == 0){
            RCLCPP_INFO(this->get_logger(), "sync_count=%ld, skew_ms=%.3f, cb_ns=%d", sync_count_, skew_ms, cb_us);
        }
    }   

    void report() {
        double last_skew = skew_samples_ms_.empty() ? -1.0 : skew_samples_ms_.back();
        double last_cb = cb_latency_us_.empty() ? -1.0 : cb_latency_us_.back();
        
        RCLCPP_INFO(
            this->get_logger(),
            "policy=%s, sync_count=%ld, last_skew_ms=%.3f, last_cb_us=%.1f",
            policy_.c_str(), sync_count_, last_skew, last_cb
        );
    }
private:
    std::string policy_;
    int queue_size_;
    int slop_ms_;
    double report_sec_;

    message_filters::Subscriber<VehicleState> sub_vs_;
    message_filters::Subscriber<ParkingSlot> sub_ps_;

    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<VehicleState, ParkingSlot>>> exact_sync_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<VehicleState, ParkingSlot>>> approx_sync_;

    rclcpp::TimerBase::SharedPtr report_timer_;

    int64_t sync_count_{0};
    std::vector<double> skew_samples_ms_;
    std::vector<double> cb_latency_us_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncHarnessNode>());
    rclcpp::shutdown();
    return 0;
}

