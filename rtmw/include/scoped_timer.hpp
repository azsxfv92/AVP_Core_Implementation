#pragma once

#include <cstdint>
#include <cstdio>
#include <ctime>

namespace rtmw {

inline uint64_t mono_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}

class ScopedTimer {

public:
    explicit ScopedTimer(const char* name){
        name_ = name;
        t0_ns_ = mono_ns();
    }

    ~ScopedTimer() {
        if(name_ == nullptr) return;
        uint64_t elapsedTime;
        elapsedTime = mono_ns() - t0_ns_;
        std::printf("%s: %llu ns\n", name_, static_cast<unsigned long long>(elapsedTime));
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator = (const ScopedTimer&) = delete;

    ScopedTimer(ScopedTimer&& other) noexcept
        : name_(other.name_), t0_ns_(other.t0_ns_){
        other.name_ = nullptr;
    }

private:
    const char* name_ = nullptr;
    uint64_t    t0_ns_ = 0;
};

} // namespace rtmw
