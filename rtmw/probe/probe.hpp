#pragma once

#include <ctime>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include <utility>
#include <builtin_interfaces/msg/time.hpp>

#include "../include/time_ns.hpp"   // rtmw::mono_ns()

namespace rtmw{


inline uint64_t real_ns(){
    timespec ts;
    // to measure the section beyind the machine
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec)*1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

inline uint64_t frame_key(const builtin_interfaces::msg::Time& t){
    return static_cast<uint64_t>(t.sec)*1000000000ULL + static_cast<uint64_t>(t.nanosec);
}

struct Rec{
    uint64_t frame;
    uint16_t point;
    uint64_t mono;
    uint64_t real;
};

class Probe{

public: 
    Probe(std::string node, std::string path, size_t cap = 300000, int cal_n = 20000)
        : node_(std::move(node)), path_(std::move(path))
    {
        // to avoid delays caused by vector reallocation during capacity expansion
        buf_.reserve(cap);
        calibrate(cal_n);
    }

    ~Probe(){
        flush();
    }
    
    inline void mark(uint64_t frame, uint16_t point){
        if(buf_.size() < buf_.capacity()){
            buf_.push_back(Rec{frame, point, mono_ns(), real_ns()});
        }
        else{
            ++dropped_;
        }   
    }

    inline void mark_at(uint64_t frame, uint16_t point, uint64_t mono, uint64_t real){
        if(buf_.size() < buf_.capacity()){
            buf_.push_back({frame, point, mono, real});
        }
        else{
            ++dropped_;
        }   
    }

    size_t dropped() const{
        return dropped_;
    }

    size_t size() const{
        return buf_.size();
    }

    void flush(){
        FILE* fp = std::fopen(path_.c_str(), "w");
        if(!fp) return;
        std::fprintf(fp, "node,frame,point,mono_ns,real_ns\n");
        for(const auto& r : buf_){
            std::fprintf(fp, "%s,%llu,%u,%llu,%llu\n", node_.c_str(),
                         static_cast<unsigned long long>(r.frame),
                         static_cast<unsigned>(r.point),
                         static_cast<unsigned long long>(r.mono),
                         static_cast<unsigned long long>(r.real));
        }
        std::fclose(fp);
    }

private:
    void calibrate(int n){
        uint64_t warm = 0;
        for(int i = 0; i < 1000; ++i){
            warm += mono_ns();
        }
        (void)warm;

        for(int i = 0; i < n; ++i){
            const uint64_t a = mono_ns();
            const uint64_t b = mono_ns();
            if(buf_.size() < buf_.capacity()){
                buf_.push_back(Rec{static_cast<uint64_t>(i), 0, a, b});
            }
            else{
                ++dropped_;
            }

        }
    }

    std::string node_, path_;
    std::vector<Rec> buf_;
    size_t dropped_ = 0;
};

} // namespace rtmw
