#ifndef CORE_OPTIMIZER_HPP
#define CORE_OPTIMIZER_HPP

#include <iostream>
#include <vector>
#include <memory>
#include <queue>
#include <algorithm> 
#include <cstdint>

struct Frame {
    uint32_t id;
    std::vector<uint8_t> data;
    Frame() { data.resize(1920*1080*3);
    // make dummy data for test
    std::fill(data.begin(), data.end(), 0xAA); 
    }
};

class ObjectPool {
private:
    std::queue<std::unique_ptr<Frame>> pool;
    const size_t max_size = 10;

public:
    std::unique_ptr<Frame> acquire() {
        //if pool is empty, make new pool
        if(pool.empty()){
            return std::make_unique<Frame>();
        }

        //if pool is not empty
        auto obj = std::move(pool.front()); // address of pool
        pool.pop();
        return obj;
    }

    void release(std::unique_ptr<Frame> frame){
        if(pool.size() < max_size){
            pool.push(std::move(frame));
        }
    }
};

class PoolGuard{
private:
    ObjectPool& pool_;
    std::unique_ptr<Frame> frame_;

public:
    PoolGuard(ObjectPool& p) : pool_(p), frame_(pool_.acquire()){}

    ~PoolGuard(){
        if (frame_){
            pool_.release(std::move(frame_));
        }
    }

    Frame* operator->() { return frame_.get();}
    Frame& operator*() {return *frame_;}

    PoolGuard(const PoolGuard&) = delete;
    PoolGuard& operator = (const PoolGuard&) = delete;
};

#endif