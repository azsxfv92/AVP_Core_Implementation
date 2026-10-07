#pragma once
//
// RT-MW v0 채널 — 같은 기계의 두 프로세스가 "최신 값 하나"를 주고받는다.
//
// 대상 구간: aeb_node (C++) → tools/week16_carla_scenario.py (Python)
//            ROS2 토픽 /avp/vehicle/target_accel (std_msgs/Float32) 를 대체한다.
//
// 설계 결정
//   - 큐가 아니라 "최신 값" 하나만 유지한다. 제어 명령이라 지나간 값은 쓸모가 없다
//   - 쓰는 쪽 1명, 읽는 쪽 N명. 쓰는 쪽은 절대 기다리지 않는다 (실시간 경로)
//   - 직렬화 없음. 같은 기계·고정 크기 POD 라 메모리 그대로 읽는다
//   - 찢김(반쯤 쓰인 값)은 시퀀스 번호를 앞뒤로 두 번 읽어서 감지한다
//
// TODO 1, 2 를 채운다. 막히면 docs/week18/d4_answers.md
//

#include <atomic>
#include <cstdint>
#include <cstring>     // std::memcpy
#include <fcntl.h>     // O_CREAT, O_RDWR
#include <sys/mman.h>  // shm_open, mmap, MAP_SHARED
#include <unistd.h>    // ftruncate, close

#include "time_ns.hpp"        // rtmw::mono_ns()
#include "shm_ptr.hpp"        // ShmPtr, ShmDeleter (W18 D3 에서 만든 것)

namespace rtmw {

// struct for shared memory
struct AccelSlot {
    std::atomic<uint64_t> seq;   // Even = stable, Odd = being written. New value detection is also based on this value
    uint64_t              stamp_ns;  // publish time
    float                 accel;     // accel data
    float                 _pad;     
};

static_assert(std::atomic<uint64_t>::is_always_lock_free,
              "atomic<uint64_t> must be lock-free to live in shared memory");
static_assert(sizeof(AccelSlot) == 24,
              "layout is shared with week16_carla_scenario.py - keep it 24 bytes");

// copy struct for reader 
struct AccelSample {
    uint64_t seq;
    uint64_t stamp_ns;
    float    accel;
};

// write accel data while seq is locked
inline void write_accel(AccelSlot* slot, float accel) {
    // check if seq is locked
    const uint64_t s0 = slot->seq.load(std::memory_order_relaxed);

    // write seq as odd value to lock seq
    slot->seq.store(s0+1, std::memory_order_release);
    // to prevent from change the execution order 
    std::atomic_thread_fence(std::memory_order_release);

    slot->stamp_ns = mono_ns();
    slot->accel = accel;
    
    // remove the seq lock
    slot->seq.store(s0+2, std::memory_order_release);
}

// read accel data while seq is not locked
inline bool read_accel(const AccelSlot* slot, AccelSample* out) {
    // use "memory_order_acquire" to prevent from changing the execution order
    const uint64_t s1 = slot->seq.load(std::memory_order_acquire);
    if((s1 % 2) == 1){
        return false;
    }
    else{
        out->accel = slot->accel;
        out->stamp_ns = slot->stamp_ns;
        std::atomic_thread_fence(std::memory_order_acquire);
        uint64_t s2 = slot->seq.load(std::memory_order_relaxed);
        if(s1 != s2){
            return false;
        }
    }
    
    out->seq = s1;
    return true;
}

// the name and size of shm using in aeb node and CARLA scenario
inline const char* accel_shm_name() { return "/avp_target_accel"; }
inline std::size_t accel_shm_size() { return sizeof(AccelSlot); }


struct AccelChannel {
    ShmPtr     mapping;          // 소멸하면 munmap
    AccelSlot* slot = nullptr;   // mapping 안을 가리킨다 (소유하지 않는다)

    bool valid() const { return slot != nullptr; }
};

// open channel for sharing accel data 
inline AccelChannel open_accel_channel(bool create) {
    const int flags = create ? (O_CREAT | O_RDWR) : O_RDWR;
    // open with the name and get fd 
    int fd = shm_open(accel_shm_name(), flags, 0600);
    if(fd == -1){
        return {};
    }
    
    // check if the size was defined
    if(create && ftruncate(fd, static_cast<off_t>(accel_shm_size())) == -1) {
        close(fd);
        return {};
    }

    // change the channel to pointer 
    void* p = mmap(nullptr, accel_shm_size(), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd); // cloase the fd but mapping is still alive
    if(p == MAP_FAILED){
        return {};
    }

    AccelChannel ch;
    // create the shm_ptr
    ch.mapping = ShmPtr(p, ShmDeleter{accel_shm_size()});
    ch.slot    = static_cast<AccelSlot*>(p);
    return ch;
}

}  // namespace rtmw
