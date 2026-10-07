//
// W18 D2 — shm_int : 두 프로세스가 정수 하나를 공유한다
//
// 빌드: g++ -std=c++17 -Wall -Wextra -O2 -g rtmw/test/shm_int.cpp -o rtmw/build/shm_int
//       (glibc 2.34 부터 shm_open 이 libc 안에 있어서 -lrt 는 필요 없다)
//
// 실행: ./rtmw/build/shm_int writer     # 0.5 초마다 1 씩 올린다
//       ./rtmw/build/shm_int reader     # 0.5 초마다 읽어서 찍는다
//       ./rtmw/build/shm_int unlink     # 이름표를 뗀다 (/dev/shm 에서 사라진다)
//
// TODO 1~3 을 채운다. 뼈대 그대로도 빌드는 되고, 실행하면 "map failed" 로 끝난다.
// 막히면 docs/week18/d2_answers.md 를 열어라.
//

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>        // std::strcmp
#include <memory>
#include <sys/mman.h>     // shm_open, mmap, munmap, shm_unlink, PROT_*, MAP_*
#include <unistd.h>       // ftruncate, close, usleep, getpid

#include "shm_channel.hpp"


// // TODO 3: 0.5 초마다 counter 를 1 올리고 찍는다.
// //   아래 run_reader 와 같은 모양이고, 읽는 대신 올린다: ++s->counter;
// static void run_writer(Shared* s) {
//     for(;;){
//         ++s->counter;
//         std::printf("writer: counter=%lu\n", (unsigned long)s->counter);
//         std::fflush(stdout);
//         usleep(500*1000);
//     }
// }

// ============================================================================
// 완성본 — 읽고 TODO 3 의 본보기로 쓴다
// ============================================================================

static void run_reader(unsigned long poll_us) {
    setvbuf(stdout, nullptr, _IOLBF, 0);

    rtmw::AccelChannel ch = rtmw::open_accel_channel(false);

    while(!ch.valid()){
        std::fprintf(stderr, "waiting for %s ...\n", rtmw::accel_shm_name());
        usleep(200 * 1000);
        ch = rtmw::open_accel_channel(false);
    }

    std::printf("# name=%s poll_us=%lu clock=CLOCK_MONOTONIC\n", rtmw::accel_shm_name(), poll_us);
    std::printf("seq,accel,age_ns\n");
    std::fflush(stdout);

    uint64_t last_seq = 0;
    for (;;) {
        rtmw::AccelSample s;
        if(rtmw::read_accel(ch.slot, &s) && s.seq != 0 && s.seq != last_seq){
            last_seq = s.seq;
            const uint64_t age_ns = rtmw::mono_ns() - s.stamp_ns;
            std::printf("%lu,%.3f,%lu\n", (unsigned long)s.seq, s.accel, (unsigned long)age_ns);
        }
        if(poll_us > 0){
            usleep(poll_us);
        }
    }
}

// // E-B: MAP_PRIVATE 로 건 뒤 내 사본에만 쓴다
// static void run_private(Shared* s) {
//     s->counter = 999;    /        // 이 쓰기가 copy-on-write 를 일으킨다
//     for (;;) {
//         std::printf("private: counter=%lu\n", (unsigned long)s->counter);
//         std::fflush(stdout);
//         usleep(500 * 1000);
//     }
// }

int main(int argc, char** argv) {
    const char* mode = (argc > 1) ? argv[1] : "";

    if (std::strcmp(mode, "unlink") == 0) {
        if (shm_unlink(rtmw::accel_shm_name()) != 0) {
            std::perror("shm_unlink");
            return 1;
        }
        std::printf("unlinked %s\n", rtmw::accel_shm_name());
        return 0;
    }

    if (std::strcmp(mode, "reader") != 0) {
        std::fprintf(stderr, "usage: %s reader [poll_us] | unlink\n", argv[0]);
        return 2;
    }

    const unsigned long poll_us = 
        (argc > 2) ? std::strtoul(argv[2], nullptr, 10) : 5000;
    run_reader(poll_us);
    return 0;
}
