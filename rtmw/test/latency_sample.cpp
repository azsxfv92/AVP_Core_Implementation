#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "scoped_timer.hpp"    

int main(int argc, char** argv) {
    const unsigned target_us = (argc > 1) ? std::strtoul(argv[1], nullptr, 10) : 1000;
    const unsigned n         = (argc > 2) ? std::strtoul(argv[2], nullptr, 10) : 2000;
    uint64_t overshoot = 0u;

    std::printf("# target_us=%u\n", target_us);
    std::printf("# n=%u\n", n);
    std::printf("# clock=CLOCK_MONOTONIC\n");
    std::printf("iter,elapsed_ns,overshoot_ns\n");

    for (int i = 0; i < 50; ++i) {
        usleep(target_us);
    }

    for (unsigned i = 0; i < n; ++i) {
        const uint64_t t0 = rtmw::mono_ns();
        usleep(target_us);
        const uint64_t elapsed = rtmw::mono_ns() - t0;
        
        const uint64_t target_ns = static_cast<uint64_t>(target_us) * 1000ULL;
        if(elapsed < target_ns){
            overshoot = 0;
        }
        else{
            overshoot = elapsed - target_ns;
        }

        std::printf("%u,%llu,%llu\n", i,
                    static_cast<unsigned long long>(elapsed),
                    static_cast<unsigned long long>(overshoot));
    }
    return 0;
}
