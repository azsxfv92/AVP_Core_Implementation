#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <pthread.h>
#include <semaphore.h>

#include "scoped_timer.hpp"     // rtmw::mono_ns()

static sem_t g_sem_a;           // signal: wake up A (main)
static sem_t g_sem_b;           // signal: wake up B (worker)
static unsigned g_rounds = 0;   // how many round trips

void* worker(void*) {
    for (unsigned i = 0; i < g_rounds; ++i) {
        sem_wait(&g_sem_b);
        sem_post(&g_sem_a);        
    }
    return nullptr;
}

int main(int argc, char** argv) {
    g_rounds = (argc > 1) ? static_cast<unsigned>(std::strtoul(argv[1], nullptr, 10)) : 5000;

    sem_init(&g_sem_a, 0, 0);
    sem_init(&g_sem_b, 0, 0);

    pthread_t th;
    if (pthread_create(&th, nullptr, worker, nullptr) != 0) {
        std::fprintf(stderr, "pthread_create failed\n");
        return 1;
    }


    std::vector<uint64_t> rtt;
    rtt.reserve(g_rounds);

    const unsigned warmup = (g_rounds > 200) ? 200 : g_rounds / 10;
    for (unsigned i = 0; i < warmup; ++i) {
        sem_post(&g_sem_b);
        sem_wait(&g_sem_a);
    }

    for (unsigned i = warmup; i < g_rounds; ++i) {
        uint64_t t0 = rtmw::mono_ns(); 
        sem_post(&g_sem_b);
        sem_wait(&g_sem_a);
        rtt.push_back(rtmw::mono_ns() - t0);
    }

    pthread_join(th, nullptr);
    sem_destroy(&g_sem_a);
    sem_destroy(&g_sem_b);

    std::printf("# rounds=%u\n", g_rounds);
    std::printf("# warmup=%u\n", warmup);
    std::printf("# clock=CLOCK_MONOTONIC\n");
    std::printf("# note=rtt is one round trip = 2 context switches\n");
    std::printf("iter,rtt_ns,per_switch_ns\n");

    for (size_t i = 0; i < rtt.size(); ++i) {
        std::printf("%zu,%llu,%llu\n", i,
                    static_cast<unsigned long long>(rtt[i]),
                    static_cast<unsigned long long>(rtt[i] / 2));
    }
    return 0;
}
