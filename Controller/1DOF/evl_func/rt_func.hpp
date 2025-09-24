#pragma once

#include <ctime>
#include <functional>
#include <pthread.h>
#include <thread>

class rt_func {
public:
    rt_func();
    ~rt_func();

    // time helper
    static void add_ms(struct timespec &dst, const struct timespec &src, long ms);

    // CPU affinity helpers
    static bool pin_to_cpu(pthread_t thread, int core_id);
    static bool pin_to_cpu(std::thread &thr, int core_id); // convenience

    // EVL-based periodic runner
    // work(sampling_ms, total_overruns)
    static void run_periodic(
        int id,
        int period_ms,
        int priority,
        const std::function<void(double, unsigned long long)> &work
    );

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};
