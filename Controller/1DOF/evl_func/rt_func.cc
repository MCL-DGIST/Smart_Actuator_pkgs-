#include "rt_func.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <unistd.h>
#include <sched.h>
#include <linux/types.h>
#include <time.h>

// EVL headers
#include <evl/evl.h>
#include <evl/timer.h>
#include <evl/clock.h>
#include <evl/syscall.h>

struct rt_func::Impl
{
    Impl() {}

    // ---- add_ms ----
    static void add_ms(struct timespec &dst, const struct timespec &src, long ms)
    {
        dst.tv_sec  = src.tv_sec + ms / 1000;
        dst.tv_nsec = src.tv_nsec + (ms % 1000) * 1000000L;
        if (dst.tv_nsec >= 1000000000L) {
            dst.tv_sec++;
            dst.tv_nsec -= 1000000000L;
        }
    }

    // ---- pin_to_cpu (pthread) ----
    static bool pin_to_cpu(pthread_t thread, int core_id)
    {
        cpu_set_t cpus;
        CPU_ZERO(&cpus);
        CPU_SET(core_id, &cpus);
        int rc = pthread_setaffinity_np(thread, sizeof(cpus), &cpus);
        return (rc == 0);
    }

    // ---- pin_to_cpu (std::thread 편의 오버로드) ----
    static bool pin_to_cpu(std::thread &thr, int core_id)
    {
        return pin_to_cpu(thr.native_handle(), core_id);
    }

    // Generic periodic handler: infinite loop, calling provided work lambda
    // ---- run_periodic (EVL) ----
    static void run_periodic(
        int id,
        int period_ms,
        int priority,
        const std::function<void(double, unsigned long long)> &work)
    {
        struct timespec prev_rt {0, 0};
        unsigned long long overrun_count = 0;

        // RT priority
        sched_param sch{};
        sch.sched_priority = priority;
        int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
        if (rc != 0) {
            std::cerr << "pthread_setschedparam failed: " << std::strerror(rc) << std::endl;
            // 계속 진행은 가능하지만 RT 특성은 약해질 수 있음
        }

        // Attach to EVL core
        int desc = evl_attach_self("rt_thread_%d:%d", id, static_cast<int>(getpid()));
        if (desc < 0) {
            std::cerr << "evl_attach_self(" << id << ") failed: " << std::strerror(-desc) << std::endl;
            return;
        }

        // Timer
        int tfd = evl_new_timer(EVL_CLOCK_MONOTONIC);
        if (tfd < 0) {
            std::cerr << "evl_new_timer(" << id << ") failed: " << std::strerror(-tfd) << std::endl;
            return;
        }

        // Start time = now + period, interval = period
        struct timespec now{};
        evl_read_clock(EVL_CLOCK_MONOTONIC, &now);

        struct itimerspec its{};
        add_ms(its.it_value, now, period_ms);
        its.it_interval.tv_sec  = period_ms / 1000;
        its.it_interval.tv_nsec = (period_ms % 1000) * 1000000L;

        if (evl_set_timer(tfd, &its, nullptr) < 0) {
            std::cerr << "evl_set_timer(" << id << ") failed" << std::endl;
            return;
        }

        // Main loop
        for (;;)
        {
            __u64 ticks = 0;
            ssize_t n = oob_read(tfd, &ticks, sizeof(ticks));
            if (n != static_cast<ssize_t>(sizeof(ticks))) {
                int err = (n < 0) ? -static_cast<int>(n) : errno;
                std::cerr << "oob_read(" << id << ") failed: " << std::strerror(err) << std::endl;
                break;
            }

            // Timestamp for perf metrics (wall time)
            struct timespec trt{};
            clock_gettime(CLOCK_REALTIME, &trt);

            long delta_ns;
            if (prev_rt.tv_sec == 0 && prev_rt.tv_nsec == 0) {
                delta_ns = static_cast<long>(period_ms) * 1000000L;
            } else {
                delta_ns = (trt.tv_sec - prev_rt.tv_sec) * 1000000000L
                         + (trt.tv_nsec - prev_rt.tv_nsec);
            }
            prev_rt = trt;

            double sampling_ms = static_cast<double>(delta_ns) * 1e-6;
            if (ticks > 1) {
                overrun_count += (ticks - 1);
            }

            // User work (sampling_ms, overrun_count)
            work(sampling_ms, overrun_count);
        }
    }
};

// ========== rt_func wrappers ==========
rt_func::rt_func() {}
rt_func::~rt_func() {}

void rt_func::add_ms(struct timespec &dst, const struct timespec &src, long ms)
{
    Impl::add_ms(dst, src, ms);
}

bool rt_func::pin_to_cpu(pthread_t thread, int core_id)
{
    return Impl::pin_to_cpu(thread, core_id);
}

bool rt_func::pin_to_cpu(std::thread &thr, int core_id)
{
    return Impl::pin_to_cpu(thr, core_id);
}

void rt_func::run_periodic(
    int id,
    int period_ms,
    int priority,
    const std::function<void(double, unsigned long long)> &work)
{
    Impl::run_periodic(id, period_ms, priority, work);
}
