#include <cmath>
#include <string>
#include <cstdio>
#include <cerrno>
#include <atomic>
#include <cstring>
#include <cstdint>
#include <csignal>
#include <iostream>

#include <time.h>
#include <sched.h>
#include <unistd.h>
#include <sys/mman.h>

#include "motor_manager/motor_manager.hpp"

static std::atomic<bool> g_stop{false};
static std::atomic<bool> g_pause{false};

static constexpr long MAX_SAFE_STACK (8 * 1024);
static constexpr long NSEC_PER_SEC (1000000000);

void on_signal(int sig)
{
    if (sig == SIGINT || sig == SIGTERM) {
        g_stop.store(true, std::memory_order_relaxed);
    } else if (sig == SIGTSTP) {
        g_pause.store(true, std::memory_order_relaxed);
    }
}

void stack_prefault()
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);   
}

double target_generator(double T, double t)
{
    double f = 1 / T;
    return 1.5*std::sin(2*M_PI*f*t);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file_path>\n";
        return 1; 
    }

    std::string config_path = argv[1];
    mmns::MotorManager manager(config_path);

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    std::cout << "Using priority " << param.sched_priority << std::endl;
    
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        std::cerr << "sched_setscheduler failed: " << std::strerror(errno) << std::endl;
        return 1;
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cerr << "Failed to lock memory: " << std::strerror(errno) << std::endl;
        return 1;
    }
    
    stack_prefault();
    std::cout << "Starting RT task ns: " << manager.period() << std::endl;

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGTSTP, on_signal);
    
    struct timespec wakeup_time;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1;
    wakeup_time.tv_nsec = 0;

    int ret = 0;
    int counter = 1000;
    double t = 0;
    double T = 20;
    bool change = false;
    
    manager.start();
    while (1) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
        if (ret == EINTR) {
            continue;
        } else if (ret) {
            std::cerr << "clock nonosleep failed: " << std::strerror(ret) << std::endl;
	        return 1;
        }
        
        const bool stop = g_stop.load(std::memory_order_relaxed);
        const bool pause = g_pause.load(std::memory_order_relaxed);

        mmns::motor_state_t states[32];
        mmns::motor_state_t cmds[32];
        if (manager.update(stop || pause, states, cmds)) break;

        wakeup_time.tv_nsec += manager.period();
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }
    manager.stop();
    return 0;
}
