#include "app/LinuxEventLoop.hpp"

#ifdef __linux__
#include <cerrno>
#include <cstring>
#include <iostream>

#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/timerfd.h>
#include <unistd.h>
#endif

namespace solar::app {

LinuxEventLoop::LinuxEventLoop(Logger& log,
                               SystemManager& sys,
                               const AppConfig& cfg
#if SOLAR_HAVE_OPENCV
                               , UiViewer* ui
#endif
                               )
    : log_(log), sys_(sys), cfg_(cfg), cli_(log, sys)
#if SOLAR_HAVE_OPENCV
    , ui_(ui)
#endif
{}

int LinuxEventLoop::run() {
#ifndef __linux__
    (void)log_; (void)sys_; (void)cfg_;
    return 0;
#else
    // Block SIGINT/SIGTERM and receive them via signalfd
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);

    if (sigprocmask(SIG_BLOCK, &mask, nullptr) < 0) {
        log_.error(std::string("sigprocmask failed: ") + std::strerror(errno));
        return 1;
    }

    const int sfd = signalfd(-1, &mask, SFD_CLOEXEC);
    if (sfd < 0) {
        log_.error(std::string("signalfd failed: ") + std::strerror(errno));
        return 1;
    }

    const int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (tfd < 0) {
        log_.error(std::string("timerfd_create failed: ") + std::strerror(errno));
        close(sfd);
        return 1;
    }

    itimerspec its{};
    its.it_interval.tv_sec  = 0;
    its.it_interval.tv_nsec = static_cast<long>(1000000000LL / (cfg_.tick_hz ? cfg_.tick_hz : 30));
    its.it_value            = its.it_interval;

    if (timerfd_settime(tfd, 0, &its, nullptr) < 0) {
        log_.error(std::string("timerfd_settime failed: ") + std::strerror(errno));
        close(tfd);
        close(sfd);
        return 1;
    }

    bool quit = false;

    while (!quit) {
        pollfd fds[3]{};
        fds[0].fd = sfd;          fds[0].events = POLLIN;
        fds[1].fd = tfd;          fds[1].events = POLLIN;
        fds[2].fd = STDIN_FILENO; fds[2].events = POLLIN;

        const int r = poll(fds, 3, -1);
        if (r < 0) {
            if (errno == EINTR) continue;
            log_.error(std::string("poll failed: ") + std::strerror(errno));
            break;
        }

        // signal
        if (fds[0].revents & POLLIN) {
            signalfd_siginfo si{};
            const ssize_t n = read(sfd, &si, sizeof(si));
            if (n == sizeof(si)) {
                if (si.ssi_signo == SIGINT || si.ssi_signo == SIGTERM) {
                    log_.info("Stop requested via signal");
                    quit = true;
                }
            }
        }

        // tick
        if (fds[1].revents & POLLIN) {
            std::uint64_t expirations = 0;
            (void)read(tfd, &expirations, sizeof(expirations));

            if (fds[2].revents & POLLIN) {
                std::string line;
                std::getline(std::cin, line);
                quit = cli_.handleLine(line);
            }

#if SOLAR_HAVE_OPENCV
            if (ui_) {
                if (!ui_->tick()) {
                    quit = true;
                } else {
                    sys_.setTrackerThreshold(static_cast<std::uint8_t>(ui_->threshold()));
                }
            }
#endif
        }
    }

    close(tfd);
    close(sfd);
    return 0;
#endif
}

} // namespace solar::app
