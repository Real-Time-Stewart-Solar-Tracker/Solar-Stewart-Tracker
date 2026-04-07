#include "app/LinuxEventLoop.hpp"

#include "app/CliController.hpp"
#include "system/SystemManager.hpp"

#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>

namespace solar::app {
namespace {

void closeIfOpen(int& fd) {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}

} // namespace

LinuxEventLoop::LinuxEventLoop(SystemManager& system, CliController& cli, const std::uint32_t tick_hz)
    : system_(system),
      cli_(cli),
      tick_hz_(tick_hz > 0U ? tick_hz : 30U) {
}

int LinuxEventLoop::run() {
    if (!system_.start()) {
        return 1;
    }

    // Block SIGINT, SIGQUIT, SIGHUP, and SIGTERM process-wide so they are
    // delivered exclusively through the signalfd below rather than through
    // the default signal disposition. This prevents the process from being
    // terminated before the shutdown path in stop() has executed.
    sigset_t mask;
    ::sigemptyset(&mask);
    ::sigaddset(&mask, SIGINT);
    ::sigaddset(&mask, SIGQUIT);
    ::sigaddset(&mask, SIGHUP);
    ::sigaddset(&mask, SIGTERM);

    if (::sigprocmask(SIG_BLOCK, &mask, nullptr) != 0) {
        system_.stop();
        return 1;
    }

    int sig_fd = ::signalfd(-1, &mask, SFD_CLOEXEC);
    if (sig_fd < 0) {
        system_.stop();
        return 1;
    }

    int timer_fd = ::timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (timer_fd < 0) {
        closeIfOpen(sig_fd);
        system_.stop();
        return 1;
    }

    // Monotonic timer period for the CLI tick.
    const std::uint64_t period_ns = 1'000'000'000ULL / tick_hz_;
    itimerspec its{};
    its.it_interval.tv_sec = static_cast<time_t>(period_ns / 1'000'000'000ULL);
    its.it_interval.tv_nsec = static_cast<long>(period_ns % 1'000'000'000ULL);
    its.it_value = its.it_interval;
    if (its.it_value.tv_sec == 0 && its.it_value.tv_nsec == 0) {
        its.it_value.tv_nsec = 1;
    }

    if (::timerfd_settime(timer_fd, 0, &its, nullptr) != 0) {
        closeIfOpen(timer_fd);
        closeIfOpen(sig_fd);
        system_.stop();
        return 1;
    }

    const int stdin_fd = ::dup(STDIN_FILENO);
    if (stdin_fd < 0) {
        closeIfOpen(timer_fd);
        closeIfOpen(sig_fd);
        system_.stop();
        return 1;
    }

    cli_.attachInputFd(stdin_fd);

    // A single poll() call multiplexes three blocking event sources:
    //   fds[0] — signalfd: SIGINT / SIGQUIT / SIGHUP / SIGTERM → clean shutdown
    //   fds[1] — timerfd:  periodic CLI tick
    //   fds[2] — stdin:    interactive command input
    pollfd fds[3];
    fds[0].fd = sig_fd;
    fds[0].events = POLLIN;
    fds[1].fd = timer_fd;
    fds[1].events = POLLIN;
    fds[2].fd = stdin_fd;
    fds[2].events = POLLIN;

    int exit_code = 0;
    bool done = false;

    while (!done) {
        const int rc = ::poll(fds, 3, -1);
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            exit_code = 1;
            break;
        }

        if (fds[0].revents & POLLIN) {
            signalfd_siginfo siginfo{};
            const ssize_t n = ::read(sig_fd, &siginfo, sizeof(siginfo));
            if (n == static_cast<ssize_t>(sizeof(siginfo))) {
                done = true;
            } else {
                exit_code = 1;
                break;
            }
        }

        if (!done && (fds[1].revents & POLLIN)) {
            std::uint64_t expirations = 0;
            const ssize_t n = ::read(timer_fd, &expirations, sizeof(expirations));
            if (n != static_cast<ssize_t>(sizeof(expirations))) {
                exit_code = 1;
                break;
            }

            if (!cli_.onTick()) {
                done = true;
            }
        }

        if (!done && (fds[2].revents & POLLIN)) {
            if (!cli_.onInputReady()) {
                done = true;
            }
        }

        if ((fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) ||
            (fds[1].revents & (POLLERR | POLLHUP | POLLNVAL)) ||
            (fds[2].revents & (POLLERR | POLLHUP | POLLNVAL))) {
            done = true;
        }
    }

    cli_.detachInputFd();
    closeIfOpen(timer_fd);
    closeIfOpen(sig_fd);
    closeIfOpen(fds[2].fd);

    system_.stop();
    return exit_code;
}

} // namespace solar::app
