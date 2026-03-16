#pragma once

#include <functional>
#include <stdexcept>
#include <string>

// Shared registry hook (implemented in test_main.cpp)
struct Register {
    Register(const std::string& name, std::function<void()> fn);
};

#define TEST(name) \
    void name(); \
    static Register reg_##name(#name, name); \
    void name()

#define REQUIRE(cond) \
    do { \
        if (!(cond)) { \
            throw std::runtime_error(std::string("REQUIRE failed: ") + #cond); \
        } \
    } while (0)

#define REQUIRE_NEAR(a,b,eps) \
    do { \
        const auto _da = (a); \
        const auto _db = (b); \
        const auto _eps = (eps); \
        if (!((_da >= (_db - _eps)) && (_da <= (_db + _eps)))) { \
            throw std::runtime_error("REQUIRE_NEAR failed"); \
        } \
    } while (0)
