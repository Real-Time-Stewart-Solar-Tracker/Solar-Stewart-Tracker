#pragma once

/**
 * @file test_common.hpp
 * @brief Minimal shared test macros.
 */

#include <cstdlib>
#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace solar::tests {

/**
 * @brief One registered test case.
 */
struct TestCase {
    std::string name;
    std::function<void()> fn;
};

/**
 * @brief Global test registry.
 *
 * @return Mutable test registry.
 */
inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

/**
 * @brief Register one test case.
 */
class Registrar {
public:
    Registrar(std::string name, std::function<void()> fn) {
        registry().push_back({std::move(name), std::move(fn)});
    }
};

/**
 * @brief Exception thrown by failed REQUIRE checks.
 */
class RequireFailure final : public std::exception {
public:
    explicit RequireFailure(std::string msg)
        : msg_(std::move(msg)) {
    }

    const char* what() const noexcept override {
        return msg_.c_str();
    }

private:
    std::string msg_;
};

} // namespace solar::tests

#define SOLAR_TEST_CONCAT_IMPL(a, b) a##b
#define SOLAR_TEST_CONCAT(a, b) SOLAR_TEST_CONCAT_IMPL(a, b)

#define TEST_CASE(name) \
    static void SOLAR_TEST_CONCAT(test_fn_, __LINE__)(); \
    static ::solar::tests::Registrar SOLAR_TEST_CONCAT(test_reg_, __LINE__)(#name, SOLAR_TEST_CONCAT(test_fn_, __LINE__)); \
    static void SOLAR_TEST_CONCAT(test_fn_, __LINE__)()

#define REQUIRE(cond) \
    do { \
        if (!(cond)) { \
            throw ::solar::tests::RequireFailure( \
                std::string("REQUIRE failed: ") + #cond + \
                " at " + __FILE__ + ":" + std::to_string(__LINE__)); \
        } \
    } while (false)
