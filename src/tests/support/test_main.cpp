/**
 * @file test_main.cpp
 * @brief Minimal test runner.
 */

#include "src/tests/support/test_common.hpp"

#include <cstring>
#include <exception>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    std::string run_one;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--run") == 0 && (i + 1) < argc) {
            run_one = argv[++i];
        }
    }

    int failures = 0;
    int executed = 0;

    for (const auto& tc : solar::tests::registry()) {
        if (!run_one.empty() && tc.name != run_one) {
            continue;
        }

        ++executed;
        try {
            tc.fn();
            std::cout << "[PASS] " << tc.name << '\n';
        } catch (const solar::tests::RequireFailure& e) {
            ++failures;
            std::cerr << "[FAIL] " << tc.name << " - " << e.what() << '\n';
        } catch (const std::exception& e) {
            ++failures;
            std::cerr << "[FAIL] " << tc.name << " - unexpected exception: " << e.what() << '\n';
        } catch (...) {
            ++failures;
            std::cerr << "[FAIL] " << tc.name << " - unknown exception\n";
        }
    }

    if (!run_one.empty() && executed == 0) {
        std::cerr << "[FAIL] no test named: " << run_one << '\n';
        return 1;
    }

    return failures == 0 ? 0 : 1;
}
