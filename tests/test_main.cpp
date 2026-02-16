#include "test_common.hpp"

#include <cstdlib>
#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct Test {
    std::string name;
    std::function<void()> fn;
};

std::vector<Test>& registry() {
    static std::vector<Test> r;
    return r;
}

// Implement the Register declared in test_common.hpp
Register::Register(const std::string& name, std::function<void()> fn) {
    registry().push_back(Test{name, std::move(fn)});
}

} // namespace

int main() {
    int failed = 0;

    for (const auto& t : registry()) {
        try {
            t.fn();
            std::cout << "[PASS] " << t.name << "\n";
        } catch (const std::exception& e) {
            failed++;
            std::cout << "[FAIL] " << t.name << " : " << e.what() << "\n";
        } catch (...) {
            failed++;
            std::cout << "[FAIL] " << t.name << " : unknown exception\n";
        }
    }

    std::cout << "Tests run: " << registry().size()
              << "  Failed: " << failed << "\n";

    return failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}