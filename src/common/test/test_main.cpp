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

void print_usage(const char* exe) {
    std::cout
        << "Usage:\n"
        << "  " << exe << "                 Run all tests\n"
        << "  " << exe << " --list          List test names\n"
        << "  " << exe << " --run <name>    Run a single test by exact name\n";
}

int run_one(const std::string& name) {
    for (const auto& t : registry()) {
        if (t.name == name) {
            try {
                t.fn();
                std::cout << "[PASS] " << t.name << "\n";
                return 0;
            } catch (const std::exception& e) {
                std::cout << "[FAIL] " << t.name << " : " << e.what() << "\n";
                return 1;
            } catch (...) {
                std::cout << "[FAIL] " << t.name << " : unknown exception\n";
                return 1;
            }
        }
    }

    std::cout << "[FAIL] " << name << " : test not found\n";
    return 1;
}

int run_all() {
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

    return failed == 0 ? 0 : 1;
}

} // namespace

// Register is declared in test_common.hpp
Register::Register(const std::string& name, std::function<void()> fn) {
    registry().push_back(Test{name, std::move(fn)});
}

int main(int argc, char** argv) {
    if (argc == 1) {
        return run_all() == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const std::string arg1 = argv[1];

    if (arg1 == "--list") {
        for (const auto& t : registry()) {
            std::cout << t.name << "\n";
        }
        return EXIT_SUCCESS;
    }

    if (arg1 == "--run") {
        if (argc < 3) {
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
        const std::string name = argv[2];
        return run_one(name) == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    print_usage(argv[0]);
    return EXIT_FAILURE;
}