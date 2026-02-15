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

struct Register {
    Register(const std::string& name, std::function<void()> fn) {
        registry().push_back(Test{name, std::move(fn)});
    }
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