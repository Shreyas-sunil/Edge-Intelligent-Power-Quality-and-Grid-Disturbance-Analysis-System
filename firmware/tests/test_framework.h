// Minimal dependency-free test framework (works on the host and could be
// linked into a Pico test image). Tests register themselves with a group name;
// `pq_tests <group>` runs one group, `pq_tests` runs all.
#pragma once
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace pqtest {

struct TestCase {
    const char* group;
    const char* name;
    void (*fn)();
};

std::vector<TestCase>& registry();
extern int g_failures;
extern const char* g_current;

struct Registrar {
    Registrar(const char* g, const char* n, void (*f)()) { registry().push_back({g, n, f}); }
};

}  // namespace pqtest

#define PQ_TEST(group, name)                                                         \
    static void group##_##name();                                                    \
    static pqtest::Registrar reg_##group##_##name(#group, #name, &group##_##name);   \
    static void group##_##name()

#define PQ_CHECK(cond)                                                                        \
    do {                                                                                      \
        if (!(cond)) {                                                                        \
            std::printf("  FAIL %s:%d [%s] %s\n", __FILE__, __LINE__, pqtest::g_current, #cond); \
            ++pqtest::g_failures;                                                             \
        }                                                                                     \
    } while (0)

#define PQ_CHECK_NEAR(a, b, tol)                                                                  \
    do {                                                                                          \
        const double _a = (a), _b = (b), _t = (tol);                                              \
        if (!(std::fabs(_a - _b) <= _t)) {                                                        \
            std::printf("  FAIL %s:%d [%s] %s = %.6g, expected %.6g +/- %.3g\n", __FILE__, __LINE__, \
                        pqtest::g_current, #a, _a, _b, _t);                                       \
            ++pqtest::g_failures;                                                                 \
        }                                                                                         \
    } while (0)

#define PQ_INFO(...)                 \
    do {                             \
        std::printf("    ");         \
        std::printf(__VA_ARGS__);    \
        std::printf("\n");           \
    } while (0)
