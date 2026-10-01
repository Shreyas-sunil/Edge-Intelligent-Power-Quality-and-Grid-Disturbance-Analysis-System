#include <cstring>

#include "test_framework.h"
#include "tinyml/classifier.h"

namespace pqtest {
std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}
int g_failures = 0;
const char* g_current = "";
}  // namespace pqtest

int main(int argc, char** argv) {
    pq::Classifier_Init();   // tests that need the model check Classifier_Info().ready
    int run = 0;
    for (const auto& t : pqtest::registry()) {
        if (argc > 1 && std::strcmp(argv[1], t.group) != 0) continue;
        pqtest::g_current = t.name;
        const int before = pqtest::g_failures;
        std::printf("[ RUN  ] %s.%s\n", t.group, t.name);
        t.fn();
        std::printf("[ %s ] %s.%s\n", pqtest::g_failures == before ? " OK " : "FAIL", t.group, t.name);
        ++run;
    }
    std::printf("\n%d tests, %d failed checks\n", run, pqtest::g_failures);
    if (run == 0) {
        std::printf("no tests matched\n");
        return 1;
    }
    return pqtest::g_failures ? 1 : 0;
}
