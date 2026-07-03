//
//  harness.cpp
//  engine::tst
//

#include "harness/harness.h"

#include <cstdio>
#include <cstring>
#include <exception>
#include <sstream>

namespace tst {

std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

Registrar::Registrar(std::string module, std::string category, std::string name, std::function<void()> fn) {
    registry().push_back({ std::move(module), std::move(category), std::move(name), std::move(fn) });
}

void failAssertion(const char* expr, const char* file, int line, const std::string& extra) {
    std::ostringstream os;
    os << "REQUIRE(" << expr << ") at " << file << ":" << line;
    if (!extra.empty()) os << " — " << extra;
    throw AssertionFailure{ os.str() };
}

namespace {
bool matches(const TestCase& t, const std::string& module, const std::string& category,
             const std::string& name) {
    if (!module.empty() && t.module != module) return false;
    if (!category.empty() && t.category != category) return false;
    if (!name.empty() && t.name != name && (t.module + "." + t.name) != name) return false;
    return true;
}
} // namespace

int runMain(int argc, char** argv) {
    std::string module, category, name;
    bool list = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--list") list = true;
        else if (a == "--module" && i + 1 < argc) module = argv[++i];
        else if (a == "--category" && i + 1 < argc) category = argv[++i];
        else if (a == "--name" && i + 1 < argc) name = argv[++i];
        else if (a.rfind("--", 0) != 0) name = a;   // positional filter
    }

    if (list) {
        for (const TestCase& t : registry())
            std::printf("%-10s %-12s %s\n", t.module.c_str(), t.category.c_str(), t.name.c_str());
        return 0;
    }

    int run = 0, failed = 0;
    for (const TestCase& t : registry()) {
        if (!matches(t, module, category, name)) continue;
        ++run;
        std::printf("[ RUN  ] %s.%s (%s)\n", t.module.c_str(), t.name.c_str(), t.category.c_str());
        try {
            t.fn();
            std::printf("[  OK  ] %s.%s\n", t.module.c_str(), t.name.c_str());
        } catch (const AssertionFailure& e) {
            std::printf("[ FAIL ] %s.%s — %s\n", t.module.c_str(), t.name.c_str(), e.message.c_str());
            ++failed;
        } catch (const std::exception& e) {
            std::printf("[ FAIL ] %s.%s — exception: %s\n", t.module.c_str(), t.name.c_str(), e.what());
            ++failed;
        }
    }
    std::printf("\n%d run, %d failed\n", run, failed);
    return failed ? 1 : 0;
}

} // namespace tst
