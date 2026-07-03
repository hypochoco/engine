//
//  harness.h
//  engine::tst
//
//  Tiny test harness shared by the tests / benchmarks / visuals runners. Tests self-register
//  via TST_CASE(module, category, name) { ... } — no central list to maintain, so adding a
//  file under tst/<module>/<category>/ (globbed by CMake) is all it takes. TST_REQUIRE throws
//  on failure so the runner reports it and moves on (no abort). The same runMain() drives all
//  three executables; each only links the source files for its category, so its registry only
//  contains those.
//

#pragma once

#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace tst {

struct AssertionFailure { std::string message; };

struct TestCase {
    std::string           module;     // core | ecs | physics | graphics | ...
    std::string           category;   // unit | integration | benchmark | visual
    std::string           name;
    std::function<void()> fn;
};

std::vector<TestCase>& registry();

struct Registrar {
    Registrar(std::string module, std::string category, std::string name, std::function<void()> fn);
};

[[noreturn]] void failAssertion(const char* expr, const char* file, int line, const std::string& extra);

// Shared entry point. Flags: --list, --module <m>, --category <c>, or a positional
// "<module>.<name>" / "<name>" filter. Returns non-zero if any selected test failed.
int runMain(int argc, char** argv);

} // namespace tst

#define TST_REQUIRE(cond) \
    do { if (!(cond)) ::tst::failAssertion(#cond, __FILE__, __LINE__, ""); } while (0)

#define TST_REQUIRE_MSG(cond, msg) \
    do { if (!(cond)) ::tst::failAssertion(#cond, __FILE__, __LINE__, (msg)); } while (0)

#define TST_APPROX(a, b, eps) \
    do { if (!(std::fabs((double)(a) - (double)(b)) <= (double)(eps))) \
             ::tst::failAssertion(#a " ~= " #b, __FILE__, __LINE__, ""); } while (0)

// Define + self-register a test case. Body follows the macro like a function body.
#define TST_CASE(mod, cat, nm)                                                            \
    static void tst_fn_##mod##_##cat##_##nm();                                            \
    static ::tst::Registrar tst_reg_##mod##_##cat##_##nm(#mod, #cat, #nm,                 \
                                                         &tst_fn_##mod##_##cat##_##nm);   \
    static void tst_fn_##mod##_##cat##_##nm()
