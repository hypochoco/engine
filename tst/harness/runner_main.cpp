//
//  runner_main.cpp
//  engine::tst
//
//  Shared main() for the tests / benchmarks / visuals executables. Each target links this plus
//  the harness plus its category's source files; the registry is populated by self-registration.
//

#include "harness/harness.h"

int main(int argc, char** argv) {
    return tst::runMain(argc, argv);
}
