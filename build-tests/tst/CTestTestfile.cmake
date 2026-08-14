# CMake generated Testfile for 
# Source directory: /Users/dseocho/Projects/research/sim-2/external/engine/tst
# Build directory: /Users/dseocho/Projects/research/sim-2/external/engine/build-tests/tst
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[core]=] "/Users/dseocho/Projects/research/sim-2/external/engine/build-tests/tst/tests" "--module" "core")
set_tests_properties([=[core]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;55;add_test;/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;0;")
add_test([=[ecs]=] "/Users/dseocho/Projects/research/sim-2/external/engine/build-tests/tst/tests" "--module" "ecs")
set_tests_properties([=[ecs]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;55;add_test;/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;0;")
add_test([=[physics]=] "/Users/dseocho/Projects/research/sim-2/external/engine/build-tests/tst/tests" "--module" "physics")
set_tests_properties([=[physics]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;55;add_test;/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;0;")
add_test([=[graphics]=] "/Users/dseocho/Projects/research/sim-2/external/engine/build-tests/tst/tests" "--module" "graphics")
set_tests_properties([=[graphics]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;55;add_test;/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;0;")
add_test([=[pathtracer]=] "/Users/dseocho/Projects/research/sim-2/external/engine/build-tests/tst/tests" "--module" "pathtracer")
set_tests_properties([=[pathtracer]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;55;add_test;/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;0;")
add_test([=[input]=] "/Users/dseocho/Projects/research/sim-2/external/engine/build-tests/tst/tests" "--module" "input")
set_tests_properties([=[input]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;55;add_test;/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;0;")
add_test([=[controls]=] "/Users/dseocho/Projects/research/sim-2/external/engine/build-tests/tst/tests" "--module" "controls")
set_tests_properties([=[controls]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;55;add_test;/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;0;")
add_test([=[physics_env]=] "/Users/dseocho/Projects/research/sim-2/external/engine/build-tests/tst/tests" "--module" "physics_env")
set_tests_properties([=[physics_env]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;55;add_test;/Users/dseocho/Projects/research/sim-2/external/engine/tst/CMakeLists.txt;0;")
