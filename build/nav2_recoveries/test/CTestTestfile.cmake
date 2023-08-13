# CMake generated Testfile for 
# Source directory: /ws/src/nav2/nav2_recoveries/test
# Build directory: /ws/src/nav2/build/nav2_recoveries/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_recoveries "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/ws/src/nav2/build/nav2_recoveries/test_results/nav2_recoveries/test_recoveries.gtest.xml" "--package-name" "nav2_recoveries" "--output-file" "/ws/src/nav2/build/nav2_recoveries/ament_cmake_gtest/test_recoveries.txt" "--command" "/ws/src/nav2/build/nav2_recoveries/test/test_recoveries" "--gtest_output=xml:/ws/src/nav2/build/nav2_recoveries/test_results/nav2_recoveries/test_recoveries.gtest.xml")
set_tests_properties(test_recoveries PROPERTIES  LABELS "gtest" REQUIRED_FILES "/ws/src/nav2/build/nav2_recoveries/test/test_recoveries" TIMEOUT "60" WORKING_DIRECTORY "/ws/src/nav2/build/nav2_recoveries/test" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/ws/src/nav2/nav2_recoveries/test/CMakeLists.txt;1;ament_add_gtest;/ws/src/nav2/nav2_recoveries/test/CMakeLists.txt;0;")
subdirs("../gtest")
