cmake_minimum_required(VERSION 3.8)
project(bunker_aubo_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_world.py scripts/generate_simple_world.py scripts/generate_textures.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_world test/test_world.py TIMEOUT 60)
endif()

ament_package()
