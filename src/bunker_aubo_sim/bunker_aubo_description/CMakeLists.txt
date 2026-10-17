cmake_minimum_required(VERSION 3.8)
project(bunker_aubo_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY urdf meshes config launch rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/print_mounts.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py TIMEOUT 120)
  ament_add_pytest_test(test_self_collision test/test_self_collision.py TIMEOUT 120)
endif()

ament_package()
