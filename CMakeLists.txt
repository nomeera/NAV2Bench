cmake_minimum_required(VERSION 3.8)
project(nav2bench)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY config launch worlds urdf rviz maps meshes models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pdf_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/performace_analysis.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/trajectory_generator.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/reset_robot.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/follow_path.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
