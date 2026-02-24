cmake_minimum_required(VERSION 3.8)
project(ros2_gui_bridge)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- GENERATE MESSAGES ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GridCell.msg"
  "msg/Brick.msg"
  "msg/BrickArray.msg"
  DEPENDENCIES geometry_msgs
)

# --- INSTALL PYTHON SCRIPTS ---
# Ensure your python files are actually in a folder named 'scripts/' 
# inside your package for this to work!
install(PROGRAMS
  scripts/firestore_bridge.py
  scripts/brick_processor.py
  scripts/assembly_plan.py
  scripts/mock_detector.py
  scripts/brick_processor2.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- INSTALL LAUNCH FILES (The Fix) ---
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# --- INSTALL JSON CONFIGS ---
install(DIRECTORY 
  scripts/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.json"
)

ament_package()