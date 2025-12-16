cmake_minimum_required(VERSION 3.8)
project(feedback_planner_interfaces)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(dependencies
  ament_cmake
  rosidl_default_generators
  std_msgs
  sensor_msgs
  geometry_msgs
)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

set(action_interfaces
  action/CustomAction
)

# uncomment for messages
# set(message_interfaces
#   msg/CustomMsg.msg
# )

set(service_interfaces
  services/TriggerFeedback.srv
  services/TriggerManipulation.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
    # ${message_interfaces}
    ${service_interfaces}
    # ${action_interfaces}
    DEPENDENCIES ${dependencies}
)

ament_export_dependencies(${dependencies})
ament_package()