cmake_minimum_required(VERSION 3.8)
project(uclv_modbus)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/connection_interface.cpp
  src/${PROJECT_NAME}/ethernet.cpp
  src/${PROJECT_NAME}/usb.cpp
  src/${PROJECT_NAME}/modbus.cpp  
  src/${PROJECT_NAME}/modbus_tcp.cpp
  src/${PROJECT_NAME}/modbus_rtu.cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "AMENT_INDEX_CPP_BUILDING_DLL")
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})

include_directories(include ${Boost_INCLUDE_DIRS})

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
