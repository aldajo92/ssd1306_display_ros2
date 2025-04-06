cmake_minimum_required(VERSION 3.5)
project(ssd1306_display)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add the library
add_library(oled_ssd1306 STATIC
    src/ssd1306/FileDescriptor.cpp
    src/ssd1306/OledHardware.cpp
    src/ssd1306/OledPixel.cpp
    src/ssd1306/OledFont8x8.cpp
    src/ssd1306/OledFont8x12.cpp
    src/ssd1306/OledFont8x16.cpp
    src/ssd1306/OledGraphics.cpp
    src/ssd1306/OledI2C.cpp
)

# Include directories for the library
target_include_directories(oled_ssd1306 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ssd1306>
    $<INSTALL_INTERFACE:include/ssd1306>
)

# Link dependencies to the library
ament_target_dependencies(oled_ssd1306 rclcpp std_msgs)

# Install the launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the configuration file (if not already added)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install the library
install(TARGETS oled_ssd1306
    EXPORT export_oled_ssd1306
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
    DESTINATION include
)

# Add the executable
add_executable(display_node src/display_node.cpp)

# Include directories for the executable
target_include_directories(display_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ssd1306
)

# Link the executable with the library and dependencies
target_link_libraries(display_node oled_ssd1306)
ament_target_dependencies(display_node rclcpp std_msgs)

# Install the executable
install(TARGETS display_node
    DESTINATION lib/${PROJECT_NAME})

# Export the package
ament_export_include_directories(include)
ament_export_libraries(oled_ssd1306)
ament_package()