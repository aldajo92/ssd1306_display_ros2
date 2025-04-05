cmake_minimum_required(VERSION 3.5)
project(waver_oled_ssd1306)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Add the library
add_library(waver_oled_ssd1306 STATIC
    src/ssd1306/FileDescriptor.cpp
    src/ssd1306/OledHardware.cpp
    src/ssd1306/OledPixel.cpp
    src/ssd1306/OledFont8x8.cpp
    src/ssd1306/OledFont8x12.cpp
    src/ssd1306/OledFont8x16.cpp
    src/ssd1306/OledGraphics.cpp
    src/ssd1306/OledI2C.cpp
)

# Include directories
target_include_directories(waver_oled_ssd1306 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ssd1306>
    $<INSTALL_INTERFACE:include/ssd1306>
)

# Install the library
install(TARGETS waver_oled_ssd1306
    EXPORT export_waver_oled_ssd1306
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
    DESTINATION include
)

# Export the package
ament_export_include_directories(include)
ament_export_libraries(waver_oled_ssd1306)
ament_package()
