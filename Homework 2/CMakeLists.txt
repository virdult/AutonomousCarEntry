cmake_minimum_required(VERSION 3.10)
project(RestaurantBot)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SOURCES
    main.cpp
    Menu.cpp
)

# Include directories (current folder)
include_directories(${PROJECT_SOURCE_DIR})

# Add executable
add_executable(RestaurantBot ${SOURCES})

# Copy JSON files to build directory
file(GLOB JSON_FILES "${PROJECT_SOURCE_DIR}/*.json")
foreach(file ${JSON_FILES})
    configure_file(${file} ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
endforeach()

# --- nlohmann/json dependency ---
# Option 1: If installed system-wide (e.g., via package manager)
find_package(nlohmann_json 3.2.0 REQUIRED)
target_link_libraries(RestaurantBot PRIVATE nlohmann_json::nlohmann_json)
