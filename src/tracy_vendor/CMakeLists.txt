cmake_minimum_required(VERSION 3.8)
project(tracy_vendor)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

# 1. Absolute path to your submodule
get_filename_component(TRACY_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/tracy" ABSOLUTE)

# 2. Build the Library (the Client part)
# This provides the tracy::TracyClient target
set(TRACY_ON_DEMAND ON CACHE BOOL "" FORCE)
add_subdirectory(${TRACY_SRC_DIR} ${CMAKE_CURRENT_BINARY_DIR}/tracy_lib_build)

# 3. Build the Profiler GUI (the Server part)
# We use the NEW profiler path for v0.13.1
include(ExternalProject)
ExternalProject_Add(tracy_profiler_build
  SOURCE_DIR ${TRACY_SRC_DIR}/profiler
  CMAKE_ARGS 
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DLEGACY=ON
  BUILD_ALWAYS 1
)

# 4. Manual installation of the binary for 'ros2 run'
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/tracy_profiler_build-prefix/src/tracy_profiler_build-build/tracy-profiler
        DESTINATION lib/${PROJECT_NAME}
        RENAME tracy_profiler)

ament_export_dependencies(Threads)

# 2. Export the internal TracyClient target
ament_export_targets(export_tracy_vendor HAS_LIBRARY_TARGET)

# 3. Create a local alias so this package can use the same name as others
if(NOT TARGET tracy::TracyClient)
    add_library(tracy::TracyClient ALIAS TracyClient)
endif()

install(TARGETS TracyClient
  EXPORT export_tracy_vendor
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 4. Export the headers correctly
ament_export_include_directories(include "${TRACY_SRC_DIR}/public")

ament_package()