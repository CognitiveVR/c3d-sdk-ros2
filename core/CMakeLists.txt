# Cognitive3D ingest core.
#
# HARD RULE: this directory must never include a ROS header or link a ROS
# target. It builds and tests standalone with plain CMake, and CI enforces that
# with a ROS-free job. That rule is what lets a consumer without ROS build and
# ship this directory on its own.
cmake_minimum_required(VERSION 3.16)

# `kSdkVersion` in include/c3d/version.hpp is the authoritative version: it is what reaches the wire
# as `c3d.version`, the only version a stored session can be attributed by. project(VERSION) is
# derived from it here rather than declared a second time. The package.xml files are checked
# against it by the ament package's `test_version_declarations` (this directory must not know that
# ROS exists), and the git tag by the release workflow. tools/bump_version.sh rewrites every
# declaration.
#
# Re-run configure when the header changes: `file(READ)` alone does not create that dependency, so
# a header-only bump on an existing build tree would otherwise build under the old project version.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/c3d/version.hpp")

# `constexpr` and the line start are both part of the pattern so that a COMMENT mentioning the
# constant cannot be matched instead: CMake's MATCHES takes the first match in the file, and the
# declaration has a comment block directly above it. Three integers, because project(VERSION) and
# a package.xml <version> both accept nothing else.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/include/c3d/version.hpp" _c3d_version_header)
if(NOT _c3d_version_header MATCHES
   "\nconstexpr const char\\* kSdkVersion = \"([0-9]+\\.[0-9]+\\.[0-9]+)\"")
  message(FATAL_ERROR
    "core/include/c3d/version.hpp does not declare kSdkVersion as three integers (x.y.z)")
endif()
set(C3D_SDK_VERSION "${CMAKE_MATCH_1}")

project(c3d_core VERSION ${C3D_SDK_VERSION} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(CURL REQUIRED)

# Cap'n Proto and zstd, for the camera and lidar payloads. Both are shippable to a non-ROS
# consumer, which is the only test a core/ dependency has to pass -- the rule at the top of this
# file is about ROS, not about having no dependencies at all.
#
# The generated C++ goes to the BUILD directory and is never committed: the schemas in capnp/ are
# vendored copies of the platform's canonical schemas (see capnp/README.md), and a checked-in
# generated header would be a third copy that nothing regenerates.
# C, in a project that has no C source at all. CapnProtoConfig.cmake probes for
# libatomic with `check_library_exists(atomic __atomic_load_8 "")`, which CMake
# compiles in C++ when C++ is the only enabled language -- and in C++ the probe's
# `extern char __atomic_load_8();` is a hard error against GCC's builtin
# declaration, not the warning it is in C. So find_package(CapnProto) fails with
# "libatomic not found" on a machine where libatomic is present and links fine.
# Enabling C makes the probe use the C compiler, where it passes.
enable_language(C)
find_package(CapnProto REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

# Relative to this directory, and SRC_PREFIX points at the same place, so the generated headers
# land flat in the build dir as `camera_payload.capnp.h`. java.capnp is an import of all three and
# is compiled as part of them; it declares annotations only and needs no output of its own.
set(CAPNPC_SRC_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/capnp")
set(CAPNPC_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
# Named, because test/ is a subdirectory and ${CMAKE_CURRENT_BINARY_DIR} means
# something else down there -- and ${CMAKE_BINARY_DIR} is wrong whenever core/ is
# built as a subproject of the ament package rather than standalone.
set(C3D_CAPNP_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}")
capnp_generate_cpp(C3D_CAPNP_SOURCES C3D_CAPNP_HEADERS
  capnp/common_payload.capnp
  capnp/camera_payload.capnp
  capnp/lidar_payload.capnp)

find_package(nlohmann_json 3.9 QUIET)
if(NOT nlohmann_json_FOUND)
  # Ubuntu ships the header without a config package on some releases.
  find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp REQUIRED)
  add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
  set_target_properties(nlohmann_json::nlohmann_json PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${NLOHMANN_JSON_INCLUDE_DIR}")
endif()

add_library(c3d_core STATIC
  src/types.cpp
  src/config.cpp
  src/environment.cpp
  src/logger.cpp
  src/util.cpp
  src/properties.cpp
  src/serialize.cpp
  src/spool.cpp
  src/session.cpp
  src/transport_curl.cpp
  src/sigpipe_block.cpp
  src/remote_variables.cpp
  src/sha256.cpp
  src/wall_clock.cpp
  src/pointcloud.cpp
  src/camera_fov.cpp
  src/capnp_payloads.cpp
  src/pose_block.cpp
  ${C3D_CAPNP_SOURCES}
)
add_library(c3d::core ALIAS c3d_core)

target_include_directories(c3d_core
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
  # The generated Cap'n Proto headers are PRIVATE, and that is the point: exactly one
  # translation unit (src/capnp_payloads.cpp) includes them, so c3d/session.hpp stays
  # includable by a consumer who has no capnp on their include path.
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_BINARY_DIR}
)
# All PRIVATE: no public header includes any of them. A static archive still carries them to the
# final link, and a consumer that uses one of them itself names it itself.
target_link_libraries(c3d_core PRIVATE CURL::libcurl nlohmann_json::nlohmann_json
                      CapnProto::capnp PkgConfig::ZSTD)
target_compile_options(c3d_core PRIVATE -Wall -Wextra -Wpedantic)
set_property(TARGET c3d_core PROPERTY POSITION_INDEPENDENT_CODE ON)

option(C3D_BUILD_TESTS "Build the core test suite" ON)
if(C3D_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

# Installed only as a standalone build. As a subproject the archive is folded into the shared
# library that links it and must not ship beside it, and the parent installs these headers under its
# own include/<package> prefix.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  include(GNUInstallDirs)
  install(TARGETS c3d_core ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
