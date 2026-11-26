find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(c3d_core_tests
  test_environment.cpp
  test_serialize.cpp
  test_properties.cpp
  test_spool.cpp
  test_session.cpp
  test_recovery.cpp
  test_remote_variables.cpp
  test_pointcloud.cpp
  test_scan_block.cpp
  test_sensor_blocks.cpp
  test_h264_encoder.cpp
  test_binary_streams.cpp
  test_upload_stall.cpp
  test_transport_curl.cpp
  test_sigpipe_block.cpp
  test_uploads_paused.cpp
  test_sha256.cpp
  test_serialization_guard.cpp
  test_upload_policy.cpp
  test_chunk_destination.cpp
  test_session_lifecycle.cpp
)
# capnp and zstd directly, not only through c3d_core: these tests decode what the
# encoder wrote rather than asking the encoder to decode it, which is the only
# arrangement in which a shared bug is visible.
target_link_libraries(c3d_core_tests PRIVATE c3d_core GTest::gtest GTest::gtest_main
                      CapnProto::capnp PkgConfig::ZSTD nlohmann_json::nlohmann_json)
target_include_directories(c3d_core_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
  ${C3D_CAPNP_GENERATED_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR})
# The environment cases are shared with tools/test_c3d_api_env.py, which reads the same file.
target_compile_definitions(c3d_core_tests PRIVATE
  C3D_ENVIRONMENT_CASES="${CMAKE_CURRENT_SOURCE_DIR}/environment_cases.json")
# A hung test fails at the timeout instead of holding the run; the slowest takes about 13 s.
gtest_discover_tests(c3d_core_tests PROPERTIES TIMEOUT 120)

# Its own executable, because it replaces fsync, fdatasync and rename for everything linked into
# it, and Linux-only, because it names each call's file through /proc/self/fd.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_executable(c3d_spool_durability_tests test_spool_durability.cpp)
  target_link_libraries(c3d_spool_durability_tests PRIVATE c3d_core GTest::gtest GTest::gtest_main
                        ${CMAKE_DL_LIBS})
  gtest_discover_tests(c3d_spool_durability_tests PROPERTIES TIMEOUT 120)
endif()
