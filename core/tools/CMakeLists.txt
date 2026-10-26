# Conformance tooling for the pose encoder. Built with the core tests and never installed.

add_library(c3d_pose_vector_input STATIC pose_vector_input.cpp)
target_include_directories(c3d_pose_vector_input PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(c3d_pose_vector_input PUBLIC c3d_core nlohmann_json::nlohmann_json)
target_compile_options(c3d_pose_vector_input PRIVATE -Wall -Wextra -Wpedantic)

# The command the formatter's reference checker drives (tools/README.md).
add_executable(c3d_pose_encode_vector c3d_pose_encode_vector.cpp)
target_link_libraries(c3d_pose_encode_vector PRIVATE c3d_pose_vector_input)
target_compile_options(c3d_pose_encode_vector PRIVATE -Wall -Wextra -Wpedantic)
