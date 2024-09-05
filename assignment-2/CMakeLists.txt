find_package(Eigen3 CONFIG REQUIRED)

add_executable(assignment-2 main.cpp)

target_link_libraries(assignment-2 PRIVATE Eigen3::Eigen)
