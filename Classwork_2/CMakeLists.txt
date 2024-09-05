find_package(Eigen3 CONFIG REQUIRED)

add_executable(Classwork_2 main.cpp)

target_link_libraries(Classwork_2 PRIVATE Eigen3::Eigen)

