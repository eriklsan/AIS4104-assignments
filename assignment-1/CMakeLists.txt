find_package(Eigen3 CONFIG REQUIRED)

add_executable(assignment-1 main.cpp
        ../math/math.cpp
)

target_link_libraries(assignment-1 PUBLIC math PRIVATE Eigen3::Eigen)
