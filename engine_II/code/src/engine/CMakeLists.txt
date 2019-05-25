
add_library(
    engine
    STATIC
    Solver.h
    Solver.cpp
    MechanicalSystem.h
    MechanicalSystem.cpp
    ArticulatedSystem.h
    ArticulatedSystem.cpp
    Link.h
    Link.cpp
    Body.h
    Body.cpp
    LinkedBody.h
    LinkedBody.cpp
    BodyModel.h
    BodyModel.cpp
    Pose.h
    Pose.cpp
    Velocity.h
    Velocity.cpp
)

target_compile_definitions(engine PUBLIC BUILD_2D)
target_compile_definitions(engine PUBLIC BUILD_3D)

target_link_libraries(engine PUBLIC Eigen3::Eigen Sophus::Sophus openscenegraph xtensor)

