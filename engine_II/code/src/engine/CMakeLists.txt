
add_library(
    engine
    STATIC
    Solver.h
    Solver.cpp
    MechanicalSystem.h
    MechanicalSystem.cpp
    ArticulatedSystem.h
    ArticulatedSystem.cpp
    Joint.h
    Joint.cpp
    FreeJoint.h
    FreeJoint.cpp
    FixedJoint.h
    FixedJoint.cpp
    HingeJoint3.h
    HingeJoint3.cpp
    CylindricalJoint3.h
    CylindricalJoint3.cpp
    PrismaticJoint3.h
    PrismaticJoint3.cpp
    Body.h
    Body.cpp
    ArticulatedBody.h
    ArticulatedBody.cpp
    BodyModel.h
    BodyModel.cpp
    Pose.h
    Pose.cpp
    Velocity.h
    Velocity.cpp
)

#target_compile_definitions(engine PUBLIC BUILD_2D)
#target_compile_definitions(engine PUBLIC BUILD_3D)

target_link_libraries(engine PUBLIC Eigen3::Eigen Sophus::Sophus openscenegraph xtensor)

