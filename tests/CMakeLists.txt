add_executable(leo_physics_tests
    test_verlet.cpp
    test_constraints.cpp
    test_collider.cpp
    test_nan_guard.cpp
    test_determinism.cpp
    test_rigidbody.cpp
)
target_link_libraries(leo_physics_tests PRIVATE leo_core gtest_main)
include(GoogleTest)
gtest_discover_tests(leo_physics_tests)
