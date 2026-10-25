add_executable(lazydash_tests
  test_robot_model.cpp
  test_backbone.cpp
  test_ik.cpp
  test_scene.cpp
  test_contacts.cpp
  test_accuracy.cpp
  test_verify.cpp
  test_task.cpp
  test_motion.cpp
  test_schedule.cpp
  test_result.cpp
)
target_link_libraries(lazydash_tests PRIVATE lazydash nlohmann_json::nlohmann_json Catch2::Catch2WithMain)

include(Catch)
catch_discover_tests(lazydash_tests)
