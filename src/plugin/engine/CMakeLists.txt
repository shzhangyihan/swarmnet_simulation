add_library(default_engine SHARED default_engine.cpp)
target_link_libraries(default_engine collision_event node config arena event_queue physics_2d)

add_library(null_engine SHARED null_engine.cpp)

add_subdirectory(id_test_engine)
