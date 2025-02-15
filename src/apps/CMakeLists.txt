add_executable(launch_correspondence_finder launch_correspondence_finder.cpp)
target_link_libraries(launch_correspondence_finder
        cad2slam_library
        ${catkin_LIBRARIES})

add_executable(launch_correspondence_visualizer launch_correspondence_visualizer.cpp)
target_link_libraries(launch_correspondence_visualizer
        cad2slam_library
        ${catkin_LIBRARIES})

add_executable(launch_projector launch_projector.cpp)
target_link_libraries(launch_projector
        cad2slam_library
        ${catkin_LIBRARIES})

add_executable(launch_experiment_runner launch_experiment_runner.cpp)
target_link_libraries(launch_experiment_runner
        cad2slam_library
        ${catkin_LIBRARIES})

add_executable(launch_experiment_1_preparation launch_experiment_1_preparation.cpp)
target_link_libraries(launch_experiment_1_preparation
        cad2slam_library
        ${catkin_LIBRARIES})

add_executable(launch_experiment_1 launch_experiment_1.cpp)
target_link_libraries(launch_experiment_1
        cad2slam_library
        ${catkin_LIBRARIES})

add_executable(launch_experiment_3 launch_experiment_3.cpp)
target_link_libraries(launch_experiment_3
        cad2slam_library
        ${catkin_LIBRARIES})


add_executable(planner_tests planner_tests.cpp)
target_link_libraries(planner_tests
        cad2slam_library
        ${catkin_LIBRARIES})
