add_library(cad2slam_library SHARED
        instances.h
        instances.cpp

        data_structures/keyframe_data.h
        data_structures/keyframe_data.cpp
        data_structures/correspondence_anchor.h
        data_structures/correspondence_anchor.cpp
        data_structures/correspondence_finder_info.h
        data_structures/path_poses.h
        data_structures/path_poses.cpp

        variables_and_factors/all_types.h
        variables_and_factors/instances.h
        variables_and_factors/instances.cpp
        variables_and_factors/keyframe_se2.h
        variables_and_factors/keyframe_se2.cpp
        variables_and_factors/factor_keyframe2keyframe.h
        variables_and_factors/factor_keyframe2keyframe.cpp
        variables_and_factors/factor_keyframe2anchor.h
        variables_and_factors/factor_keyframe2anchor.cpp

        config_master/config_master.h
        config_master/config_master.cpp

        correspondence_finders/manual_correspondence_finder/manual_correspondence_finder.h
        correspondence_finders/manual_correspondence_finder/manual_correspondence_finder.cpp

        graph_builder/graph_builder.h
        graph_builder/graph_builder.cpp

        map_manager/map_manager.h
        map_manager/map_manager.cpp

        graph_optimizer/graph_optimizer.h
        graph_optimizer/graph_optimizer.cpp

        projector/projector.h
        projector/projector.cpp

        utils/yaml_utils.h
        utils/yaml_utils.cpp
        utils/image_utils.h
        utils/image_utils.cpp
        utils/comparator_utils.h
        utils/comparator_utils.cpp
        utils/graph_utils.h
        utils/graph_utils.cpp
        utils/lidar_simulator.h
        utils/lidar_simulator.cpp
        utils/experiments_runner.h
        utils/experiments_runner.cpp
        utils/pointcloud_utils.h
        utils/pointcloud_utils.cpp
        utils/path_planner.h
        utils/path_planner.cpp
        utils/results_utils.h
        utils/results_utils.cpp
)

set (MY_LIBRARIES
    "${CUDA_LIBRARIES}"
    )

target_link_libraries(cad2slam_library
        srrg2_system_utils_library
        srrg2_property_library
        srrg2_config_library
        srrg2_boss_library
        srrg2_messages_library
        srrg2_solver_core_library
        srrg2_solver_types_common_library
        srrg2_solver_types2d_library
        srrg2_solver_types_common_library
        srrg2_navigation_2d_library
        ${MY_LIBRARIES}
        ${YAMLCPP_LIBRARIES}
        ${Boost_LIBRARIES}
)
