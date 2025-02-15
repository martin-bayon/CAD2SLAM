#include "instances.h"

#include "config_master/config_master.h"
#include "map_manager/map_manager.h"
#include "graph_builder/graph_builder.h"
#include "graph_optimizer/graph_optimizer.h"
#include "projector/projector.h"
#include "data_structures/correspondence_anchor.h"
#include "data_structures/path_poses.h"
#include "correspondence_finders/manual_correspondence_finder/manual_correspondence_finder.h"
#include "utils/lidar_simulator.h"
#include "utils/experiments_runner.h"

namespace cad2slam{
  void cad2slam_configurables(){
    BOSS_REGISTER_CLASS(ConfigMaster);
    BOSS_REGISTER_CLASS(MapManager);
    BOSS_REGISTER_CLASS(GraphBuilder);
    BOSS_REGISTER_CLASS(GraphOptimizer);
    BOSS_REGISTER_CLASS(Projector);
    BOSS_REGISTER_CLASS(ManualCorrespondenceFinder);
    BOSS_REGISTER_CLASS(LidarSimulator);
    BOSS_REGISTER_CLASS(ExperimentsRunner);
  }

  void cad2slam_serializables(){
    BOSS_REGISTER_CLASS(CorrespondenceAnchor);
    BOSS_REGISTER_CLASS(PathPoses);
  }
}