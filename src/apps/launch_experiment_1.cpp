#include "srrg_config/configurable_manager.h"
#include "srrg_solver/solver_core/factor_graph.h"

#include "cad2slam/projector/projector.h"
#include "cad2slam/utils/experiments_runner.h"

#include <iomanip>

using namespace srrg2_core;
using namespace cad2slam;

int main(int argc, char **argv){

  if(argc!=2){
    throw std::runtime_error("Usage: launch_experiment_1_preparation [config_name]");
  }

  std::string config_filename = argv[1];
  std::string root_directory = "/home/martin/workspace/src/cad2slam/config/";
  std::string config_file = root_directory + config_filename;
  std::cerr << "[INFO] Config file: " << config_file << std::endl;

  ConfigurableManager configurable_manager;
  configurable_manager.read(config_file);

  ConfigMaster* config_master = dynamic_cast<ConfigMaster *>(configurable_manager.getByName("config_master").get());

  Projector* projector = dynamic_cast<Projector *>(configurable_manager.getByName("projector").get());
  projector->setGraph(FactorGraph::read(root_directory + config_master->getOptimizedGraphFilename()));

  std::string path_limits_path = root_directory + "data/experiment_1_runs/" + config_filename.substr(0, config_filename.find_last_of(".")) + ".txt";
  ExperimentsRunner* experiments_runner = dynamic_cast<ExperimentsRunner *>(configurable_manager.getByName("experiments_runner").get());

  std::vector<float> threshold_values = {0.01, 0.05, 0.1, 0.5};
  int run_id = -1;
  int threshold_id = -1;
  int experiment_id = -1;
  if(config_filename == "demo.conf"){
    experiment_id = 1;
  }
  if(config_filename == "diag.conf"){
    experiment_id = 29;
  }
  if(config_filename == "eiii-0.conf"){
    experiment_id = 49;
  }
  if(config_filename == "eiii-3.conf"){
    experiment_id = 85;
  }
  if(config_filename == "mic.conf"){
    experiment_id = 109;
  }
  if(experiment_id == -1){
    std::cerr << "[Error - Invalid config] -- experiment_id: " << experiment_id << " for environment: " << config_filename << std::endl;
  }

  auto runs = experiments_runner->loadPathLimits(path_limits_path);
  auto runs_cad_start_pixels_resized = runs.first;
  auto runs_cad_goal_pixels_resized = runs.second;

  for(run_id=0; run_id<runs_cad_start_pixels_resized.size(); run_id++){
    for(threshold_id=0; threshold_id<threshold_values.size(); threshold_id++){
      //experiment_id += run_id * runs_cad_start_pixels_resized.size() + threshold_id;
      std::cerr << "###############################################################" << std::endl;
      std::cerr << "################# STARTING EXPERIMENT " << std::setw(3) << std::setfill('0') << experiment_id << "/132 #################" << std::endl;
      std::cerr << "###############################################################" << std::endl;
      experiments_runner->setExperimentId(experiment_id);
      experiments_runner->setRunId(run_id+1);
      experiments_runner->param_p2p_threshold.setValue(threshold_values[threshold_id]);
      auto poses = experiments_runner->generatePath(runs_cad_start_pixels_resized[run_id], runs_cad_goal_pixels_resized[run_id]);
      experiments_runner->generateLidarReadings(poses.first, poses.second);
      experiments_runner->calculateP2P();
      cv::destroyAllWindows();
      experiment_id ++;
    }
  }
  
  std::cerr << "[Done]" << std::endl;
}