#include "srrg_config/configurable_manager.h"
#include "srrg_solver/solver_core/factor_graph.h"

#include "cad2slam/projector/projector.h"
#include "cad2slam/utils/experiments_runner.h"

#include <iomanip>

using namespace srrg2_core;
using namespace cad2slam;

int main(int argc, char **argv){

  if(argc!=2){
    throw std::runtime_error("Usage: launch_experiment_runner [config_name]");
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

  ExperimentsRunner* experiments_runner = dynamic_cast<ExperimentsRunner *>(configurable_manager.getByName("experiments_runner").get());

  //TODO -- Experiment 1 demo
  /*
  auto runs = experiments_runner->getPathLimits();
  auto poses = experiments_runner->generatePath(runs.first[0], runs.second[0]);
  experiments_runner->generateLidarReadings(poses.first, poses.second);
  experiments_runner->calculateP2P();
  cv::destroyAllWindows();
   */

  //TODO -- Generate a set of random poses and LiDAR measurements in those poses -- Evaluate the p2p metric in those clouds

  experiments_runner->generateRandomLidarReadings();
  //experiments_runner->calculateP2P();


  //TODO -- Transform every pixel on the CAD map to the SLAM map and visualize it
  /*
  experiments_runner->reset();
  experiments_runner->allCad2slam();
   */


  std::cerr << "[Done]" << std::endl;
}