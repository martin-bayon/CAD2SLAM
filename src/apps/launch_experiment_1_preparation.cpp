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

  auto runs = experiments_runner->getPathLimits();
  experiments_runner->savePathLimits(runs, path_limits_path);

  std::cerr << "[Done]" << std::endl;
}