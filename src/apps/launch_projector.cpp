#include "srrg_config/configurable_manager.h"
#include "srrg_solver/solver_core/factor_graph.h"

#include "cad2slam/projector/projector.h"

int main(int argc, char **argv){
  using namespace srrg2_core;
  using namespace cad2slam;

  if(argc!=2){
    throw std::runtime_error("Usage: launch_projector [config_name]");
  }

  std::string root_directory = "/home/martin/workspace/src/cad2slam/config/";
  std::string config_file = root_directory + argv[1];
  std::cerr << "[INFO] Config file: " << config_file << std::endl;

  ConfigurableManager configurable_manager;
  configurable_manager.read(config_file);

  ConfigMaster* config_master = dynamic_cast<ConfigMaster *>(configurable_manager.getByName("config_master").get());

  Projector* projector = dynamic_cast<Projector *>(configurable_manager.getByName("projector").get());
  projector->setGraph(FactorGraph::read(root_directory + config_master->getOptimizedGraphFilename()));
  projector->launchProjector();

  std::cerr << "[Done]" << std::endl;
}