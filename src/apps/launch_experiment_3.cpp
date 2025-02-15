#include "srrg_config/configurable_manager.h"
#include "srrg_solver/solver_core/factor_graph.h"

#include "cad2slam/projector/projector.h"
#include "cad2slam/graph_builder/graph_builder.h"
#include "cad2slam/graph_optimizer/graph_optimizer.h"
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

  /*
  GraphBuilder* graph_builder = dynamic_cast<GraphBuilder *>(configurable_manager.getByName("graph_builder").get());
  auto start = std::chrono::high_resolution_clock::now();
  graph_builder->saveAugmentedGraph();
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float, std::milli> elapsed = end-start;
  std::cout << "\n\nBuild and augment (including save) graph elapsed time: " << elapsed.count() << std::endl;

  GraphOptimizer* graph_optimizer = dynamic_cast<GraphOptimizer *>(configurable_manager.getByName("graph_optimizer").get());
  graph_optimizer->loadGraph();


  auto start_2 = std::chrono::high_resolution_clock::now();
  graph_optimizer->optimize();
  auto end_2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float, std::milli> elapsed_2 = end_2-start_2;
  std::cout << "\n\nGraph optimization elapsed time: " << elapsed_2.count() << std::endl;
  */

  auto _graph = FactorGraph::read(config_master->getAugmentedGraphFilename());
  std::cerr << "Number of variables: " << _graph->variables().size() << std::endl;

  /*
  Projector* projector = dynamic_cast<Projector *>(configurable_manager.getByName("projector").get());
  projector->setGraph(FactorGraph::read(root_directory + config_master->getOptimizedGraphFilename()));



  std::string path_limits_path = root_directory + "data/experiment_1_runs/" + config_filename.substr(0, config_filename.find_last_of(".")) + ".txt";
  ExperimentsRunner* experiments_runner = dynamic_cast<ExperimentsRunner *>(configurable_manager.getByName("experiments_runner").get());

  int number_of_samples = 1000;
  experiments_runner->experiment_3(number_of_samples);
  std::cerr << "[Done]" << std::endl;
  */


}