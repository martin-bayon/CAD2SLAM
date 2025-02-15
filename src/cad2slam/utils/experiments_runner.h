#pragma once

#include "srrg_config/configurable.h"
#include "srrg_data_structures/path_matrix_distance_search.h"
#include "cad2slam/utils/lidar_simulator.h"
#include "cad2slam/utils/image_utils.h"
#include "cad2slam/utils/results_utils.h"
#include "cad2slam/projector/projector.h"

#include "opencv2/opencv.hpp"

namespace cad2slam {
  using namespace srrg2_core;

  class ExperimentsRunner : public Configurable {
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      using ThisType = ExperimentsRunner;
      using BaseType = Configurable;

      PARAM(PropertyConfigurable_<Projector>, projector, "CAD2SLAM projector", nullptr, nullptr);
      PARAM(PropertyConfigurable_<LidarSimulator>, lidar_simulator, "LiDAR Simulator", nullptr, nullptr);
      PARAM(PropertyConfigurable_<MapManager>, cad_map_manager, "CAD Map Manager", nullptr, nullptr);
      PARAM(PropertyConfigurable_<MapManager>, slam_map_manager, "SLAM Map Manager", nullptr, nullptr);
      PARAM(PropertyConfigurable_<PathMatrixDistanceSearch>, dmap_calculator, "Distance Map Calculator", nullptr, nullptr);
      PARAM(PropertyInt, num_lidar_readings, "Number of LiDAR Readings", 100, nullptr);
      PARAM(PropertyFloat, p2p_threshold, "Threshold for the point2point metric", 0.05, nullptr);
      PARAM(PropertyBool , save_lidar_cloud, "Save the lidar clouds to folder", false, nullptr);
      PARAM(PropertyBool, visualize, "Enable visualization", false, nullptr);
      PARAM(PropertyString, experiments_root_path, "Path to save the experiments", ".", nullptr);
      PARAM(PropertyString, folder_cad_clouds, "Folder to save the CAD Lidar clouds", "cad_clouds", nullptr);
      PARAM(PropertyString, folder_slam_clouds, "Folder to save the SLAM Lidar clouds", "slam_clouds", nullptr);
      PARAM(PropertyString, folder_cad2slam_clouds, "Folder to save the CAD2SLAM Lidar clouds", "cad2slam_clouds", nullptr);
      PARAM(PropertyString, folder_slam2cad_clouds, "Folder to save the SLAM2CAD Lidar clouds", "slam2cad_clouds", nullptr);
      PARAM(PropertyString, folder_results, "Folder to save the numeric results", "results", nullptr);


      ExperimentsRunner();

      void reset() override;

      void generateRandomLidarReadings();
      void generateLidarReadings(std::vector<Vector3f> cad_poses, std::vector<Vector3f> slam_poses);
      std::pair<std::vector<Vector3f>,std::vector<Vector3f>> generatePath(Vector2i cad_start_pixel_resized, Vector2i cad_goal_pixel_resized);
      void calculateP2P();
      void allCad2slam();

      std::pair<std::vector<Vector2i>, std::vector<Vector2i>> getPathLimits();
      std::pair<std::vector<Vector2i>, std::vector<Vector2i>> loadPathLimits(std::string file_path);
      void savePathLimits(std::pair<std::vector<Vector2i>, std::vector<Vector2i>>, std::string file_path);

      void experiment_3(int number_of_samples);

      void setExperimentId(int experiment_id);
      int getExperimentId();

      void setRunId(int run_id);
      int getRunId();

  protected:
      bool _initialized = false;
      std::string _start_time_str;
      int _experiment_id = -1;
      int _run_id = -1;

      std::string _cad_clouds_folder_path;
      std::string _slam_clouds_folder_path;
      std::string _cad2slam_clouds_folder_path;
      std::string _slam2cad_clouds_folder_path;
      std::string _results_folder_path;
      std::string _experiments_base_path;

      std::vector<Point2fVectorCloud> _cad_clouds;
      std::vector<Point2fVectorCloud> _slam_clouds;
      std::vector<Point2fVectorCloud> _cad2slam_clouds;
      std::vector<Point2fVectorCloud> _slam2cad_clouds;

      std::vector<Vector3f> _cad_poses;
      std::vector<Vector3f> _slam_poses;

      float _p2p_min;
      float _p2p_max;
      int _outliers_min;
      int _outliers_max;

      void initialize();
      void checkParams();
      bool cmdGenerateRandomLidarReadings(std::string& response);
      bool cmdPrepareFolders(std::string& response);

      void prepareFolders();
      void createFolder(std::string path);
      void setExperimentsDate();

      bool isPixelWhite(cv::Mat image, Vector2i pixel);
      std::pair<Vector2i, float> getRandomWhitePixel(cv::Mat image);
      Point2fVectorCloud pixels2Cloud(MapManager* map_manager, std::vector<Vector2i> pixels);

      Point2fVectorCloud cadCloud2SlamFrame(Point2fVectorCloud cad_cloud);
      Point2fVectorCloud slamCloud2CadFrame(Point2fVectorCloud slam_cloud);

      std::pair<float, int> P2P(Point2fVectorCloud source, Point2fVectorCloud target, int idx);
      float minDistanceToCloud(Point2fVectorCloud cloud, Point2f query_point);
      float P2P(Point2f source, Point2f target);

      void saveResults(std::vector<float> values, std::vector<int> outliers, std::string path);
      void saveStats(float min, float max, float mean, float std_dev, std::string path);
      void saveInfo(int experiment_id, int run_id, float threshold, float p2p_mean, float outliers_mean, std::string path);
      void prepareGnuplotFile(std::string path, bool cad, int type);
  };
}
