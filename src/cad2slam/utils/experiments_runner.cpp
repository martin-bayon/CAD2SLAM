#include <random>
#include <filesystem>
#include <ctime>
#include "experiments_runner.h"
#include <srrg_config/configurable_command.h>

#include "cad2slam/utils/pointcloud_utils.h"
#include "cad2slam/utils/path_planner.h"
#include "cad2slam/data_structures/path_poses.h"

#include "srrg_system_utils/system_utils.h"


namespace fs = std::filesystem;

namespace cad2slam {
  ExperimentsRunner::ExperimentsRunner() {
    addCommand(new ConfigurableCommand_<ExperimentsRunner, typeof(&ExperimentsRunner::cmdGenerateRandomLidarReadings), std::string>(
            this,
            "generateRandomLidarReadings",
            "Generates LiDAR readings from both the CAD and SLAM maps that can later be compared",
            &ExperimentsRunner::cmdGenerateRandomLidarReadings));
  }

  void ExperimentsRunner::setExperimentId(int experiment_id) {
    _experiment_id = experiment_id;
    reset();
  }

  int ExperimentsRunner::getExperimentId() {
    return _experiment_id;
  }

  void ExperimentsRunner::setRunId(int run_id) {
    _run_id = run_id;
  }

  int ExperimentsRunner::getRunId() {
    return _run_id;
  }

  void ExperimentsRunner::reset() {
    _start_time_str = "";
    _initialized = false;
  }

  void ExperimentsRunner::initialize() {
    if(!_initialized){
      checkParams();
      prepareFolders();
      _initialized = true;
    }
  }

  void ExperimentsRunner::checkParams() {
    if(!param_lidar_simulator.value()) {
      throw std::runtime_error("[ExperimentsRunner::checkParams] lidar_simulator is not set - Exiting...");
    }
    if(!param_projector.value()) {
      throw std::runtime_error("[ExperimentsRunner::checkParams] projector is not set - Exiting...");
    }
    if(!param_cad_map_manager.value()) {
      throw std::runtime_error("[ExperimentsRunner::checkParams] cad_map_manager is not set - Exiting...");
    }
    if(!param_slam_map_manager.value()) {
      throw std::runtime_error("[ExperimentsRunner::checkParams] slam_map_manager is not set - Exiting...");
    }
    /*
    if(!param_dmap_calculator.value()) {
      throw std::runtime_error("[ExperimentsRunner::checkParams] dmap_calculator is not set - Exiting...");
    }
     */
    std::cerr << "[INFO] [ExperimentsRunner] checkParams is successful" << std::endl;
  }

  void ExperimentsRunner::setExperimentsDate(){
    //Get start time for the experiments
    std::time_t t = std::time(nullptr);
    std::tm* now = std::localtime(&t);
    std::stringstream ss;
    ss << std::put_time(now, "%Y-%m-%d_%H:%M:%S");
    _start_time_str = ss.str();
  }

  bool ExperimentsRunner::cmdGenerateRandomLidarReadings(std::string &response) {
    generateRandomLidarReadings();
    return true;
  }

  void ExperimentsRunner::prepareFolders() {
    if(_start_time_str==""){
      setExperimentsDate();
    }

    //Using start time_stamp
    //_experiments_base_path = "./" + param_experiments_root_path.value() + "_" + std::to_string(param_p2p_threshold.value()) + "_" + _start_time_str + "/";

    //Using experiment_id
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << _experiment_id;
    _experiments_base_path = "./" + param_experiments_root_path.value() + "_" + ss.str() + "/";

    _cad_clouds_folder_path = _experiments_base_path + param_folder_cad_clouds.value();
    _slam_clouds_folder_path = _experiments_base_path + param_folder_slam_clouds.value();
    _cad2slam_clouds_folder_path = _experiments_base_path + param_folder_cad2slam_clouds.value();
    _slam2cad_clouds_folder_path = _experiments_base_path + param_folder_slam2cad_clouds.value();

    _results_folder_path = _experiments_base_path + param_folder_results.value();

    createFolder(_cad_clouds_folder_path);
    createFolder(_slam_clouds_folder_path);
    createFolder(_cad2slam_clouds_folder_path);
    createFolder(_slam2cad_clouds_folder_path);

    createFolder(_results_folder_path);
  }

  void ExperimentsRunner::createFolder(std::string path){
    if (!fs::exists(path)) {
      if (!fs::create_directories(path)) {
        throw std::runtime_error("[ExperimentsRunner::createFolder] Could not create the experiments folder at " + path);
      }
    }
  }

  bool ExperimentsRunner::isPixelWhite(cv::Mat image, Vector2i pixel){
    cv::Vec3b pixel_color = image.at<cv::Vec3b>(pixel.y(), pixel.x());
    return (pixel_color[0] == 255 && pixel_color[1] == 255 && pixel_color[2] == 255);
  }

  std::pair<Vector2i, float> ExperimentsRunner::getRandomWhitePixel(cv::Mat image){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> rows_distribution(0, image.rows - 1);
    std::uniform_int_distribution<int> columns_distribution(0, image.cols - 1);

    int max_tries = 2000;
    for(int i=0; i<max_tries; i++){
      Vector2i pixel(columns_distribution(gen), rows_distribution(gen));
      if(isPixelWhite(image, pixel)){
        float theta_degress = (rand() % 361);
        float theta = theta_degress * CV_PI / 180;
        return std::make_pair(pixel, theta);
      }
    }
    throw std::runtime_error("[ExperimentsRunner::getRandomWhitePixel] Could not get a white pixel in " + std::to_string(max_tries) + " tries");
  }

  Point2fVectorCloud ExperimentsRunner::pixels2Cloud(MapManager* map_manager, std::vector<Vector2i> pixels){
    std::shared_ptr<Point2fVectorCloud> cloud(new Point2fVectorCloud());
    for(auto pixel : pixels){
      std::shared_ptr<Point2f> point(new Point2f);
      point->coordinates() = map_manager->pixel2Point(map_manager->resized2original(pixel));
      cloud->push_back(*point);
    }
    return *cloud;
  }

  Point2fVectorCloud ExperimentsRunner::cadCloud2SlamFrame(Point2fVectorCloud cad_cloud){
    std::shared_ptr<Point2fVectorCloud> cloud_slam_frame(new Point2fVectorCloud());
    for(auto point : cad_cloud){
      auto point_slam_frame = param_projector->cad2slam(Vector3f(point.coordinates().x(), point.coordinates().y(), 0));
      //auto point_slam_frame = param_projector->cad2slam_NEW(Vector3f(point.coordinates().x(), point.coordinates().y(), 0));

      std::shared_ptr<Point2f> slam_point(new Point2f);
      slam_point->coordinates() = point_slam_frame.head<2>();
      cloud_slam_frame->push_back(*slam_point);
    }
    return *cloud_slam_frame;
  }

  Point2fVectorCloud ExperimentsRunner::slamCloud2CadFrame(Point2fVectorCloud slam_cloud){
    std::shared_ptr<Point2fVectorCloud> slam_cad_frame(new Point2fVectorCloud());
    for(auto point : slam_cloud){
      auto point_cad_frame = param_projector->slam2cad(Vector3f(point.coordinates().x(), point.coordinates().y(), 0));

      std::shared_ptr<Point2f> cad_point(new Point2f);
      cad_point->coordinates() = point_cad_frame.head<2>();
      slam_cad_frame->push_back(*cad_point);
    }
    return *slam_cad_frame;
  }

  void ExperimentsRunner::generateRandomLidarReadings() {
    initialize();
    cv::Mat cad_resized_image = param_cad_map_manager->getResizedImage().clone();

    cv::Mat cad_resized_image_debug = param_cad_map_manager->getResizedImage().clone();
    cv::Mat slam_resized_image_debug = param_slam_map_manager->getResizedImage().clone();

    std::cerr << "[INFO] Generating " << param_num_lidar_readings.value() << " random LiDAR Readings" << std::endl;

    for(int i=0; i<param_num_lidar_readings.value(); i++){
      std::cerr << "\t[INFO] Generating reading " << i << "/" << param_num_lidar_readings.value() << std::endl;
      //Generate a random point in the CAD image from which to simulate the LiDAR
      auto cad_pixel_heading = getRandomWhitePixel(cad_resized_image);
      Vector2i cad_pixel = cad_pixel_heading.first;
      float cad_heading = cad_pixel_heading.second;

      cv::Point cad_pt1(cad_pixel.x(), cad_pixel.y());
      cv::Point cad_pt2(static_cast<int>(cad_pt1.x + 10 * cos(cad_heading)), static_cast<int>(cad_pt1.y - 10 * sin(cad_heading)));
      cv::arrowedLine(cad_resized_image_debug, cad_pt1, cad_pt2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA, 0, 0.3);
      cv::circle(cad_resized_image_debug, cad_pt1, 2, cv::Scalar(0, 0, 255), -1);

      auto slam_pixel_heading = param_projector->cad_pixel2slam_pixel(cad_pixel, cad_heading);
      Vector2i slam_pixel = slam_pixel_heading.first;
      float slam_heading = slam_pixel_heading.second;

      cv::Point slam_pt1(slam_pixel.x(), slam_pixel.y());
      cv::Point slam_pt2(static_cast<int>(slam_pt1.x + 10 * cos(slam_heading)), static_cast<int>(slam_pt1.y - 10 * sin(slam_heading)));
      cv::arrowedLine(slam_resized_image_debug, slam_pt1, slam_pt2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA, 0, 0.3);
      cv::circle(slam_resized_image_debug, slam_pt1, 2, cv::Scalar(0, 0, 255), -1);

      //Call the LiDAR simulator -- Get both clouds
      auto cad_pixels = param_lidar_simulator->simulateLidarPixels(param_cad_map_manager->getResizedImage(), cad_pixel, cad_heading);
      auto slam_pixels = param_lidar_simulator->simulateLidarPixels(param_slam_map_manager->getResizedImage(), slam_pixel, slam_heading);

      //TODO -- Check the max distance in meters

      auto cad_cloud = pixels2Cloud(param_cad_map_manager.value().get(), cad_pixels);
      auto slam_cloud = pixels2Cloud(param_slam_map_manager.value().get(), slam_pixels);

      auto cad_cloud_slam_frame = cadCloud2SlamFrame(cad_cloud);
      auto slam_cloud_cad_frame = slamCloud2CadFrame(slam_cloud);

      _cad_clouds.push_back(cad_cloud);
      _slam_clouds.push_back(slam_cloud);
      _cad2slam_clouds.push_back(cad_cloud_slam_frame);
      _slam2cad_clouds.push_back(slam_cloud_cad_frame);

      //Save the clouds
      if(param_save_lidar_cloud.value()) {
        std::string filename = to_string(i) + ".pcd";
        saveCloudAsPCD(cad_cloud, _cad_clouds_folder_path + "/" + filename);
        saveCloudAsPCD(slam_cloud, _slam_clouds_folder_path + "/" + filename);
        saveCloudAsPCD(cad_cloud_slam_frame, _cad2slam_clouds_folder_path + "/" + filename);
        saveCloudAsPCD(slam_cloud_cad_frame, _slam2cad_clouds_folder_path + "/" + filename);
      }
    }

    std::cerr << "[INFO] All LiDAR readings were generated" << std::endl;

    cv::imshow("Cad_image_debug", cad_resized_image_debug);
    cv::imshow("Slam_image_debug", slam_resized_image_debug);
    cv::waitKey(0);
  }

  void ExperimentsRunner::generateLidarReadings(std::vector<Vector3f> cad_poses, std::vector<Vector3f> slam_poses) {
    initialize();
    cv::Mat cad_resized_image = param_cad_map_manager->getResizedImage().clone();

    cv::Mat cad_resized_image_debug = param_cad_map_manager->getResizedImage().clone();
    cv::Mat slam_resized_image_debug = param_slam_map_manager->getResizedImage().clone();

    std::cerr << "[INFO] Generating " << cad_poses.size() << " LiDAR Readings" << std::endl;

    _cad_clouds.clear();
    _slam_clouds.clear();
    _cad2slam_clouds.clear();
    _slam2cad_clouds.clear();

    for(int i=0; i<cad_poses.size(); i++){
      std::cerr << "\r\t[INFO] Generating reading " << i+1 << "/" << cad_poses.size();
      auto cad_pose = cad_poses[i];
      auto cad_pixel_original = param_cad_map_manager->point2Pixel(cad_pose.head<2>());
      auto cad_pixel_resized = param_cad_map_manager->original2resized(cad_pixel_original);
      auto cad_heading = cad_pose.z();

      cv::Point cad_pt1(cad_pixel_resized.x(), cad_pixel_resized.y());
      cv::Point cad_pt2(static_cast<int>(cad_pt1.x + 10 * cos(cad_heading)), static_cast<int>(cad_pt1.y - 10 * sin(cad_heading)));
      cv::arrowedLine(cad_resized_image_debug, cad_pt1, cad_pt2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA, 0, 0.3);
      cv::circle(cad_resized_image_debug, cad_pt1, 1, cv::Scalar(0, 0, 255), -1);

      auto slam_pose = slam_poses[i];
      auto slam_pixel_original = param_slam_map_manager->point2Pixel(slam_pose.head<2>());
      auto slam_pixel_resized = param_slam_map_manager->original2resized(slam_pixel_original);
      auto slam_heading = slam_pose.z();

      cv::Point slam_pt1(slam_pixel_resized.x(), slam_pixel_resized.y());
      cv::Point slam_pt2(static_cast<int>(slam_pt1.x + 10 * cos(slam_heading)), static_cast<int>(slam_pt1.y - 10 * sin(slam_heading)));
      cv::arrowedLine(slam_resized_image_debug, slam_pt1, slam_pt2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA, 0, 0.3);
      cv::circle(slam_resized_image_debug, slam_pt1, 1, cv::Scalar(0, 0, 255), -1);

      //Call the LiDAR simulator -- Get both clouds
      auto cad_pixels = param_lidar_simulator->simulateLidarPixels(param_cad_map_manager->getResizedImage(), cad_pixel_resized, cad_heading);
      auto slam_pixels = param_lidar_simulator->simulateLidarPixels(param_slam_map_manager->getResizedImage(), slam_pixel_resized, slam_heading);

      //TODO -- Check the max distance in meters

      auto cad_cloud = pixels2Cloud(param_cad_map_manager.value().get(), cad_pixels);
      auto slam_cloud = pixels2Cloud(param_slam_map_manager.value().get(), slam_pixels);

      auto cad_cloud_slam_frame = cadCloud2SlamFrame(cad_cloud);

      _cad_clouds.push_back(cad_cloud);
      _slam_clouds.push_back(slam_cloud);
      _cad2slam_clouds.push_back(cad_cloud_slam_frame);

      //Save the clouds
      if(param_save_lidar_cloud.value()) {
        std::string filename = to_string(i) + ".pcd";
        saveCloudAsPCD(cad_cloud, _cad_clouds_folder_path + "/" + filename);
        saveCloudAsPCD(slam_cloud, _slam_clouds_folder_path + "/" + filename);
        saveCloudAsPCD(cad_cloud_slam_frame, _cad2slam_clouds_folder_path + "/" + filename);
      }
    }
    std::cerr << std::endl;
    std::cerr << "[INFO] All LiDAR readings were generated" << std::endl;

    if(param_visualize.value()){
      cv::imshow("Cad_image_debug", cad_resized_image_debug);
      cv::imshow("Slam_image_debug", slam_resized_image_debug);
      cv::waitKey(0);
    }
  }






  //TODO -- From here to the bottom it is mess


  std::vector<Vector2i> cad_start_pixels_resized;
  std::vector<Vector2i> cad_goal_pixels_resized;

  void selectPixelsCallback(int event, int x, int y, int flags, void* param) {
    if (event == cv::EVENT_LBUTTONDOWN) {
      cv::Mat* image = reinterpret_cast<cv::Mat*>(param);
      if(cad_start_pixels_resized.size() == cad_goal_pixels_resized.size()){
        cv::circle(*image, cv::Point(x,y), 3, index2hueValue(cad_goal_pixels_resized.size()), -1);
        std::cerr << "Start pixel selected at (" << x << ", " << y << ")" << std::endl;
        cad_start_pixels_resized.push_back({x,y});
      }else{
        cv::circle(*image, cv::Point(x,y), 3, index2hueValue(cad_goal_pixels_resized.size()), -1);
        std::cerr << "Goal pixel selected at (" << x << ", " << y << ")" << std::endl;
        std::cerr << ":::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;
        cad_goal_pixels_resized.push_back({x,y});

      }
      cv::imshow("Select start/goal pixels", *image);
    }
  }

  std::pair<std::vector<Vector2i>, std::vector<Vector2i>> ExperimentsRunner::getPathLimits(){
    initialize();

    cv::Mat cad_map = param_cad_map_manager->getResizedImage().clone();
    std::string window_name = "Select start/goal pixels";

    cv::imshow(window_name, cad_map);
    cv::setMouseCallback(window_name, selectPixelsCallback, &cad_map);
    cv::waitKey(0);

    //TODO -- Check that the number of pixels in each vector is the same
    if(cad_start_pixels_resized.size() == cad_goal_pixels_resized.size() + 1){
      std::cerr << "[INFO] [ExperimentsRunner::getPathLimits] Removing the last element of cad_start_pixels_resized as the sets are unbalanced" << std::endl;
      cad_start_pixels_resized.pop_back();
    }
    return std::make_pair(cad_start_pixels_resized, cad_goal_pixels_resized);
  }

  std::pair<std::vector<Vector2i>, std::vector<Vector2i>> ExperimentsRunner::loadPathLimits(std::string file_path) {
    std::vector<Vector2i> _cad_start_pixels;
    std::vector<Vector2i> _cad_goal_pixels;

    Deserializer des;
    SerializablePtr o;
    des.setFilePath(file_path);
    while((o=des.readObjectShared())){
      auto path_poses = std::dynamic_pointer_cast<PathPoses>(o);
      if(path_poses){
        _cad_start_pixels.push_back(path_poses->_start);
        _cad_goal_pixels.push_back(path_poses->_end);
      }
    }
    if(_cad_start_pixels.empty() || _cad_goal_pixels.empty()){
      throw std::runtime_error("[ExperimentsRunner::loadPathLimits] the list of path_poses is empty. Maybe the input file was empty - Exiting...");
    }
    return std::make_pair(_cad_start_pixels, _cad_goal_pixels);
  }

  void ExperimentsRunner::savePathLimits(std::pair<std::vector<Vector2i>, std::vector<Vector2i>> path_limits, std::string path) {
    std::cerr << "[INFO] Saving path limits to: " << path << std::endl;
    Serializer ser;
    ser.setFilePath(path);

    auto cad_start_pixels = path_limits.first;
    auto cad_goal_pixels = path_limits.second;

    for(int i=0; i<cad_start_pixels.size(); i++){
      auto path_poses = PathPoses(cad_start_pixels[i], cad_goal_pixels[i]);
      ser.writeObject(path_poses);
    }
  }



  int _arrow_length = 5;
  int _arrow_thickness = 2;
  cv::Scalar arrow_color(0,255,0);

  void ExperimentsRunner::saveResults(std::vector<float> values, std::vector<int> outliers, std::string path){
    std::ofstream out_file;
    out_file.open(path);

    if (!out_file) {
      throw std::runtime_error("[ExperimentsRunner::saveResults] Could not open file [" + path + "] - Exiting...");
    }

    for(int i=0; i<values.size(); i++){
      Vector3f cad_pose = _cad_poses[i];
      Vector3f slam_pose = _slam_poses[i];
      int outliers_counter = outliers[i];
      float error_value = values[i];

      auto cad_pixel = param_cad_map_manager->point2Pixel(cad_pose.head<2>());
      auto slam_pixel = param_slam_map_manager->point2Pixel(slam_pose.head<2>());

      if(error_value == -1){
        out_file << cad_pose.x() << " " << cad_pose.y() << " " << cad_pose.z() << " " << slam_pose.x() << " " << slam_pose.y() << " " << slam_pose.z() << " " << cad_pixel.x() << " " << cad_pixel.y() << " " << slam_pixel.x() << " " << slam_pixel.y() << " -1 " << outliers_counter << std::endl;
      }else{
        out_file << cad_pose.x() << " " << cad_pose.y() << " " << cad_pose.z() << " " << slam_pose.x() << " " << slam_pose.y() << " " << slam_pose.z() << " " << cad_pixel.x() << " " << cad_pixel.y() << " " << slam_pixel.x() << " " << slam_pixel.y() << " " << error_value << " " << outliers_counter << std::endl;
      }
    }
    out_file.close();
  }

  void ExperimentsRunner::saveStats(float min, float max, float mean, float std_dev, std::string path) {
    std::ofstream out_file;
    out_file.open(path);

    if (!out_file) {
      throw std::runtime_error("[ExperimentsRunner::saveStats] Could not open file [" + path + "] - Exiting...");
    }

    out_file << min << " " << max << " " << mean << " " << std_dev << std::endl;
    out_file.close();
  }

  void ExperimentsRunner::saveInfo(int experiment_id, int run_id, float threshold, float p2p_mean, float outliers_mean, std::string path){
    std::ofstream out_file;
    out_file.open(path);

    if (!out_file) {
      throw std::runtime_error("[ExperimentsRunner::saveInfo] Could not open file [" + path + "] - Exiting...");
    }
    out_file << "Experiment_id: " << experiment_id << std::endl;
    out_file << "Run_id: " << run_id << std::endl;
    out_file << "Threshold: " << threshold << std::endl;
    out_file << "P2p mean: " << p2p_mean << std::endl;
    out_file << "Outliers mean: " << outliers_mean << std::endl;
    out_file.close();
  }

  //Type 1 -- Draw the standard path
  //Type 2 -- Draw the p2p colored metric
  //Type 3 -- Draw the outliers metric
  void ExperimentsRunner::prepareGnuplotFile(std::string path, bool cad, int type){
    if(type < 1 || type > 3){
      throw std::runtime_error("[ExperimentsRunner::prepareGnuplotFile] invalid type [" + std::to_string(type) + "] received - Exiting...");
    }
    cv::Mat image;
    std::string image_path;
    std::string output_filename;
    if(cad){
      output_filename = "cad";
      image = param_cad_map_manager->getImage();
      image_path = param_cad_map_manager->getImagePath();
    }else{
      output_filename = "slam";
      image = param_slam_map_manager->getImage();
      image_path = param_slam_map_manager->getImagePath();
    }
    if(type == 2){
      output_filename += "_p2p";
    }
    if(type == 3){
      output_filename += "_outliers";
    }
    path = path.substr(0, path.find_last_of("/")+1) + output_filename + ".gp";

    std::ofstream out_file;
    out_file.open(path);
    if (!out_file) {
      throw std::runtime_error("[ExperimentsRunner::prepareGnuplotFile] Could not open file [" + path + "] - Exiting...");
    }
    out_file << "set xrange [0:" << image.cols << "]" << std::endl;
    out_file << "set yrange [0:" << image.rows << "]" << std::endl;

    out_file << "set terminal postscript eps enhanced color solid" << std::endl;
    out_file << "set output '" << output_filename << ".eps'" << std::endl;
    out_file << "set xtics" << std::endl;

    out_file << "adapt_y(y) = " << image.rows << "-y" << std::endl;

    out_file << "set size ratio -1" << std::endl;
    if(type == 1){
      out_file << "set style line 1 linewidth 10 linecolor rgb \"green\"" << std::endl;
    }else{
      out_file << "set style line 1 linewidth 10" << std::endl;
    }

    out_file << "set xtics" << std::endl;
    out_file << "set ytics" << std::endl;

    if(type != 1){
      out_file << "set palette defined (0 \"blue\", 1 \"red\")" << std::endl;
    }
    if(type == 2){
      out_file << "set cbrange [" << _p2p_min << ":" << _p2p_max << "]" << std::endl;
    }
    if(type == 3){
      out_file << "set cbrange [0:" << param_lidar_simulator->param_num_measurements.value() << "]" << std::endl;
    }

    out_file << "plot '" << image_path << "' binary filetype=png with rgbimage notitle, \\" << std::endl;
    if(type == 1){
      if(cad){
        out_file << "     'p2p.txt' using 7:(adapt_y($8)) with lines linestyle 1 notitle" << std::endl;
      }else{
        out_file << "     'p2p.txt' using 9:(adapt_y($10)) with lines linestyle 1 notitle" << std::endl;
      }
    }
    if(type == 2){
      if(cad){
        out_file << "     'p2p.txt' using 7:(adapt_y($8)):11 with lines linestyle 1 linecolor palette notitle" << std::endl;
      }else{
        out_file << "     'p2p.txt' using 9:(adapt_y($10)):11 with lines linestyle 1 linecolor palette notitle" << std::endl;
      }
    }
    if(type == 3){
      if(cad){
        out_file << "     'p2p.txt' using 7:(adapt_y($8)):12 with lines linestyle 1 linecolor palette notitle" << std::endl;
      }else{
        out_file << "     'p2p.txt' using 9:(adapt_y($10)):12 with lines linestyle 1 linecolor palette notitle" << std::endl;
      }
    }

    out_file << "unset output" << std::endl;
    out_file.close();

    char command[1024];
    boost::filesystem::path current_dir = boost::filesystem::current_path();
    sprintf(command,"cd %s; gnuplot %s.gp",_results_folder_path.c_str(),output_filename.c_str());
    system(command);
    sprintf(command,"cd %s; ps2pdf %s.eps %s_large.pdf",_results_folder_path.c_str(),output_filename.c_str(),output_filename.c_str());
    system(command);
    sprintf(command,"cd %s; pdfcrop %s_large.pdf %s.pdf",_results_folder_path.c_str(),output_filename.c_str(),output_filename.c_str());
    system(command);
    sprintf(command,"cd %s; rm %s_large.pdf",_results_folder_path.c_str(),output_filename.c_str());
    system(command);
  }

  void ExperimentsRunner::calculateP2P(){
    initialize();
    std::vector<float> p2p_results;
    std::vector<int> outliers;

    std::cerr << "[INFO] Calculating the p2p metric for " << _slam_clouds.size() << " clouds" << std::endl;
    for(int i=0; i < _slam_clouds.size(); i++){
      auto slam_cloud = _slam_clouds[i];
      auto cad2slam_cloud = _cad2slam_clouds[i];
      auto p2p_and_outliers = P2P(cad2slam_cloud, slam_cloud, i);
      std::cerr << "\r\t[INFO] Cloud " << i+1 << "/" << _slam_clouds.size() << " --> p2p: " << p2p_and_outliers.first << " -- Points/Outliers: " << slam_cloud.size() << "/" << p2p_and_outliers.second;
      p2p_results.push_back(p2p_and_outliers.first);
      outliers.push_back(p2p_and_outliers.second);
    }
    std::cerr << std::endl;

    float mean = calculateMean(p2p_results, false);
    float std_deviation = calculateStdDev(p2p_results, false);
    _p2p_min = calculateMin(p2p_results, false);
    _p2p_max = calculateMax(p2p_results, false);

    std::vector<float> outliers_float(outliers.begin(), outliers.end());
    float mean_outliers = calculateMean(outliers_float, true);
    float std_deviation_outliers = calculateStdDev(outliers_float, true);
    _outliers_min = calculateMin(outliers_float, true);
    _outliers_max = calculateMax(outliers_float, true);

    std::cerr << std::endl;
    std::cerr << "\t#############################" << std::endl;
    std::cerr << "\tMean --> " << mean << std::endl;
    std::cerr << "\tStd deviation --> " << std_deviation << std::endl;
    std::cerr << "\tMin --> " << _p2p_min << std::endl;
    std::cerr << "\tMax --> " << _p2p_max << std::endl;
    std::cerr << "\t#############################" << std::endl;
    std::cerr << "\tMean outliers --> " << mean_outliers << std::endl;
    std::cerr << "\tStd deviation outliers --> " << std_deviation_outliers << std::endl;
    std::cerr << "\tMin outliers --> " << _outliers_min << std::endl;
    std::cerr << "\tMax outliers --> " << _outliers_max << std::endl;
    std::cerr << "\t#############################" << std::endl;
    std::cerr << std::endl;

    std::string file_path = _results_folder_path + "/p2p.txt";
    saveResults(p2p_results, outliers, file_path);
    saveStats(_p2p_min, _p2p_max, mean, std_deviation, _results_folder_path + "/p2p_stats.txt");
    saveStats(_outliers_min, _outliers_max, mean_outliers, std_deviation_outliers, _results_folder_path + "/outlier_stats.txt");
    saveInfo(_experiment_id, _run_id, param_p2p_threshold.value(), mean, mean_outliers, _experiments_base_path + "/info.txt");
    prepareGnuplotFile(file_path, true, 1);
    prepareGnuplotFile(file_path, false, 1);
    prepareGnuplotFile(file_path, true, 2);
    prepareGnuplotFile(file_path, false, 2);
    prepareGnuplotFile(file_path, true, 3);
    prepareGnuplotFile(file_path, false, 3);
  }

  std::pair<float, int> ExperimentsRunner::P2P(Point2fVectorCloud source, Point2fVectorCloud target, int idx){
    std::vector<float> distances;
    int outliers = 0;
    for(auto p : source){
      //std::cerr << "\tPoint id " << i <<" (" << p.coordinates().x() << "," << p.coordinates().y() << ")";
      float distance = minDistanceToCloud(target, p);
      if(distance <= param_p2p_threshold.value()){
        distances.push_back(distance);
      }else{
        outliers++;
      }
    }
    if(distances.empty()){
      return std::make_pair(-1, outliers);
    }
    float mean = calculateMean(distances, true);
    return std::make_pair(mean, outliers);
  }

  float ExperimentsRunner::minDistanceToCloud(Point2fVectorCloud cloud, Point2f query_point){
    float min_distance = std::numeric_limits<float>::max();
    int closest_index = -1;
    Point2f closest_point;
    int i=0;
    for(auto point : cloud){
      float dist = P2P(point, query_point);
      if(dist < min_distance){
        min_distance = dist;
        closest_index = i;
        closest_point = point;
      }
      i++;
    }
    if(min_distance == std::numeric_limits<float>::max()){
      throw std::runtime_error("[ExperimentsRunner::minDistanceToCloud] Could not find the closest point - Exiting...");
    }
    return min_distance;
  }

  float ExperimentsRunner::P2P(Point2f source, Point2f target){
    return std::sqrt(std::pow(source.coordinates().x() - target.coordinates().x(), 2) + std::pow(source.coordinates().y() - target.coordinates().y(), 2));
  }



  //TODO -- Check if it is -atan2 or atan2 -- Now it works, it would need changes in other parts of the code
  double getTheta(Vector2i p1, Vector2i p2){
    int dx = p2.x() - p1.x();
    int dy = p2.y() - p1.y();
    double theta = -std::atan2(dy, dx);
    return theta;
  }

  std::vector<double> calculateOrientations(std::vector<Vector2i> pixels){
    std::vector<double> orientations;
    for (size_t i = 1; i < pixels.size(); ++i) {
      double orientation = getTheta(pixels[i-1], pixels[i]);
      orientations.push_back(orientation);
    }
    orientations.push_back(orientations.back());
    return orientations;
  }

  std::pair<std::vector<Vector3f>,std::vector<Vector3f>> ExperimentsRunner::generatePath(Vector2i cad_start_pixel_resized, Vector2i cad_goal_pixel_resized) {
    std::cerr << "Generate a path from " << cad_start_pixel_resized.transpose() << " to " << cad_goal_pixel_resized.transpose() << std::endl;
    initialize();

    std::vector<Vector3f> cad_poses;
    std::vector<Vector3f> slam_poses;

    cv::Mat cad_map = param_cad_map_manager->getResizedImage().clone();
    cv::Mat slam_map = param_slam_map_manager->getResizedImage().clone();

    std::string cad_window_name = param_cad_map_manager->param_window_name.value();
    std::string slam_window_name = param_slam_map_manager->param_window_name.value();

    std::string cad_yaml_file = param_cad_map_manager->param_yaml_path.value();

    auto cad_start_pixel_original = param_cad_map_manager->resized2original(cad_start_pixel_resized);
    auto cad_goal_pixel_original = param_cad_map_manager->resized2original(cad_goal_pixel_resized);

    auto theta_test = getTheta(cad_start_pixel_resized, cad_goal_pixel_resized);

    auto slam_start_pixel_resized = param_projector->cad_pixel2slam_pixel(cad_start_pixel_resized, theta_test);
    auto slam_goal_pixel_resized = param_projector->cad_pixel2slam_pixel(cad_goal_pixel_resized, 0);

    cv::circle(cad_map, cv::Point(cad_start_pixel_resized.x(), cad_start_pixel_resized.y()), 3, cv::Scalar(0, 255, 0), -1);
    cv::circle(cad_map, cv::Point(cad_goal_pixel_resized.x(), cad_goal_pixel_resized.y()), 3, cv::Scalar(0, 0, 255), -1);

    cv::circle(slam_map, cv::Point(slam_start_pixel_resized.first.x(), slam_start_pixel_resized.first.y()), 3, cv::Scalar(0, 255, 0), -1);
    cv::circle(slam_map, cv::Point(slam_goal_pixel_resized.first.x(), slam_goal_pixel_resized.first.y()), 3, cv::Scalar(0, 0, 255), -1);

    PathPlanner path_planner;
    auto px_path_original = path_planner.doPathPlanning(cad_start_pixel_original, cad_goal_pixel_original, cad_yaml_file);
    auto cad_orientations = calculateOrientations(px_path_original);

    for(int i=0; i<px_path_original.size(); i++){
      auto cad_px_original = px_path_original[i];
      auto cad_px_resized = param_cad_map_manager->original2resized(cad_px_original);

      auto cad_point = param_cad_map_manager->pixel2Point(cad_px_original);
      auto cad_theta = cad_orientations[i];
      cad_poses.push_back(Vector3f(cad_point.x(), cad_point.y(), cad_theta));

      auto slam_px_resized = param_projector->cad_pixel2slam_pixel(cad_px_resized, cad_theta);

      auto slam_px_original = param_slam_map_manager->resized2original(slam_px_resized.first);
      auto slam_point = param_slam_map_manager->pixel2Point(slam_px_original);
      auto slam_theta = slam_px_resized.second;
      slam_poses.push_back(Vector3f(slam_point.x(), slam_point.y(), slam_theta));

      cv::circle(cad_map, cv::Point(cad_px_resized.x(), cad_px_resized.y()), 1, cv::Scalar(255, 0, 0), -1);
      cv::circle(slam_map, cv::Point(slam_px_resized.first.x(), slam_px_resized.first.y()), 1, cv::Scalar(255, 0, 0), -1);
    }

    if(param_visualize.value()){
      cv::namedWindow(cad_window_name);
      cv::namedWindow(slam_window_name);
      cv::imshow(cad_window_name, cad_map);
      cv::imshow(slam_window_name, slam_map);
      cv::waitKey(0);
    }
    _cad_poses = cad_poses;
    _slam_poses = slam_poses;
    return std::make_pair(cad_poses, slam_poses);
  }






  //TODO -- New experiment

  void ExperimentsRunner::allCad2slam(){
    initialize();
    cv::Mat cad_image = param_cad_map_manager->getImage().clone();

    cv::Mat cad_resized_image_debug = param_cad_map_manager->getResizedImage().clone();
    cv::Mat slam_resized_image_debug = param_slam_map_manager->getResizedImage().clone();

    cv::Mat cad_resized_image_debug_mask = param_cad_map_manager->getResizedImage().clone();
    cv::Mat slam_resized_image_debug_mask = param_slam_map_manager->getResizedImage().clone();

    for(int col=0; col<cad_image.cols; col++){
      std::cerr << col << "/" << cad_image.cols << std::endl;
      for(int row=0; row<cad_image.rows; row++){
        Vector2i cad_pixel(col, row);
        if(isPixelWhite(cad_image, cad_pixel)){
          Vector2i slam_pixel = param_projector->cad_pixel2slam_pixel_no_resize(cad_pixel, 0).first;

          Vector2i cad_resized = param_cad_map_manager->original2resized(cad_pixel);
          Vector2i slam_resized = param_slam_map_manager->original2resized(slam_pixel);

          cv::circle(cad_resized_image_debug_mask, cv::Point(cad_resized.x(), cad_resized.y()), 1, cv::Scalar(0, 0, 255), -1);
          cv::circle(slam_resized_image_debug_mask, cv::Point(slam_resized.x(), slam_resized.y()), 1, cv::Scalar(0, 255, 0), -1);
        }
      }
    }
    float alpha = 0.6;
    cv::addWeighted(cad_resized_image_debug_mask, alpha, cad_resized_image_debug, 1 - alpha, 0, cad_resized_image_debug);
    cv::addWeighted(slam_resized_image_debug_mask, alpha, slam_resized_image_debug, 1 - alpha, 0, slam_resized_image_debug);


    cv::imshow("CAD", cad_resized_image_debug);
    cv::imshow("SLAM", slam_resized_image_debug);
    cv::waitKey(0);
  }

  void ExperimentsRunner::experiment_3(int number_of_samples) {
    initialize();
    std::vector<float> times;
    for(int i=0; i<number_of_samples; i++){
      auto random_pixel_and_theta = getRandomWhitePixel(param_cad_map_manager->getImage());
      auto random_pixel = random_pixel_and_theta.first;
      auto random_theta = random_pixel_and_theta.second;
      auto random_point = param_cad_map_manager->pixel2Point(random_pixel);
      auto random_pose = Vector3f(random_point.x(), random_point.y(), random_theta);

      //Evaluate time
      auto start = std::chrono::high_resolution_clock::now();



      auto cad2slam_pose = param_projector->cad2slam(random_pose);


      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<float, std::milli> elapsed = end-start;
      times.push_back(elapsed.count());
      std::cout << i << " -- Elapsed time: " << elapsed.count() << std::endl;
      //Stop evaluating time
    }

    float mean = calculateMean(times, true);
    float std_dev = calculateStdDev(times, true);

    std::cerr << "#####################################" << std::endl;
    std::cerr << "Mean running time: " << mean << std::endl;
    std::cerr << "Std deviation time: " << std_dev << std::endl;
    std::cerr << "#####################################" << std::endl;
  }

}