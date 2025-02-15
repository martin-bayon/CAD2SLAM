#include "manual_correspondence_finder.h"

#include <srrg_config/configurable_command.h>
#include <opencv2/opencv.hpp>


namespace cad2slam {

  ManualCorrespondenceFinder::ManualCorrespondenceFinder() {
    addCommand(new ConfigurableCommand_<ManualCorrespondenceFinder, typeof(&ManualCorrespondenceFinder::cmdOpenCorrespondenceFinder), std::string>(
            this,
            "selectCorrespondences",
            "Opens the correspondence finder",
            &ManualCorrespondenceFinder::cmdOpenCorrespondenceFinder));
  }

  void ManualCorrespondenceFinder::initialize() {
    if(!_is_initialized){
      checkParams();
      cad_info.map_manager = param_cad_map_manager.value().get();
      slam_info.map_manager = param_slam_map_manager.value().get();
      _is_initialized = true;
    }
  }

  void ManualCorrespondenceFinder::checkParams() {
    if(!param_config_master.value()) {
      throw std::runtime_error("[ManualCorrespondenceFinder::checkParams] config_master is not set - Exiting...");
    }
    if(!param_cad_map_manager.value()) {
      throw std::runtime_error("[ManualCorrespondenceFinder::checkParams] cad_map_manager is not set - Exiting...");
    }
    if(!param_slam_map_manager.value()) {
      throw std::runtime_error("[ManualCorrespondenceFinder::checkParams] slam_map_manager is not set - Exiting...");
    }
  }

  bool ManualCorrespondenceFinder::cmdOpenCorrespondenceFinder(std::string &response) {
    selectCorrespondences();
    return false;
  }

  void ManualCorrespondenceFinder::selectCorrespondences() {
    initialize();

    std::string cad_window_name = param_cad_map_manager->param_window_name.value();
    std::string slam_window_name = param_slam_map_manager->param_window_name.value();

    cv::namedWindow(cad_window_name);
    cv::namedWindow(slam_window_name);

    cv::Mat cad_image = param_cad_map_manager->getResizedImage().clone();
    cv::Mat slam_image = param_slam_map_manager->getResizedImage().clone();

    cv::imshow(cad_window_name, cad_image);
    cv::imshow(slam_window_name, slam_image);

    cv::setMouseCallback(cad_window_name, correspondence_finder_callback, (void*)&cad_info);
    cv::setMouseCallback(slam_window_name, correspondence_finder_callback, (void*)&slam_info);

    cv::waitKey(0);

    saveCorrespondences();
  }

  void ManualCorrespondenceFinder::visualizeCorrespondences() {
    initialize();

    std::string cad_window_name = param_cad_map_manager->param_window_name.value();
    std::string slam_window_name = param_slam_map_manager->param_window_name.value();

    cv::namedWindow(cad_window_name);
    cv::namedWindow(slam_window_name);

    cv::Mat cad_image = param_cad_map_manager->getResizedImage().clone();
    cv::Mat slam_image = param_slam_map_manager->getResizedImage().clone();



    std::vector<CorrespondenceAnchor> correspondenceAnchors = param_config_master->getCorrespondenceAnchors();


    //Serializer ser;
    //ser.setFilePath(param_config_master->getCorrespondenceAnchorsFilename()+"_test.txt");

    for(int i=0; i<correspondenceAnchors.size(); i++){
      auto correspondence = correspondenceAnchors[i];

      auto cad_point = correspondence.getCadPoint();
      auto cad_pixel_original = param_cad_map_manager->point2Pixel(cad_point);
      auto cad_pixel_resize = param_cad_map_manager->original2resized(cad_pixel_original);

      auto slam_point = correspondence.getSlamPoint();

      auto slam_pixel_original = param_slam_map_manager->point2Pixel(slam_point);
      auto slam_pixel_resize = param_slam_map_manager->original2resized(slam_pixel_original);

      cv::circle(cad_image, cv::Point(cad_pixel_resize.x(), cad_pixel_resize.y()), param_visualizer_correspondence_size.value(), index2hueValue(i), -1);
      cv::circle(slam_image, cv::Point(slam_pixel_resize.x(), slam_pixel_resize.y()), param_visualizer_correspondence_size.value(), index2hueValue(i), -1);
    }

    cv::imshow(cad_window_name, cad_image);
    cv::imshow(slam_window_name, slam_image);

    cv::waitKey(0);
  }

  void ManualCorrespondenceFinder::correspondence_finder_callback(int event, int x, int y, int flags, void* userdata){
    CorrespondenceFinderInfo* info = static_cast<CorrespondenceFinderInfo*>(userdata);
    if(!info) {
      throw std::runtime_error("[ManualCorrespondenceFinder::correspondence_finder_callback] the userdata received is not a compatible - Exiting...");
    }

    cv::Mat image =  info->map_manager->getResizedImage().clone();

    cv::line(image, cv::Point(0, y), cv::Point(image.cols-1, y), index2hueValue(info->points.size()), 1, cv::LINE_AA);
    cv::line(image, cv::Point(x, 0), cv::Point(x, image.rows-1), index2hueValue(info->points.size()), 1, cv::LINE_AA);

    if(event == cv::EVENT_LBUTTONDOWN){
      auto resized_pixel = Vector2i(x, y);
      auto original_pixel = info->map_manager->resized2original(resized_pixel);
      auto point = info->map_manager->pixel2Point(original_pixel);
      info->points.push_back(point);
      std::cerr << "[INFO] " << info->points.size() << " correspondence points in " << info->map_manager->param_window_name.value() << std::endl;
    }
    if(event == cv::EVENT_RBUTTONDOWN){
      if(!info->points.empty()){
        info->points.pop_back();
      }
      std::cerr << "[INFO] " << info->points.size() << " correspondence points in " << info->map_manager->param_window_name.value() << std::endl;
    }

    for(int i=0; i<info->points.size(); i++){
      auto point = info->points[i];
      auto original_pixel = info->map_manager->point2Pixel(point);
      auto resized_pixel = info->map_manager->original2resized(original_pixel);
      cv::circle(image, cv::Point(resized_pixel.x(), resized_pixel.y()), 3, index2hueValue(i), -1);
    }
    cv::imshow(info->map_manager->param_window_name.value(), image);
  }

  void ManualCorrespondenceFinder::saveCorrespondences() {
    if(cad_info.points.size() != slam_info.points.size()){
      throw std::runtime_error("[ManualCorrespondenceFinder::selectCorrespondences] The number of correspondences is different in both images - Exiting...");
    }
    if(cad_info.points.empty()){
      throw std::runtime_error("[ManualCorrespondenceFinder::selectCorrespondences] You have to select at least one correspondence - Exiting...");
    }

    Serializer ser;
    ser.setFilePath(param_config_master->getCorrespondenceAnchorsFilename());

    for(int i=0; i<cad_info.points.size(); i++){
      auto cad_point = cad_info.points[i];
      auto slam_point = slam_info.points[i];

      CorrespondenceAnchor anchor(cad_point, slam_point);
      ser.writeObject(anchor);
    }
  }
}