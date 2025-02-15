#include "projector.h"

#include "srrg_config/configurable_command.h"

namespace cad2slam {

  float epsilon = 10e-4;

  Projector::Projector() {
    addCommand(new ConfigurableCommand_<Projector, typeof(&Projector::cmdLoadGraph), std::string>(
            this,
            "loadGraph",
            "Loads the optimized Graph file indicated in the Config Master",
            &Projector::cmdLoadGraph));
  }

  bool Projector::cmdLoadGraph(std::string &response) {
    loadGraph();
    return true;
  }

  void Projector::initialize() {
    if(!_initialized){
      checkParams();
      _initialized = true;
    }
  }

  void Projector::checkParams() {
    if(!param_config_master.value()){
      throw std::runtime_error("[Projector::checkParams] config_master is not set - Exiting...");
    }
    if(!param_cad_map_manager.value()){
      throw std::runtime_error("[Projector::checkParams] cad_map_manager is not set - Exiting...");
    }
    if(!param_slam_map_manager.value()){
      throw std::runtime_error("[Projector::checkParams] slam_map_manager is not set - Exiting...");
    }
  }

  void Projector::checkGraph(){
    if(!_is_graph_ready){
      throw std::runtime_error("[Projector::checkGraph] graph is not ready, please load the graph before trying to use it - Exiting...");
    }
    if(_graph->variables().empty() || _graph->factors().empty()){
      throw std::runtime_error("[Projector::checkGraph] the loaded graph is empty - Exiting...");
    }
  }

  Vector3f Projector::cad2slam(Vector3f cad_pose) {
    if(param_use_new_cad2slam.value()){
      return cad2slam_NEW(cad_pose);
    }else{
      return cad2slam_OLD(cad_pose);
    }
  }

  Vector3f Projector::cad2slam_OLD(Vector3f cad_pose) {
    //std::cerr << "using old cad2slam" << std::endl;
    initialize();
    checkGraph();

    KeyframeSE2* closest_keyframe = closestCadPoseKeyframe(cad_pose.head<2>(), _graph);
    Isometry2f cad_pose_wrt_keyframe = closest_keyframe->getCadIso().inverse() * geometry2d::v2t(cad_pose);
    Isometry2f slam_pose = closest_keyframe->getSlamIso() * cad_pose_wrt_keyframe;

    return geometry2d::t2v(slam_pose);
  }

  Vector3f Projector::cad2slam_NEW(Vector3f cad_pose){
    initialize();
    checkGraph();

    //std::vector<std::pair<float, KeyframeSE2*>> neighboringKeyframes = neighborhoodCadPoseKeyframe(cad_pose.head<2>(), _graph, param_k_neighbours.value());
    std::vector<std::pair<float, KeyframeSE2*>> neighboringKeyframes = neighborhoodCadPoseKeyframe(cad_pose.head<2>(), _graph, param_threshold.value());
    std::vector<float> normalized_weights;
    std::vector<float> weights;
    float pose_x = 0;
    float pose_y = 0;
    float pose_theta = 0;

    float theta_x_sum = 0;
    float theta_y_sum = 0;

    float weights_sum = 0;
    for(std::pair<float, KeyframeSE2*> neighborKeyframe : neighboringKeyframes) {
      float weight = 1 / (neighborKeyframe.first+epsilon);
      weights.push_back(weight);
      weights_sum += weight;
    }
    for(float weight : weights) {
      float normalized_weight = weight / weights_sum;
      normalized_weights.push_back(normalized_weight);
    }

    for(int i=0; i<neighboringKeyframes.size(); i++){
      KeyframeSE2* keyframe = neighboringKeyframes[i].second;
      Isometry2f cad_pose_wrt_keyframe = keyframe->getCadIso().inverse() * geometry2d::v2t(cad_pose);
      Isometry2f slam_pose_iso = keyframe->getSlamIso() * cad_pose_wrt_keyframe;
      Vector3f slam_pose = geometry2d::t2v(slam_pose_iso);
      //std::cerr << "\tWeight:" << normalized_weights[i] << " -------- Subpose:" << slam_pose.x() << ", " << slam_pose.y() << ", " << slam_pose.z()*180/M_PI << std::endl;

      pose_x += normalized_weights[i]*slam_pose.x();
      pose_y += normalized_weights[i]*slam_pose.y();
      theta_x_sum += normalized_weights[i]*cos(slam_pose.z());
      theta_y_sum += normalized_weights[i]*sin(slam_pose.z());
    }
    Vector3f mean_pose(pose_x, pose_y, atan2(theta_y_sum,theta_x_sum));
    //std::cerr << "Mean pose: " << mean_pose.x() << ", " << mean_pose.y() << ", " << mean_pose.z()*180/M_PI << std::endl;
    return mean_pose;
  }


  Vector3f Projector::slam2cad(Vector3f slam_pose) {
    //std::cerr << "using old slam2cad" << std::endl;
    initialize();
    checkGraph();

    auto closest_keyframe = closestSlamPoseKeyframe(slam_pose.head<2>(), _graph);

    Isometry2f slam_pose_wrt_keyframe = closest_keyframe->getSlamIso().inverse() * geometry2d::v2t(slam_pose);
    Isometry2f cad_pose = closest_keyframe->getCadIso() * slam_pose_wrt_keyframe;

    return geometry2d::t2v(cad_pose);
  }

  std::pair<Vector2i, float> Projector::cad_pixel2slam_pixel(Vector2i cad_pixel, float cad_theta){
    if(param_use_new_cad2slam.value()){
      return cad_pixel2slam_pixel_NEW(cad_pixel, cad_theta);
    }else{
      return cad_pixel2slam_pixel_OLD(cad_pixel, cad_theta);
    }
  }

  std::pair<Vector2i, float> Projector::cad_pixel2slam_pixel_OLD(Vector2i cad_pixel, float cad_theta){
    //std::cerr << "using old cad_pixel2slam_pixel" << std::endl;
    initialize();
    checkGraph();

    Vector2i original_cad_pixel = param_cad_map_manager->resized2original(cad_pixel);
    Vector2f cad_point = param_cad_map_manager->pixel2Point(original_cad_pixel);
    Vector3f cad_pose(cad_point.x(), cad_point.y(), cad_theta);

    Vector3f slam_pose = cad2slam(cad_pose);
    Vector2f slam_point = slam_pose.head<2>();
    float slam_theta = slam_pose.z();

    Vector2i original_slam_pixel = param_slam_map_manager->point2Pixel(slam_point);
    Vector2i resized_slam_pixel = param_slam_map_manager->original2resized(original_slam_pixel);

    return std::make_pair(resized_slam_pixel, slam_theta);
  }

  std::pair<Vector2i, float> Projector::cad_pixel2slam_pixel_NEW(Vector2i cad_pixel, float cad_theta){
    initialize();
    checkGraph();

    Vector2i original_cad_pixel = param_cad_map_manager->resized2original(cad_pixel);
    Vector2f cad_point = param_cad_map_manager->pixel2Point(original_cad_pixel);
    Vector3f cad_pose(cad_point.x(), cad_point.y(), cad_theta);

    Vector3f slam_pose = cad2slam_NEW(cad_pose);
    Vector2f slam_point = slam_pose.head<2>();
    float slam_theta = slam_pose.z();

    Vector2i original_slam_pixel = param_slam_map_manager->point2Pixel(slam_point);
    Vector2i resized_slam_pixel = param_slam_map_manager->original2resized(original_slam_pixel);

    return std::make_pair(resized_slam_pixel, slam_theta);
  }

  std::pair<Vector2i, float> Projector::slam_pixel2cad_pixel(Vector2i slam_pixel, float slam_theta){
    //std::cerr << "using old slam_pixel2cad_pixel" << std::endl;
    initialize();
    checkGraph();

    Vector2i original_slam_pixel = param_slam_map_manager->resized2original(slam_pixel);
    Vector2f slam_point = param_slam_map_manager->pixel2Point(original_slam_pixel);
    Vector3f slam_pose(slam_point.x(), slam_point.y(), slam_theta);

    Vector3f cad_pose = slam2cad(slam_pose);
    Vector2f cad_point = cad_pose.head<2>();
    float cad_theta = cad_pose.z();

    Vector2i original_cad_pixel = param_cad_map_manager->point2Pixel(cad_point);
    Vector2i resized_cad_pixel = param_cad_map_manager->original2resized(original_cad_pixel);
    return std::make_pair(resized_cad_pixel, cad_theta);
  }

  std::pair<Vector2i, float> Projector::cad_pixel2slam_pixel_no_resize(Vector2i original_cad_pixel, float cad_theta){
    initialize();
    checkGraph();

    Vector2f cad_point = param_cad_map_manager->pixel2Point(original_cad_pixel);
    Vector3f cad_pose(cad_point.x(), cad_point.y(), cad_theta);

    Vector3f slam_pose = cad2slam(cad_pose);
    Vector2f slam_point = slam_pose.head<2>();
    float slam_theta = slam_pose.z();

    Vector2i original_slam_pixel = param_slam_map_manager->point2Pixel(slam_point);

    return std::make_pair(original_slam_pixel, slam_theta);
  }

  std::pair<Vector2i, float> Projector::slam_pixel2cad_pixel_no_resize(Vector2i original_slam_pixel, float slam_theta){
    initialize();
    checkGraph();

    Vector2f slam_point = param_slam_map_manager->pixel2Point(original_slam_pixel);
    Vector3f slam_pose(slam_point.x(), slam_point.y(), slam_theta);

    Vector3f cad_pose = slam2cad(slam_pose);
    Vector2f cad_point = cad_pose.head<2>();
    float cad_theta = cad_pose.z();

    Vector2i original_cad_pixel = param_cad_map_manager->point2Pixel(cad_point);
    return std::make_pair(original_cad_pixel, cad_theta);
  }

  bool Projector::cadOriginalIsWhite(Vector2i resized_pixel, Projector* projector){
    Vector2i original_cad_pixel = projector->param_cad_map_manager->resized2original(resized_pixel);
    cv::Vec3b color = projector->param_cad_map_manager->getImage().at<cv::Vec3b>(cv::Point(original_cad_pixel.x(), original_cad_pixel.y()));
    if (color[0] > _white_pixel_threshold && color[1] > _white_pixel_threshold && color[2] > _white_pixel_threshold) {
      return true;
    }
    return false;
  }

  bool Projector::slamOriginalIsWhite(Vector2i resized_pixel, Projector* projector){
    Vector2i original_slam_pixel = projector->param_slam_map_manager->resized2original(resized_pixel);
    cv::Vec3b color = projector->param_slam_map_manager->getImage().at<cv::Vec3b>(cv::Point(original_slam_pixel.x(), original_slam_pixel.y()));
    if (color[0] > _white_pixel_threshold && color[1] > _white_pixel_threshold && color[2] > _white_pixel_threshold) {
      return true;
    }
    return false;
  }

  void Projector::setGraph(FactorGraphPtr graph_){
    initialize();
    _graph = std::move(graph_);
    _is_graph_ready = true;
  }

  void Projector::loadGraph() {
    initialize();
    _graph = FactorGraph::read(param_config_master->getOptimizedGraphFilename());
    _is_graph_ready = true;
  }

  std::vector<Vector2i> Projector::_prev_cad_pixels;
  void Projector::cad2slam_callback(int event, int x, int y, int flags, void* userdata){
    Projector* projector = static_cast<Projector*>(userdata);
    if(!projector) {
      throw std::runtime_error("[Projector::cad2slam_callback] the userdata received is not a Projector object - Exiting...");
    }

    cv::Mat cad_resized_image_copy = projector->param_cad_map_manager->getResizedImage().clone();
    cv::Mat slam_resized_image_copy = projector->param_slam_map_manager->getResizedImage().clone();

    Vector2i cad_pixel = Vector2i(x, y);
    if(!cadOriginalIsWhite(cad_pixel, projector)){
      cv::imshow(projector->param_cad_map_manager->param_window_name.value(), cad_resized_image_copy);
      cv::imshow(projector->param_slam_map_manager->param_window_name.value(), slam_resized_image_copy);
      return;
    }
    float cad_theta = 0;
    if(_prev_cad_pixels.size() == 2){
      cad_theta = -std::atan2(y - _prev_cad_pixels[0].y(), x - _prev_cad_pixels[0].x());
    }
    _prev_cad_pixels.push_back(cad_pixel);
    if(_prev_cad_pixels.size() > 2){
      _prev_cad_pixels.erase(_prev_cad_pixels.begin());
    }

    //auto slam_pixel_and_theta = projector->cad_pixel2slam_pixel_OLD(cad_pixel, cad_theta);
    auto slam_pixel_and_theta_NEW = projector->cad_pixel2slam_pixel_NEW(cad_pixel, cad_theta);

    //Vector2i slam_pixel = slam_pixel_and_theta.first;
    //float slam_theta = slam_pixel_and_theta.second;

    Vector2i slam_pixel_NEW = slam_pixel_and_theta_NEW.first;
    float slam_theta_NEW = slam_pixel_and_theta_NEW.second;

    cv::Scalar arrow_color(0,0,255);
    cv::Scalar arrow_color_NEW(0,0,255);
    //if(!slamOriginalIsWhite(slam_pixel, projector)){
    //  arrow_color = cv::Scalar(0, 0, 255);
    //}

    if(!slamOriginalIsWhite(slam_pixel_NEW, projector)){
      arrow_color_NEW = cv::Scalar(0, 0, 255);
    }

    //Plot arrow in CAD image
    cv::Point pt1(x, y);
    cv::Point pt2(x + _arrow_length * cos(cad_theta), y - _arrow_length * sin(cad_theta));
    cv::arrowedLine(cad_resized_image_copy, pt1, pt2, arrow_color, _arrow_thickness);

    //Plot arrow in SLAM image
    //cv::Point pt1_r(slam_pixel.x(), slam_pixel.y());
    //cv::Point pt2_r(slam_pixel.x() + _arrow_length * cos(slam_theta), slam_pixel.y() - _arrow_length * sin(slam_theta));
    //cv::arrowedLine(slam_resized_image_copy, pt1_r, pt2_r, arrow_color, _arrow_thickness);

    cv::Point pt1_r_NEW(slam_pixel_NEW.x(), slam_pixel_NEW.y());
    cv::Point pt2_r_NEW(slam_pixel_NEW.x() + _arrow_length * cos(slam_theta_NEW), slam_pixel_NEW.y() - _arrow_length * sin(slam_theta_NEW));
    cv::arrowedLine(slam_resized_image_copy, pt1_r_NEW, pt2_r_NEW, arrow_color_NEW, _arrow_thickness);

    cv::imshow(projector->param_cad_map_manager->param_window_name.value(), cad_resized_image_copy);
    cv::imshow(projector->param_slam_map_manager->param_window_name.value(), slam_resized_image_copy);
  }

  std::vector<Vector2i> Projector::_prev_slam_pixels;
  void Projector::slam2cad_callback(int event, int x, int y, int flags, void* userdata){
    Projector* projector = static_cast<Projector*>(userdata);
    if(!projector) {
      throw std::runtime_error("[Projector::slam2cad_callback] the userdata received is not a Projector object - Exiting...");
    }

    cv::Mat cad_resized_image_copy = projector->param_cad_map_manager->getResizedImage().clone();
    cv::Mat slam_resized_image_copy = projector->param_slam_map_manager->getResizedImage().clone();

    Vector2i slam_pixel = Vector2i(x, y);
    if(!slamOriginalIsWhite(slam_pixel, projector)){
      cv::imshow(projector->param_cad_map_manager->param_window_name.value(), cad_resized_image_copy);
      cv::imshow(projector->param_slam_map_manager->param_window_name.value(), slam_resized_image_copy);
      return;
    }
    float slam_theta = 0;
    if(_prev_slam_pixels.size() == 2){
      slam_theta = -std::atan2(y - _prev_slam_pixels[0].y(), x - _prev_slam_pixels[1].x());
    }
    _prev_slam_pixels.push_back(slam_pixel);
    if(_prev_slam_pixels.size() > 2){
      _prev_slam_pixels.erase(_prev_slam_pixels.begin());
    }

    auto cad_pixel_and_theta = projector->slam_pixel2cad_pixel(slam_pixel, slam_theta);
    Vector2i cad_pixel = cad_pixel_and_theta.first;
    float cad_theta = cad_pixel_and_theta.second;

    cv::Scalar arrow_color(0,255,0);
    if(!cadOriginalIsWhite(cad_pixel, projector)){
      arrow_color = cv::Scalar(0, 0, 255);
    }

    //Plot arrow in SLAM image
    cv::Point pt1(x, y);
    cv::Point pt2(x + _arrow_length * cos(slam_theta), y - _arrow_length * sin(slam_theta));
    cv::arrowedLine(slam_resized_image_copy, pt1, pt2, arrow_color, _arrow_thickness);

    //Plot arrow in CAD image
    cv::Point pt1_r(cad_pixel.x(), cad_pixel.y());
    cv::Point pt2_r(cad_pixel.x() + _arrow_length * cos(cad_theta), cad_pixel.y() - _arrow_length * sin(cad_theta));
    cv::arrowedLine(cad_resized_image_copy, pt1_r, pt2_r, arrow_color, _arrow_thickness);

    cv::imshow(projector->param_cad_map_manager->param_window_name.value(), cad_resized_image_copy);
    cv::imshow(projector->param_slam_map_manager->param_window_name.value(), slam_resized_image_copy);
  }

  void Projector::launchProjector(){
    initialize();
    checkGraph();

    std::string cad_window_name = param_cad_map_manager->param_window_name.value();
    std::string slam_window_name = param_slam_map_manager->param_window_name.value();

    cv::namedWindow(cad_window_name);
    cv::namedWindow(slam_window_name);

    cv::Mat cad_image = param_cad_map_manager->getResizedImage().clone();
    cv::Mat slam_image = param_slam_map_manager->getResizedImage().clone();

    cv::imshow(cad_window_name, cad_image);
    cv::imshow(slam_window_name, slam_image);

    cv::setMouseCallback(cad_window_name, cad2slam_callback, this);
    cv::setMouseCallback(slam_window_name, slam2cad_callback, this);

    cv::waitKey(0);
  }
}