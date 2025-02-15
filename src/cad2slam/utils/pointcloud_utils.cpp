#include <fstream>
#include "pointcloud_utils.h"

namespace cad2slam {

  PcdFile cloud2PCD(Point2fVectorCloud cloud){
    PcdFile pcd_file;
    PcdHeader pcd_header;
    pcd_header.version =    "VERSION .7";
    pcd_header.fields =     "FIELDS x y z ";
    pcd_header.size =       "SIZE 4 4 4";
    pcd_header.type =       "TYPE F F F";
    pcd_header.count =      "COUNT 1 1 1";
    pcd_header.width =      "WIDTH " + to_string(cloud.size());
    pcd_header.height =     "HEIGHT 1";
    pcd_header.viewpoint =  "VIEWPOINT 0 0 0 1 0 0 0";
    pcd_header.points =     "POINTS " + to_string(cloud.size());
    pcd_header.data =       "DATA ascii";
    shared_ptr<vector<string>> points = shared_ptr<vector<string>>(new vector<string>());
    for(const auto& point : cloud){
      points->push_back(to_string(point.coordinates().x()) + " " + to_string(point.coordinates().y()) + " 0 ");
    }
    pcd_file.header = pcd_header;
    pcd_file.points = points;
    return pcd_file;
  }

  bool saveCloudAsPCD(Point2fVectorCloud cloud, string filename){
    PcdFile pcd_file = cloud2PCD(cloud);
    PcdHeader pcd_header = pcd_file.header;
    shared_ptr<vector<string>> points = pcd_file.points;

    ofstream output_file;
    output_file.open (filename);
    output_file << pcd_header.version + "\n";
    output_file << pcd_header.fields + "\n";
    output_file << pcd_header.size + "\n";
    output_file << pcd_header.type + "\n";
    output_file << pcd_header.count + "\n";
    output_file << pcd_header.width + "\n";
    output_file << pcd_header.height + "\n";
    output_file << pcd_header.viewpoint + "\n";
    output_file << pcd_header.points + "\n";
    output_file << pcd_header.data + "\n";
    for(string point: *points){
      output_file << point + "\n";
    }
    output_file.close();
    //std::cerr << "The cloud has been saved as [" << filename << "]" << std::endl;
    return true;
  }
}