#include "iostream"
#include "srrg_pcl/point.h"


namespace cad2slam {
  using namespace std;
  using namespace srrg2_core;

  struct PcdHeader{
      string version;
      string fields;
      string size;
      string type;
      string count;
      string width;
      string height;
      string viewpoint;
      string points;
      string data;
  };

  struct PcdFile{
      PcdHeader header;
      shared_ptr<vector<string>> points;
  };

  PcdFile cloud2PCD(Point2fVectorCloud cloud);
  bool saveCloudAsPCD(Point2fVectorCloud cloud, string filename);
}