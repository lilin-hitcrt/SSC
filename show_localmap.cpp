#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include<random>
#include <pcl/common/transforms.h>
#include "ssc.h"
int main (int argc, char** argv)
{
    std::string conf_file="../config/config.yaml";
    SSC ssc(conf_file);
    // pcl::visualization::CloudViewer viewer("viewer");
    auto cloudl=ssc.getLCloud("local_map200.pcd");
    auto desc=ssc.calculate_ssc(cloudl);
    auto img=ssc.getColorImage(desc);
    cv::imshow("image",img);
    cv::waitKey(0);
  return (0);
}
