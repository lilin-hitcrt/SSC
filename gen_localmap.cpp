#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include<random>
#include <pcl/common/transforms.h>
#include "ssc.h"
void myCrop(pcl::PointCloud<pcl::PointXYZL>::Ptr in_cloud,pcl::PointCloud<pcl::PointXYZL>::Ptr& out_cloud,Eigen::Isometry3f pose){
     pcl::PassThrough<pcl::PointXYZL> pass;
     pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZL>),cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZL>);
     out_cloud.reset(new pcl::PointCloud<pcl::PointXYZL>);
     pass.setInputCloud (in_cloud);
     pass.setFilterFieldName ("x");
     pass.setFilterLimits(pose(0,3)-50, pose(0,3)+50);
     pass.setNegative(false);
     pass.filter (*cloud_filtered_x);

     pass.setInputCloud (cloud_filtered_x);
     pass.setFilterFieldName ("y");
     pass.setFilterLimits(pose(1,3)-50, pose(1,3)+50);
     pass.setNegative(false);
     pass.filter (*cloud_filtered_y);
    pcl::transformPointCloud(*cloud_filtered_y,*out_cloud,pose.inverse().matrix());
}
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZL>);

  if (pcl::io::loadPCDFile<pcl::PointXYZL> ("map.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  
    std::string pose_file="/media/l/yp2/KITTI/odometry/data_odometry_labels/dataset/sequences/00/poses.txt";
    std::string calib_file="/media/l/yp2/KITTI/odometry/dataset/sequences/00/calib.txt";
        std::ifstream cf(calib_file);
    Eigen::Isometry3f T=Eigen::Isometry3f::Identity();
    for(int j=0;j<5;++j){
        std::string c;
        cf>>c;
        for(int i=0;i<12;++i){
            float temp;
            cf>>temp;
            if(j==4){
                T(i/4,i%4)=temp;
            }
        }
    }
    std::vector<Eigen::Isometry3f,Eigen::aligned_allocator<Eigen::Isometry3f> > poses;
    std::vector<float> temp_pose;
    std::ifstream pf(pose_file);
    while(1){
        float v=10000000;
        pf>>v;
        if(v==10000000){
            break;
        }
        temp_pose.emplace_back(v);
    }
    if(temp_pose.size()%12!=0){
        std::cerr<<"pose size error:"<<temp_pose.size()<<" "<<temp_pose.size()/12<<" "<<temp_pose.size()%12;
    }
    for(int i=0;i<temp_pose.size();++i){
        int num_id=i%12;
        if(num_id==0){
            poses.emplace_back(Eigen::Isometry3f::Identity());
        }
        poses.back()(num_id/4,num_id%4)=temp_pose[i];
    }
    std::string conf_file="../config/config.yaml";
    SSC ssc(conf_file);
     pcl::visualization::CloudViewer viewer("viewer");
    for(int i=0;i<poses.size();++i){
        pcl::PointCloud<pcl::PointXYZL>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZL>);
        auto posei=T.inverse()*poses[i]*T;
        myCrop(cloud,out_cloud,posei);
        std::stringstream ss;
        ss<<"local_map"<<i<<".pcd";
        pcl::io::savePCDFileASCII (ss.str(), *out_cloud);
        auto cloudc=ssc.getColorCloud(out_cloud);
        viewer.showCloud(cloudc);
    }

  return (0);
}
