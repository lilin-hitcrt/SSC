#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <dirent.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include "ssc.h"
std::vector<std::string> listDir(std::string path,std::string end){
        DIR* pDir;
        struct dirent* ptr;
        std::vector<std::string> files;
        if(!(pDir=opendir(path.c_str()))){
            return files;
        }
        std::string subFile;
        while ((ptr=readdir(pDir))!=0){
            subFile=ptr->d_name;
            auto rt=subFile.find(end);
            if(rt != std::string::npos){
                files.emplace_back(path+subFile);
            }
        }
        std::sort(files.begin(),files.end());
        return files;
    }
pcl::PointCloud<pcl::PointXYZL>::Ptr myDownSampling(pcl::PointCloud<pcl::PointXYZL>::Ptr in_cloud){
    std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr > clouds_list(20);
    std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr > clouds_filtered(20);
    pcl::PointCloud<pcl::PointXYZL>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZL>());
    for(auto p:in_cloud->points){
        int id=p.label;
        if(clouds_list[id]==NULL){
            clouds_list[id].reset(new pcl::PointCloud<pcl::PointXYZL>);
        }
        clouds_list[id]->points.emplace_back(p);
    }
    pcl::VoxelGrid<pcl::PointXYZL> sor;
    for(int i=0;i<clouds_list.size();++i){
        auto& v=clouds_list[i];
        if(v==NULL){
            continue;
        }
        v->height=1;
        v->width=v->points.size();
        v->is_dense=false;
        if(clouds_filtered[i]==NULL){
            clouds_filtered[i].reset(new pcl::PointCloud<pcl::PointXYZL>);
        }
        if(i==9||i==10||i==11||i==12||i==13||i==15||i==17){
            sor.setLeafSize (0.4f, 0.4f, 0.4f);
        }else if(i==1||i==4||i==5||i==14||i==16){
            sor.setLeafSize (0.2f, 0.2f, 0.2f);
        }else{
            sor.setLeafSize (0.05f, 0.05f, 0.05f);
        }
        sor.setInputCloud (clouds_list[i]);
        sor.filter (*clouds_filtered[i]);
        *out_cloud+=*clouds_filtered[i];
    }
    return out_cloud;
}
void myCrop(pcl::PointCloud<pcl::PointXYZL>::Ptr in_cloud,pcl::PointCloud<pcl::PointXYZL>::Ptr& out_cloud,pcl::PointCloud<pcl::PointXYZL>::Ptr& out_cloud_neg,Eigen::Isometry3f pose){
     pcl::PassThrough<pcl::PointXYZL> pass;
     pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZL>),cloud_filtered_x_neg(new pcl::PointCloud<pcl::PointXYZL>);
     out_cloud.reset(new pcl::PointCloud<pcl::PointXYZL>);
     out_cloud_neg.reset(new pcl::PointCloud<pcl::PointXYZL>);
     pass.setInputCloud (in_cloud);
     pass.setFilterFieldName ("x");
     pass.setFilterLimits(pose(0,3)-50, pose(0,3)+50);
     pass.setNegative(false);
     pass.filter (*cloud_filtered_x);
     pass.setNegative(true);
     pass.filter (*cloud_filtered_x_neg);

     pass.setInputCloud (cloud_filtered_x);
     pass.setFilterFieldName ("y");
     pass.setFilterLimits(pose(1,3)-50, pose(1,3)+50);
     pass.setNegative(false);
     pass.filter (*out_cloud);
     pass.setNegative(true);
     pass.filter (*out_cloud_neg);
     *out_cloud_neg+=*cloud_filtered_x_neg;
}
int main(){
    std::string conf_file="../config/config.yaml";
    auto data_cfg = YAML::LoadFile(conf_file);
    auto cloud_path=data_cfg["eval_seq"]["cloud_path"].as<std::string>();
    auto label_path=data_cfg["eval_seq"]["label_path"].as<std::string>();
    auto pose_file=data_cfg["eval_seq"]["pose_file"].as<std::string>();
    auto calib_file=data_cfg["eval_seq"]["calib_file"].as<std::string>();
    auto cloud_files=listDir(cloud_path,".bin");
    auto label_files=listDir(label_path,".label");
    SSC ssc(conf_file);
    std::ifstream pf(pose_file);
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
    pcl::PointCloud<pcl::PointXYZL>::Ptr map(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::visualization::CloudViewer viewer("viewer");
    for(int i=0;i<cloud_files.size();++i){
        std::cout<<cloud_files[i]<<"  "<<label_files[i]<<std::endl;
        Eigen::Isometry3f pose=Eigen::Isometry3f::Identity();
        for(int j=0;j<12;++j){
            float v;
            pf>>v;
            pose(j/4,j%4)=v;
        }
        pose=T.inverse()*pose*T;
        auto cloudl=ssc.getLCloud(cloud_files[i],label_files[i]);
        pcl::PointCloud<pcl::PointXYZL>::Ptr transformd_cloud(new pcl::PointCloud<pcl::PointXYZL>);
        pcl::transformPointCloud(*cloudl,*transformd_cloud,pose.matrix());
        // std::cout<<pose(0,3)<<" "<<pose(1,3)<<" "<<pose(2,3)<<std::endl;
        *map+=*transformd_cloud;
        pcl::PointCloud<pcl::PointXYZL>::Ptr map1(new pcl::PointCloud<pcl::PointXYZL>);
        pcl::PointCloud<pcl::PointXYZL>::Ptr map2(new pcl::PointCloud<pcl::PointXYZL>);
        pcl::PointCloud<pcl::PointXYZL>::Ptr map3(new pcl::PointCloud<pcl::PointXYZL>);
        myCrop(map,map1,map2,pose);

        map3=myDownSampling(map1);
        map.reset(new pcl::PointCloud<pcl::PointXYZL>);
        *map+=*map2;
        *map+=*map3;
        auto cloudc=ssc.getColorCloud(map);
        viewer.showCloud(cloudc);
    }
    pcl::io::savePCDFileASCII ("map.pcd", *map);
    pf.close();
    return 0;
}