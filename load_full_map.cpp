#include <iostream>
#include <string>
#include <dirent.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include "my_voxel_grid.hpp"
#include "ssc.h"
typedef std::tuple<u_char, u_char, u_char> Color;
    std::map<uint32_t, Color> _color_map, _argmax_to_rgb;
    YAML::Node learning_map;
    std::vector<int> label_map;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud(std::string file)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    int32_t num = 60000000;
    float *data = (float *)malloc(num * sizeof(float));
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pi = data + 3;
    float *pl1 = data + 4;
    float *pl2 = data + 5;
    FILE *stream;
    stream = fopen(file.c_str(), "rb");
    if (stream == NULL)
    {
        std::cerr << "stream is NULL!" << std::endl;
        return NULL;
    }
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    num = fread(data, sizeof(float), num, stream) / 6;
    cloud->points.resize(num);
    for (int32_t i = 0; i < num; i++)
    {
        pcl::PointXYZRGBA p;
        p.x = (*px);
        p.y = (*py);
        p.z = (*pz);
        p.b = (*pi)*255;
        p.g = (*pl1);
        p.r = (*pl2);
        px += 6;
        py += 6;
        pz += 6;
        pi += 6;
        pl1 += 6;
        pl2 += 6;
        cloud->points[i] = p;
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    fclose(stream);
    free(data);
    return cloud;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColorCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_in)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    outcloud->points.resize(cloud_in->points.size());
    for (size_t i = 0; i < outcloud->points.size(); i++)
    {
        uint8_t label=cloud_in->points[i].g;
        // std::cout<<+label<<std::endl;
        if(label==0){
            continue;
        }
        outcloud->points[i].x = cloud_in->points[i].x;
        outcloud->points[i].y = cloud_in->points[i].y;
        outcloud->points[i].z = cloud_in->points[i].z;
        outcloud->points[i].r = std::get<0>(_argmax_to_rgb[label]);
        outcloud->points[i].g = std::get<1>(_argmax_to_rgb[label]);
        outcloud->points[i].b = std::get<2>(_argmax_to_rgb[label]);
    }
    return outcloud;
}
int main(){
         auto data_cfg = YAML::LoadFile("../config/config.yaml");
    auto color_map = data_cfg["color_map"];
    learning_map = data_cfg["learning_map"];
    label_map.resize(260);
    for (auto it = learning_map.begin(); it != learning_map.end(); ++it)
    {
        label_map[it->first.as<int>()] = it->second.as<int>();
    }
    YAML::const_iterator it;
    for (it = color_map.begin(); it != color_map.end(); ++it)
    {
        // Get label and key
        int key = it->first.as<int>(); // <- key
        Color color = std::make_tuple(
            static_cast<u_char>(color_map[key][0].as<unsigned int>()),
            static_cast<u_char>(color_map[key][1].as<unsigned int>()),
            static_cast<u_char>(color_map[key][2].as<unsigned int>()));
        _color_map[key] = color;
    }
    auto learning_class = data_cfg["learning_map_inv"];
    auto _n_classes = learning_class.size();
    for (it = learning_class.begin(); it != learning_class.end(); ++it)
    {
        int key = it->first.as<int>(); // <- key
        _argmax_to_rgb[key] = _color_map[learning_class[key].as<unsigned int>()];
    }


    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("map.pcd", *cloud) == -1) //* load the file
    // {
    //     PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    //     return NULL;
    // }
    auto cloud=getCloud("map.bin");
    std::cout<<"size:"<<cloud->size()<<std::endl;
    auto ccloud=getColorCloud(cloud);
    pcl::visualization::CloudViewer viewer("viewer");
    viewer.showCloud(ccloud);
    while(!viewer.wasStopped()){
        continue;
    }
    return 0;
}