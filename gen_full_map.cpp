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
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr myDownSampling(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in_cloud){
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > clouds_list(20);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > clouds_filtered(20);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    for(auto p:in_cloud->points){
        int id=p.g;
        if(clouds_list[id]==NULL){
            clouds_list[id].reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
        }
        clouds_list[id]->points.emplace_back(p);
    }
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    for(int i=0;i<clouds_list.size();++i){
        auto& v=clouds_list[i];
        if(v==NULL){
            continue;
        }
        v->height=1;
        v->width=v->points.size();
        v->is_dense=false;
        if(clouds_filtered[i]==NULL){
            clouds_filtered[i].reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
        }
        if(i==9||i==10||i==11||i==12||i==13||i==15||i==17){
            sor.setLeafSize (0.4f, 0.4f, 0.4f);
        }else if(i==1||i==4||i==5||i==14||i==16){
            sor.setLeafSize (0.2f, 0.2f, 0.2f);
        }else{
            sor.setLeafSize (0.1f, 0.1f, 0.1f);
        }
        sor.setInputCloud (clouds_list[i]);
        sor.filter (*clouds_filtered[i]);
        *out_cloud+=*clouds_filtered[i];
    }
    return out_cloud;
}
void myCrop(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in_cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& out_cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& out_cloud_neg,Eigen::Isometry3f pose){
     pcl::PassThrough<pcl::PointXYZRGBA> pass;
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZRGBA>),cloud_filtered_x_neg(new pcl::PointCloud<pcl::PointXYZRGBA>);
     out_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
     out_cloud_neg.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
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
void getLabel(std::string file, std::vector<uint32_t> &sem_labels)
{
    int32_t num = 1000000;
    uint32_t *data = (uint32_t *)malloc(num * sizeof(uint32_t));
    uint32_t *px = data + 0;
    FILE *stream;
    stream = fopen(file.c_str(), "rb");
    if (stream == NULL)
    {
        std::cerr << "stream is NULL!" << std::endl;
    }
    num = fread(data, sizeof(uint32_t), num, stream);
    for (int32_t i = 0; i < num; i++)
    {
        uint32_t label = (*px);
        uint32_t sem_label;
        sem_label = label_map[(int)(label & 0x0000ffff)];

        // uint32_t ins_label = (label & 0xffff0000) >> 16;
        sem_labels[i] = sem_label;
        px += 1;
        // std::cout<<label<<" "<<sem_label<<" "<<ins_label<<std::endl;
    }
    fclose(stream);
    free(data);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr getCloud(std::string file)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    int32_t num = 600000;
    float *data = (float *)malloc(num * sizeof(float));
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;
    FILE *stream;
    stream = fopen(file.c_str(), "rb");
    if (stream == NULL)
    {
        std::cerr << "stream is NULL!" << std::endl;
        return NULL;
    }
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    num = fread(data, sizeof(float), num, stream) / 4;
    cloud->points.resize(num);
    for (int32_t i = 0; i < num; i++)
    {
        pcl::PointXYZI p;
        p.x = (*px);
        p.y = (*py);
        p.z = (*pz);
        p.intensity = (*pr);
        px += 4;
        py += 4;
        pz += 4;
        pr += 4;
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

    auto cloud_path="/media/l/yp2/KITTI/odometry/dataset/sequences/10/velodyne/";
    auto sk_label_path="/media/l/yp2/KITTI/odometry/dataset/sequences/10/labels/";
    auto rn_label_path="/media/l/yp2/KITTI/darknet53-knn/sequences/10/predictions/";
    auto pose_file="/media/l/yp2/KITTI/odometry/dataset/sequences/10/poses.txt";
    auto calib_file="/media/l/yp2/KITTI/odometry/dataset/sequences/10/calib.txt";
    auto cloud_files=listDir(cloud_path,".bin");
    auto sk_label_files=listDir(sk_label_path,".label");
    auto rn_label_files=listDir(rn_label_path,".label");
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
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr map(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::visualization::CloudViewer viewer("viewer");
    for(int i=0;i<cloud_files.size();++i){
        std::cout<<cloud_files[i]<<"  "<<sk_label_files[i]<<" "<<rn_label_files[i]<<std::endl;
        Eigen::Isometry3f pose=Eigen::Isometry3f::Identity();
        for(int j=0;j<12;++j){
            float v;
            pf>>v;
            pose(j/4,j%4)=v;
        }
        pose=T.inverse()*pose*T;
        auto cloud=getCloud(cloud_files[i]);
        std::vector<uint32_t> sem_labels_sk,sem_labels_rn;
        sem_labels_sk.resize(cloud->points.size());
        sem_labels_rn.resize(cloud->points.size());
        getLabel(sk_label_files[i], sem_labels_sk);
        getLabel(rn_label_files[i], sem_labels_rn);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr re_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        re_cloud->points.resize(cloud->points.size());
        for (int i = 0; i < cloud->points.size(); ++i)
        {
            if(sqrt( cloud->points[i].x* cloud->points[i].x+ cloud->points[i].y* cloud->points[i].y)>50){
                continue;
            }
            if (sem_labels_sk[i] == 0&&sem_labels_rn[i]==0)
            {
                continue;
            }
            re_cloud->points[i].x = cloud->points[i].x;
            re_cloud->points[i].y = cloud->points[i].y;
            re_cloud->points[i].z = cloud->points[i].z;
            re_cloud->points[i].b = 255*cloud->points[i].intensity;
            re_cloud->points[i].g=sem_labels_sk[i];
            re_cloud->points[i].r=sem_labels_rn[i];
        }
        re_cloud->height = 1;
        re_cloud->width = re_cloud->points.size();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformd_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::transformPointCloud(*re_cloud,*transformd_cloud,pose.matrix());
        // std::cout<<pose(0,3)<<" "<<pose(1,3)<<" "<<pose(2,3)<<std::endl;
        *map+=*transformd_cloud;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr map1(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr map2(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr map3(new pcl::PointCloud<pcl::PointXYZRGBA>);
        myCrop(map,map1,map2,pose);

        map3=myDownSampling(map1);
        map.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
        *map+=*map2;
        *map+=*map3;
        auto ccloud=getColorCloud(map);
        viewer.showCloud(ccloud);
    }
    pcl::io::savePCDFileASCII ("map.pcd", *map);
    pf.close();
    std::ofstream fout("map.bin",ios::binary);
    for (auto p : map->points){
        fout.write((char*)(&p.x),sizeof(p.x));
        fout.write((char*)(&p.y),sizeof(p.y));
        fout.write((char*)(&p.z),sizeof(p.z));
        float b,g,r;
        b=p.b;
        g=p.g;
        r=p.r;
        fout.write((char*)(&b),sizeof(b));
        fout.write((char*)(&g),sizeof(g));
        fout.write((char*)(&r),sizeof(r));
    }
    fout.close();
    return 0;
}