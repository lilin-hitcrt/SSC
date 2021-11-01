#include "ssc.h"
#include "pcl/impl/point_types.hpp"
SSC::SSC(std::string conf_file)
{
    auto data_cfg = YAML::LoadFile(conf_file);
    show = data_cfg["show"].as<bool>();
    remap = data_cfg["remap"].as<bool>();
    if (show)
    {
        viewer.reset(new pcl::visualization::CloudViewer("viewer"));
    }
    rotate = data_cfg["rotate"].as<bool>();
    occlusion = data_cfg["occlusion"].as<bool>();
    gettimeofday(&time_t, nullptr);
    random_generator.reset(new std::default_random_engine(time_t.tv_usec));
    random_distribution.reset(new std::uniform_int_distribution<int>(-18000, 18000));
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
    for (it = learning_class.begin(); it != learning_class.end(); ++it)
    {
        int key = it->first.as<int>(); // <- key
        _argmax_to_rgb[key] = _color_map[learning_class[key].as<unsigned int>()];
    }
}

SSC::~SSC()
{
}

pcl::PointCloud<pcl::PointXYZL>::Ptr SSC::getLCloud(std::string file_cloud, std::string file_label)
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr re_cloud(new pcl::PointCloud<pcl::PointXYZL>());
    std::ifstream in_label(file_label, std::ios::binary);
    if (!in_label.is_open())
    {
        std::cerr << "No file:" << file_label << std::endl;
        exit(-1);
    }
    in_label.seekg(0, std::ios::end);
    uint32_t num_points = in_label.tellg() / sizeof(uint32_t);
    in_label.seekg(0, std::ios::beg);
    std::vector<uint32_t> values_label(num_points);
    in_label.read((char *)&values_label[0], num_points * sizeof(uint32_t));
    std::ifstream in_cloud(file_cloud, std::ios::binary);
    std::vector<float> values_cloud(4 * num_points);
    in_cloud.read((char *)&values_cloud[0], 4 * num_points * sizeof(float));
    re_cloud->points.resize(num_points);
    float random_angle = 0, max_angle = 0;
    float cs = 1, ss = 0;
    if (rotate || occlusion)
    {
        random_angle = (*random_distribution)(*random_generator) * M_PI / 18000.0;
        max_angle = random_angle + M_PI / 6.;
        cs = cos(random_angle);
        ss = sin(random_angle);
    }
    for (uint32_t i = 0; i < num_points; ++i)
    {
        if (occlusion)
        {
            float theta = atan2(values_cloud[4 * i + 1], values_cloud[4 * i]);
            if (theta > random_angle && theta < max_angle)
            {
                continue;
            }
        }
        uint32_t sem_label;
        if (remap)
        {
            sem_label = label_map[(int)(values_label[i] & 0x0000ffff)];
        }
        else
        {
            sem_label = values_label[i];
        }
        if (sem_label == 0)
        {
            re_cloud->points[i].x = 0;
            re_cloud->points[i].y = 0;
            re_cloud->points[i].z = 0;
            re_cloud->points[i].label = 0;
            continue;
        }
        if (rotate)
        {
            re_cloud->points[i].x = values_cloud[4 * i] * cs - values_cloud[4 * i + 1] * ss;
            re_cloud->points[i].y = values_cloud[4 * i] * ss + values_cloud[4 * i + 1] * cs;
        }
        else
        {
            re_cloud->points[i].x = values_cloud[4 * i];
            re_cloud->points[i].y = values_cloud[4 * i + 1];
        }
        re_cloud->points[i].z = values_cloud[4 * i + 2];
        re_cloud->points[i].label = sem_label;
    }
    in_label.close();
    in_cloud.close();
    return re_cloud;
}
pcl::PointCloud<pcl::PointXYZL>::Ptr SSC::getLCloud(std::string file_cloud)
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);
    if (pcl::io::loadPCDFile<pcl::PointXYZL>(file_cloud, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return NULL;
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr SSC::getColorCloud(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud_in)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    outcloud->points.resize(cloud_in->points.size());
    for (size_t i = 0; i < outcloud->points.size(); i++)
    {
        outcloud->points[i].x = cloud_in->points[i].x;
        outcloud->points[i].y = cloud_in->points[i].y;
        outcloud->points[i].z = cloud_in->points[i].z;
        outcloud->points[i].r = std::get<0>(_argmax_to_rgb[cloud_in->points[i].label]);
        outcloud->points[i].g = std::get<1>(_argmax_to_rgb[cloud_in->points[i].label]);
        outcloud->points[i].b = std::get<2>(_argmax_to_rgb[cloud_in->points[i].label]);
    }
    outcloud->height = 1;
    outcloud->width = outcloud->points.size();
    return outcloud;
}

cv::Mat SSC::project(pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_pointcloud)
{
    auto sector_step = 2. * M_PI / sectors_range;
    cv::Mat ssc_dis = cv::Mat::zeros(cv::Size(sectors_range, 1), CV_32FC4);
    for (uint i = 0; i < filtered_pointcloud->points.size(); i++)
    {
        auto label = filtered_pointcloud->points[i].label;
        if (label == 13 || label == 14 || label == 16 || label == 18 || label == 19)
        {
            float distance = std::sqrt(filtered_pointcloud->points[i].x * filtered_pointcloud->points[i].x + filtered_pointcloud->points[i].y * filtered_pointcloud->points[i].y);
            if (distance < 1e-2)
            {
                continue;
            }
            // int sector_id = cv::fastAtan2(filtered_pointcloud->points[i].y, filtered_pointcloud->points[i].x);
            float angle = M_PI + std::atan2(filtered_pointcloud->points[i].y, filtered_pointcloud->points[i].x);
            int sector_id = std::floor(angle / sector_step);
            if (sector_id >= sectors_range || sector_id < 0)
                continue;
            // if(ssc_dis.at<cv::Vec4f>(0, sector_id)[3]<10||distance<ssc_dis.at<cv::Vec4f>(0, sector_id)[0]){
            ssc_dis.at<cv::Vec4f>(0, sector_id)[0] = distance;
            ssc_dis.at<cv::Vec4f>(0, sector_id)[1] = filtered_pointcloud->points[i].x;
            ssc_dis.at<cv::Vec4f>(0, sector_id)[2] = filtered_pointcloud->points[i].y;
            ssc_dis.at<cv::Vec4f>(0, sector_id)[3] = label;
            // }
        }
    }
    return ssc_dis;
}

cv::Mat SSC::calculateSSC(pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_pointcloud)
{
    auto ring_step = (max_dis - min_dis) / rings;
    auto sector_step = 360. / sectors;
    cv::Mat ssc = cv::Mat::zeros(cv::Size(sectors, rings), CV_8U);
    for (int i = 0; i < (int)filtered_pointcloud->points.size(); i++)
    {
        auto label = filtered_pointcloud->points[i].label;
        if (order_vec[label] > 0)
        {
            double distance = std::sqrt(filtered_pointcloud->points[i].x * filtered_pointcloud->points[i].x + filtered_pointcloud->points[i].y * filtered_pointcloud->points[i].y);
            if (distance >= max_dis || distance < min_dis)
                continue;
            int sector_id = cv::fastAtan2(filtered_pointcloud->points[i].y, filtered_pointcloud->points[i].x)/ sector_step;
            int ring_id = (distance - min_dis) / ring_step;
            if (ring_id >= rings || ring_id < 0)
                continue;
            if (sector_id >= sectors || sector_id < 0)
                continue;
            if (order_vec[label] > order_vec[ssc.at<unsigned char>(ring_id, sector_id)])
            {
                ssc.at<unsigned char>(ring_id, sector_id) = label;
            }
        }
    }
    return ssc;
}

cv::Mat SSC::getColorImage(cv::Mat &desc)
{
    cv::Mat out = cv::Mat::zeros(desc.size(), CV_8UC3);
    for (int i = 0; i < desc.rows; ++i)
    {
        for (int j = 0; j < desc.cols; ++j)
        {
            out.at<cv::Vec3b>(i, j)[0] = std::get<2>(_argmax_to_rgb[(int)desc.at<uchar>(i, j)]);
            out.at<cv::Vec3b>(i, j)[1] = std::get<1>(_argmax_to_rgb[(int)desc.at<uchar>(i, j)]);
            out.at<cv::Vec3b>(i, j)[2] = std::get<0>(_argmax_to_rgb[(int)desc.at<uchar>(i, j)]);
        }
    }
    return out;
}

void SSC::globalICP(cv::Mat &ssc_dis1, cv::Mat &ssc_dis2, double &angle, float &diff_x, float &diff_y)
{
    double similarity = 100000;
    int sectors = ssc_dis1.cols;
    for (int i = 0; i < sectors; ++i)
    {
        float dis_count = 0;
        for (int j = 0; j < sectors; ++j)
        {
            int new_col = j + i >= sectors ? j + i - sectors : j + i;
            cv::Vec4f vec1 = ssc_dis1.at<cv::Vec4f>(0, j);
            cv::Vec4f vec2 = ssc_dis2.at<cv::Vec4f>(0, new_col);
            // if(vec1[3]==vec2[3]){
            dis_count += fabs(vec1[0] - vec2[0]);
            // }
        }
        if (dis_count < similarity)
        {
            similarity = dis_count;
            angle = i;
        }
    }
    int angle_o = angle;
    angle = M_PI * (360. - angle * 360. / sectors) / 180.;
    auto cs = cos(angle);
    auto sn = sin(angle);
    auto temp_dis1 = ssc_dis1.clone();
    auto temp_dis2 = ssc_dis2.clone();
    for (int i = 0; i < sectors; ++i)
    {
        temp_dis2.at<cv::Vec4f>(0, i)[1] = ssc_dis2.at<cv::Vec4f>(0, i)[1] * cs - ssc_dis2.at<cv::Vec4f>(0, i)[2] * sn;
        temp_dis2.at<cv::Vec4f>(0, i)[2] = ssc_dis2.at<cv::Vec4f>(0, i)[1] * sn + ssc_dis2.at<cv::Vec4f>(0, i)[2] * cs;
    }

    for (int i = 0; i < 100; ++i)
    {
        float dx = 0, dy = 0;
        int diff_count = 1;
        for (int j = 0; j < sectors; ++j)
        {
            cv::Vec4f vec1 = temp_dis1.at<cv::Vec4f>(0, j);
            if (vec1[0] <= 0)
            {
                continue;
            }
            int min_id = -1;
            float min_dis = 1000000.;
            for (int k = j + angle_o - 10; k < j + angle_o + 10; ++k)
            {
                cv::Vec4f vec_temp;
                int temp_id = k;
                if (k < 0)
                {
                    temp_id = k + sectors;
                }
                else if (k >= sectors)
                {
                    temp_id = k - sectors;
                }
                vec_temp = temp_dis2.at<cv::Vec4f>(0, temp_id);
                if (vec_temp[0] <= 0)
                {
                    continue;
                }
                float temp_dis = (vec1[1] - vec_temp[1]) * (vec1[1] - vec_temp[1]) + (vec1[2] - vec_temp[2]) * (vec1[2] - vec_temp[2]);
                if (temp_dis < min_dis)
                {
                    min_dis = temp_dis;
                    min_id = temp_id;
                }
            }
            if (min_id < 0)
            {
                continue;
            }
            cv::Vec4f vec2 = temp_dis2.at<cv::Vec4f>(0, min_id);
            if (fabs(vec1[1] - vec2[1]) < 3 && fabs(vec1[2] - vec2[2]) < 3)
            {
                dx += vec1[1] - vec2[1];
                dy += vec1[2] - vec2[2];
                diff_count++;
            }
        }
        dx = 1. * dx / diff_count;
        dy = 1. * dy / diff_count;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int j = 0; j < sectors; ++j)
        {
            if (temp_dis2.at<cv::Vec4f>(0, j)[0] != 0)
            {
                temp_dis2.at<cv::Vec4f>(0, j)[1] += dx;
                temp_dis2.at<cv::Vec4f>(0, j)[2] += dy;
                if (show)
                {
                    pcl::PointXYZRGB p;
                    p.x = temp_dis2.at<cv::Vec4f>(0, j)[1];
                    p.y = temp_dis2.at<cv::Vec4f>(0, j)[2];
                    p.z = 0;
                    p.r = std::get<0>(_argmax_to_rgb[(int)temp_dis2.at<cv::Vec4f>(0, j)[3]]);
                    p.g = std::get<1>(_argmax_to_rgb[(int)temp_dis2.at<cv::Vec4f>(0, j)[3]]);
                    p.b = std::get<2>(_argmax_to_rgb[(int)temp_dis2.at<cv::Vec4f>(0, j)[3]]);
                    temp_cloud->points.emplace_back(p);
                }
            }

            if (show && temp_dis1.at<cv::Vec4f>(0, j)[0] != 0)
            {
                pcl::PointXYZRGB p;
                p.x = temp_dis1.at<cv::Vec4f>(0, j)[1];
                p.y = temp_dis1.at<cv::Vec4f>(0, j)[2];
                p.z = 0;
                p.r = std::get<0>(_argmax_to_rgb[(int)temp_dis1.at<cv::Vec4f>(0, j)[3]]);
                p.g = std::get<1>(_argmax_to_rgb[(int)temp_dis1.at<cv::Vec4f>(0, j)[3]]);
                p.b = std::get<2>(_argmax_to_rgb[(int)temp_dis1.at<cv::Vec4f>(0, j)[3]]);
                temp_cloud->points.emplace_back(p);
            }
        }
        if (show)
        {
            temp_cloud->height = 1;
            temp_cloud->width = temp_cloud->points.size();
            viewer->showCloud(temp_cloud);
            usleep(1000000);
        }

        diff_x += dx;
        diff_y += dy;
        if (show)
        {
            std::cout << i << " diff " << diff_x << " " << diff_y << " " << dx << " " << dy << std::endl;
        }
        if (fabs(dx) < 1e-5 && fabs(dy) < 1e-5)
        {
            break;
        }
    }
}

Eigen::Matrix4f SSC::globalICP(cv::Mat &ssc_dis1, cv::Mat &ssc_dis2){
    double similarity = 100000;
    float angle=0;
    int sectors = ssc_dis1.cols;
    for (int i = 0; i < sectors; ++i)
    {
        float dis_count = 0;
        for (int j = 0; j < sectors; ++j)
        {
            int new_col = j + i >= sectors ? j + i - sectors : j + i;
            cv::Vec4f vec1 = ssc_dis1.at<cv::Vec4f>(0, j);
            cv::Vec4f vec2 = ssc_dis2.at<cv::Vec4f>(0, new_col);
            // if(vec1[3]==vec2[3]){
            dis_count += fabs(vec1[0] - vec2[0]);
            // }
        }
        if (dis_count < similarity)
        {
            similarity = dis_count;
            angle = i;
        }
    }
    angle = M_PI * (360. - angle * 360. / sectors) / 180.;
    auto cs = cos(angle);
    auto sn = sin(angle);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>),cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < sectors; ++i)
    {
        if(ssc_dis1.at<cv::Vec4f>(0, i)[3]>0){
            cloud1->push_back(pcl::PointXYZ(ssc_dis1.at<cv::Vec4f>(0, i)[1],ssc_dis1.at<cv::Vec4f>(0, i)[2],0.));
        }
        if(ssc_dis2.at<cv::Vec4f>(0, i)[3]>0){
            float tpx = ssc_dis2.at<cv::Vec4f>(0, i)[1] * cs - ssc_dis2.at<cv::Vec4f>(0, i)[2] * sn;
            float tpy = ssc_dis2.at<cv::Vec4f>(0, i)[1] * sn + ssc_dis2.at<cv::Vec4f>(0, i)[2] * cs;
            cloud2->push_back(pcl::PointXYZ(tpx,tpy,0.));
        }
    }
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud2);
    icp.setInputTarget(cloud1);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    auto trans=icp.getFinalTransformation();
    Eigen::Affine3f trans1 = Eigen::Affine3f::Identity();
    trans1.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
    return trans*trans1.matrix();
}

double SSC::calculateSim(cv::Mat &desc1, cv::Mat &desc2)
{
    double similarity = 0;
    int sectors = desc1.cols;
    int rings = desc1.rows;
    int valid_num = 0;
    for (int p = 0; p < sectors; p++)
    {
        for (int q = 0; q < rings; q++)
        {
            if (desc1.at<unsigned char>(q, p) == 0 && desc2.at<unsigned char>(q, p) == 0)
            {
                continue;
            }
            valid_num++;

            if (desc1.at<unsigned char>(q, p) == desc2.at<unsigned char>(q, p))
            {
                similarity++;
            }
        }
    }
    // std::cout<<similarity<<std::endl;
    return similarity / valid_num;
}

double SSC::getScore(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud2, double &angle, float &diff_x, float &diff_y)
{
    angle = 0;
    diff_x = 0;
    diff_y = 0;
    cv::Mat ssc_dis1 = project(cloud1);
    cv::Mat ssc_dis2 = project(cloud2);
    globalICP(ssc_dis1, ssc_dis2, angle, diff_x, diff_y);
    if (fabs(diff_x)>5 || fabs(diff_y) > 5)
    {
        diff_x = 0;
        diff_y = 0;
    }
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << diff_x, diff_y, 0;
    transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
    pcl::PointCloud<pcl::PointXYZL>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    transformPointCloud(*cloud2, *trans_cloud, transform);
    auto desc1 = calculateSSC(cloud1);
    auto desc2 = calculateSSC(trans_cloud);
    auto score = calculateSim(desc1, desc2);
    if (show)
    {
        transform.translation() << diff_x, diff_y, 0;
        transformPointCloud(*cloud2, *trans_cloud, transform);
        auto color_cloud1 = getColorCloud(cloud1);
        auto color_cloud2 = getColorCloud(trans_cloud);
        *color_cloud2 += *color_cloud1;
        viewer->showCloud(color_cloud2);
        auto color_image1 = getColorImage(desc1);
        cv::imshow("color image1", color_image1);
        auto color_image2 = getColorImage(desc2);
        cv::imshow("color image2", color_image2);
        cv::waitKey(0);
    }

    return score;
}

double SSC::getScore(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud2, Eigen::Matrix4f& transform)
{
    cv::Mat ssc_dis1 = project(cloud1);
    cv::Mat ssc_dis2 = project(cloud2);
    transform=globalICP(ssc_dis1, ssc_dis2);
    pcl::PointCloud<pcl::PointXYZL>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    transformPointCloud(*cloud2, *trans_cloud, transform);
    auto desc1 = calculateSSC(cloud1);
    auto desc2 = calculateSSC(trans_cloud);
    auto score = calculateSim(desc1, desc2);
    if (show)
    {
        transform(2,3)=0.;
        transformPointCloud(*cloud2, *trans_cloud, transform);
        auto color_cloud1 = getColorCloud(cloud1);
        auto color_cloud2 = getColorCloud(trans_cloud);
        *color_cloud2 += *color_cloud1;
        viewer->showCloud(color_cloud2);
        auto color_image1 = getColorImage(desc1);
        cv::imshow("color image1", color_image1);
        auto color_image2 = getColorImage(desc2);
        cv::imshow("color image2", color_image2);
        cv::waitKey(0);
    }
    return score;
}

double SSC::getScore(std::string cloud_file1, std::string cloud_file2, std::string label_file1, std::string label_file2, double &angle, float &diff_x, float &diff_y)
{
    auto cloudl1 = getLCloud(cloud_file1, label_file1);
    auto cloudl2 = getLCloud(cloud_file2, label_file2);
    auto score = getScore(cloudl1, cloudl2, angle, diff_x, diff_y);
    return score;
}

double SSC::getScore(std::string cloud_file1, std::string cloud_file2, std::string label_file1, std::string label_file2, Eigen::Matrix4f& transform)
{
    auto cloudl1 = getLCloud(cloud_file1, label_file1);
    auto cloudl2 = getLCloud(cloud_file2, label_file2);
    auto score = getScore(cloudl1, cloudl2, transform);
    return score;
}


double SSC::getScore(std::string cloud_file1, std::string cloud_file2, double &angle, float &diff_x, float &diff_y)
{
    auto cloudl1 = getLCloud(cloud_file1);
    auto cloudl2 = getLCloud(cloud_file2);
    auto score = getScore(cloudl1, cloudl2, angle, diff_x, diff_y);
    return score;
}

double SSC::getScore(std::string cloud_file1, std::string cloud_file2, Eigen::Matrix4f& transform)
{
    auto cloudl1 = getLCloud(cloud_file1);
    auto cloudl2 = getLCloud(cloud_file2);
    auto score = getScore(cloudl1, cloudl2, transform);
    return score;
}
