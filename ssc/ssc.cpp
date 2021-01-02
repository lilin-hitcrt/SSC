#include "ssc.h"
SSC::SSC(std::string conf_file)
{
    auto data_cfg = YAML::LoadFile(conf_file);
    if (data_cfg["name"].as<std::string>() == "rn")
    {
        use_sk = false;
    }
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
}

SSC::~SSC()
{
}
pcl::PointCloud<pcl::PointXYZI>::Ptr SSC::getCloud(std::string file)
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

void SSC::getLabel(std::string file, std::vector<uint32_t> &sem_labels, std::vector<uint32_t> &ins_labels)
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
        if (use_sk)
        {
            sem_label = label_map[(int)(label & 0x0000ffff)];
        }
        else
        {
            sem_label = label;
        }
        uint32_t ins_label = (label & 0xffff0000) >> 16;
        sem_labels[i] = sem_label;
        ins_labels[i] = ins_label;
        px += 1;
        // std::cout<<label<<" "<<sem_label<<" "<<ins_label<<std::endl;
    }
    fclose(stream);
    free(data);
}

pcl::PointCloud<pcl::PointXYZL>::Ptr SSC::getLCloud(std::string file_cloud, std::string file_label)
{
    struct timeval time_t;
    double time1, time2;
    pcl::PointCloud<pcl::PointXYZL>::Ptr re_cloud(new pcl::PointCloud<pcl::PointXYZL>());
    gettimeofday(&time_t, NULL);
    time1 = time_t.tv_sec * 1e3 + time_t.tv_usec * 1e-3;
    auto cloud = getCloud(file_cloud);
    gettimeofday(&time_t, NULL);
    time2 = time_t.tv_sec * 1e3 + time_t.tv_usec * 1e-3;
    if (time2 - time1 > 100)
    {
        std::cout << "get cloud:" << time2 - time1 << " ms" << std::endl;
    }
    // std::cout<<"cloud time:"<<time2-time1<<" "<<cloud->points.size()<<std::endl;
    std::vector<uint32_t> ins_labels, sem_labels;
    ins_labels.resize(cloud->points.size());
    sem_labels.resize(cloud->points.size());
    getLabel(file_label, sem_labels, ins_labels);
    gettimeofday(&time_t, NULL);
    time1 = time_t.tv_sec * 1e3 + time_t.tv_usec * 1e-3;
    // std::cout<<"label time:"<<time1-time2<<std::endl;
    // std::cout<<sem_labels.size()<<" "<<cloud->points.size()<<std::endl;
    re_cloud->points.resize(cloud->points.size());
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        if (sem_labels[i] == 0)
        {
            continue;
        }
        re_cloud->points[i].x = cloud->points[i].x;
        re_cloud->points[i].y = cloud->points[i].y;
        re_cloud->points[i].z = cloud->points[i].z;
        re_cloud->points[i].label = sem_labels[i];
    }
    re_cloud->height = 1;
    re_cloud->width = re_cloud->points.size();
    return re_cloud;
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
    return outcloud;
}

void SSC::calculate_ssc_range(const pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_pointcloud, cv::Mat3f &ssc_dis)
{
    int sectors = 360;
    auto sector_step = 2. * M_PI / sectors;
    ssc_dis = cv::Mat::zeros(cv::Size(sectors, 1), CV_32FC3);
    for (int i = 0; i < (int)filtered_pointcloud->points.size(); i++)
    {
        double distance = std::sqrt(filtered_pointcloud->points[i].x * filtered_pointcloud->points[i].x + filtered_pointcloud->points[i].y * filtered_pointcloud->points[i].y);
        double angle = M_PI + std::atan2(filtered_pointcloud->points[i].y, filtered_pointcloud->points[i].x);
        int sector_id = std::floor(angle / sector_step);
        if (sector_id >= sectors)
            continue;
        auto label = filtered_pointcloud->points[i].label;
        if (label == 13 || label == 14 || label == 16 || label == 18 || label == 19)
        {
            ssc_dis.at<cv::Vec3f>(0, sector_id)[0] = distance;
            ssc_dis.at<cv::Vec3f>(0, sector_id)[1] = filtered_pointcloud->points[i].x;
            ssc_dis.at<cv::Vec3f>(0, sector_id)[2] = filtered_pointcloud->points[i].y;
        }
    }
}

cv::Mat1b SSC::calculate_ssc(const pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_pointcloud)
{
    int sectors = 360, rings = 50;
    double max_dis = 50;
    auto ring_step = max_dis / rings;
    auto sector_step = 2. * M_PI / sectors;
    cv::Mat1b ssc = cv::Mat::zeros(cv::Size(sectors, rings), CV_8U);
    for (int i = 0; i < (int)filtered_pointcloud->points.size(); i++)
    {
        double distance = std::sqrt(filtered_pointcloud->points[i].x * filtered_pointcloud->points[i].x + filtered_pointcloud->points[i].y * filtered_pointcloud->points[i].y);
        if (distance >= max_dis)
            continue;
        double angle = M_PI + std::atan2(filtered_pointcloud->points[i].y, filtered_pointcloud->points[i].x);
        int ring_id = std::floor(distance / ring_step);
        int sector_id = std::floor(angle / sector_step);
        if (ring_id >= rings)
            continue;
        if (sector_id >= sectors)
            continue;
        auto label = filtered_pointcloud->points[i].label;

        if (order_vec[label] > order_vec[ssc.at<unsigned char>(ring_id, sector_id)])
        {
            ssc.at<unsigned char>(ring_id, sector_id) = label;
        }
    }
    return ssc;
}

cv::Mat3b SSC::getColorImage(cv::Mat1b &desc)
{
    cv::Mat3b out = cv::Mat::zeros(desc.size(), CV_8UC3);
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

void SSC::calculate_trans(cv::Mat3f &ssc_dis1, cv::Mat3f &ssc_dis2, double &angle, float &diff_x, float &diff_y)
{
    double similarity = 100000;
    int sectors = ssc_dis1.cols;
    for (int i = 0; i < sectors; ++i)
    {
        float dis_count = 0;
        for (int j = 0; j < sectors; ++j)
        {
            int new_col = j + i >= sectors ? j + i - sectors : j + i;
            cv::Vec3f vec1 = ssc_dis1.at<cv::Vec3f>(0, j);
            cv::Vec3f vec2 = ssc_dis2.at<cv::Vec3f>(0, new_col);
            dis_count += fabs(vec1[0] - vec2[0]);
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
        temp_dis2.at<cv::Vec3f>(0, i)[1] = ssc_dis2.at<cv::Vec3f>(0, i)[1] * cs - ssc_dis2.at<cv::Vec3f>(0, i)[2] * sn;
        temp_dis2.at<cv::Vec3f>(0, i)[2] = ssc_dis2.at<cv::Vec3f>(0, i)[1] * sn + ssc_dis2.at<cv::Vec3f>(0, i)[2] * cs;
    }
    for (int i = 0; i < 100; ++i)
    {
        float dx = 0, dy = 0;
        int diff_count = 1;
        for (int j = 0; j < sectors; ++j)
        {
            cv::Vec3f vec1 = temp_dis1.at<cv::Vec3f>(0, j);
            if (vec1[0] <= 0)
            {
                continue;
            }
            int min_id = -1;
            float min_dis = 1000000.;
            for (int k = j + angle_o - 10; k < j + angle_o + 10; ++k)
            {
                cv::Vec3f vec_temp;
                int temp_id = k;
                if (k < 0)
                {
                    temp_id = k + sectors;
                }
                else if (k >= sectors)
                {
                    temp_id = k - sectors;
                }
                vec_temp = temp_dis2.at<cv::Vec3f>(0, temp_id);
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
            cv::Vec3f vec2 = temp_dis2.at<cv::Vec3f>(0, min_id);
            // std::cout<<fabs(vec1[1]-vec2[1])<<" "<<fabs(vec1[2]-vec2[2])<<std::endl;
            if (fabs(vec1[1] - vec2[1]) < 3 && fabs(vec1[2] - vec2[2]) < 3)
            {
                dx += vec1[1] - vec2[1];
                dy += vec1[2] - vec2[2];
                diff_count++;
            }
        }
        dx = 1. * dx / diff_count;
        dy = 1. * dy / diff_count;
#if SHOW
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#endif
        for (int j = 0; j < sectors; ++j)
        {
            if (temp_dis2.at<cv::Vec3f>(0, j)[0] != 0)
            {
                temp_dis2.at<cv::Vec3f>(0, j)[1] += dx;
                temp_dis2.at<cv::Vec3f>(0, j)[2] += dy;
#if SHOW
                pcl::PointXYZRGB p;
                p.x = temp_dis2.at<cv::Vec3f>(0, j)[1];
                p.y = temp_dis2.at<cv::Vec3f>(0, j)[2];
                p.z = 0;
                p.r = 255;
                temp_cloud->points.emplace_back(p);
#endif
            }
#if SHOW
            if (temp_dis1.at<cv::Vec3f>(0, j)[0] != 0)
            {
                pcl::PointXYZRGB p;
                p.x = temp_dis1.at<cv::Vec3f>(0, j)[1];
                p.y = temp_dis1.at<cv::Vec3f>(0, j)[2];
                p.z = 0;
                p.b = 255;
                temp_cloud->points.emplace_back(p);
            }
#endif
        }
#if SHOW
        viewer.showCloud(temp_cloud);
        usleep(100000);
#endif
        diff_x += dx;
        diff_y += dy;
#if SHOW
        std::cout << i << " diff " << diff_x << " " << diff_y << " " << dx << " " << dy << std::endl;
#endif
        if (fabs(dx) < 1e-5 && fabs(dy) < 1e-5)
        {
            break;
        }
    }
}

double SSC::calculate_dis(cv::Mat1b &desc1, cv::Mat1b &desc2)
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
    cv::Mat3f ssc_dis1, ssc_dis2;
    calculate_ssc_range(cloud1, ssc_dis1);
    calculate_ssc_range(cloud2, ssc_dis2);
    calculate_trans(ssc_dis1, ssc_dis2, angle, diff_x, diff_y);
    if (fabs(diff_x > 5) || fabs(diff_y) > 5)
    {
        diff_x = 0;
        diff_y = 0;
    }
    pcl::PointCloud<pcl::PointXYZL>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << diff_x, diff_y, 0;
    transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud2, *trans_cloud, transform);
    auto desc1 = calculate_ssc(cloud1);
    auto desc2 = calculate_ssc(trans_cloud);
    auto score = calculate_dis(desc1, desc2);

#if SHOW
    auto color_cloud1 = getColorCloud(cloud1);
    auto color_cloud2 = getColorCloud(trans_cloud);
    *color_cloud2 += *color_cloud1;
    viewer.showCloud(color_cloud2);
    auto color_image1 = getColorImage(desc1);
    cv::imshow("color image1", color_image1);
    auto color_image2 = getColorImage(desc2);
    cv::imshow("color image2", color_image2);
    cv::waitKey(0);
#endif
    return score;
}

double SSC::getScore(std::string cloud_file1, std::string cloud_file2, std::string label_file1, std::string label_file2, double &angle, float &diff_x, float &diff_y)
{
    angle=0;
    diff_x=0;
    diff_y=0;
    auto cloudl1 = getLCloud(cloud_file1, label_file1);
    auto cloudl2 = getLCloud(cloud_file2, label_file2);
    auto score = getScore(cloudl1, cloudl2, angle,diff_x,diff_y);
    return score;
}