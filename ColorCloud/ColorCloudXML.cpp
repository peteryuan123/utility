#include<pcl-1.8/pcl/point_cloud.h>
#include<pcl-1.8/pcl/io/ply_io.h>
#include<pcl-1.8/pcl/PCLPointField.h>
#include<pcl-1.8/pcl/point_types.h>
#include<pcl-1.8/pcl/pcl_macros.h>
#include<pcl/common/transforms.h>
#include<pcl/filters/conditional_removal.h>

#include<opencv2/calib3d.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc_c.h>
#include<opencv2/core/eigen.hpp>

#include<Eigen/Core>
#include<Eigen/Geometry>

#include<string.h>
#include<iostream>
#include<vector>
#include<tinyxml2.h>
#include<omp.h>

struct PointXYZITN
{
    PCL_ADD_POINT4D;
    float ts;
    float i;
    bool visited = false;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN32;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZITN,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, ts, scalar_ts)
                                    (float, i, intensity)
)


void getMinMaxTime(pcl::PointCloud<PointXYZITN>::Ptr cloud, double& min, double& max) {
    min = std::numeric_limits<double>::max();
    max = std::numeric_limits<double>::min();
    for (auto& point: cloud->points) {
        if (point.ts < min) {
            min = point.ts;
        }
        if (point.ts > max) {
            max = point.ts;
        }
    }
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr FilterTimeInterval(pcl::PointCloud<PointXYZITN>::Ptr cloud, double left, double right) { 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
    // todo:  start id as param
    for (size_t start_id = 0; start_id < cloud->points.size(); ++start_id) 
    {
        auto& point = cloud->points[start_id];
        if (point.ts >= left) 
        {
            if (point.ts < right) 
            {
                pcl::PointXYZRGB out_point ;
                out_point.x = point.x;
                out_point.y = point.y;
                out_point.z = point.z;
                out->push_back(out_point);
            }
            else 
            {
                break;
            }
        }
    }
    return out;
}

void read_transform(std::string file, Eigen::Matrix4d &transform)
{
    std::ifstream src;
    src.open(file);
    
    for (int i = 0; i < 4; i++)
    {
        src >> transform(i,0);
        src >> transform(i,1);
        src >> transform(i,2);
        src >> transform(i,3);
    }
    src.close();
}


int ColorImageCloud(cv::Mat &image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     Eigen::Matrix4d transform, cv::Mat &intrinsic, cv::Mat &distortion, cv::Mat &mask)
{
    int count = 0;
    // Transform rotation matrix to rotation vector because of the requirement of projectPoints()
    cv::Mat rmat = cv::Mat::eye(3,3,CV_32F);
    cv::Mat tvec = cv::Mat::zeros(1,3,CV_32F);
    cv::Mat rvec;
    cv::Rodrigues(rmat, rvec);

    // Transform cloud into camera coordinates
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // Gets all the 3D points and project them on 2D plane
    cv::Mat pts(cloud->points.size(), 1, CV_32FC3);
    //#pragma omp parallel for
    for (size_t p_index = 0; p_index < transformed_cloud->points.size(); p_index++)
    {
        pts.at<cv::Vec3f>(p_index, 0)[0] = transformed_cloud->points[p_index].x;
        pts.at<cv::Vec3f>(p_index, 0)[1] = transformed_cloud->points[p_index].y;
        pts.at<cv::Vec3f>(p_index, 0)[2] = transformed_cloud->points[p_index].z;
    }

    // Project points on uv
    cv::Mat uvs(cloud->points.size(), 1, CV_32FC2);
    cv::fisheye::projectPoints(pts, uvs, rvec, tvec, intrinsic, distortion);

    // Coloring
    //#pragma omp parallel for
    for (size_t p_index = 0; p_index < transformed_cloud->points.size(); p_index++)
    {
        auto &point = cloud->points[p_index];
        cv::Vec2f uv = uvs.at<cv::Vec2f>(p_index, 0);
        // std::cout << transform << std::endl;
        // std::cout << cloud->points[p_index].x <<  " " << cloud->points[p_index].y << " " << cloud->points[p_index].z << std::endl;
        // std::cout << transformed_cloud->points[p_index].x <<  " " << transformed_cloud->points[p_index].y << " " << transformed_cloud->points[p_index].z << std::endl;
        // std::cout << uv[0] <<  " " << uv[1] << std::endl;
        // Points that we don't need to color
        if (transformed_cloud->points[p_index].z < 0 /*|| transformed_cloud->points[p_index].y > 5 ||
            transformed_cloud->points[p_index].x > 5 || transformed_cloud->points[p_index].z > 5*/)
        {
            continue;
        }

        // Coloring
        
        if (uv[0]>0 && uv[0] < image.cols && uv[1] > 0 && uv[1] < image.rows)
        {
            //std::cout << (mask.at<uint8_t>(uv[1],uv[0]) == 255) << std::endl;
            if (point.b == 0 && point.g == 0 && point.r == 0 /*&& mask.at<uint8_t>(uv[1],uv[0]) != 0*/)
            {  
                point.b = image.at<cv::Vec3b>(uv[1],uv[0])[0];
                point.g = image.at<cv::Vec3b>(uv[1],uv[0])[1];
                point.r = image.at<cv::Vec3b>(uv[1], uv[0])[2];
                count++;
            }   
        }
    }
    return count;
}

Eigen::Matrix4d getCameraPose(tinyxml2::XMLElement *pose_node)
{
    Eigen::Matrix4d pose = Eigen::Matrix4d::Zero();

    tinyxml2::XMLElement *rotation_node = pose_node->FirstChildElement("Rotation");
    tinyxml2::XMLElement *translation_node = pose_node->FirstChildElement("Center");
    std::string key = "M_";
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            std::string index = key + std::to_string(i) + std::to_string(j);
            pose(j,i) = rotation_node->FirstChildElement(index.c_str())->DoubleText();
        }
    }
    pose(0,3) = translation_node->FirstChildElement("x")->DoubleText();
    pose(1,3) = translation_node->FirstChildElement("y")->DoubleText();
    pose(2,3) = translation_node->FirstChildElement("z")->DoubleText();
    pose(3,3) = 1.0;
    //std::cout << pose << std::endl;
    return pose.inverse();
}


void ColorVideoCloud(pcl::PointCloud<PointXYZITN>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud,
                        tinyxml2::XMLDocument& doc, std::vector<cv::Mat> intrinsic, std::vector<cv::Mat> distortion, 
                        std::vector<cv::Mat> masks)
{
    double fps = 29.97;

    tinyxml2::XMLElement *root = doc.RootElement();
    tinyxml2::XMLElement *photogroups = root->FirstChildElement("Block")->FirstChildElement("Photogroups");
    tinyxml2::XMLElement *group_left = photogroups->FirstChildElement();
    tinyxml2::XMLElement *group_right = group_left->NextSiblingElement();
    tinyxml2::XMLElement *n_left_photo = group_left->FirstChildElement("Photo");
    tinyxml2::XMLElement *n_right_photo = group_right->FirstChildElement("Photo");
    double cur_time = 0;
    do
    {
        std::string Limage_path = n_left_photo->FirstChildElement("ImagePath")->GetText();
        std::string Rimage_path = n_right_photo->FirstChildElement("ImagePath")->GetText();

        Eigen::Matrix4d Ltransform = getCameraPose(n_left_photo->FirstChildElement("Pose"));
        Eigen::Matrix4d Rtransform = getCameraPose(n_right_photo->FirstChildElement("Pose"));

        cv::Mat Limage = cv::imread(Limage_path);
        cv::Mat Rimage = cv::imread(Rimage_path);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = FilterTimeInterval(cloud, cur_time, cur_time + 0.5);

        int left_c = ColorImageCloud(Limage, filtered_cloud, Ltransform, intrinsic[0], distortion[0], masks[0]);
        int right_c = ColorImageCloud(Rimage, filtered_cloud, Rtransform, intrinsic[1], distortion[1], masks[1]);
        
        *(colored_cloud) += *(filtered_cloud);

        cur_time += 0.5;
        n_left_photo = n_left_photo->NextSiblingElement();
        n_right_photo = n_right_photo->NextSiblingElement(); 
        if (n_left_photo == NULL || n_right_photo == NULL)
        {
            break;
        }

        std::cout << "--------------\n";
        //std::cout << Ltransform << std::endl;
        std::cout << filtered_cloud->points.size() << std::endl;
        std::cout << Limage_path << std::endl;
        std::cout << Rimage_path << std::endl;
        std::cout << cur_time << std::endl;
        std::cout << "left_count: " << left_c << std::endl;
        std::cout << "right_count: " << right_c << std::endl;


    }while(cur_time < 490);

    // Remove black points
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range(new pcl::ConditionAnd<pcl::PointXYZRGB> ());

    // range->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
    //         new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GE, 1)));
    // range->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
    //         new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GE, 1)));
    // range->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
    //         new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GE, 1)));

    // pcl::ConditionalRemoval<pcl::PointXYZRGB> condream;
    // condream.setCondition(range);
    // condream.setInputCloud(colored_cloud);
    // condream.setKeepOrganized(true);
    // condream.filter(*colored_cloud);

    std::cout << "Coloring finish !" << std::endl;
}


int main(int argc, char *argv[]){
    if (argc != 3)
    {
        std::cout << "Usage ./colorXml cloud_path xml_path\n";
        return 0;
    }
    // Load ply file
    std::cout << "Loading ply..." << std::endl;
    pcl::PointCloud<PointXYZITN>::Ptr cloud(new pcl::PointCloud<PointXYZITN>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile(argv[1], *cloud);

    // Get lidar pose file
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError ret = doc.LoadFile(argv[2]);
    
    /*********** Load mask ***********/
    std::vector<cv::Mat> masks;
    cv::Mat mask_left;
    cv::Mat mask_right;
    mask_left = cv::imread("/home/peteryuan/Downloads/F2/left_mask.png");
    mask_right = cv::imread("/home/peteryuan/Downloads/F2/right_mask.png");
    masks.push_back(mask_left);
    masks.push_back(mask_right);
    // cv::imwrite("left.png", mask_left);
    // cv::imwrite("right.png", mask_right);
    /*********** Load mask ***********/

    /*********** Load video ***********/
    cv::VideoCapture video_left("/home/peteryuan/color/color_cloud/data/F4/VID_20200924_101927_10_002.insv.mp4");
    cv::VideoCapture video_right("/home/peteryuan/color/color_cloud/data/F4/VID_20200924_101927_00_002.insv.mp4");
    std::vector<cv::VideoCapture> videos;
    videos.push_back(video_left);
    videos.push_back(video_right);
    /*********** Load video ***********/

    /*********** Get Intrinsic and distortion ***********/

    // For left
    Eigen::Matrix3d intrinsic_left;
    Eigen::Vector4d distortion_left;
    intrinsic_left << 8.6120225460954066e+02, 0., 1.4375127005478748e+03, 0.,
                      8.6065035115142234e+02, 1.4418096527828075e+03, 0., 0., 1.;
    distortion_left << 3.4024548043502460e-02, 1.1038130872236002e-02, -1.2664634538608322e-02, 1.5672309306780908e-03;
    
    cv::Mat intrinsic_l_cv, distortion_l_cv;
    cv::eigen2cv(intrinsic_left, intrinsic_l_cv);
    cv::eigen2cv(distortion_left, distortion_l_cv);

    // For right
    Eigen::Matrix3d intrinsic_right;
    Eigen::Vector4d distortion_right;
    intrinsic_right << 8.6264757161231410e+02, 0., 1.4419426569209147e+03, 0.,
                        8.6179474700008700e+02, 1.4373417306128667e+03, 0., 0., 1. ;
    distortion_right << 3.5270559653026744e-02, 8.7541438756161601e-03, -1.1365295066544340e-02, 1.3060458954774687e-03;
    
    cv::Mat intrinsic_r_cv, distortion_r_cv;
    cv::eigen2cv(intrinsic_right, intrinsic_r_cv);
    cv::eigen2cv(distortion_right, distortion_r_cv);

    std::vector<cv::Mat> intrin;
    std::vector<cv::Mat> distor;
    intrin.push_back(intrinsic_l_cv);
    intrin.push_back(intrinsic_r_cv);
    distor.push_back(distortion_l_cv);
    distor.push_back(distortion_r_cv);
    /*********** Get Intrinsic and distortion ***********/


    // Rendering cloud
    std::cout << "Rendering..." << std::endl;
    //ColorVideoCloud(videos, cloud, colored_cloud, pose_file, transforms, intrin, distor, masks, 0);
    ColorVideoCloud(cloud, colored_cloud, doc, intrin, distor, masks);
    // Saving colored cloud
    std::cout << "Saving result..." << std::endl;
    pcl::io::savePLYFile("color_xml.ply", *colored_cloud, 1);

    return 0;

}


