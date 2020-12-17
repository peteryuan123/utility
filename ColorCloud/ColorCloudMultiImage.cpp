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
#include<omp.h>
#include<boost/filesystem.hpp>
namespace fs = boost::filesystem;


struct PointXYZITN
{
    PCL_ADD_POINT4D;
    double ts;
    PCL_ADD_NORMAL4D;
    float i;
    bool visited = false;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN32;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZITN,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (double, ts, ts)
                                    (float, normal_x, nx)
                                    (float, normal_y, ny)
                                    (float, normal_z, nz)
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

void ColorImageCloud(cv::Mat &image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     Eigen::Matrix4d transform, cv::Mat &intrinsic, cv::Mat &distortion, cv::Mat &mask)
{
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
        if (transformed_cloud->points[p_index].z < 0 || transformed_cloud->points[p_index].y > 5 ||
            transformed_cloud->points[p_index].x > 5 || transformed_cloud->points[p_index].z > 5)
        {
            continue;
        }

        // Coloring
        
        if (uv[0]>0 && uv[0] < image.cols && uv[1] > 0 && uv[1] < image.rows)
        {
            //std::cout << (mask.at<uint8_t>(uv[1],uv[0]) == 255) << std::endl;
            if (point.b == 0 && point.g == 0 && point.r == 0 )
            {  
                point.b = image.at<cv::Vec3b>(uv[1],uv[0])[0];
                point.g = image.at<cv::Vec3b>(uv[1],uv[0])[1];
                point.r = image.at<cv::Vec3b>(uv[1], uv[0])[2];
            }   
        }

    }
}

void ColorMultiImageCloud(std::vector<std::string> Left_path, std::vector<std::string> Right_path, pcl::PointCloud<PointXYZITN>::Ptr cloud, 
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud, std::string geo_pose_file, std::vector<Eigen::Matrix4d> geo_to_cam, 
                          std::vector<cv::Mat> intrinsic, std::vector<cv::Mat> distortion, std::vector<cv::Mat> masks, double offset)
{
    int start_index = 19;
    double start_time = 0.666;
    int count = 0;
    int frame_interval = 30;
    int time_interval = 1;

    // Get the min and max time in cloud
    double cloud_t_min, cloud_t_max;
    getMinMaxTime(cloud, cloud_t_min, cloud_t_max);
    cloud_t_min = 0.666;
    //cloud_t_max = 60;

    // Open lidar pose file
    std::ifstream src;
    std::string header;
    src.open(geo_pose_file);
    for (int i = 0; i < start_index; i++)
    {
        getline(src, header);
    }
    
    // Get the base time of lidar pose
    double t_x,t_y,t_z;
    double r_x,r_y,r_z,r_w;
    double ref_time,pose_time;
    src >> ref_time >> t_x >> t_y >> t_z >> r_w >> r_x >> r_y >> r_z;
    //pose_next_time = pose_time + offset/fps + start_time;

   
    while ((cloud_t_min < cloud_t_max) && (src >> pose_time >> t_x >> t_y >> t_z >> r_w >> r_x >> r_y >> r_z) )
    {
        
        //Getlidar pose
        Eigen::Vector3d poseT(t_x,t_y,t_z);
        Eigen::Quaterniond poseR(r_w, r_x,r_y,r_z);
        Eigen::Isometry3d geo_to_world(poseR);
        geo_to_world.pretranslate(poseT);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = FilterTimeInterval(cloud, cloud_t_min, cloud_t_min + 1);

        Eigen::Matrix4d Ltransform = geo_to_cam[0] * geo_to_world.matrix().inverse();
        Eigen::Matrix4d Rtransform = geo_to_cam[1] * geo_to_world.matrix().inverse();
        cv::Mat left_im = cv::imread(Left_path[start_index]);
        cv::Mat right_im = cv::imread(Right_path[start_index]);
        

        cv::flip(left_im.t(), left_im, 0);
        cv::flip(right_im.t(), right_im, 1);
        ColorImageCloud(left_im, filtered_cloud, Ltransform, intrinsic[0], distortion[0], masks[0]);
        ColorImageCloud(right_im, filtered_cloud, Rtransform, intrinsic[1], distortion[1], masks[1]);
        
        //pcl::io::savePLYFile("/home/peteryuan/color/color_cloud/data/huajian/1/" + std::to_string(count) + ".ply", *filtered_cloud, 0);
        //cv::imwrite("/home/peteryuan/color/color_cloud/data/huajian/l/" + std::to_string(count) + ".jpg", left_im);
        //cv::imwrite("/home/peteryuan/color/color_cloud/data/huajian/r/" + std::to_string(count) + ".jpg", right_im);
        count++;

        for (int i = 0; i < frame_interval; i++)
        {
            getline(src, header);
        }
        //Indicate the next pose

        //Coloring

        *(colored_cloud) += *(filtered_cloud);
        std::cout << "-----------------\n" ;
        std::cout << Left_path[start_index] << std::endl;
        std::cout << Right_path[start_index] << std::endl;
        std::cout << Ltransform << std::endl;
        std::cout << Rtransform << std::endl;
        std::cout << "-----------------\n" ;
        cloud_t_min += time_interval;
        start_index += frame_interval;
        if (start_index > 3340)
        {
            break;
        }
        std::cout << cloud_t_min << std::endl;
    }


    // Remove black points
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range(new pcl::ConditionAnd<pcl::PointXYZRGB> ());

    range->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GE, 1)));
    range->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GE, 1)));
    range->addComparison(pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr(
            new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GE, 1)));

    pcl::ConditionalRemoval<pcl::PointXYZRGB> condream;
    condream.setCondition(range);
    condream.setInputCloud(colored_cloud);
    condream.setKeepOrganized(true);
    condream.filter(*colored_cloud);

    std::cout << "Coloring finish !" << std::endl;
    src.close();
}




int main(int argc, char *argv[]){

    // Load ply file
    std::cout << "Loading ply..." << std::endl;
    pcl::PointCloud<PointXYZITN>::Ptr cloud(new pcl::PointCloud<PointXYZITN>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile("/home/peteryuan/color/color_cloud/data/huajian/2020-11-30_15-21-30_100pct_normals_scan.ply", *cloud);

    // Get lidar pose file
    std::string pose_file = "/home/peteryuan/color/color_cloud/data/huajian/pose_for_imu_sync.txt";

    // Load transform matrix
    Eigen::Matrix4d transform_left;
    Eigen::Matrix4d transform_right;
    std::vector<Eigen::Matrix4d> transforms;
    read_transform("/home/peteryuan/color/color_cloud/data/F4/Tgeo2leftcam.txt", transform_left);
    read_transform("/home/peteryuan/color/color_cloud/data/F4/Tgeo2rightcam.txt", transform_right);
    transforms.push_back(transform_left);
    transforms.push_back(transform_right);

    // Load mask
    std::vector<cv::Mat> masks;
    cv::Mat mask_left;
    cv::Mat mask_right;
    mask_left = cv::imread("../data/colorizelibrary3f/useful/left_mask.png");
    mask_right = cv::imread("../data/colorizelibrary3f/useful/right_mask.png");
    cv::flip(mask_left.t(), mask_left, 0);
    cv::flip(mask_right.t(), mask_right, 1);
    masks.push_back(mask_left);
    masks.push_back(mask_right);
    cv::imwrite("left.png", mask_left);
    cv::imwrite("right.png", mask_right);

    // Load videos
    std::vector<std::string> Left_image_path;
    std::vector<std::string> Right_image_path;
    fs::path left_dir("/home/peteryuan/color/color_cloud/data/huajian/left_full");
    fs::path right_dir("/home/peteryuan/color/color_cloud/data/huajian/right_full");
    for(fs::directory_entry& x: fs::directory_iterator(left_dir))
        Left_image_path.push_back(x.path().string());
    for(fs::directory_entry& x: fs::directory_iterator(right_dir))
        Right_image_path.push_back(x.path().string());
    std::sort(Left_image_path.begin(),Left_image_path.end());
    std::sort(Right_image_path.begin(),Right_image_path.end());

    // Get Intrinsic and distortion
    Eigen::Matrix3d intrinsic_left;
    Eigen::Vector4d distortion_left;
    intrinsic_left << 8.6120225460954066e+02, 0., 1.4375127005478748e+03, 0.,
                      8.6065035115142234e+02, 1.4418096527828075e+03, 0., 0., 1.;
    distortion_left << 3.4024548043502460e-02, 1.1038130872236002e-02, -1.2664634538608322e-02, 1.5672309306780908e-03;
    
    // Transform eigen matrix into opencv matrix
    cv::Mat intrinsic_l_cv, distortion_l_cv;
    cv::eigen2cv(intrinsic_left, intrinsic_l_cv);
    cv::eigen2cv(distortion_left, distortion_l_cv);

    Eigen::Matrix3d intrinsic_right;
    Eigen::Vector4d distortion_right;
    intrinsic_right << 8.6264757161231410e+02, 0., 1.4419426569209147e+03, 0.,
                        8.6179474700008700e+02, 1.4373417306128667e+03, 0., 0., 1. ;
    distortion_right << 3.5270559653026744e-02, 8.7541438756161601e-03, -1.1365295066544340e-02, 1.3060458954774687e-03;
    
    // Transform eigen matrix into opencv matrix
    cv::Mat intrinsic_r_cv, distortion_r_cv;
    cv::eigen2cv(intrinsic_right, intrinsic_r_cv);
    cv::eigen2cv(distortion_right, distortion_r_cv);

    std::vector<cv::Mat> intrin;
    std::vector<cv::Mat> distor;
    intrin.push_back(intrinsic_l_cv);
    intrin.push_back(intrinsic_r_cv);
    distor.push_back(distortion_l_cv);
    distor.push_back(distortion_r_cv);


    // Rendering cloud
    std::cout << "Rendering..." << std::endl;
    ColorMultiImageCloud(Left_image_path, Right_image_path, cloud, colored_cloud, pose_file, transforms, intrin, distor, masks, 0);

    // Saving colored cloud
    std::cout << "Saving result..." << std::endl;
    pcl::io::savePLYFile("multiimage.ply", *colored_cloud, 0);

    return 0;

}


