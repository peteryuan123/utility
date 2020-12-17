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

struct PointXYZITNRGB
{
    PCL_ADD_POINT4D;
    double ts;
    PCL_ADD_NORMAL4D;
    float i;
    PCL_ADD_RGB;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN32;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZITNRGB,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (double, ts, ts)
                                    (float, normal_x, nx)
                                    (float, normal_y, ny)
                                    (float, normal_z, nz)
                                    (float, i, intensity)
                                    (uchar, r, r)
                                    (uchar, g, g)
                                    (uchar, b, b)

)

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

struct PointXYZRGBD
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    double d;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN32;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBD,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (uchar, r, r)
                                    (uchar, g, g)
                                    (uchar, b, b)
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
            std::cout << (mask.at<uint8_t>(uv[1],uv[0]) == 255) << std::endl;
            if (point.b == 0 && point.g == 0 && point.r == 0 && mask.at<uint8_t>(uv[1],uv[0]) != 0)
            {  
                point.b = image.at<cv::Vec3b>(uv[1],uv[0])[0];
                point.g = image.at<cv::Vec3b>(uv[1],uv[0])[1];
                point.r = image.at<cv::Vec3b>(uv[1], uv[0])[2];
            }   
        }

    }
}

void ColorVideoCloud(std::vector<cv::VideoCapture> videos, pcl::PointCloud<PointXYZITN>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud,
                        std::string geo_pose_file, std::vector<Eigen::Matrix4d> geo_to_cam, std::vector<cv::Mat> intrinsic, std::vector<cv::Mat> distortion, std::vector<cv::Mat> masks,
                        double offset)
{
    int frame = 0;

    double start_time = 13.0;
    double fps = videos[0].get(CV_CAP_PROP_FPS);

    std::cout << "fps:" << fps << std::endl;

    // Get the min and max time in cloud
    double cloud_t_min, cloud_t_max;
    getMinMaxTime(cloud, cloud_t_min, cloud_t_max);
    cloud_t_min = start_time;
    //cloud_t_max = 60;
    cloud_t_min += offset/fps;

    // Open lidar pose file
    std::ifstream src;
    std::string header;
    src.open(geo_pose_file);
    getline(src, header);
    
    // Get the base time of lidar pose
    double pose_t_offset = 13.0;
    double t_x,t_y,t_z;
    double r_x,r_y,r_z,r_w;
    double ref_time,pose_time;
    src >> ref_time >> t_x >> t_y >> t_z >> r_w >> r_x >> r_y >> r_z; 
    //pose_next_time = pose_time + offset/fps + start_time;

    double start_frame_pos = 200.0 + start_time * 1000.0;

    while (cloud_t_min < cloud_t_max)
    {
        //Getlidar pose
        do
        {
            src >> pose_time >> t_x >> t_y >> t_z >> r_w >> r_x >> r_y >> r_z; 
            if (src.peek() == EOF)
            {
                src.close();
                std::cout << "pose file end" << std::endl;
                return;
            }
        }while((pose_time - ref_time) < pose_t_offset && src.peek()!=EOF);
        
        printf("%f\n", pose_time);
        Eigen::Vector3d poseT(t_x,t_y,t_z);
        Eigen::Quaterniond poseR(r_w,r_x,r_y,r_z);
        Eigen::Isometry3d geo_to_world(poseR);
        geo_to_world.pretranslate(poseT);

        //Indicate the next pose
        pose_t_offset += 0.1665;

        //Coloring
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud = FilterTimeInterval(cloud, cloud_t_min, cloud_t_min + 0.1665);
        for (size_t v_index = 0; v_index < videos.size(); v_index++)
        {
            cv::Mat image;
            videos[v_index].set(cv::CAP_PROP_POS_MSEC, start_frame_pos);
            videos[v_index] >> image;

            if (image.empty()) 
            {
                std::cout << "Image empty! return!" << std::endl;
                src.close();
                return;
            }
    
            cv::flip(image.t(), image, (v_index)%2);
            Eigen::Matrix4d transform = geo_to_cam[v_index] * geo_to_world.matrix().inverse();
            // std::cout << geo_to_cam[v_index] << std::endl;
            // std::cout << geo_to_world.matrix() << std::endl;
            // std::cout << geo_to_world.matrix().inverse() << std::endl;
            // std::cout << "------------------\n";
            ColorImageCloud(image, filtered_cloud, transform, intrinsic[v_index], distortion[v_index], masks[v_index]);
        
            
            // if (v_index == 0)
            // {
            //     cv::imwrite("/home/peteryuan/color/color_cloud/result/left/frame" + std::to_string(frame) + ".png", image);
            //     pcl::io::savePLYFile("/home/peteryuan/color/color_cloud/result/cloud_l/frame" + std::to_string(frame) + ".ply", *filtered_cloud, 0);
            //     // pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            //     // pcl::transformPointCloud(*filtered_cloud, *transformed_cloud, transform);
            //     // pcl::io::savePLYFile("/home/peteryuan/color/color_cloud/result/cloud_l/trans" + std::to_string(frame) + ".ply", *transformed_cloud, 0);

            // }   
            // else
            // {                
            //     cv::imwrite("/home/peteryuan/color/color_cloud/result/right/frame" + std::to_string(frame) + ".png", image);
            //     pcl::io::savePLYFile("/home/peteryuan/color/color_cloud/result/cloud_r/frame" + std::to_string(frame) + ".ply", *filtered_cloud, 0);
            //     // pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            //     // pcl::transformPointCloud(*filtered_cloud, *transformed_cloud, transform);
            //     // pcl::io::savePLYFile("/home/peteryuan/color/color_cloud/result/cloud_r/trans" + std::to_string(frame) + ".ply", *transformed_cloud, 0);

            // }
 
        }

        *(colored_cloud) += *(filtered_cloud);
        start_frame_pos += 166.5;
        frame += 1;
        cloud_t_min += 0.1665;

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


//--------------------------------------------------------------------------------------------------


// void CapTwoFrame(const std::string& geo_video, const std::string& insta_video, int geo_start_num) {
//     int insta_start_num = 0;
//     if (geo_start_num < 0) {
//         insta_start_num = -geo_start_num;
//         geo_start_num = 0;
//     }
//     cv::VideoCapture cap_geo(geo_video);
//     cv::VideoCapture cap_insta(insta_video);
//     if (!cap_geo.isOpened() || !cap_insta.isOpened()) {
//         return;
//     }
//     int nFrame_geo = cap_geo.get(cv::CAP_PROP_FRAME_COUNT);
//     double rate_geo = cap_geo.get(CV_CAP_PROP_FPS);
//     // UINFO("geo nFrame: %d    rate: %f", nFrame_geo, rate_geo);
//     int nFrame_insta = cap_insta.get(cv::CAP_PROP_FRAME_COUNT);
//     double rate_insta = cap_insta.get(CV_CAP_PROP_FPS);
//     // UINFO("insta nFrame: %d    rate: %f", nFrame_insta, rate_insta);
//     cv::Mat frame_geo, frame_insta;
//     // set start frame
//     cap_geo.set(cv::CAP_PROP_POS_FRAMES, geo_start_num);
//     cap_insta.set(cv::CAP_PROP_POS_FRAMES, insta_start_num);
//     // show two frame
//     while (true) {
//         cap_geo >> frame_geo;
//         cap_insta >> frame_insta;
//         cv::flip(frame_insta.t(), frame_insta, 1);
//         cv::Mat resized_frame_geo, resized_frame_insta;
//         cv::resize(frame_geo, resized_frame_geo, cv::Size((float)frame_geo.cols/frame_geo.rows*500, 500));
//         cv::resize(frame_insta, resized_frame_insta, cv::Size(500, 500));
//         cv::Mat des;
//         des.create(resized_frame_geo.rows, resized_frame_geo.cols+resized_frame_insta.cols, resized_frame_geo.type());
//         cv::Mat r1 = des(cv::Rect(0, 0, resized_frame_geo.cols, resized_frame_geo.rows));
//         resized_frame_geo.copyTo(r1);
//         cv::Mat r2 = des(cv::Rect(resized_frame_geo.cols, 0, resized_frame_insta.cols, resized_frame_insta.rows));
//         resized_frame_insta.copyTo(r2);
//         cv::imshow("", des);
//         cv::waitKey(10);
//     }
// }
// // //--------------------------------------------------------------------------------------------------


int main(int argc, char *argv[]){

    // Load ply file
    std::cout << "Loading ply..." << std::endl;
    pcl::PointCloud<PointXYZITN>::Ptr cloud(new pcl::PointCloud<PointXYZITN>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile("/home/peteryuan/color/color_cloud/data/F4/wenlianF2_100pct_normals_scan.ply", *cloud);

    // Get lidar pose file
    std::string pose_file = "/home/peteryuan/color/color_cloud/data/F4/wenlianF2_results_traj.txt";

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
    cv::VideoCapture video_left("/home/peteryuan/color/color_cloud/data/F4/VID_20200924_101927_10_002.insv.mp4");
    cv::VideoCapture video_right("/home/peteryuan/color/color_cloud/data/F4/VID_20200924_101927_00_002.insv.mp4");
    std::vector<cv::VideoCapture> videos;
    videos.push_back(video_left);
    videos.push_back(video_right);

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
    ColorVideoCloud(videos, cloud, colored_cloud, pose_file, transforms, intrin, distor, masks, 0);

    // Saving colored cloud
    std::cout << "Saving result..." << std::endl;
    pcl::io::savePLYFile("colored_cloud.ply", *colored_cloud, 0);

    return 0;

}


