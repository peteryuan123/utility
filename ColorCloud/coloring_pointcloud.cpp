#include<pcl-1.8/pcl/point_cloud.h>
#include<pcl-1.8/pcl/io/ply_io.h>
#include<pcl-1.8/pcl/io/ascii_io.h>
#include<pcl-1.8/pcl/PCLPointField.h>
#include<pcl-1.8/pcl/point_types.h>
#include<pcl-1.8/pcl/pcl_macros.h>
#include<pcl/common/transforms.h>
#include<pcl/filters/passthrough.h> 

#include<string.h>
#include<iostream>
#include<opencv2/calib3d.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc_c.h>
#include<opencv2/core/eigen.hpp>
#include<Eigen/Core>
#include<Eigen/Geometry>


struct PointXYZITNormalRGB
{
    PCL_ADD_POINT4D;
    double ts;
    PCL_ADD_NORMAL4D;
    PCL_ADD_RGB;
    float i;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN32;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZITNormalRGB,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (double, ts, ts)
                                    (float, normal_x, nx)
                                    (float, normal_y, ny)
                                    (float, normal_z, nz)
                                    (float, r, r)
                                    (float, g, g)
                                    (float, b, b)
                                    (float, i, intensity)
)

struct PointXYZITNormal
{
    PCL_ADD_POINT4D;
    double ts;
    PCL_ADD_NORMAL4D;
    float i;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN32;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZITNormal,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (double, ts, ts)
                                    (float, normal_x, nx)
                                    (float, normal_y, ny)
                                    (float, normal_z, nz)
                                    (float, i, intensity)
)

// void read_transform(Eigen::Matrix4f &rotation, cv::Vec3f &translation, cv::Mat& K, cv::Vec4f& dist, std::string file)
// {
//     std::ifstream src;
//     src.open(file);
//     std::string header;
//     float x;
//     while(src >> header)
//     {
//         if (header == "Rotation")
//         {
//             cv::Mat_<float> tmp(3,3);
//             for (int i = 0; i < 9; i++)
//             {
//                 src >> x;
//                 tmp << x;
//             }
//             rotation = cv::Mat(tmp);
//         }

//         if (header =="translation")
//         {
//             for (int i = 0; i < 3; i++)
//             {
//                 src >> x;
//                 translation[i] = x;
//             }
//         }

//         if (header == "camera")
//         {
//             cv::Mat_<float> tmp(3,4);
//             for (int i = 0; i < 12; i++)
//             {
//                 src >> x;
//                 tmp << x;
//             }
//             K = cv::Mat(tmp);
//         }

//         if (header == "distortion")
//         {
//             for (int i = 0; i < 4; i++)
//             {
//                 src >> x;
//                 dist[i] = x;
//             }
//         }

//     }
//     src.close();
// }




void ColorImageCloud(cv::Mat &image, pcl::PointCloud<PointXYZITNormal>::Ptr cloud, pcl::PointCloud<PointXYZITNormalRGB>::Ptr colored_cloud, 
                     Eigen::Matrix4d transform, cv::Mat &intrinsic, cv::Mat &distortion, double start)
{
    // Transform rotation matrix to rotation vector because of the requirement of projectPoints()
    cv::Mat rmat = cv::Mat::eye(3,3,CV_32F);
    cv::Mat rvec;
    cv::Rodrigues(rmat, rvec);

    // Transform cloud into camera coordinates
    pcl::PointCloud<PointXYZITNormal>::Ptr transformed_cloud(new pcl::PointCloud<PointXYZITNormal>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    cv::Mat tvec = cv::Mat::zeros(1,3,CV_32F);
    //std::cout << "rvec: " << rvec << std::endl;
    //std::cout << tvec << std::endl;

    // Gets all the 3D points and project them on 2D plane
    cv::Mat pts(cloud->points.size(),1, CV_32FC3);
    for (size_t p_index = 0; p_index < transformed_cloud->points.size(); p_index++)
    {
        pts.at<cv::Vec3f>(p_index, 0)[0] = transformed_cloud->points[p_index].x;
        pts.at<cv::Vec3f>(p_index, 0)[1] = transformed_cloud->points[p_index].y;
        pts.at<cv::Vec3f>(p_index, 0)[2] = transformed_cloud->points[p_index].z;
    }
    cv::Mat uvs(cloud->points.size(), 2, CV_32F);
    cv::fisheye::projectPoints(pts, uvs, rvec, tvec, intrinsic, distortion);

    for (size_t p_index = 0; p_index < transformed_cloud->points.size(); p_index++)
    {
        PointXYZITNormal point = transformed_cloud->points[p_index];
        cv::Vec2f uv = uvs.at<cv::Vec2f>(p_index, 0);
        if (point.z < 0)
        {
            colored_cloud->points[p_index].x = point.x;
            colored_cloud->points[p_index].y = point.y;
            colored_cloud->points[p_index].z = point.z;
            colored_cloud->points[p_index].normal_x = point.normal_x;
            colored_cloud->points[p_index].normal_y = point.normal_y;
            colored_cloud->points[p_index].normal_z = point.normal_z;
            colored_cloud->points[p_index].i = point.i;
            colored_cloud->points[p_index].ts = point.ts;
            colored_cloud->points[p_index].b = 0;
            colored_cloud->points[p_index].g = 0;
            colored_cloud->points[p_index].r = 0;
            continue;
        }

        // Coloring
        if (uv[0]>0 && uv[0] < image.cols && uv[1] > 0 && uv[1] < image.rows)
        {
            colored_cloud->points[p_index].x = point.x;
            colored_cloud->points[p_index].y = point.y;
            colored_cloud->points[p_index].z = point.z;
            colored_cloud->points[p_index].normal_x = point.normal_x;
            colored_cloud->points[p_index].normal_y = point.normal_y;
            colored_cloud->points[p_index].normal_z = point.normal_z;
            colored_cloud->points[p_index].i = point.i;
            colored_cloud->points[p_index].ts = point.ts;
            colored_cloud->points[p_index].b = image.at<cv::Vec3b>(uv[1],uv[0])[0];
            colored_cloud->points[p_index].g = image.at<cv::Vec3b>(uv[1],uv[0])[1];
            colored_cloud->points[p_index].r = image.at<cv::Vec3b>(uv[1], uv[0])[2];
        }
        else
        {
            colored_cloud->points[p_index].x = point.x;
            colored_cloud->points[p_index].y = point.y;
            colored_cloud->points[p_index].z = point.z;
            colored_cloud->points[p_index].normal_x = point.normal_x;
            colored_cloud->points[p_index].normal_y = point.normal_y;
            colored_cloud->points[p_index].normal_z = point.normal_z;
            colored_cloud->points[p_index].i = point.i;
            colored_cloud->points[p_index].ts = point.ts;
            colored_cloud->points[p_index].b = 0;
            colored_cloud->points[p_index].g = 0;
            colored_cloud->points[p_index].r = 0;
        }

    }
}

void ColorVideoCloud(cv::VideoCapture &video, pcl::PointCloud<PointXYZITNormal>::Ptr cloud, pcl::PointCloud<PointXYZITNormalRGB>::Ptr colored_cloud,
                        std::string geo_pose_file, Eigen::Matrix4d geo_to_cam, cv::Mat &intrinsic, cv::Mat &distortion, float offset)
{
    double frame_rate = video.get(cv::CAP_PROP_FPS);
    double start_frame_pos = offset * frame_rate;
    video.set(cv::CAP_PROP_POS_FRAMES, start_frame_pos);
    std::ifstream src;
    std::string header;
    src.open(geo_pose_file);
    getline(src, header);

    size_t cur_index = 0;
    double cur_cloud_time = 0.;
    while (cur_index != colored_cloud->points.size())
    {
        cv::Mat image;
        double x,y,z,w;
        double time;
        // Reading lidar pose
        src >> time;
        src >> x >> y >> z;
        Eigen::Vector3d poseT(x,y,z);
        src >> x >> y >> z >> w;
        Eigen::Quaterniond poseR(x,y,z,w);
        Eigen::Isometry3d geo_to_world(poseR);
        geo_to_world.pretranslate(poseT);

        // Get current cloud 
        pcl::PointCloud<PointXYZITNormal>::Ptr cur_cloud(new pcl::PointCloud<PointXYZITNormal>);
        pcl::PassThrough<PointXYZITNormal> passthrough;
        passthrough.setInputCloud(cloud);
        passthrough.setFilterFieldName("ts");
        passthrough.setFilterLimits();// TODO
        passthrough.filter(*filtered_cloud);

        pcl::PointCloud<PointXYZITNormalRGB>::Ptr cur_colored_cloud(new pcl::PointCloud<PointXYZITNormalRGB>);
        cur_colored_cloud.resize(cur_cloud->points.size());

        video >> image;
        ColorImageCloud(image, cur_cloud, cur_colored_cloud, geo_to_cam*geo_to_world.matrix().inverse(), intrinsic, distortion); 
        

    }


    

    src.close();
}


int main(){

    pcl::PointCloud<PointXYZITNormal>::Ptr cloud(new pcl::PointCloud<PointXYZITNormal>);
    pcl::PointCloud<PointXYZITNormalRGB>::Ptr colored_cloud(new pcl::PointCloud<PointXYZITNormalRGB>);
    pcl::io::loadPLYFile("../data/colorizelibrary3f/cloud.ply", *cloud);
    
    // pcl::ASCIIReader reader;
    // reader.setInputFields<pcl::PointXYZ>();
    // reader.read("frame1.txt", *cloud, 0);

    colored_cloud->points.resize(cloud->points.size());
    cv::Mat image = cv::imread("pano.png");


    Eigen::Matrix4f camera_to_geo_optical_rotate;
    camera_to_geo_optical_rotate << -1, 0, 0, 0,
                                    0, 0, -1, 0,
                                    0, -1, 0, 0,
                                    0, 0, 0, 1;

    Eigen::Matrix4f geo_to_world;// camera pose
    geo_to_world << 0.1115510, -0.9682245,  0.2238251, 0.171192916661659,
                    0.9145918,  0.0119322, -0.4042022,  0.118193745582378,
                    0.3886878,  0.2497977,  0.8868613, -0.0144739742756305,
                    0,0,0,1;


    Eigen::Matrix4f pano_in_geo_to_geo;
    pano_in_geo_to_geo << 0.947,0.006,0.320,0.102,
                        0.006,0.999,-0.038,-0.068,
                        -0.320,0.038,0.947,0.404,
                        0.000,0.000,0.000,1.000;

    // this is extrinsic from geo to pano
    Eigen::Matrix4f cloud_to_pano;
    cloud_to_pano = camera_to_geo_optical_rotate.inverse() * pano_in_geo_to_geo.inverse() * geo_to_world.inverse();
    //std::cout << cloud_to_pano.inverse() << std::endl;

    // Intrinsic and distortion
    Eigen::Matrix3f intrinsic;
    intrinsic << 8.4098849868201216e+02, 0., 1.4434440625372513e+03, 
                 0.,8.4010382923520899e+02, 1.4418184901855836e+03, 
                 0., 0., 1.;
    Eigen::Vector4f distortion;
    distortion << 3.8232475690236148e-02, 1.5403609873562242e-02, -1.8340056869076908e-02, 3.5725672649046612e-03;
    
    // Transform eigen matrix into opencv matrix
    cv::Mat intrinsic_cv, distortion_cv;
    cv::eigen2cv(intrinsic, intrinsic_cv);
    cv::eigen2cv(distortion, distortion_cv);
    //std::cout << intrinsic_cv << std::endl;
    //std::cout << distortion_cv << std::endl;

    //ColorImageCloud(image, cloud, colored_cloud, cloud_to_pano, intrinsic_cv, distortion_cv);
    
    pcl::io::savePLYFile("colored_cloud.ply", *colored_cloud, 0);

    return 0;
    // // PointCloud
    // pcl::PointCloud<PointXYZT>::Ptr cloud(new pcl::PointCloud<PointXYZT>);
    // pcl::PointCloud<PointXYZRGBT>::Ptr colored_cloud(new pcl::PointCloud<PointXYZRGBT>);

    // //Read point cloud
    // pcl::ASCIIReader reader;
    // reader.setInputFields<PointXYZT>();
    // reader.read("cloud.txt", *cloud, 0);

    // colored_cloud->points.resize(cloud->points.size());

    // //Read video
    // cv::VideoCapture video("video.mp4");
    // cv::Mat colored_image;

    // //Read transformation
    // cv::Mat rotation;
    // cv::Vec3f tvec;
    // cv::Vec4f dist;
    // cv::Mat K;
    // read_transform(rotation, tvec, K, dist, "transform.txt");
    
    // //Transform rotation matrix to vector
    // cv::Mat rvec;
    // cv::Rodrigues(rotation, rvec);


    // size_t p_index = 0;
    // float time_interval = 1;
    // float cur_time = 0;

    
    // do{
        
    //     video >> colored_image;
    //     cur_time += time_interval;

    //     for (;p_index < cloud->points.size(); p_index++)
    //     {
    //         PointXYZT point = cloud->points[p_index];
    //         if (point.time > cur_time)
    //         {
    //             break;
    //         }

    //         // Project
    //         cv::Vec3f pt_cv(point.x, point.y, point.z);
    //         cv::Vec2f uv;
    //         cv::fisheye::projectPoints(pt_cv, uv, rvec, tvec, K, dist);
            
    //         // Coloring
    //         if (uv[0]>0 && uv[0] < colored_image.cols && uv[1] > 0 && uv[1] < colored_image.rows)
    //         {
    //             colored_cloud->points[p_index].x = point.x;
    //             colored_cloud->points[p_index].y = point.y;
    //             colored_cloud->points[p_index].z = point.z;
    //             colored_cloud->points[p_index].time = point.time;
    //             colored_cloud->points[p_index].r = colored_image.at<cv::Vec3b>(uv[0],uv[1])[0];
    //             colored_cloud->points[p_index].g = colored_image.at<cv::Vec3b>(uv[0],uv[1])[1];
    //             colored_cloud->points[p_index].b = colored_image.at<cv::Vec3b>(uv[0], uv[1])[2];
    //         }
    //         else
    //         {
    //             colored_cloud->points[p_index].x = point.x;
    //             colored_cloud->points[p_index].y = point.y;
    //             colored_cloud->points[p_index].z = point.z;
    //             colored_cloud->points[p_index].time = point.time;
    //             colored_cloud->points[p_index].r = 0;
    //             colored_cloud->points[p_index].g = 0;
    //             colored_cloud->points[p_index].b = 0;
    //         }
            
    //     }
        
    // }while(!colored_image.empty());

    // pcl::io::savePLYFile("colored_cloud.ply", *colored_cloud, 0);



}