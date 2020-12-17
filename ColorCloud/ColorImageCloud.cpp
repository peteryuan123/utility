#include<pcl-1.8/pcl/point_cloud.h>
#include<pcl-1.8/pcl/io/ply_io.h>
#include<pcl-1.8/pcl/PCLPointField.h>
#include<pcl-1.8/pcl/point_types.h>
#include<pcl-1.8/pcl/pcl_macros.h>
#include<pcl/common/transforms.h>

#include<opencv2/calib3d.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc_c.h>
#include<opencv2/core/eigen.hpp>

#include<Eigen/Core>
#include<Eigen/Geometry>

void ColorImageCloud(cv::Mat &image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     Eigen::Matrix4d transform, cv::Mat &intrinsic, cv::Mat &distortion)
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
    //cv::projectPoints(pts,  rvec, tvec, intrinsic, distortion, uvs);
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

        // Coloring
        //std::cout << uv[0] << " " << uv[1] << "\n";
        if (uv[0]>0 && uv[0] < image.cols && uv[1] > 0 && uv[1] < image.rows)
        {
            point.b = image.at<cv::Vec3b>(uv[1],uv[0])[0];
            point.g = image.at<cv::Vec3b>(uv[1],uv[0])[1];
            point.r = image.at<cv::Vec3b>(uv[1], uv[0])[2];
        }

    }
}


int main()
{
    // Load ply file
    std::cout << "Loading ply..." << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile("/home/peteryuan/Desktop/calib_geo/00/0 - Cloud.ply", *colored_cloud);

    Eigen::Matrix4d transform;
    transform <<  0.946609, 0.0623083, -0.316305, 0.187408,
                0.32151, -0.110244,  0.940467, -0.747746,
                0.0237281,  -0.99195, -0.124391, -7.97813,
                0, 0, 0, 1;

    
    Eigen::Matrix3d intrinsic_eigen;
    intrinsic_eigen << 8.6264757161231410e+02, 0., 1.4419426569209147e+03, 0.,
       8.6179474700008700e+02, 1.4373417306128667e+03, 0., 0., 1.;
    cv::Mat intrinsic_cv;
    cv::eigen2cv(intrinsic_eigen, intrinsic_cv);

    double distortion_cv_data[] = {3.5270559653026744e-02, 8.7541438756161601e-03,
       -1.1365295066544340e-02, 1.3060458954774687e-03};
    Eigen::VectorXd distortion_eigen;
    cv::Mat distortion_cv(1, 4, CV_64FC1, distortion_cv_data);
    //cv::eigen2cv(distortion_eigen, distortion_cv);

    cv::Mat image;
    image = cv::imread("/home/peteryuan/Desktop/calib_geo/00/image/01.png");

    ColorImageCloud(image,colored_cloud, transform, intrinsic_cv, distortion_cv);
    pcl::io::savePLYFile("test_calib.ply", *colored_cloud, 0);

}