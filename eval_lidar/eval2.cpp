#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
namespace fs = boost::filesystem;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

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

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "Usage ./eval path\n";
    }
    std::string path = argv[1];
    // Read all clouds
    PointCloud::Ptr cloud(new PointCloud);
    Eigen::Matrix4d plane;

    // std::cout << "Reading clouds...\n";
    // std::cout << "-------------------------\n";
    pcl::io::loadPCDFile(path + "/cloud.pcd", *cloud);
    read_transform(path + "/plane.txt", plane);
    Eigen::Matrix4d plane_inverse;
    plane_inverse = plane.inverse();

    pcl::transformPointCloud(*cloud, *cloud, plane_inverse);

    float min_z = 9999;
    float max_z = -9999;
    float mean_error = 0;
    float variance = 0;

    for (int i = 0; i < cloud->points.size(); i++)
    {
        std::cout << cloud->points[i].z << std::endl;
        mean_error += std::abs(cloud->points[i].z);
        min_z = std::min(min_z, cloud->points[i].z);
        max_z = std::max(max_z, cloud->points[i].z);
    }
    mean_error /= cloud->points.size();

    for (int i = 0; i < cloud->points.size(); i++)
    {
        variance += std::abs(cloud->points[i].z - mean_error);
    }
    variance /= cloud->points.size();

    pcl::io::savePCDFile(path+"/transformed.pcd", *cloud, 1);
    std::cout << "-----------------------------\n";
    std::cout << "mean error: " << mean_error << ", variance: " << variance << std::endl;
    std::cout << "min z: " << min_z << ", max z: " << max_z << ",thick: "  << max_z - min_z << std::endl;


    return 0;
}