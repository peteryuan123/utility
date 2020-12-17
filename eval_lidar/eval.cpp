#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <vector>
namespace fs = boost::filesystem;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Mycloud
{
    public:
        
        Mycloud(int scanNum = 16): scanNum(scanNum) 
        {
            for (int i = 0; i < scanNum; i++)
            {
                PointCloud::Ptr temp(new PointCloud);
                temp->resize(2000);
                laserScan.push_back(temp);
            }
        }
        
        ~Mycloud(){};

        void orgnize(PointCloud::Ptr InCloud);

    int scanNum;
    std::vector<PointCloud::Ptr> laserScan;     
};

void Mycloud::orgnize(PointCloud::Ptr InCloud)
{
    std::cout << InCloud->points.size() << std::endl;
    for (uint i = 0; i < InCloud->points.size(); i++)
    {
        auto p = InCloud->points[i];
        double elevationRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
        double elevationDeg = elevationRad * 180 / M_PI + 15;
        int laseridx = std::round(elevationDeg) / 2;
        int ptidx = int(std::round((180 + atan2(p.y, p.x) * 180 / M_PI) / 0.2)) + 1 ;

        laserScan[laseridx]->points[ptidx].x = p.x;
        laserScan[laseridx]->points[ptidx].y = p.y;
        laserScan[laseridx]->points[ptidx].z = p.z;
    }
}


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "Usage ./eval pcd_folder\n";
    }

    // Get all the cloud path
    std::vector<std::string> cloud_names;
    fs::path cloud_path(argv[1]);

    for(fs::directory_entry& x: fs::directory_iterator(cloud_path))
        cloud_names.push_back(x.path().string());
    
    std::sort(cloud_names.begin(), cloud_names.end());
    // Get all the cloud path

    // Read all clouds
    std::cout << "Reading clouds...\n";
    std::vector<Mycloud> clouds;
    for (uint i = 0; i < cloud_names.size(); i++)
    {   
        std::cout << "Reading cloud " << i << std::endl;
        PointCloud::Ptr cloud(new PointCloud);
        Mycloud mycloud;

        // Reorganize the cloud

        pcl::io::loadPCDFile(cloud_names[i], *cloud);
        
        mycloud.orgnize(cloud);

        clouds.push_back(mycloud);
    }
    std::cout << cloud_names[0] << std::endl;

    double xsum = 0;
    double ysum = 0;
    double zsum = 0;
    int total_sum = 0;

    Mycloud mincloud;
    Mycloud maxcloud;

    for (uint scanidx = 0; scanidx < 16; scanidx++)
    {
        for (uint ptidx = 0; ptidx < 1800; ptidx++)
        {   
            std::cout << "-------------------\n";
            double x = 0, y = 0, z = 0;
            int validnum = 0;

            for (uint cloudidx = 0; cloudidx < clouds.size(); cloudidx++)
            {
                auto point = clouds[cloudidx].laserScan[scanidx]->points[ptidx];
                if (point.x != 0 && point.x != 0 && point.z != 0 && !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
                {
                    x += point.x;
                    y += point.y;
                    z += point.z;
                    validnum++;
                }

            }

            if (validnum != 0)
            {
                std::cout << "sum x y z: " << x << " " << y << " " << z << std::endl;
                x /= validnum;
                y /= validnum;
                z /= validnum;

                double sumlossSquareX = 0;
                double sumlossSquareY = 0;
                double sumlossSquareZ = 0;

                pcl::PointXYZ min;
                pcl::PointXYZ max;
                max.x = x; max.y = y; max.z = z;
                
                for (uint cloudidx = 0; cloudidx < clouds.size(); cloudidx++)
                {
                    auto point = clouds[cloudidx].laserScan[scanidx]->points[ptidx];
                    if (point.x != 0 && point.x != 0 && point.z != 0 && !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
                    {
                        double errorx = sqrt((x - point.x) * (x - point.x));
                        double errory = sqrt((y - point.y) * (y - point.y));
                        double errorz = sqrt((z - point.z) * (z - point.z));

                        double min_errorx = sqrt((x - min.x) * (x - min.x));
                        double min_errory = sqrt((y - min.y) * (y - min.y));
                        double min_errorz = sqrt((z - min.z) * (z - min.z));

                        double max_errorx = sqrt((x - max.x) * (x - max.x));
                        double max_errory = sqrt((y - max.y) * (y - max.y));
                        double max_errorz = sqrt((z - max.z) * (z - max.z));

                        sumlossSquareX += errorx; 
                        sumlossSquareY += errory; 
                        sumlossSquareZ += errorz; 

                        min.x = (min_errorx < errorx) ? min.x : point.x;
                        min.y = (min_errory < errory) ? min.y : point.y;
                        min.z = (min_errorz < errorz) ? min.z : point.z;

                        max.x = (max_errorx > errorx) ? max.x : point.x;
                        max.y = (max_errory > errory) ? max.y : point.y;
                        max.z = (max_errorz > errorz) ? max.z : point.z;
                    }

                }

                mincloud.laserScan[scanidx]->points[ptidx] = min;
                maxcloud.laserScan[scanidx]->points[ptidx] = max;

                sumlossSquareX /= validnum;
                sumlossSquareY /= validnum;
                sumlossSquareZ /= validnum;
                std::cout << "mean x y z: " << x << " " << y << " " << z << std::endl;
                std::cout << "variance x y z: " << sumlossSquareX << " " << sumlossSquareY << " " << sumlossSquareZ << std::endl;
                std::cout << "-------------------\n";

                xsum += sumlossSquareX;
                ysum += sumlossSquareY;
                zsum += sumlossSquareZ;

                total_sum++;

            }
            
        }

    }
    //std::cout << total_sum << std::endl;
    std::cout << "final variance x y z: " << xsum/total_sum << " " << ysum/total_sum << " " << zsum/total_sum << std::endl;

    for (uint scanidx = 0; scanidx < 16; scanidx++)
    {
        pcl::io::savePCDFile("/home/peteryuan/eval_lidar/scan/min" + std::to_string(scanidx) + ".pcd", *mincloud.laserScan[scanidx]);
        pcl::io::savePCDFile("/home/peteryuan/eval_lidar/scan/max" + std::to_string(scanidx) + ".pcd", *maxcloud.laserScan[scanidx]);

    }

    //  double ringRad = atan2(p.z, sqrt(p.x * p.x + p.y * p.y));
    //  double ringDegree = ringRad * 180 / PI; 
    //  char *ringDegreeChar = (char *) malloc(32 * sizeof(char)); 
    //  sprintf(ringDegreeChar, "%.2f", ringDegree); 
    //  string ringDegreeString(ringDegreeChar); 
    //  free(ringDegreeChar); 
    //  int ring = ringMap[ringDegreeString]; 
    //  int pIndex = int( round((180 + atan2(p.y, p.x) * 180 / PI) / 0.16)) + 1; 
    return 0;
}