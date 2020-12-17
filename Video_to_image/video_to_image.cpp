#include<opencv2/opencv.hpp>
#include<iostream>
#include<boost/filesystem.hpp>
#include<string.h>
#include<opencv2/imgproc/imgproc_c.h>
#include<algorithm>
#include<omp.h>

namespace fs = boost::filesystem;

bool blur_detection(cv::Mat img)
{
    cv::Mat gray;
    cv::Scalar mean, sdv;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(3,3),0);
    //cv::Mat temp;
    // cv::resize(gray, temp, cv::Size(gray.rows/2,gray.cols/4));
    // while (cv::waitKey(30) != 27)
    //     cv::imshow("a",temp);
    cv::Laplacian(gray,gray,3);
    cv::meanStdDev(gray,mean,sdv);
    //std::cout << mean << " " << sdv << std::endl;
    //std::cout << sdv[0] << std::endl;
    if (sdv[0] < 5.70) 
        return true;
    else 
        return false; 
        
}

int main(int argc, char** argv)
{

    fs::path input(argv[1]);

    std::vector<std::string> filenames;
    std::string cam_folder[6];

    cv::VideoCapture videos[6];

/*     cv::Mat test;
    cv::Scalar mean,sdv;
    test = cv::imread("/home/peteryuan/Desktop/calib/video_to_image/build/output/cam0/1520526030623340022.png");

    blur_detection(test, mean, sdv);  */

    if(fs::is_directory(input))
    {
        std::cout << "read filenames" << std::endl;
        for(fs::directory_entry& x: fs::directory_iterator(input))
            filenames.push_back(x.path().string());

        std::sort(filenames.begin(),filenames.end());

        for (unsigned int i = 0; i < filenames.size(); i++)
        {
            std::cout << filenames[i] <<std::endl;
            videos[i] = cv::VideoCapture(filenames[i]);
        }
            


        std::cout << "create folders" << std::endl;
        fs::path output(argv[2]);
        fs::create_directory(output);

        for (unsigned int i = 0; i < filenames.size(); i++)
        {
            std::string outfolder = "cam";
            outfolder = output.string() + "/" + outfolder + std::to_string(i);
            std::cout << outfolder << std::endl;
            fs::create_directory(outfolder);
            cam_folder[i] = outfolder;
        }


        std::cout << "output files" << std::endl;
        int count = 0;
        cv::Mat frame_list[6];
        long long num = 1520526024123132022;
        while (cv::waitKey(30) != 27)
        {
            bool blur = true;
            for (unsigned int i = 0; i < filenames.size(); i++)
            {
                videos[i] >> frame_list[i];
                cv::transpose(frame_list[i], frame_list[i]);
                flip(frame_list[i] ,frame_list[i], 0);
                //cv::imshow("a",frame_list[i]);
                blur = blur && blur_detection(frame_list[i]);
            }
            if (frame_list[0].empty()) break;

            if (count % 10 == 0 && !blur)
            {
                for (unsigned int i = 0 ; i < filenames.size(); i++)
                {
                    std::string outfile = cam_folder[i] + "/" + std::to_string(num) + ".png";
                    cv::imwrite(outfile, frame_list[i]);
                }
                num+=250008000;
            }
            count++;
        }
        
    }

}