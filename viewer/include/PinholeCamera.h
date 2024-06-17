#ifndef PINHOLECAMERA_H
#define PINHOLECAMERA_H

#include <opencv2/core/core.hpp>
#include "CameraBase.h"

class PinholeCamera : public CameraBase
{
private:
    cv::Mat
public:
    PinholeCamera(/* args */);
    ~PinholeCamera();
};

PinholeCamera::PinholeCamera(/* args */)
{
}

PinholeCamera::~PinholeCamera()
{
}



#endif