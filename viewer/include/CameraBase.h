//
// Created by mpl on 22-5-13.
//

#ifndef VIEWER_CAMERABASE_H
#define VIEWER_CAMERABASE_H

#include <vector>
#include <Eigen/Core>

class CameraBase
{
public:
    CameraBase() {};
    CameraBase(const std::vector<float> &params) : params_(params){}
    ~CameraBase() {};

    virtual Eigen::Vector2d project(Eigen::Vector3d p3d) = 0;
    virtual Eigen::Vector3d invProject(Eigen::Vector2d p2d) = 0;


    static unsigned int camIdTotal_;
protected:
    std::vector<float> params_;
    unsigned int id_;
    unsigned int type_;
};



#endif //VIEWER_CAMERABASE_H
