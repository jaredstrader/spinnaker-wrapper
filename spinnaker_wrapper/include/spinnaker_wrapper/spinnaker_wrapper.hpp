//ros
#include <ros/ros.h>
#include <ros/package.h>

//opencv
#include <opencv2/opencv.hpp>

//spinnaker
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

//other
#include <iostream>
#include <sstream>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

class ROS_Spinnaker {
  public:
    ROS_Spinnaker();

    int start();
    int acquire_images();

    //wrapper
    int load_cameras();
    void init();
    void deinit();
    void begin_acquisition();
    void end_acquisition();
    cv::Mat grab_frame();
    // cv::Mat grab_mat_frame();
    cv::Mat convert_to_mat(ImagePtr pImage);
    void set_enum_value(std::string setting, std::string value);
    void set_int_value(std::string setting, int value);
    void set_float_value(std::string setting, float value);
    void set_bool_value(std::string setting, bool value);
    void set_buffer_size(int numBuf);

  private:
    // int AcquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice);
    int PrintDeviceInfo(INodeMap & nodeMap);
    // int RunSingleCamera(CameraPtr pCam);

    //later change these variable names (only used for acquire images function)
    // SystemPtr system;
    // CameraList camList;
    // unsigned int numCameras;

    //custom variables
    cv::Mat _image;
    CameraPtr pCam_;
    SystemPtr system_;
    CameraList camList_;
    unsigned int numCameras_;
    uint64_t timestamp_;
    int frameID_;
    int lastFrameID_;
    int GET_NEXT_IMAGE_TIMEOUT_ = 2000;

};

