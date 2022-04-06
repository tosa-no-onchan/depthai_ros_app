/*
*
*  depthai_ros_app/include/ImageConverter_ex.hpp
*
*/
#pragma once

//#include <iostream>
//#include <opencv2/opencv.hpp>
//#include <sstream>
//#include <unordered_map>

//#include "depthai-shared/common/CameraBoardSocket.hpp"
//#include "depthai/depthai.hpp"

#include <depthai_bridge/ImageConverter.hpp>

#ifdef IS_ROS2
#else
//    #include <ros/ros.h>
//    #include <boost/make_shared.hpp>
//    #include <boost/range/algorithm.hpp>
//    #include "sensor_msgs/CameraInfo.h"
//    #include "sensor_msgs/Image.h"
//    #include "std_msgs/Header.h"
#endif

namespace dai {

namespace ros {

#ifdef IS_ROS2
namespace StdMsgs = std_msgs::msg;
namespace ImageMsgs = sensor_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;
#else
namespace StdMsgs = std_msgs;
namespace ImageMsgs = sensor_msgs;
using ImagePtr = ImageMsgs::ImagePtr;
#endif
//using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

class ImageConverter_ex : public dai::rosBridge::ImageConverter {
   public:
    // ImageConverter() = default;

    ImageConverter_ex(bool interleaved);
    ImageConverter_ex(const std::string frameName, bool interleaved);

    void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, ImageMsgs::Image& outImageMsg);
    void AData2RosMsg(std::shared_ptr<dai::ADatatype> aData, ImageMsgs::Image& outImageMsg);
    //ImagePtr toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData);

    //void toDaiMsg(const ImageMsgs::Image& inMsg, dai::ImgFrame& outData);

    /** TODO(sachin): Add support for ros msg to cv mat since we have some
     *  encodings which cv supports but ros doesn't
     **/
    //cv::Mat rosMsgtoCvMat(ImageMsgs::Image& inMsg);

    //ImageMsgs::CameraInfo calibrationToCameraInfo(dai::CalibrationHandler calibHandler,
    //                                              dai::CameraBoardSocket cameraId,
    //                                              int width = -1,
    //                                              int height = -1,
    //                                              Point2f topLeftPixelId = Point2f(),
    //                                              Point2f bottomRightPixelId = Point2f());

    long int _wait_d=0;

   private:
    static std::unordered_map<dai::RawImgFrame::Type, std::string> encodingEnumMap;
    static std::unordered_map<dai::RawImgFrame::Type, std::string> planarEncodingEnumMap;

    // dai::RawImgFrame::Type _srcType;
    bool _daiInterleaved;
    // bool c
    const std::string _frameName = "";
    char _ftype = 0;
    int _type = 0;
    std::string _temp_str;
    cv::Size _size = {0, 0};
    uint32_t _step;
    uint32_t _inData_height;
    uint32_t _inData_width;

    //void planarToInterleaved(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp);
    //void interleavedToPlanar(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h, int numPlanes, int bpp);
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
