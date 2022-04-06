/*
*
*  depthai_ros_app/src/ImageConverter_ex.cpp
*
*  https://itsakura.com/cpp-inheritance
*  https://ez-net.jp/article/85/b639Y5xC/PKb02YqrhK8A/
*/
#include <cv_bridge/cv_bridge.h>

#include <depthai/depthai.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include "depthai_ros_app/ImageConverter_ex.hpp"
//#include <ratio>
//#include <tuple>

namespace dai {

namespace ros {

std::unordered_map<dai::RawImgFrame::Type, std::string> ImageConverter_ex::encodingEnumMap = {{dai::RawImgFrame::Type::YUV422i, "yuv422"},
                                                                                           {dai::RawImgFrame::Type::RGBA8888, "rgba8"},
                                                                                           {dai::RawImgFrame::Type::RGB888i, "rgb8"},
                                                                                           {dai::RawImgFrame::Type::BGR888i, "bgr8"},
                                                                                           {dai::RawImgFrame::Type::GRAY8, "mono8"},
                                                                                           {dai::RawImgFrame::Type::RAW8, "mono8"},
                                                                                           {dai::RawImgFrame::Type::RAW16, "16UC1"},
                                                                                           {dai::RawImgFrame::Type::YUV420p, "YUV420"}};
// TODO(sachin) : Move Planare to encodingEnumMap and use default planar namings. And convertt those that are not supported in ROS using ImageTransport in the
// bridge.
std::unordered_map<dai::RawImgFrame::Type, std::string> ImageConverter_ex::planarEncodingEnumMap = {
    {dai::RawImgFrame::Type::BGR888p, "rgb8"},  // 3_1_bgr8 represents 3 planes/channels and 1 byte per pixel in BGR format
    {dai::RawImgFrame::Type::RGB888p, "rgb8"},
    {dai::RawImgFrame::Type::NV12, "rgb8"},
    {dai::RawImgFrame::Type::YUV420p, "rgb8"}};

ImageConverter_ex::ImageConverter_ex(bool interleaved) : dai::rosBridge::ImageConverter(interleaved) {
    _ftype=0;
}

ImageConverter_ex::ImageConverter_ex(const std::string frameName, bool interleaved) : _frameName(frameName) , dai::rosBridge::ImageConverter(frameName, interleaved) {
    _ftype=0;
}

void ImageConverter_ex::toRosMsg(std::shared_ptr<dai::ImgFrame> inData, ImageMsgs::Image& outImageMsg) {
    auto tstamp = inData->getTimestamp();

    StdMsgs::Header header;
    header.frame_id = _frameName;

#ifdef IS_ROS2
    auto rclNow = rclcpp::Clock().now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    auto rclStamp = rclNow - diffTime;
    header.stamp = rclStamp;
#else
    auto rosNow = ::ros::Time::now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    if(_wait_d==0.0)
        _wait_d = diffTime.count(); //時間のズレ [ns]
    else{
        _wait_d += diffTime.count(); //時間のズレ [ns]
        _wait_d /= 2;
    }
    long int nsec = rosNow.toNSec() - diffTime.count();
    auto rosStamp = rosNow.fromNSec(nsec);
    header.stamp = rosStamp;
    header.seq = inData->getSequenceNum();
#endif
    if(_ftype==0){
        if(planarEncodingEnumMap.find(inData->getType()) != planarEncodingEnumMap.end()) {
            switch(inData->getType()) {
                case dai::RawImgFrame::Type::BGR888p:
                case dai::RawImgFrame::Type::RGB888p:
                    _size = cv::Size(inData->getWidth(), inData->getHeight());
                    _type = CV_8UC3;
                    break;
                case dai::RawImgFrame::Type::YUV420p:
                case dai::RawImgFrame::Type::NV12:
                    _size = cv::Size(inData->getWidth(), inData->getHeight() * 3 / 2);
                    _type = CV_8UC1;
                    break;

                default:
                    std::runtime_error("ImageConverter_ex.cpp : #1 Invalid dataType inputs..");
                    break;
            }
            _ftype=1;
        } 
        else if(encodingEnumMap.find(inData->getType()) != encodingEnumMap.end()) {
            //std::string temp_str(encodingEnumMap[inData->getType()]);
            _temp_str=encodingEnumMap[inData->getType()];
            //outImageMsg.height = inData->getHeight();
            _inData_height = inData->getHeight();
            //outImageMsg.width = inData->getWidth();
            _inData_width = inData->getWidth();
            //outImageMsg.step = inData->getData().size() / inData->getHeight();
            _step = inData->getData().size() / inData->getHeight();
            _ftype=2;
        }
        else{
            std::runtime_error("ImageConverter_ex.cpp : #2 Invalid dataType inputs..");
            _ftype=3;
        }
    }
    if(_ftype==1){
        // cv::Mat inImg = inData->getCvFrame();
        cv::Mat mat, output;
        //cv::Size size = {0, 0};
        //int type = 0;
        std::vector<cv::Mat> channels;

        switch(inData->getType()) {
            case dai::RawImgFrame::Type::RGB888p:
                // RGB
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 2));
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 1));
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 0));
                cv::merge(channels, output);
                break;

            case dai::RawImgFrame::Type::BGR888p: 
                // BGR
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 0));
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 1));
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 2));
                cv::merge(channels, output);
                break;

            case dai::RawImgFrame::Type::YUV420p:       // Center RGB Camera
                mat = cv::Mat(_size, _type, inData->getData().data());
                cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2BGR_IYUV);
                break;

            case dai::RawImgFrame::Type::NV12:
                mat = cv::Mat(_size, _type, inData->getData().data());
                cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
                break;

            default:
                //output = mat.clone();
                output = cv::Mat(_size, _type, inData->getData().data());
                break;
        }
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, output).toImageMsg(outImageMsg);        
    }
    else if(_ftype==2){
        // Depth data
        // copying the data to ros msg
        outImageMsg.header = header;
        //std::string temp_str(encodingEnumMap[inData->getType()]);
        outImageMsg.encoding = _temp_str;
        //outImageMsg.height = inData->getHeight();
        outImageMsg.height = _inData_height;
        //outImageMsg.width = inData->getWidth();
        outImageMsg.width = _inData_width;
        //outImageMsg.step = inData->getData().size() / inData->getHeight();
        outImageMsg.step = _step;
        if(outImageMsg.encoding == "16UC1")
            outImageMsg.is_bigendian = false;
        else
            outImageMsg.is_bigendian = true;

        size_t size = inData->getData().size();
        if(outImageMsg.data.size() != size){
            outImageMsg.data.resize(size);
        }
        unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(&outImageMsg.data[0]);
        unsigned char* daiImgData = reinterpret_cast<unsigned char*>(inData->getData().data());

        // TODO(Sachin): Try using assign since it is a vector
        // img->data.assign(packet.data->cbegin(), packet.data->cend());
        memcpy(imageMsgDataPtr, daiImgData, size);

    }
    return;
}

void ImageConverter_ex::AData2RosMsg(std::shared_ptr<dai::ADatatype> aData, ImageMsgs::Image& outImageMsg){
    std::shared_ptr<dai::ImgFrame> inData = std::dynamic_pointer_cast<dai::ImgFrame>(aData);
    //aData.reset();

    auto tstamp = inData->getTimestamp();

    StdMsgs::Header header;
    header.frame_id = _frameName;

#ifdef IS_ROS2
    auto rclNow = rclcpp::Clock().now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    auto rclStamp = rclNow - diffTime;
    header.stamp = rclStamp;
#else
    auto rosNow = ::ros::Time::now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    if(_wait_d==0.0)
        _wait_d = diffTime.count(); //時間のズレ [ns]
    else{
        _wait_d += diffTime.count(); //時間のズレ [ns]
        _wait_d /= 2;
    }
    long int nsec = rosNow.toNSec() - diffTime.count();
    auto rosStamp = rosNow.fromNSec(nsec);
    header.stamp = rosStamp;
    header.seq = inData->getSequenceNum();
#endif
    if(_ftype==0){
        if(planarEncodingEnumMap.find(inData->getType()) != planarEncodingEnumMap.end()) {
            switch(inData->getType()) {
                case dai::RawImgFrame::Type::BGR888p:
                case dai::RawImgFrame::Type::RGB888p:
                    _size = cv::Size(inData->getWidth(), inData->getHeight());
                    _type = CV_8UC3;
                    break;
                case dai::RawImgFrame::Type::YUV420p:
                case dai::RawImgFrame::Type::NV12:
                    _size = cv::Size(inData->getWidth(), inData->getHeight() * 3 / 2);
                    _type = CV_8UC1;
                    break;

                default:
                    std::runtime_error("ImageConverter_ex.cpp : #1 Invalid dataType inputs..");
                    break;
            }
            _ftype=1;
        } 
        else if(encodingEnumMap.find(inData->getType()) != encodingEnumMap.end()) {
            //std::string temp_str(encodingEnumMap[inData->getType()]);
            _temp_str=encodingEnumMap[inData->getType()];
            //outImageMsg.height = inData->getHeight();
            _inData_height = inData->getHeight();
            //outImageMsg.width = inData->getWidth();
            _inData_width = inData->getWidth();
            //outImageMsg.step = inData->getData().size() / inData->getHeight();
            _step = inData->getData().size() / inData->getHeight();
            _ftype=2;
        }
        else{
            std::runtime_error("ImageConverter_ex.cpp : #2 Invalid dataType inputs..");
            _ftype=3;
        }
    }
    if(_ftype==1){
        // cv::Mat inImg = inData->getCvFrame();
        cv::Mat mat, output;
        //cv::Size size = {0, 0};
        //int type = 0;
        std::vector<cv::Mat> channels;

        switch(inData->getType()) {
            case dai::RawImgFrame::Type::RGB888p:
                // RGB
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 2));
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 1));
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 0));
                cv::merge(channels, output);
                break;

            case dai::RawImgFrame::Type::BGR888p: 
                // BGR
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 0));
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 1));
                channels.push_back(cv::Mat(_size, CV_8UC1, inData->getData().data() + _size.area() * 2));
                cv::merge(channels, output);
                break;

            case dai::RawImgFrame::Type::YUV420p:       // Center RGB Camera
                mat = cv::Mat(_size, _type, inData->getData().data());
                cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2BGR_IYUV);
                break;

            case dai::RawImgFrame::Type::NV12:
                mat = cv::Mat(_size, _type, inData->getData().data());
                cv::cvtColor(mat, output, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
                break;

            default:
                //output = mat.clone();
                output = cv::Mat(_size, _type, inData->getData().data());
                break;
        }
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, output).toImageMsg(outImageMsg);        
    }
    else if(_ftype==2){
        // Depth data
        // copying the data to ros msg
        outImageMsg.header = header;
        //std::string temp_str(encodingEnumMap[inData->getType()]);
        outImageMsg.encoding = _temp_str;
        //outImageMsg.height = inData->getHeight();
        outImageMsg.height = _inData_height;
        //outImageMsg.width = inData->getWidth();
        outImageMsg.width = _inData_width;
        //outImageMsg.step = inData->getData().size() / inData->getHeight();
        outImageMsg.step = _step;
        if(outImageMsg.encoding == "16UC1")
            outImageMsg.is_bigendian = false;
        else
            outImageMsg.is_bigendian = true;

        size_t size = inData->getData().size();
        if(outImageMsg.data.size() != size){
            outImageMsg.data.resize(size);
        }
        unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(&outImageMsg.data[0]);
        unsigned char* daiImgData = reinterpret_cast<unsigned char*>(inData->getData().data());

        // TODO(Sachin): Try using assign since it is a vector
        // img->data.assign(packet.data->cbegin(), packet.data->cend());
        memcpy(imageMsgDataPtr, daiImgData, size);

    }
    return;    
}

}  // namespace ros
}  // namespace dai