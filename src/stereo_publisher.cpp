/*
* stereo_publisher.cpp
* reffer
* https://sy-base.com/myrobotics/ros/ros_cvbridge/
*/

#include "ros/ros.h"

#include <iostream>
#include <cstdio>
#include <tuple>
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

//#include <depthai_bridge/BridgePublisher.hpp>
//#include <depthai_bridge/ImageConverter.hpp>
#include "depthai_ros_app/ImageConverter_ex.hpp"
//#include <depthai_bridge/DisparityConverter.hpp>
#include <image_transport/image_transport.h>

// for time
//#include <iostream>
#include <chrono>

static std::atomic<bool> lrcheck{true};
static std::atomic<bool> extended{false};
static std::atomic<bool> subpixel{false};

//void pub_sub(ImageMsgs::Image& img, image_transport::Publisher& img_pub, 
//                    ImageMsgs::CameraInfo& info,ros::Publisher& info_pub);

namespace ImageMsgs = sensor_msgs;

void pub_sub(ImageMsgs::Image img, image_transport::Publisher *img_pub, 
                    ImageMsgs::CameraInfo info,ros::Publisher *info_pub){
    img_pub->publish(img);
    info.header.stamp = img.header.stamp;
    info.header.frame_id = img.header.frame_id;
    info_pub->publish(info);
}

#define FREQUENCY_CAPUTION_HZ               15   // 15[hz]
//#define PUB_IMG

int main(int argc, char** argv){

    using namespace std;
    //namespace ImageMsgs = sensor_msgs;
    int confidence = 200;
    int LRchecktresh = 5;

    int rate = FREQUENCY_CAPUTION_HZ;    // own publish rate

    ros::init(argc, argv, "stereo_depth");
    ros::NodeHandle pnh("~");

    image_transport::ImageTransport it(pnh);

    image_transport::Publisher left_image_pub = it.advertise("/left/image", 1);
    image_transport::Publisher right_image_pub = it.advertise("/right/image", 1);

    //camera_info_manager::CameraInfoManager left_camInfoManager(pnh,"left");
    //camera_info_manager::CameraInfoManager right_camInfoManager(pnh,"right");

    ros::Publisher left_cameraInfoPublisher;
    ros::Publisher right_cameraInfoPublisher;
    //ros::Publisher depth_cameraInfoPublisher;

    left_cameraInfoPublisher = pnh.advertise<ImageMsgs::CameraInfo>("/left/camera_info",1);
    right_cameraInfoPublisher = pnh.advertise<ImageMsgs::CameraInfo>("/right/camera_info",1);

    std::string tfPrefix="oak";
    string resolution = "400p";

    dai::node::MonoCamera::Properties::SensorResolution monoResolution;

    int width, height;

    // set default
    monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P; 
    width  = 640;
    height = 480;

    if(resolution == "720p"){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P; 
        width  = 1280;
        height = 720;
    }
    else if(resolution == "400p" ){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P; 
        width  = 640;
        height = 400;
    }
    else if(resolution == "800p" ){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P; 
        width  = 1280;
        height = 800;
    }
    else if(resolution == "480p" ){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P; 
        width  = 640;
        height = 480;
    }
    //else{
    //    ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
    //    throw std::runtime_error("Invalid mono camera resolution.");
    //}

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();

    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();

    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setFps(rate);
    //monoLeft->setFps(15);

    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setFps(rate);
    //monoRight->setFps(15);

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    // Link plugins CAM -> XLINK
    monoLeft->out.link(xoutLeft->input);
    monoRight->out.link(xoutRight->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto leftQueue = device.getOutputQueue("left", 1, false);
    auto rightQueue = device.getOutputQueue("right", 1, false);

    auto calibrationHandler = device.readCalibration();

    dai::rosBridge::ImageConverter_ex leftConverter(tfPrefix + "_left_camera_optical_frame", true);
    dai::rosBridge::ImageConverter_ex rightConverter(tfPrefix + "_right_camera_optical_frame", true);

    auto leftCameraInfo = leftConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, width, height); 
    //ImageMsgs::CameraInfo leftCameraInfo = leftConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, width, height); 
    auto rightCameraInfo = rightConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, width, height);

    //left_camInfoManager.setCameraInfo(leftCameraInfo);
    //right_camInfoManager.setCameraInfo(rightCameraInfo);

    ImageMsgs::Image left_img;
    ImageMsgs::Image right_img;

    int right_cnt=0;
    int depth_cnt=0;

    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間

    unsigned int sleep_nt = 27*1000;    // 17[ms]

    int sw=0;

    while(true) {
        std::shared_ptr<dai::ADatatype> left;
        std::shared_ptr<dai::ADatatype> right;
        // left Mono image get
        left = leftQueue->get<dai::ADatatype>();
        //cv::imshow("left", left->getFrame());
        // right Mono image get
        right = rightQueue->get<dai::ADatatype>();
        //cv::imshow("right", right->getFrame());
        sw++;
        if(sw>=2){
            //convert left dai::ImgFrame to sensor_msgs/Image
            leftConverter.AData2RosMsg(left, left_img);
            //convert right dai::ImgFrame to sensor_msgs/Image
            rightConverter.AData2RosMsg(right, right_img);

            // publish image data
            //left_image_pub.publish(left_img);
            //right_image_pub.publish(right_img);

            pub_sub(left_img, &left_image_pub, leftCameraInfo, &left_cameraInfoPublisher);
            pub_sub(right_img, &right_image_pub, rightCameraInfo, &right_cameraInfoPublisher);

            // publish camera info
            //leftCameraInfo.header.stamp = left_img.header.stamp;
            //leftCameraInfo.header.frame_id = left_img.header.frame_id;
            //left_cameraInfoPublisher.publish(leftCameraInfo);
            //rightCameraInfo.header.stamp =right_img.header.stamp;
            //rightCameraInfo.header.frame_id = right_img.header.frame_id;
            //right_cameraInfoPublisher.publish(rightCameraInfo);

            right_cnt++;

            sw=0;
        }
        if(right_cnt >= 30){
            end = std::chrono::system_clock::now();  // 計測終了時間
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); //処理に要した時間をミリ秒に変換
            double fs = (double)right_cnt * 1000.0 / elapsed;
            //std::cout << right_cnt - depth_cnt << std::endl;
            std::cout << "leftConverter._wait_d="<< leftConverter._wait_d/1000000 << "[ms]"<<  std::endl;
            std::cout << "rightConverter._wait_d="<< rightConverter._wait_d/1000000 << "[ms]" <<std::endl;
            std::cout << fs << " [Hz]"<< std::endl;
            right_cnt=0;
            depth_cnt=0;
            start = std::chrono::system_clock::now();
        }
        if(ros::isShuttingDown()==true){
            break;
        }
        //int key = cv::waitKey(1);
        //if(key == 'q' || key == 'Q') {
        //    return 0;
        //}
        //ros::spinOnce();
        //ros::Duration(0.0001).sleep();
        usleep(sleep_nt);
    }
    //ros::spin();
    ros::shutdown();
    return 0;
}