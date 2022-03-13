/*
*
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
#include <depthai_bridge/ImageConverter.hpp>
//#include <depthai_bridge/DisparityConverter.hpp>
#include <image_transport/image_transport.h>


// for time
//#include <iostream>
#include <chrono>

static std::atomic<bool> withDepth{true};
//static std::atomic<bool> outputDepth{false};
static std::atomic<bool> outputDepth{true};

//static std::atomic<bool> outputRectified{true};       // ここは、エラーになる。
static std::atomic<bool> outputRectified{false};
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

int main(int argc, char** argv){

    using namespace std;
    //namespace ImageMsgs = sensor_msgs;
    int confidence = 200;
    int LRchecktresh = 5;

    ros::init(argc, argv, "stereo_depth");
    ros::NodeHandle pnh("~");

    image_transport::ImageTransport it(pnh);
    image_transport::Publisher left_image_pub = it.advertise("/left/image", 3);
    image_transport::Publisher right_image_pub = it.advertise("/right/image", 3);
    image_transport::Publisher depth_image_pub = it.advertise("/stereo_publisher/stereo/depth", 3);


    //camera_info_manager::CameraInfoManager left_camInfoManager(pnh,"left");
    //camera_info_manager::CameraInfoManager right_camInfoManager(pnh,"right");


    ros::Publisher left_cameraInfoPublisher;
    ros::Publisher right_cameraInfoPublisher;
    ros::Publisher depth_cameraInfoPublisher;

    left_cameraInfoPublisher = pnh.advertise<ImageMsgs::CameraInfo>("/left/camera_info",3);
    right_cameraInfoPublisher = pnh.advertise<ImageMsgs::CameraInfo>("/right/camera_info",3);
    depth_cameraInfoPublisher = pnh.advertise<ImageMsgs::CameraInfo>("/stereo_publisher/stereo/camera_info",3);


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
    }else if(resolution == "400p" ){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P; 
        width  = 640;
        height = 400;
    }else if(resolution == "800p" ){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P; 
        width  = 1280;
        height = 800;
    }else if(resolution == "480p" ){
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
    auto stereo = withDepth ? pipeline.create<dai::node::StereoDepth>() : nullptr;

    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutDisp = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifL = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifR = pipeline.create<dai::node::XLinkOut>();

    monoLeft->setResolution(monoResolution);
    //monoLeft->setFps(15);

    monoRight->setResolution(monoResolution);
    //monoRight->setFps(15);

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    if(withDepth) {
        xoutDisp->setStreamName("disparity");
        xoutDepth->setStreamName("depth");
        xoutRectifL->setStreamName("rectified_left");
        xoutRectifR->setStreamName("rectified_right");
    }

    if(withDepth) {
        // StereoDepth
        #define ORG_USE_1
        #ifdef ORG_USE_1
        stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
        stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
        // stereo->setInputResolution(1280, 720);
        stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);
        #else
        // StereoDepth form stereo_publisher.cpp
        stereo->initialConfig.setConfidenceThreshold(confidence);
        stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
        stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);
        #endif


        // Linking
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);
        stereo->disparity.link(xoutDisp->input);

        if(outputRectified) {
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
        }

        if(outputDepth) {
            stereo->depth.link(xoutDepth->input);
        }

    } else {
        // Link plugins CAM -> XLINK
        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);
    }

    // Connect to device and start pipeline
    dai::Device device(pipeline);


    //auto leftQueue = device.getOutputQueue("left", 2, false);
    auto leftQueue = device.getOutputQueue("left", 2, true);
    //auto rightQueue = device.getOutputQueue("right", 2, false);
    auto rightQueue = device.getOutputQueue("right", 2, true);
    //auto dispQueue = withDepth ? device.getOutputQueue("disparity", 2, false) : nullptr;
    auto dispQueue = withDepth ? device.getOutputQueue("disparity", 2, true) : nullptr;
    //auto depthQueue = withDepth ? device.getOutputQueue("depth", 2, false) : nullptr;
    auto depthQueue = withDepth ? device.getOutputQueue("depth", 2, true) : nullptr;
    auto rectifLeftQueue = withDepth ? device.getOutputQueue("rectified_left", 2, false) : nullptr;
    auto rectifRightQueue = withDepth ? device.getOutputQueue("rectified_right", 2, false) : nullptr;

    // Disparity range is used for normalization
    float disparityMultiplier = withDepth ? 255 / stereo->initialConfig.getMaxDisparity() : 0;

    auto calibrationHandler = device.readCalibration();

    dai::rosBridge::ImageConverter leftConverter(tfPrefix + "_left_camera_optical_frame", true);
    auto leftCameraInfo = leftConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, width, height); 
    //ImageMsgs::CameraInfo leftCameraInfo = leftConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, width, height); 

    dai::rosBridge::ImageConverter rightConverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = rightConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, width, height);


    //left_camInfoManager.setCameraInfo(leftCameraInfo);
    //right_camInfoManager.setCameraInfo(rightCameraInfo);

    ImageMsgs::Image left_img;
    ImageMsgs::Image right_img;
    ImageMsgs::Image depth_img;

    ros::Rate rate(15);

    int right_cnt=0;
    int depth_cnt=0;

    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間

    while(true) {
        // left Mono image get
        auto left = leftQueue->get<dai::ImgFrame>();
        //dai::ImgFrame left = leftQueue->get<dai::ImgFrame>();
        
        //cv::imshow("left", left->getFrame());

        // right Mono image get
        auto right = rightQueue->get<dai::ImgFrame>();
        //cv::imshow("right", right->getFrame());

        //convert left dai::ImgFrame to sensor_msgs/Image
        leftConverter.toRosMsg(left, left_img);
        //convert right dai::ImgFrame to sensor_msgs/Image
        rightConverter.toRosMsg(right, right_img);

        // publish image data
        //left_image_pub.publish(left_img);
        //right_image_pub.publish(right_img);

        pub_sub(left_img, &left_image_pub, leftCameraInfo, &left_cameraInfoPublisher);
        pub_sub(right_img, &right_image_pub, rightCameraInfo, &right_cameraInfoPublisher);
        right_cnt++;

        // publish camera info
        //leftCameraInfo.header.stamp = left_img.header.stamp;
        //leftCameraInfo.header.frame_id = left_img.header.frame_id;
        //left_cameraInfoPublisher.publish(leftCameraInfo);
        //rightCameraInfo.header.stamp =right_img.header.stamp;
        //rightCameraInfo.header.frame_id = right_img.header.frame_id;
        //right_cameraInfoPublisher.publish(rightCameraInfo);


        if(withDepth) {
            // Note: in some configurations (if depth is enabled), disparity may output garbage data
            auto disparity = dispQueue->get<dai::ImgFrame>();
            //cv::Mat disp(disparity->getCvFrame());
            //disp.convertTo(disp, CV_8UC1, disparityMultiplier);  // Extend disparity range
            //cv::imshow("disparity", disp);
            //cv::Mat disp_color;
            //cv::applyColorMap(disp, disp_color, cv::COLORMAP_JET);
            //cv::imshow("disparity_color", disp_color);

            if(outputDepth) {
                auto depth = depthQueue->get<dai::ImgFrame>();
                //cv::imshow("depth", depth->getCvFrame());
                if(depth->getWidth() >= 100){
                    depth_cnt++;

                    rightConverter.toRosMsg(depth, depth_img);
                    depth_image_pub.publish(depth_img);

                    rightCameraInfo.header.stamp = depth_img.header.stamp;
                    rightCameraInfo.header.frame_id = depth_img.header.frame_id;
                    depth_cameraInfoPublisher.publish(rightCameraInfo);
                }
            }

            if(outputRectified) {
                auto rectifL = rectifLeftQueue->get<dai::ImgFrame>();
                //cv::imshow("rectified_left", rectifL->getFrame());

                auto rectifR = rectifRightQueue->get<dai::ImgFrame>();
                //cv::imshow("rectified_right", rectifR->getFrame());
            }
        }

        if(right_cnt >= 30){
            end = std::chrono::system_clock::now();  // 計測終了時間
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); //処理に要した時間をミリ秒に変換
            double fs = (double)right_cnt * 1000.0 / elapsed;

            std::cout << right_cnt - depth_cnt << std::endl;
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
        ros::spinOnce();
        ros::Duration(0.0001).sleep();
        //rate.sleep();

    }


    //ros::spin();
    ros::shutdown();
    return 0;
}