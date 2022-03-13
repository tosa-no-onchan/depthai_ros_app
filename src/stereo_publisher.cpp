
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
#include <depthai_bridge/DisparityConverter.hpp>


// add by nishi start
#include <image_transport/image_transport.h>
namespace ImageMsgs = sensor_msgs;

// for time
#include <chrono>

// add by nishi end


std::tuple<dai::Pipeline, int, int> createPipeline(bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution){
    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution; 
    auto monoLeft    = pipeline.create<dai::node::MonoCamera>();
    auto monoRight   = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft    = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight   = pipeline.create<dai::node::XLinkOut>();
    auto stereo      = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth   = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    if (withDepth) {
        xoutDepth->setStreamName("depth");
    }
    else {
        xoutDepth->setStreamName("disparity");
    }

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
    }else{
        //ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
        //throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->syncedLeft.link(xoutLeft->input);
    stereo->syncedRight.link(xoutRight->input);

    if(withDepth){
        stereo->depth.link(xoutDepth->input);
    }
    else{
        stereo->disparity.link(xoutDepth->input);
    }

    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv){
    
    std::string tfPrefix = "oak";
    std::string mode = "depth";
    std::string cameraParamUri="package://depthai_examples/params/camera";
    int badParams = 0;
    bool lrcheck=true;
    bool extended=false;
    bool subpixel=true;
    bool enableDepth;
    int confidence = 200;
    int monoWidth, monoHeight;
    int LRchecktresh = 5;
    std::string monoResolution = "480p";
    dai::Pipeline pipeline;


    ros::init(argc, argv, "stereo_node");
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


    //badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
    //badParams += !pnh.getParam("tf_prefix",        tfPrefix);
    //badParams += !pnh.getParam("mode",             mode);
    //badParams += !pnh.getParam("lrcheck",          lrcheck);
    //badParams += !pnh.getParam("extended",         extended);
    //badParams += !pnh.getParam("subpixel",         subpixel);
    //badParams += !pnh.getParam("confidence",       confidence);
    //badParams += !pnh.getParam("LRchecktresh",     LRchecktresh);
    //badParams += !pnh.getParam("monoResolution",   monoResolution);

    //if (badParams > 0)
    //{   
    //    std::cout << " Bad parameters -> " << badParams << std::endl;
    //    throw std::runtime_error("Couldn't find %d of the parameters");
    //}

    if(mode == "depth"){
        enableDepth = true;
    }
    else{
        enableDepth = false;
    }

    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution);

    dai::Device device(pipeline);

    //auto leftQueue = device.getOutputQueue("left", 30, false);
    auto leftQueue = device.getOutputQueue("left", 2, true);
    //auto rightQueue = device.getOutputQueue("right", 30, false);
    auto rightQueue = device.getOutputQueue("right", 2, true);
    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if (enableDepth) {
        //stereoQueue = device.getOutputQueue("depth", 30, false);
        stereoQueue = device.getOutputQueue("depth", 2, true);
    }
    else{
        //stereoQueue = device.getOutputQueue("disparity", 30, false);
        stereoQueue = device.getOutputQueue("disparity", 2, true);
    }

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    //if (monoHeight > 480 && boardName == "OAK-D-LITE") {
    //    monoWidth = 640;
    //    monoHeight = 480;
    //}
   
    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, monoWidth, monoHeight); 

    //dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(leftQueue,
    //                                                                                pnh, 
    //                                                                                std::string("left/image"),
    //                                                                                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
    //                                                                                &converter, 
    //                                                                                std::placeholders::_1, 
    //                                                                                std::placeholders::_2) , 
    //                                                                                30,
    //                                                                                leftCameraInfo,
    //                                                                                "left");

    //leftPublish.addPublisherCallback();

    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight);

    //dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(rightQueue,
    //                                                                                 pnh, 
    //                                                                                 std::string("right/image"),
    //                                                                                 std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
    //                                                                                 &rightconverter, 
    //                                                                                 std::placeholders::_1, 
    //                                                                                 std::placeholders::_2) , 
    //                                                                                 30,
    //                                                                                 rightCameraInfo,
    //                                                                                 "right");

    //rightPublish.addPublisherCallback();

     if(mode == "depth"){
        //dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(stereoQueue,
        //                                                                             pnh, 
        //                                                                             std::string("stereo/depth"),
        //                                                                             std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
        //                                                                             &rightconverter, // since the converter has the same frame name
        //                                                                                              // and image type is also same we can reuse it
        //                                                                             std::placeholders::_1, 
        //                                                                             std::placeholders::_2) , 
        //                                                                             30,
        //                                                                             rightCameraInfo,
        //                                                                             "stereo");
        //depthPublish.addPublisherCallback();
        //ros::spin();
    }
    else{
        //dai::rosBridge::DisparityConverter dispConverter(tfPrefix + "_right_camera_optical_frame", 880, 7.5, 20, 2000);
        //dai::rosBridge::BridgePublisher<stereo_msgs::DisparityImage, dai::ImgFrame> dispPublish(stereoQueue,
        //                                                                             pnh, 
        //                                                                             std::string("stereo/disparity"),
        //                                                                             std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, 
        //                                                                             &dispConverter, 
        //                                                                             std::placeholders::_1, 
        //                                                                             std::placeholders::_2) , 
        //                                                                             30,
        //                                                                             rightCameraInfo,
        //                                                                             "stereo");
        //dispPublish.addPublisherCallback();
        //ros::spin();
    }


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
        converter.toRosMsg(left, left_img);
        //convert right dai::ImgFrame to sensor_msgs/Image
        rightconverter.toRosMsg(right, right_img);

        // publish image data
        left_image_pub.publish(left_img);
        right_image_pub.publish(right_img);

        //pub_sub(left_img, &left_image_pub, leftCameraInfo, &left_cameraInfoPublisher);
        //pub_sub(right_img, &right_image_pub, rightCameraInfo, &right_cameraInfoPublisher);

        right_cnt++;

        // publish camera info
        leftCameraInfo.header.stamp = left_img.header.stamp;
        leftCameraInfo.header.frame_id = left_img.header.frame_id;
        left_cameraInfoPublisher.publish(leftCameraInfo);
        rightCameraInfo.header.stamp =right_img.header.stamp;
        rightCameraInfo.header.frame_id = right_img.header.frame_id;
        right_cameraInfoPublisher.publish(rightCameraInfo);


        auto depth = stereoQueue->get<dai::ImgFrame>();
        //cv::imshow("depth", depth->getCvFrame());
        if(depth->getWidth() >= 100){
            depth_cnt++;

            rightconverter.toRosMsg(depth, depth_img);
            depth_image_pub.publish(depth_img);

            rightCameraInfo.header.stamp = depth_img.header.stamp;
            rightCameraInfo.header.frame_id = depth_img.header.frame_id;
            depth_cameraInfoPublisher.publish(rightCameraInfo);
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
