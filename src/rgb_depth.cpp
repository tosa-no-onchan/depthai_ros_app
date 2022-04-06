/*
*  rgb_depth.cpp
*  copy and reffer from 
*      depthai-core/examples/StereoDepth/rgb_depth_aligned.cpp
*      depthai-core/examples/MonoCamera/mono_preview.cpp
*      luxonis/depthai-ros-examples/depthai_examples/ros1_src/stereo_publisher.cpp
*
* time
*  https://qiita.com/yukiB/items/01f8e276d906bf443356
*  https://qiita.com/srs/items/4c1f8111ce82e32e6117
*/
#include "ros/ros.h"
#include <iostream>
#include <cstdio>
#include <tuple>
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>

// for time
//#include <iostream>
#include <chrono>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>

//#define USE_CAMERA_SYNC

#define USE_CONV_EX
#ifndef USE_CONV_EX
#include <depthai_bridge/ImageConverter.hpp>
#else
#include "depthai_ros_app/ImageConverter_ex.hpp"
#endif

//#include <depthai_bridge/DisparityConverter.hpp>
#include <image_transport/image_transport.h>
//#include <geometry_msgs/Point.h>
#include <sensor_msgs/Temperature.h>

namespace ImageMsgs = sensor_msgs;

// Optional. If set (true), the ColorCamera is downscaled from 1080p to 720p.
// Otherwise (false), the aligned depth is automatically upscaled to 1080p
static std::atomic<bool> downscaleColor{true};
//static constexpr int fps = 30;
//static constexpr int fps = 15;
// The disparity is computed at this resolution, then upscaled to RGB resolution
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_400_P;

static float rgbWeight = 0.4f;
static float depthWeight = 0.6f;

static std::atomic<bool> lrcheck{true};
static std::atomic<bool> extended{false};
static std::atomic<bool> subpixel{false};

static void updateBlendWeights(int percentRgb, void* ctx) {
    rgbWeight = float(percentRgb) / 100.f;
    depthWeight = 1.f - rgbWeight;
}

#define FREQUENCY_CAPUTION_HZ               30   // 15[hz]

int main(int argc, char** argv){
    using namespace std;
    int odom_off=0;                 // foxbot_core3  tf publish timming off set [ms]
    int rate = FREQUENCY_CAPUTION_HZ;    // own publish rate

    int LRchecktresh = 5;
    int confidence = 200;

    ros::init(argc, argv, "rgb_depth");
    ros::NodeHandle pnh("~");

    //pnh.getParam("rate", rate);

    pnh.getParam("LRchecktresh", LRchecktresh);
    pnh.getParam("confidence", confidence);

    std::cout << "rate:=" << rate << std::endl;
    std::cout << "LRchecktresh:=" << LRchecktresh << std::endl;
    std::cout << "confidence:=" << confidence << std::endl;

    image_transport::ImageTransport it(pnh);
    image_transport::Publisher rgb_image_pub = it.advertise("/rgb/image", 1);
    image_transport::Publisher depth_image_pub = it.advertise("/stereo_publisher/stereo/depth",1);

    ros::Publisher rgb_cameraInfoPublisher;
    ros::Publisher depth_cameraInfoPublisher;

    rgb_cameraInfoPublisher = pnh.advertise<ImageMsgs::CameraInfo>("/rgb/camera_info",1);
    depth_cameraInfoPublisher = pnh.advertise<ImageMsgs::CameraInfo>("/stereo_publisher/stereo/camera_info",1);

    #ifdef USE_CAMERA_SYNC
    // /camera/sync publisher
    // syncolonize with tf-base_footprint of foxbot_core3. 
    sensor_msgs::Temperature camera_sync;
    //ros::Publisher camera_sync_publisher("/rgb/sync", &camera_sync);
   	//pnh.advertise(&camera_sync_publisher);
    ros::Publisher camera_sync_publisher = pnh.advertise<sensor_msgs::Temperature>("/camera/sync", 1);   
    #endif

    // Create pipeline
    dai::Pipeline pipeline;
    //std::vector<std::string> queueNames;

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

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto rgbOut = pipeline.create<dai::node::XLinkOut>();
    auto depthOut = pipeline.create<dai::node::XLinkOut>();

    rgbOut->setStreamName("rgb");
    //queueNames.push_back("rgb");
    depthOut->setStreamName("depth");
    //queueNames.push_back("depth");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    //camRgb->setPreviewSize(300, 300);       // 試し。 2022.3.3 by nishi
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(rate);

    //if(downscaleColor) camRgb->setIspScale(2, 3);
    //if(downscaleColor) camRgb->setIspScale(1, 3);       // width 640 height 360
    if(downscaleColor) camRgb->setIspScale(1, 4);       // width 480 height 270
    // For now, RGB needs fixed focus to properly align with depth.
    // This value was used during calibration
    camRgb->initialControl.setManualFocus(135);

    left->setResolution(monoRes);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setFps(rate);
    right->setResolution(monoRes);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setFps(rate);

    // StereoDepth
    #define ORG_USE_3
    #ifdef ORG_USE_1
    // org from rgb_depth_aligned.cpp
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // LR-check is required for depth alignment
    stereo->setLeftRightCheck(true);
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
    #endif
 
    #ifdef ORG_USE_2
    // from stereo_depth.cpp  --> OK こちらで、動作します。
    //stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    // stereo->setInputResolution(1280, 720);
    stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
 
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);     // 上の指定をまねる。
    #endif

    #ifdef ORG_USE_3
    // from stereo_publisher.cpp
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);     // 上の指定をまねる。
    #endif

    // Linking
    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    //stereo->disparity.link(depthOut->input);        // オリジナルの設定
    stereo->depth.link(depthOut->input);               // こちらを試す。 2022.3.3

    std::cout << "rgb_depth:#5 " << std::endl;
    // Connect to device and start pipeline
    dai::Device device(pipeline);
    std::cout << "rgb_depth:#6 " << std::endl;

    // Sets queues size and behavior
    //for(const auto& name : queueNames) {
        //device.getOutputQueue(name, 4, false);
        //device.getOutputQueue(name, 3, false);
    //    device.getOutputQueue(name, 1, false);
    //}

    auto qRgb =device.getOutputQueue("rgb", 1, false);
    auto qDepth =device.getOutputQueue("depth", 1, false);

    //std::unordered_map<std::string, cv::Mat> frame;

    //auto rgbWindowName = "rgb";
    //auto depthWindowName = "depth";
    //auto blendedWindowName = "rgb-depth";
    //cv::namedWindow(rgbWindowName);
    //cv::namedWindow(depthWindowName);
    //cv::namedWindow(blendedWindowName);
    //int defaultValue = (int)(rgbWeight * 100);
    //cv::createTrackbar("RGB Weight %", blendedWindowName, &defaultValue, 100, updateBlendWeights);

    auto calibrationHandler = device.readCalibration();

    // depth camera
    // width=640 height=400
    // width 480 height 270
    width=480;
    height=270;

    #ifndef USE_CONV_EX
    dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame", true);
    #else
    dai::rosBridge::ImageConverter_ex depthConverter(tfPrefix + "_right_camera_optical_frame", true);
    #endif
    auto depthCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, width, height);

    // RGB camera
    // camRgb->setIspScale(x, y) の指定に拠る。
    // width 640 height 360
    //width=640;
    //height=360;

    #ifndef USE_CONV_EX
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", true);
    #else
    dai::rosBridge::ImageConverter_ex rgbConverter(tfPrefix + "_rgb_camera_optical_frame", true);
    #endif
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, width, height);

    ImageMsgs::Image depth_img;
    ImageMsgs::Image rgb_img;

    int rgb_cnt=0;
    int depth_cnt=0;

    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間

    double fs=(double)rate;
    double fs_ave=(double)rate;
    //unsigned int sleep_nt = (unsigned int)((1000000.0 / rate)/6.0) ;  // micro sec
    unsigned int sleep_nt = 15*1000;
    //sleep_nt = 3*1000;
    //sleep_nt = 1*1000;
    //sleep_nt = 4*1000;
    //unsigned int sleep_nt = 40*1000;    // 40[ms]

    int sw=0;

    while(true) {
        std::shared_ptr<dai::ADatatype> rgb_sp;
        std::shared_ptr<dai::ADatatype> depth_sp;

        rgb_sp = qRgb->get<dai::ADatatype>();
        depth_sp = qDepth->get<dai::ADatatype>();
        sw++;

        if(sw>=2){
            //rgbConverter.toRosMsg(rgb_sp, rgb_img);
            rgbConverter.AData2RosMsg(rgb_sp, rgb_img);
            // publish image data
            rgb_image_pub.publish(rgb_img);
            rgbCameraInfo.header.stamp =rgb_img.header.stamp;
            rgbCameraInfo.header.frame_id = rgb_img.header.frame_id;
            rgb_cameraInfoPublisher.publish(rgbCameraInfo);
            rgb_cnt++;

            //depthConverter.toRosMsg(depth_sp, depth_img);
            depthConverter.AData2RosMsg(depth_sp, depth_img);
            // publish image data
            depth_image_pub.publish(depth_img);
            depthCameraInfo.header.stamp =depth_img.header.stamp;
            depthCameraInfo.header.frame_id = depth_img.header.frame_id;
            depth_cameraInfoPublisher.publish(depthCameraInfo);
            depth_cnt++;
            sw=0;
        }

        if(rgb_cnt >= 30){
            end = std::chrono::system_clock::now();  // 計測終了時間
            double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count(); //処理に要した時間をマイクロ秒に変換
            start = std::chrono::system_clock::now();
            //fs = (double)(rgb_cnt+depth_cnt) / 2.0 * 1000.0 / elapsed;
            fs = (double)rgb_cnt * 1000000.0 / elapsed;
            fs_ave = (fs+fs_ave) / 2.0;
            std::cout << rgb_cnt - depth_cnt << std::endl;

            std::cout << "rgbConverter._wait_d="<< rgbConverter._wait_d/1000000 << "[ms]"<<  std::endl;
            std::cout << "depthConverter._wait_d="<< depthConverter._wait_d/1000000 << "[ms]" <<std::endl;

            std::cout << fs << " [Hz]"<< std::endl;
            //std::cout << "off_fox="<< off_fox <<  std::endl;
            rgb_cnt=0;
            depth_cnt=0;
        }
        if(ros::isShuttingDown()==true){
            break;
        }
        //ros::spinOnce();
        //ratex.sleep();
        //ros::Duration(0.001).sleep();
        //int sleep_nt = 10 * 1000;
        usleep(sleep_nt);
    }
    //ros::spin();
    ros::shutdown();
    return 0;
}