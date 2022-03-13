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
static constexpr int fps = 15;
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

#define FREQUENCY_CAPUTION_HZ               15.2f   // 15[hz]

int main(int argc, char** argv){
    using namespace std;
    int odom_off=0;     // foxbot_core3  tf publish timming off set [ms]
    double rate = FREQUENCY_CAPUTION_HZ;

    ros::init(argc, argv, "rgb_depth");
    ros::NodeHandle pnh("~");

    pnh.getParam("odom_off", odom_off);
    pnh.getParam("rate", rate);

    image_transport::ImageTransport it(pnh);
    image_transport::Publisher rgb_image_pub = it.advertise("/rgb/image", 1);
    image_transport::Publisher depth_image_pub = it.advertise("/stereo_publisher/stereo/depth",1);

    ros::Publisher rgb_cameraInfoPublisher;
    ros::Publisher depth_cameraInfoPublisher;

    rgb_cameraInfoPublisher = pnh.advertise<ImageMsgs::CameraInfo>("/rgb/camera_info",1);
    depth_cameraInfoPublisher = pnh.advertise<ImageMsgs::CameraInfo>("/stereo_publisher/stereo/camera_info",1);

    // /camera/sync publisher
    // syncolonize with tf-base_footprint of foxbot_core3. 
    sensor_msgs::Temperature camera_sync;
    //ros::Publisher camera_sync_publisher("/rgb/sync", &camera_sync);
   	//pnh.advertise(&camera_sync_publisher);
    ros::Publisher camera_sync_publisher = pnh.advertise<sensor_msgs::Temperature>("/camera/sync", 1);   

    // Create pipeline
    dai::Pipeline pipeline;
    //std::vector<std::string> queueNames;

    std::string tfPrefix="oak";
    string resolution = "400p";

    int LRchecktresh = 5;
    int confidence = 200;

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
    //camRgb->setFps(fps);

    //if(downscaleColor) camRgb->setIspScale(2, 3);
    //if(downscaleColor) camRgb->setIspScale(1, 3);       // width 640 height 360
    if(downscaleColor) camRgb->setIspScale(1, 4);       // width 480 height 270
    // For now, RGB needs fixed focus to properly align with depth.
    // This value was used during calibration
    camRgb->initialControl.setManualFocus(135);

    left->setResolution(monoRes);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    //left->setFps(fps);
    right->setResolution(monoRes);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    //right->setFps(fps);

    // StereoDepth
    #define ORG_USE_2
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
    #endif

    // Linking
    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    //stereo->disparity.link(depthOut->input);        // オリジナルの設定
    stereo->depth.link(depthOut->input);               // こちらを試す。 2022.3.3

    // Connect to device and start pipeline
    dai::Device device(pipeline);

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

    //ros::Rate ratex(15);

    int rgb_cnt=0;
    int depth_cnt=0;
    u_int32_t camera_sync_cnt=0;

    std::chrono::system_clock::time_point  start, end, cur_t,now_t; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間
    cur_t = std::chrono::system_clock::now(); // 計測開始時間

    bool first_f = true;

    double fs=rate;
    int sleep_nt = (int)(1000.f / rate)/6 * 1000;  // micro sec
    //int sleep_nt = 10 * 1000;
    sleep_nt = 3*1000;

    uint32_t camera_sync_off = 10;  // 10[ms]

    // foxbot_core3 の publish 時刻を、rgb カメラの、キャプション時刻より、off_fox[ms] 早くする。
    double off_fox = (double)odom_off;    // [ms]
    double off_my=0.0;

    while(true) {
        now_t = std::chrono::system_clock::now(); // 今の時間
        double wait_d = std::chrono::duration_cast<std::chrono::milliseconds>(now_t-cur_t).count(); //処理に要した時間をミリ秒に変換
        if (wait_d >= ((double)(1000.0 / rate)+off_my) || first_f == true){

            //std::cout << "wait_d: " << wait_d << std::endl;
            // for camera sync with foxbot_core3.
            if((camera_sync_cnt % 5) == 0){
                // /camera/sync Publish
                ros::Time now_tmp = ros::Time::now();
                // foxbot_core3 の publish 時刻を、off_fox[ms] だけ、早くする。
                if(off_fox > 0.0){
                    double tx = now_tmp.toSec() - off_fox / 1000.0;
                    now_tmp.fromSec(tx);
                }
                camera_sync.header.stamp= now_tmp;
                camera_sync.temperature=fs;
       		    camera_sync_publisher.publish(camera_sync);

            }
            camera_sync_cnt++;

            double off_x = (double_t)((rgbConverter._wait_d+depthConverter._wait_d)/2/1000000);
            off_my=0.0;
            if(off_x > 10.0){
                off_my = -4.0;
            }
            //else if(off_x > 5.0){
            //    off_my = -2.0;
            //}
            else if(off_x < -10.0){
                off_my = 4.0;
            }
            //else if(off_x < -5.0){
            //    off_my = 2.0;
            //}

            //std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

            first_f = false;
            cur_t = std::chrono::system_clock::now();

            bool rgb_f=false;
            bool depth_f=false;

            while(true){
                std::shared_ptr<dai::ImgFrame> rgb_sp;
                std::shared_ptr<dai::ImgFrame> depth_sp;
                if(rgb_f==false){
                    rgb_sp = qRgb->tryGet<dai::ImgFrame>();     // if <dai::ImgFrame> is comming,then return <dai::ImgFrame>. 
                                                                //  from depthai-core/src/device/DataQueue.cpp
                }
                if(depth_f==false){
                    depth_sp = qDepth->tryGet<dai::ImgFrame>();
                }
                if(rgb_f==false){
                    if(rgb_sp){
                        //cv::imshow("left", inLeft->getCvFrame());

                        //std::cout << "+" << std::endl;
                        //frame[name] = latestPacket[name]->getFrame();
                        //auto maxDisparity = stereo->initialConfig.getMaxDisparity();
                        // Optional, extend range 0..95 -> 0..255, for a better visualisation
                        //if(1) frame[name].convertTo(frame[name], CV_8UC1, 255. / maxDisparity);
                        // Optional, apply false colorization
                        //if(1) cv::applyColorMap(frame[name], frame[name], cv::COLORMAP_HOT);

                        //auto rgb = latestPacket[name];
                        rgbConverter.toRosMsg(rgb_sp, rgb_img);

                        // publish image data
                        rgb_image_pub.publish(rgb_img);

                        rgbCameraInfo.header.stamp =rgb_img.header.stamp;
                        rgbCameraInfo.header.frame_id = rgb_img.header.frame_id;
                        rgb_cameraInfoPublisher.publish(rgbCameraInfo);

                        rgb_cnt++;
                        rgb_f=true;
                        //rgb_sp.~shared_ptr();
                    }
                }
                if(depth_f==false){
                    if(depth_sp){
                        //cv::imshow("left", inLeft->getCvFrame());

                        //std::cout << "-" << std::endl;
                        //auto depth = latestPacket[name];
                        depthConverter.toRosMsg(depth_sp, depth_img);

                        // publish image data
                        depth_image_pub.publish(depth_img);

                        depthCameraInfo.header.stamp =depth_img.header.stamp;
                        depthCameraInfo.header.frame_id = depth_img.header.frame_id;
                        depth_cameraInfoPublisher.publish(depthCameraInfo);

                        depth_cnt++;
                        depth_f=true;
                        //depth_sp.~shared_ptr();
                    }                    
                }
                if(rgb_f != false && depth_f != false){
                    break;
                }
                usleep(10);
            }
        }
        if(rgb_cnt >= 30){
            end = std::chrono::system_clock::now();  // 計測終了時間
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); //処理に要した時間をミリ秒に変換
            fs = (double)rgb_cnt * 1000.0 / elapsed;

            std::cout << rgb_cnt - depth_cnt << std::endl;

            std::cout << "rgbConverter._wait_d="<< rgbConverter._wait_d/1000000 << "[ms]"<<  std::endl;
            std::cout << "depthConverter._wait_d="<< depthConverter._wait_d/1000000 << "[ms]" <<std::endl;

            std::cout << fs << " [Hz]"<< std::endl;
            //std::cout << "off_fox="<< off_fox <<  std::endl;
            rgb_cnt=0;
            depth_cnt=0;
            start = std::chrono::system_clock::now();
        }
        // Blend when both received
        //if(frame.find(rgbWindowName) != frame.end() && frame.find(depthWindowName) != frame.end()) {
        //    // Need to have both frames in BGR format before blending
        //    if(frame[depthWindowName].channels() < 3) {
        //        cv::cvtColor(frame[depthWindowName], frame[depthWindowName], cv::COLOR_GRAY2BGR);
        //    }
        //    cv::Mat blended;
        //    cv::addWeighted(frame[rgbWindowName], rgbWeight, frame[depthWindowName], depthWeight, 0, blended);
        //    cv::imshow(blendedWindowName, blended);
        //    frame.clear();
        //}
        //int key = cv::waitKey(1);
        //if(key == 'q' || key == 'Q') {
        //    return 0;
        //}
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