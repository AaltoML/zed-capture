#include "videocapture.hpp"
#include "sensorcapture.hpp"

#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <thread>
#include <vector>
#include <string>
#include <atomic>
#include <math.h>

#include <opencv2/opencv.hpp>

#include <jsonl-recorder/recorder.hpp>

std::string currentISO8601TimeUTC() {
  auto now = std::chrono::system_clock::now();
  auto itt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << std::put_time(gmtime(&itt), "%FT%H-%M-%SZ");

  return ss.str();
}

int main(int argc, char * argv[]) {

    if (argc != 3) {
        std::cout << "Requires FPS and resolution: ./zed_capture <VGA/720/1080/2K> <15/30/60/100>" << std::endl;
        std::cout << "2K: 2208*1242, available framerates: 15 fps." << std::endl;
        std::cout << "1080: 1920*1080, available framerates: 15, 30 fps." << std::endl;
        std::cout << "720: 1280*720, available framerates: 15, 30, 60 fps." << std::endl;
        std::cout << "VGA: 672*376, available framerates: 15, 30, 60, 100 fps." << std::endl;
        return EXIT_FAILURE;
    }

    std::string res = argv[1];
    std::string fps = argv[2];

    // Setup recording

    auto startTimeString = currentISO8601TimeUTC();
    auto outputPrefix = "output/recording-" + startTimeString;
    auto recorder = recorder::Recorder::build(outputPrefix + ".jsonl", outputPrefix + "-video.avi");

    // Zed

    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;
    sl_oc::video::VideoParams params;

    if (res == "2K") params.res = sl_oc::video::RESOLUTION::HD2K;
    else if (res == "1080") params.res = sl_oc::video::RESOLUTION::HD1080;
    else if (res == "720") params.res = sl_oc::video::RESOLUTION::HD720;
    else if (res == "VGA") params.res = sl_oc::video::RESOLUTION::VGA;
    else {
        std::cout << "Unrecognized resolution: " << res << std::endl;
        return EXIT_FAILURE;
    }
    if (fps == "100") params.fps = sl_oc::video::FPS::FPS_100;
    else if (fps == "60") params.fps = sl_oc::video::FPS::FPS_60;
    else if (fps == "30") params.fps = sl_oc::video::FPS::FPS_30;
    else if (fps == "15") params.fps = sl_oc::video::FPS::FPS_15;
    else {
        std::cout << "Unrecognized fps: " << fps << std::endl;
        return EXIT_FAILURE;
    }

    recorder->setVideoRecordingFps((int)params.fps);

    std::atomic<bool> running(true);
    std::atomic<bool> recording(false);

    sl_oc::video::VideoCapture videoCap(params);
    if(!videoCap.initializeVideo(-1)) {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "Try to enable verbose to get more info" << std::endl;
        return EXIT_FAILURE;
    }
    int camSn = videoCap.getSerialNumber();
    std::cout << "Video Capture connected to camera sn: " << camSn << std::endl;
    
    sl_oc::sensors::SensorCapture sensCap(verbose);
    if(!sensCap.initializeSensors(camSn)) { // Note: we use the serial number acquired by the VideoCapture object
        std::cerr << "Cannot open sensors capture" << std::endl;
        std::cerr << "Try to enable verbose to get more info" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Sensors Capture connected to camera sn: " << sensCap.getSerialNumber() << std::endl;
    uint16_t fw_maior;
    uint16_t fw_minor;
    sensCap.getFirmwareVersion( fw_maior, fw_minor );
    std::cout << "Firmware version: " << std::to_string(fw_maior) << "." << std::to_string(fw_minor) << std::endl;
    
    videoCap.enableSensorSync(&sensCap);

    int stereoWidth, height; // stereoWidth == width of both cameras combined
    videoCap.getFrameSize(stereoWidth, height);
    int width = stereoWidth / 2;
    cv::Rect leftRect(0, 0, width, height);
    cv::Rect rightRect(width, 0, width, height);
    std::vector<cv::Mat> frames;
    cv::Mat stereoFrameYUV;
    cv::Mat leftFrameYUV;
    cv::Mat rightFrameYUV;

    // TODO: Could adjust settings
    videoCap.resetBrightness();
    videoCap.resetSharpness();
    videoCap.resetContrast();
    videoCap.resetHue();
    videoCap.resetSaturation();
    videoCap.resetGamma();
    videoCap.resetAECAGC();
    videoCap.resetAutoWhiteBalance();
    videoCap.resetROIforAECAGC(sl_oc::video::CAM_SENS_POS::LEFT);
    videoCap.resetROIforAECAGC(sl_oc::video::CAM_SENS_POS::RIGHT);
    std::cout << "Automatic White Balance control: " << ((videoCap.getAutoWhiteBalance())?"ENABLED":"DISABLED") << std::endl;
    std::cout << "Automatic Exposure and Gain control: " << ((videoCap.getAECAGC())?"ENABLED":"DISABLED") << std::endl;

    // Start the sensor capture thread. Note: since sensor data can be retrieved at 400Hz and video data frequency is
    // minor (max 100Hz), we use a separated thread for sensors.
    std::mutex syncMutex;
    double lastImuTimestamp = -1;
    std::thread sensThread([&](){
        while(running.load()) {
            constexpr double D2R = M_PI / 180.0; // Zed 2 uses degrees, convert to radians
            const sl_oc::sensors::data::Imu imuData = sensCap.getLastIMUData(2000);
            if(imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL ) {
                double timestamp = static_cast<double>(imuData.timestamp) / 1e9;
                syncMutex.lock();
                lastImuTimestamp = timestamp;
                syncMutex.unlock();
                if (recording.load()) {
                    recorder->addGyroscope(timestamp, imuData.gX * D2R, imuData.gY * D2R, imuData.gZ * D2R);
                    recorder->addAccelerometer(timestamp, imuData.aX, imuData.aY, imuData.aZ);
                }
            }
        }
    });

    double lastVideoTimestamp = -1;
    std::thread videoThread([&](){
        while(running.load()) {
            const sl_oc::video::Frame frame = videoCap.getLastFrame(1);
            std::stringstream videoTs;
            double timestamp = static_cast<double>(frame.timestamp) / 1e9;

            syncMutex.lock();
            double imuTime = lastImuTimestamp;
            syncMutex.unlock();

            double timeDiff = std::abs(imuTime - timestamp);
            bool rec = recording.load();
            constexpr double MAX_ALLOWED_TIME_DIFF_SECONDS = 0.5;
            if (!rec && timeDiff < MAX_ALLOWED_TIME_DIFF_SECONDS) {
                std::cout << "Achieved time difference between IMU and Video: " << timeDiff << std::endl;
                std::cout << std::endl << "Recording! Press Enter to finish recording" << std::endl;
                rec = true;
                recording.store(true);
            } else if (rec && timeDiff > MAX_ALLOWED_TIME_DIFF_SECONDS) {
                std::cout << "Error! Timediff increased outside allow ranged, stopping: " << timeDiff << std::endl;
                running.store(false);
                return;
            }
            if (lastVideoTimestamp == timestamp)
                continue;
            lastVideoTimestamp = timestamp;
            if(rec && frame.data != nullptr) {
                double timestamp = static_cast<double>(frame.timestamp) / 1e9;

                // stereoFrameYUV contains both camera frames, it must be split and conerted to BGR
                assert(height == frame.height);
                assert(stereoWidth == frame.width);
                stereoFrameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
                // This doesn't copy the data, just references portion of the stereo frame
                leftFrameYUV = cv::Mat(stereoFrameYUV, leftRect);
                rightFrameYUV = cv::Mat(stereoFrameYUV, rightRect);
                // This copies split image and does color conversion
                if (!recorder->getEmptyFrames(2, timestamp, width, height, CV_8UC3, frames)) {
                    // No free frames in buffer, skip frame
                    continue;
                }
                cv::cvtColor(leftFrameYUV, frames[0], cv::COLOR_YUV2BGR_YUYV);
                cv::cvtColor(rightFrameYUV, frames[1], cv::COLOR_YUV2BGR_YUYV);
                
                // Save both frames
                std::vector<recorder::FrameData> frameGroup {
                    recorder::FrameData {
                        .t = timestamp,
                        .cameraInd = 0,
                        .focalLengthX = -1,
                        .focalLengthY = -1,
                        .px = -1,
                        .py = -1,
                        .frameData = &frames[0]
                    },
                    recorder::FrameData {
                        .t = timestamp,
                        .cameraInd = 1,
                        .focalLengthX = -1,
                        .focalLengthY = -1,
                        .px = -1,
                        .py = -1,
                        .frameData = &frames[1]
                    }
                };
                recorder->addFrameGroup(frameGroup[0].t, frameGroup, false);
            }
        }
    });

    std::cout << std::endl << "Initializing sensors, please wait until IMU and Video are synced" << std::endl;

    // Wait for user input before stopping
    std::cin.ignore();

    std::cout << "Exiting. Waiting recorder thread to finish..." << std::endl;

    running.store(false);
    sensThread.join();
    videoThread.join();

    std::cout << "Bye!\n";

    return EXIT_SUCCESS;
}
