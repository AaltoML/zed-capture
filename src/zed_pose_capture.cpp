#include <sl/Camera.hpp>

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
#include <nlohmann/json.hpp>

#include <jsonl-recorder/recorder.hpp>

// using namespace sl;

std::string currentISO8601TimeUTC() {
  auto now = std::chrono::system_clock::now();
  auto itt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  ss << std::put_time(gmtime(&itt), "%FT%H-%M-%SZ");
  return ss.str();
}

int getOCVtype(sl::MAT_TYPE type) {
    int cv_type = -1;
    switch (type) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}

cv::Mat slMat2cvMat(sl::Mat& input) {
    return cv::Mat(
        input.getHeight(), 
        input.getWidth(), 
        getOCVtype(input.getDataType()), 
        input.getPtr<sl::uchar1>(sl::MEM::CPU), 
        input.getStepBytes(sl::MEM::CPU)
    );
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

    std::atomic<bool> running(true);

    // Zed

    sl::Camera zed;
    
    // Set configuration parameters for the ZED
    sl::InitParameters init_parameters;
    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    // init_parameters.sdk_verbose = true;

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Camera Open: " << returned_state << ". Exit program." << std::endl;
        return EXIT_FAILURE;
    }

    // Set parameters for Positional Tracking
    sl::PositionalTrackingParameters positional_tracking_param;
    positional_tracking_param.enable_area_memory = true;
    returned_state = zed.enablePositionalTracking(positional_tracking_param);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Enabling positionnal tracking failed: " << returned_state << std::endl;
        zed.close();
        return EXIT_FAILURE;
    }

    // Display camera information (model, serial number, firmware version)
    auto info = zed.getCameraInformation();
    std::cout << "Camera Model: " << info.camera_model << std::endl;
    std::cout << "Serial Number: " << info.serial_number << std::endl;
    std::cout << "Camera Firmware: " << info.camera_configuration.firmware_version << std::endl;
    std::cout << "Sensors Firmware: " << info.sensors_configuration.firmware_version << std::endl;

    // Display accelerometer sensor configuration
    // sl::SensorParameters& sensor_parameters = info.sensors_configuration.accelerometer_parameters;
    // std::cout << "Sensor Type: " << sensor_parameters.type << std::endl;
    // std::cout << "Sampling Rate: " << sensor_parameters.sampling_rate << std::endl;
    // std::cout << "Range: "       << sensor_parameters.range << std::endl;
    // std::cout << "Resolution: "  << sensor_parameters.resolution << std::endl;
    // if (isfinite(sensor_parameters.noise_density)) std::cout << "Noise Density: " << sensor_parameters.noise_density << std::endl;
    // if (isfinite(sensor_parameters.random_walk)) std::cout << "Random Walk: " << sensor_parameters.random_walk << std::endl;

    sl::Pose camera_pose;
    sl::POSITIONAL_TRACKING_STATE tracking_state;
    sl::SensorsData sensors_data;
    
    double previousImuTimestamp = -1;
    // double previousVideoTimestamp = -1;
    double previousPoseTimestamp = -1;

    std::thread sensThread([&](){
        while(running.load()) {
            // Using sl::TIME_REFERENCE::CURRENT decouples this from Camera.grab() so we get full 400Hz sampling rate
            if (zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) {
                constexpr double D2R = M_PI / 180.0; // Zed 2 uses degrees, convert to radians
                double timestamp = (double)sensors_data.imu.timestamp.getNanoseconds() / 1e9;
                if (timestamp != previousImuTimestamp) {
                    auto angular_velocity = sensors_data.imu.angular_velocity;
                    auto linear_acceleration = sensors_data.imu.linear_acceleration;
                    recorder->addGyroscope(timestamp, angular_velocity.x * D2R, angular_velocity.y * D2R, angular_velocity.z * D2R);
                    recorder->addAccelerometer(timestamp, linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);
                    previousImuTimestamp = timestamp;
                }
                // TODO: Temperature
                // TODO: Magnetometer
            }
        }
    });

    sl::Mat leftZedImage;
    sl::Mat rightZedImage;
    cv::Mat leftImage;
    cv::Mat rightImage;

    nlohmann::json jPose = R"({
        "time": 0.0,
        "zed": {
            "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
            "orientation": { "w": 0.0, "x": 0.0, "y": 0.0, "z": 0.0 }
        }
    })"_json;
         
    std::thread videoThread([&](){
        while(running.load()) {
            if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
                // Get the position of the camera in a fixed reference frame (the World Frame)
                
                tracking_state = zed.getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD);
                if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK 
                    || tracking_state == sl::POSITIONAL_TRACKING_STATE::SEARCHING) {
                    double timestamp = (double)camera_pose.timestamp.getNanoseconds() / 1e9;
                    if (timestamp != previousPoseTimestamp) {
                        auto translation = camera_pose.getTranslation();
                        auto orientation = camera_pose.getOrientation();
                        jPose["time"] = timestamp;
                        jPose["zed"]["position"]["x"] = translation.x;
                        jPose["zed"]["position"]["y"] = translation.y;
                        jPose["zed"]["position"]["z"] = translation.z;
                        jPose["zed"]["orientation"]["x"] = orientation.x;
                        jPose["zed"]["orientation"]["y"] = orientation.y;
                        jPose["zed"]["orientation"]["z"] = orientation.z;
                        jPose["zed"]["orientation"]["w"] = orientation.w;
                        recorder->addJson(jPose);
                        previousPoseTimestamp = timestamp;
                    }
                }

                zed.retrieveImage(leftZedImage, sl::VIEW::LEFT, sl::MEM::CPU);
                zed.retrieveImage(rightZedImage, sl::VIEW::RIGHT, sl::MEM::CPU);

                leftImage = slMat2cvMat(leftZedImage);
                rightImage = slMat2cvMat(rightZedImage);

                double timestamp = (double)zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds() / 1e9;

                std::vector<recorder::FrameData> frameGroup {
                    recorder::FrameData {
                        .t = timestamp,
                        .cameraInd = 0,
                        .focalLengthX = -1,
                        .focalLengthY = -1,
                        .px = -1,
                        .py = -1,
                        .frameData = &leftImage
                    },
                    recorder::FrameData {
                        .t = timestamp,
                        .cameraInd = 1,
                        .focalLengthX = -1,
                        .focalLengthY = -1,
                        .px = -1,
                        .py = -1,
                        .frameData = &rightImage
                    }
                };
                recorder->addFrameGroup(frameGroup[0].t, frameGroup);
            } else {
                sl::sleep_ms(1);
            }
        }
    });

    std::cout << std::endl << "Recording! Press Enter to finish recording" << std::endl;

    // Wait for user input before stopping
    std::cin.ignore();

    std::cout << "Exiting. Waiting recorder thread to finish..." << std::endl;

    running.store(false);
    sensThread.join();
    videoThread.join();
    zed.disablePositionalTracking();
    zed.close();

    std::cout << "Bye!\n";

    return EXIT_SUCCESS;
}
