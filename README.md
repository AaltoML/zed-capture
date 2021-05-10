# Zed Capture

Tool for capturing IMU sensor and video data through [zed-open-capture](https://github.com/stereolabs/zed-open-capture) library (Zed 2, Zed, Zed mini).

## Dependencies

Install basic bulid tools if you don't have them yet:

`sudo apt install build-essential cmake`

Install dependencies:

`sudo apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev opencv-dev`

## Setup

You need to install udev rule to be able to use the device:

`cd zed-open-capture/udev && bash install_udev_rule.sh && cd ../..`

## Build

Create a build directory:

`mkdir target && cd target`

Build:

`cmake .. && make -j$(nproc)`

## Running

Create output dir:

`mkdir output`

Run tool with resolution and framerates as arguments. Wait until you get "Recording..." message, it will take a while to initialize and sync IMU/Video. Recording won't start until they are within 500ms and will halt if they dritft over 500ms afterwards.

`./zed_capture 720 30`

Supported resolution and framerate combinations:

* 2K: 2208 * 1242, available framerates: 15 fps.
* 1080: 1920 * 1080, available framerates: 15, 30 fps.
* 720: 1280 * 720, available framerates: 15, 30, 60 fps.
* VGA: 672 * 376, available framerates: 15, 30, 60, 100 fps.

## License

This code is licensed under the Apache-2.0 License.
