/*
 * The Example of the Withrobot oCam-1MGN-U API using with OpenCV. (Linux only.)
 *
 * This example shows how to get image from the oCam-1MGN using the Withrobot camera API.
 * And also shows how to control the oCam-1MGN using the Withrobot camera API.
 *
 * This example program usage.:
 * 	- Press the key 'q' for exit this program.
 *  - Press the key ']' for increase the exposure.
 *  - Press the key '[' for decrease the exposure.
 *  - Press the key '=' for increase the brightness.
 *  - Press the key '-' for decrease the brightness.
 */

#include <filesystem>
#include <fstream>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <errno.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "api/withrobot_camera.hpp"	/* withrobot camera API */

/*
 *	Main
 */
int main (int argc, char* argv[])
{
    /*
     * initialize oCam-1CGN
     *
     * [ supported image formats ]
     *
     * USB 3.0
     * 	[0] "8-bit Greyscale 1280 x 720 60 fps"
	 *	[1] "8-bit Greyscale 1280 x 960 45 fps"
	 *	[2] "8-bit Greyscale 320 x 240 160 fps"
	 * 	[3] "8-bit Greyscale 640 x 480 80 fps"
	 *
	 * USB 2.0
     * 	[0] "8-bit Greyscale 1280 x 720 30 fps"
	 *	[1] "8-bit Greyscale 1280 x 960 22.5 fps"
	 *	[2] "8-bit Greyscale 320 x 240 160 fps"
	 * 	[3] "8-bit Greyscale 640 x 480 80 fps"
	 *
     */

    // Use designated port when given
    std::string devPath = "";
    if (argc == 2){
    devPath = argv[1];
    }
    else {
        std::vector<std::string> paths;
        // Withrobot camera id would be like "usb-WITHROBOT_Inc._oCam-1CGN-U-T_SN_35E27013-video-index0"
        for (const auto& entry : std::filesystem::directory_iterator("/dev/v4l/by-id") ) {
            if (entry.is_character_file() && (entry.path().filename().string().find("1CGN-U-T") != std::string::npos)) {
                auto path = entry.path().parent_path();
                path /= std::filesystem::read_symlink(entry.path());
                path = std::filesystem::canonical(path);
                paths.push_back(path);
            }
        }
        // Singel camera can contain two video pahts, normally ealier one gives image
        std::sort(paths.begin(), paths.end());
        devPath = paths.front();
    }

    Withrobot::Camera camera(devPath);

    /* USB 3.0 */
    /* bayer RBG 1280 x 720 60 fps */
    // camera.set_format(1280, 720, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, 60);
    camera.set_format(640, 480, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, 30);

    /* bayer RBG 1280 x 960 45 fps */
    //camera.set_format(1280, 960, Withrobot::fourcc_to_pixformat('G','B','G','R')), 1, 45);

    /* bayer RBG 320 x 240 160 fps */
    //camera.set_format(320, 240, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, 160);

    /* bayer RBG 640 x 480 80 fps */
    //camera.set_format(640, 480, Withrobot::fourcc_to_pixformat('G','B','G','R')'), 1, 80);

    /* USB 2.0 */
    /* bayer RBG 1280 x 720 30 fps */
    //camera.set_format(1280, 720, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 1, 30);

    /* bayer RBG 1280 x 960 22.5 fps */
    //camera.set_format(1280, 960, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 2, 45);

    /* bayer RBG 320 x 240 160 fps */
    //camera.set_format(320, 240, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 1, 160);

    /* bayer RBG 640 x 480 80 fps */
    //camera.set_format(640, 480, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 1, 80);

    /*
     * get current camera format (image size and frame rate)
     */
    Withrobot::camera_format camFormat;
    camera.get_current_format(camFormat);

    /*
     * Print infomations
     */
    std::string camName = camera.get_dev_name();
    std::string camSerialNumber = camera.get_serial_number();

    printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
    printf("----------------- Current format informations -----------------\n");
    camFormat.print();
    printf("---------------------------------------------------------------\n");

    /*
     * [ supported camera controls; The double quotes are the 'get_control' and the 'set_control' function string argument values. ]
     *
     *  [0] "Gain",          Value(default [min, step, max]): 64 ( 64 [0, 1, 127] )
     *  [1] "Exposure (Absolute)", Value(default [min, step, max]): 39 ( 39 [1, 1, 625] )
     *
     */
    
    const int32_t INTI_EXPOSURE_ABS = 130;
    camera.set_control("Exposure Time, Absolute", INTI_EXPOSURE_ABS);

    const int32_t INIT_BRIGHTNESS = 110;
    camera.set_control("Gain", INIT_BRIGHTNESS);

    // 1 - Manual mode, 3 - Apeture priority mode
    // see v4l2-ctl -d[#] --all
    const int32_t INTI_AUTO_EXPOSURE = 1;
    const auto auto_exposure = camera.get_control("Auto Exposure");
    camera.set_control("Auto Exposure", 1);

    std::cout << "Current Gain: " << camera.get_control("Gain") << std::endl;
    std::cout << "Current Exposure Time: " << camera.get_control("Exposure Time, Absolute") << std::endl;
    std::cout << "Current Auto Exposure Mode: " << camera.get_control("Auto Exposure") << std::endl;

    /*
     * Start streaming
     */
    if (!camera.start()) {
    	perror("Failed to start.");
    	exit(0);
    }

    /*
     * Initialize OpenCV
     */
    std::string windowName = camName + " " + camSerialNumber;
    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    cv::Mat colorImg(cv::Size(camFormat.width, camFormat.height), CV_8UC3);
    cv::namedWindow(windowName.c_str());

    /*
     * Main loop
     */
    bool quit = false;
    while (!quit) {
    	/* Copy a single frame(image) from camera(oCam-1MGN). This is a blocking function. */
    	int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);

    	/* If the error occured, restart the camera. */
    	if (size == -1) {
    	    printf("error number: %d\n", errno);
    	    perror("Cannot get image from camera");
    	    camera.stop();
    	    camera.start();
    	    continue;
    	}

		cv::cvtColor(srcImg, colorImg, cv::COLOR_BayerGB2BGR);
    	/* Show image */
    	cv::imshow(windowName.c_str(), colorImg);
    	char key = cv::waitKey(1);

    	/* Keyboard options */
    	switch (key) {
    	/* When press the 'q' key then quit. */
    	case 'q':
    		quit = true;
    		break;

    	/* When press the '[' key then decrease the exposure time. */
    	case '[': {
    		int exposure = camera.get_control("Exposure Time, Absolute");
    		camera.set_control("Exposure Time, Absolute", --exposure);
            std::cout << exposure << std::endl;
    		break;
        }

		/* When press the ']' key then increase the exposure time. */
    	case ']': {
    		int exposure = camera.get_control("Exposure Time, Absolute");
    		camera.set_control("Exposure Time, Absolute", ++exposure);
            std::cout << exposure << std::endl;
    		break;
        }

		/* When press the '-' key then decrease the brightness. */
    	case '-': {
    		int brightness = camera.get_control("Gain");
    		camera.set_control("Gain", --brightness);
            std::cout << brightness << std::endl;
    		break;
        }

		/* When press the '=' key then increase the brightness. */
    	case '=': {
    		int brightness = camera.get_control("Gain");
    		camera.set_control("Gain", ++brightness);
            std::cout << brightness << std::endl;
    		break;
        }

    	default:
    		break;
    	}
    }

    cv::destroyAllWindows();

    /*
     * Stop streaming
     */
    camera.stop();

	printf("Done.\n");

	return 0;
}
