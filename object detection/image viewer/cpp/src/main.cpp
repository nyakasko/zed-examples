///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*****************************************************************************************
 ** This sample demonstrates how to detect objects and retrieve their 3D position **
 **         with the ZED SDK and display the result in an OpenGL window.                **
 *****************************************************************************************/

 // ZED includes
#include <sl/Camera.hpp>
// Sample includes
#include "GLViewer.hpp"
 // Standard includes
#include <iostream>
#include <fstream>
#include <chrono>
// OpenCV includes
#include <opencv2/opencv.hpp>
// OpenCV dep
#include <opencv2/cvconfig.h>
#include <boost/filesystem.hpp>
// Using std and sl namespaces
using namespace std;
using namespace sl;

cv::Mat slMat2cvMat(Mat& input);
#ifdef HAVE_CUDA
cv::cuda::GpuMat slMat2cvMatGPU(Mat& input);
#endif // HAVE_CUDA

void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");
void parseArgs(int argc, char **argv, InitParameters& param);

int count_files(std::string directory, std::string ext)
{
	namespace fs = boost::filesystem;
	fs::path Path(directory);
	int Nb_ext = 0;
	fs::directory_iterator end_iter; // Default constructor for an iterator is the end iterator

	for (fs::directory_iterator iter(Path); iter != end_iter; ++iter)
		if (iter->path().extension() == ext)
			++Nb_ext;

	return Nb_ext;
}

int main(int argc, char **argv) {

#ifdef _SL_JETSON_
	const bool isJetson = true;
#else
	const bool isJetson = false;
#endif

	// Create ZED objects
	Camera zed;
	InitParameters init_parameters;
	init_parameters.camera_resolution = RESOLUTION::HD2K;
	// On Jetson the object detection combined with an heavy depth mode could reduce the frame rate too much
	init_parameters.depth_mode = isJetson ? DEPTH_MODE::PERFORMANCE : DEPTH_MODE::ULTRA;
	init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	init_parameters.coordinate_units = UNIT::METER;
	parseArgs(argc, argv, init_parameters);

	// Open the camera
	auto returned_state = zed.open(init_parameters);
	if (returned_state != ERROR_CODE::SUCCESS) {
		print("Open Camera", returned_state, "\nExit program.");
		zed.close();
		return EXIT_FAILURE;
	}
	// Set runtime parameters after opening the camera
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;

	// Enable Positional tracking (mandatory for object detection)
	PositionalTrackingParameters positional_tracking_parameters;
	//If the camera is static, uncomment the following line to have better performances and boxes sticked to the ground.
	//positional_tracking_parameters.set_as_static = true;
	returned_state = zed.enablePositionalTracking(positional_tracking_parameters);
	if (returned_state != ERROR_CODE::SUCCESS) {
		print("enable Positional Tracking", returned_state, "\nExit program.");
		zed.close();
		return EXIT_FAILURE;
	}

	// Enable the Objects detection module
	ObjectDetectionParameters obj_det_params;
	obj_det_params.enable_tracking = true;
	obj_det_params.detection_model = DETECTION_MODEL::MULTI_CLASS_BOX_ACCURATE;
	obj_det_params.image_sync = true;
	returned_state = zed.enableObjectDetection(obj_det_params);
	if (returned_state != ERROR_CODE::SUCCESS) {
		print("enable Object Detection", returned_state, "\nExit program.");
		zed.close();
		return EXIT_FAILURE;
	}
	Resolution image_size = zed.getCameraInformation().camera_resolution;
	int new_width = image_size.width;
	int new_height = image_size.height;
	Resolution new_image_size(new_width, new_height);
	auto camera_info = zed.getCameraInformation().camera_configuration;
	camera_info.calibration_parameters.left_cam.image_size = new_image_size;
	// Create OpenGL Viewer
	GLViewer viewer;
	viewer.init(argc, argv, camera_info.calibration_parameters.left_cam);

	// Configure object detection runtime parameters
	ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
	objectTracker_parameters_rt.detection_confidence_threshold = 20;
	// To select a set of specific object classes, like persons, vehicles and animals for instance:
	//objectTracker_parameters_rt.object_class_filter = {OBJECT_CLASS::PERSON /*, OBJECT_CLASS::VEHICLE, OBJECT_CLASS::ANIMAL*/ };
	// To set a specific threshold
	objectTracker_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::PERSON] = 65;
	//detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::CAR] = 35;

	// Create ZED Objects filled in the main loop
	Objects objects;
	char key = ' ';
	// Main Loop
	bool need_floor_plane = positional_tracking_parameters.set_as_static;

	// To share data between sl::Mat and cv::Mat, use slMat2cvMat()
	// Only the headers and pointer to the sl::Mat are copied, not the data itself
	Mat image(new_width, new_height, MAT_TYPE::U8_C4);
	Mat zed_image(new_width, new_height, MAT_TYPE::U8_C4);
	cv::Mat image_ocv = slMat2cvMat(zed_image);
	int count_save = count_files("D:/zed codes/zed-examples/object detection/image viewer/cpp/images/", ".png");
	while (viewer.isAvailable()) {
		cin.ignore();
		//key = cv::waitKey();
		// Grab images
		if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {

			// Retrieve left image
			zed.retrieveImage(image, VIEW::LEFT, MEM::GPU, new_image_size);
			zed.retrieveImage(zed_image, VIEW::LEFT, MEM::CPU, new_image_size);
			// Retrieve objects
			std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
			zed.retrieveObjects(objects, objectTracker_parameters_rt);
			std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
			std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
			// cv::imshow("proba", image_ocv);
			string filename = "D:/zed codes/zed-examples/object detection/image viewer/cpp/images/left_img" + to_string(count_save) + string(".png");
			string detection_name = "D:/zed codes/zed-examples/object detection/image viewer/cpp/detections/left_img" + to_string(count_save) + string(".txt");
			ofstream detection_file(detection_name);
			count_save++;
			auto state = zed_image.write(filename.c_str());

			for (int index = 0; index < objects.object_list.size(); index++) {
				cout << objects.object_list.at(index).label << " " << objects.object_list.at(index).sublabel << endl;
				cout << " Bounding Box 2D \n";
				for (auto it : objects.object_list.at(index).bounding_box_2d)
					cout << "    " << it << "\n";
				cout << " Confidence:   " << objects.object_list.at(index).confidence << "\n";
			}
			if (state == sl::ERROR_CODE::SUCCESS) {
				cout << "Left image has been save under " << filename << endl;
				for (int index = 0; index < objects.object_list.size(); index++) {
					detection_file << objects.object_list.at(index).sublabel << " ." << int(objects.object_list.at(index).confidence) << " " <<
						objects.object_list.at(index).bounding_box_2d[0] << " " << objects.object_list.at(index).bounding_box_2d[2] << endl;
				}
				cout << "Detections has been save under " << detection_name << endl;
				detection_file.close();
			}
			else {
				cout << "Failed to save image... Please check that you have permissions to write at this location (" << filename << "). Re-run the sample with administrator rights under windows" << endl;
			}
			//Update GL View
			viewer.updateView(image, objects);
		}
	}

	// Release objects
	image.free();
	objects.object_list.clear();

	// Disable modules
	zed.disableObjectDetection();
	zed.disablePositionalTracking();
	zed.close();
	return EXIT_SUCCESS;
}

void parseArgs(int argc, char **argv, InitParameters& param) {
	if (argc > 1 && string(argv[1]).find(".svo") != string::npos) {
		// SVO input mode
		param.input.setFromSVOFile(argv[1]);
		cout << "[Sample] Using SVO File input: " << argv[1] << endl;
	}
	else if (argc > 1 && string(argv[1]).find(".svo") == string::npos) {
		string arg = string(argv[1]);
		unsigned int a, b, c, d, port;
		if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
			// Stream input mode - IP + port
			string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
			param.input.setFromStream(String(ip_adress.c_str()), port);
			cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
		}
		else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
			// Stream input mode - IP only
			param.input.setFromStream(String(argv[1]));
			cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
		}
		else if (arg.find("HD2K") != string::npos) {
			param.camera_resolution = RESOLUTION::HD2K;
			cout << "[Sample] Using Camera in resolution HD2K" << endl;
		}
		else if (arg.find("HD1080") != string::npos) {
			param.camera_resolution = RESOLUTION::HD1080;
			cout << "[Sample] Using Camera in resolution HD1080" << endl;
		}
		else if (arg.find("HD720") != string::npos) {
			param.camera_resolution = RESOLUTION::HD720;
			cout << "[Sample] Using Camera in resolution HD720" << endl;
		}
		else if (arg.find("VGA") != string::npos) {
			param.camera_resolution = RESOLUTION::VGA;
			cout << "[Sample] Using Camera in resolution VGA" << endl;
		}
	}
}

void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix) {
	cout << "[Sample]";
	if (err_code != ERROR_CODE::SUCCESS)
		cout << "[Error]";
	cout << " " << msg_prefix << " ";
	if (err_code != ERROR_CODE::SUCCESS) {
		cout << " | " << toString(err_code) << " : ";
		cout << toVerbose(err_code);
	}
	if (!msg_suffix.empty())
		cout << " " << msg_suffix;
	cout << endl;
}

// Mapping between MAT_TYPE and CV_TYPE
int getOCVtype(sl::MAT_TYPE type) {
	int cv_type = -1;
	switch (type) {
	case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
	case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
	case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
	case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
	case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
	case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
	case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
	case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
	default: break;
	}
	return cv_type;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}
#ifdef HAVE_CUDA
/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::cuda::GpuMat slMat2cvMatGPU(Mat& input) {
	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::cuda::GpuMat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::GPU), input.getStepBytes(sl::MEM::GPU));
}
#endif