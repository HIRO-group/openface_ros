#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <OpenFace/LandmarkCoreIncludes.h>
#include <OpenFace/GazeEstimation.h>
#include <OpenFace/FaceAnalyserParameters.h>
#include <OpenFace/FaceAnalyser.h>
#include <OpenFace/SequenceCapture.h>
#include <OpenFace/Visualizer.h>
#include <OpenFace/VisualizationUtils.h>

#define fx 615.6707153320312
#define fy 615.962158203125
#define cx 328.0010681152344
#define cy 241.31031799316406


static const std::string DEPTH_OPENCV_WINDOW = "Depth Image";

std::shared_ptr<LandmarkDetector::FaceModelParameters> det_parameters;
std::shared_ptr<LandmarkDetector::CLNF> face_model;
std::shared_ptr<FaceAnalysis::FaceAnalyserParameters> face_analysis_params;
std::shared_ptr<FaceAnalysis::FaceAnalyser> face_analyser;
std::shared_ptr<Utilities::Visualizer> visualizer;
Utilities::FpsTracker fps_tracker;


void colorCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_color_ptr;
    try
    {
        cv_color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat grayscale_image;
    cv::Mat rgb_image = cv_color_ptr->image;
    cv::cvtColor(rgb_image, grayscale_image, cv::COLOR_BGR2GRAY);
    bool detection_success = LandmarkDetector::DetectLandmarksInVideo(rgb_image, *face_model, *det_parameters, grayscale_image);
    cv::Point3f gazeDirection0(0, 0, -1);
    cv::Point3f gazeDirection1(0, 0, -1);
    // If tracking succeeded and we have an eye model, estimate gaze
    if (detection_success && face_model->eye_model)
    {
        GazeAnalysis::EstimateGaze(*face_model, gazeDirection0, fx, fy, cx, cy, true);
        GazeAnalysis::EstimateGaze(*face_model, gazeDirection1, fx, fy, cx, cy, false);
    }

    cv::Mat sim_warped_img;
    cv::Mat_<double> hog_descriptor; int num_hog_rows = 0, num_hog_cols = 0;    
    face_analyser->AddNextFrame(rgb_image, face_model->detected_landmarks, face_model->detection_success, ros::Time::now().toSec(), true);
    face_analyser->GetLatestAlignedFace(sim_warped_img);
    face_analyser->GetLatestHOG(hog_descriptor, num_hog_rows, num_hog_cols);

    // Work out the pose of the head from the tracked model
    cv::Vec6d pose_estimate = LandmarkDetector::GetPose(*face_model, fx, fy, cx, cy);

    // Displaying the tracking visualizations
    visualizer->SetImage(rgb_image, fx, fy, cx, cy);
    visualizer->SetObservationLandmarks(face_model->detected_landmarks, face_model->detection_certainty, face_model->GetVisibilities());
    visualizer->SetObservationPose(pose_estimate, face_model->detection_certainty);
    visualizer->SetObservationGaze(gazeDirection0, gazeDirection1, LandmarkDetector::CalculateAllEyeLandmarks(*face_model), LandmarkDetector::Calculate3DEyeLandmarks(*face_model, fx, fy, cx, cy), face_model->detection_certainty);
    visualizer->SetObservationActionUnits(face_analyser->GetCurrentAUsReg(), face_analyser->GetCurrentAUsClass());
    char character_press = visualizer->ShowObservation();
    // restart the tracker
    if (character_press == 'r')
    {
        face_model->Reset();
    }
    // // display AU
    // for (auto au : face_analyser->GetCurrentAUsClass())
    // {
    //     std::cout << au.first << ": " << au.second << std::endl;
    // }

}

void depthCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_depth_ptr;
    cv::namedWindow(DEPTH_OPENCV_WINDOW);
    try
    {
        cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow(DEPTH_OPENCV_WINDOW, cv_depth_ptr->image);
    cv::waitKey(1);
}

vector<string> get_arguments(int argc, char **argv)
{
    std::vector<std::string> arguments;
    for (int i = 0; i < argc; ++i)
    {
        arguments.push_back(std::string(argv[i]));
    }
    return arguments;
}


int main(int argc, char** argv)
{
    vector<string> arguments = get_arguments(argc, argv);
    ros::init(argc, argv, "openface_ros");

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber color_image_sub = it.subscribe("/camera/color/image_raw", 1, colorCb);
    image_transport::Subscriber depth_image_sub = it.subscribe("/camera/depth/image_rect_raw", 1, depthCb);

    det_parameters = std::make_shared<LandmarkDetector::FaceModelParameters>(arguments);
    face_model = std::make_shared<LandmarkDetector::CLNF>(det_parameters->model_location);
    face_analysis_params = std::make_shared<FaceAnalysis::FaceAnalyserParameters>(arguments);
    face_analyser = std::make_shared<FaceAnalysis::FaceAnalyser>(*face_analysis_params);
    if (!face_model->loaded_successfully)
    {
        std::cout << "ERROR: Could not load the landmark detector" << std::endl;
        return 1;
    }

    if (!face_model->eye_model)
    {
        std::cout << "WARNING: no eye model found" << std::endl;
    }

    visualizer = std::make_shared<Utilities::Visualizer>(true, false, false, false);

    ros::spin();

    return 0;
}
