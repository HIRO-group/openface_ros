#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/fast_math.hpp>
#include <sensor_msgs/image_encodings.h>

#include <OpenFace/LandmarkCoreIncludes.h>
#include <OpenFace/GazeEstimation.h>
#include <OpenFace/FaceAnalyserParameters.h>
#include <OpenFace/FaceAnalyser.h>
#include <OpenFace/SequenceCapture.h>
#include <OpenFace/Visualizer.h>
#include <OpenFace/VisualizationUtils.h>

class OpenFaceRos
{
private:
    std::vector<std::string> args;
    ros::NodeHandle n;
    image_transport::ImageTransport it;

    LandmarkDetector::FaceModelParameters det_parameters;
    LandmarkDetector::CLNF face_model;
    FaceAnalysis::FaceAnalyserParameters face_analysis_params;
    FaceAnalysis::FaceAnalyser face_analyser;
    Utilities::Visualizer visualizer;

    ros::Publisher head_status_pub;
    ros::Publisher gripper_status_pub;
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    image_transport::Subscriber color_image_sub;
    image_transport::Subscriber depth_image_sub;
    cv_bridge::CvImagePtr cv_depth_ptr;

    bool cv_depth_valid;
    float depth_left, depth_right;
    double fx;
    double fy;
    double cx;
    double cy;
    double threshold;

    void colorCb(const sensor_msgs::ImageConstPtr& msg);
    void depthCb(const sensor_msgs::ImageConstPtr& msg);
    void calculatePupil(cv::Point3f& pupil_left, cv::Point3f& pupil_right, const std::vector<cv::Point3f>& eye_landmarks3d);
    void Project(cv::Mat_<float>& dest, const cv::Mat_<float>& mesh, float _fx, float _fy, float _cx, float _cy);

    std::vector<float> realDistanceTransform(float distance_x, float distance_y, float depth);
    std::vector<std::string> get_arguments(int argc, char **argv);

    double distanceOfPointToLine(std::vector<float> a, std::vector<float> b, std::vector<float> s);

    unsigned short getMedianDepth(int row, int col);

    cv::Matx33f Euler2RotationMatrix(const cv::Vec3f& eulerAngles);


public:
    OpenFaceRos(std::string name, double fx, double fy, double cx, double cy, double threshold);
    ~OpenFaceRos();
    
    // void move(moveit_msgs::RobotTrajectory& traj);
};