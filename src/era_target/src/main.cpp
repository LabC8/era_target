#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"

#include "era_target_recogn.hpp"

#include "era_target/era_camera.h"

using namespace cv;

static const std::string TOPIC_NAME_IMAGE = "era_camera/rgb/image";
static const std::string TOPIC_NAME_CARTESIAN = "era_camera/cartesian";
static const std::string TOPIC_NAME_FINDED = "era_camera/finded";

enum ERRORS {err_vcap_argument = 1, err_vcap_number, err_vcap_opening};

class_ERA_target_recognizer recognizer;  
TARGET _target;

image_transport::Publisher _image_pub;   
ros::Publisher _cartesian_pub;
ros::Publisher _finded_pub;

std::string gstreamer_pipeline (int camera_id, int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return 
    "nvarguscamerasrc sensor-id=" + std::to_string(camera_id) + 
    // "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + 
    " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + 
    ", height=(int)" + std::to_string(capture_height) + 
    ", framerate=(fraction)" + std::to_string(framerate) +
    "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + 
    " ! video/x-raw, width=(int)" + std::to_string(display_width) + 
    ", height=(int)" + std::to_string(display_height) + 
    ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

// void handler (cv::Mat _image, image_transport::Publisher _image_pub, ros::Publisher _cartesian_pub, ros::Publisher _finded_pub, ros::NodeHandle nh)
void handler (cv::Mat _image)
{
    geometry_msgs::Pose cartesian;
    std_msgs::Bool finded;

    static cv::Point3f previous_position = {0, 0, 0};
    static cv::Point3f previous_rotation = {0, 0, 0};
    // era_target::era_camera era_camera_msg;
    // ros::Publisher publisher = nh.advertise<era_target::era_camera>("era_camera", 20);
    
    // recognizer.setup (cv::Size2d(3264, 2464), FocalLength, cv::Size2f(0.00000112, 0.00000112), _target);
    // recognizer.set_viewer_params (true, true, true, true);

    cv::Mat gray;
    cv::cvtColor(_image, gray, cv::COLOR_BGR2GRAY);
    
    if (recognizer.FindTarget (gray))
    {
        std::cout << "position " << recognizer.get_position() 
            << " rotation " << recognizer.to_degrees(recognizer.get_rotation()) << std::endl;

        previous_position = recognizer.get_position();
        previous_rotation = recognizer.get_rotation();
        cartesian.position.x = recognizer.get_position().x;
        cartesian.position.y = recognizer.get_position().y;
        cartesian.position.z = recognizer.get_position().z;
        cartesian.orientation.x = recognizer.get_rotation().x;
        cartesian.orientation.y = recognizer.get_rotation().y;
        cartesian.orientation.z = recognizer.get_rotation().z;
        finded.data = true;
    }
    else 
    {
        cartesian.position.x = 0;
        cartesian.position.y = 0;
        cartesian.position.z = 0;
        cartesian.orientation.x = 0;
        cartesian.orientation.y = 0;
        cartesian.orientation.z = 0;
        std::cout << "Target wasn't found!" << std::endl;
        finded.data = false;
    }

    sensor_msgs::ImagePtr image_msg;  
    std_msgs::Header hdr;

    hdr.frame_id = "era_target";

    if (!_image.empty())
    {
        cv::Mat viewer = recognizer.CreateAndGetPicture(_image);
        if (!viewer.empty()) 
        {
            image_msg = cv_bridge::CvImage(hdr, "bgr8", viewer).toImageMsg();
            _image_pub.publish (image_msg);
            // std::cout << "Viewer sent" << std::endl;
        }
        // else std::cout << "Viewer is empty" << std::endl;
    }
    // else std::cout << "Image is empty" << std::endl;
    _finded_pub.publish(finded);
    _cartesian_pub.publish(cartesian); 

 /*   if (recognizer.FindTarget (gray))
    {
        std::cout << "position " << recognizer.get_position() 
            << " rotation " << recognizer.to_degrees(recognizer.get_rotation()) << std::endl;

        previous_position = recognizer.get_position();
        previous_rotation = recognizer.get_rotation();
        output_msg.position.x = recognizer.get_position().x;
        output_msg.position.y = recognizer.get_position().y;
        output_msg.position.z = recognizer.get_position().z;
        output_msg.orientation.x = recognizer.get_rotation().x;
        output_msg.orientation.y = recognizer.get_rotation().y;
        output_msg.orientation.z = recognizer.get_rotation().z;
        _finded_pub.publish(_is_finded);
    }
    else 
    {
        output_msg.position.x = 0;
        output_msg.position.y = 0;
        output_msg.position.z = 0;
        output_msg.orientation.x = 0;
        output_msg.orientation.y = 0;
        output_msg.orientation.z = 0;
        std::cout << "Target wasn't found!" << std::endl;
        _is_finded = false;
        _finded_pub.publish(_is_finded);
    }
    _cartesian_pub.publish(output_msg); 

    sensor_msgs::ImagePtr out_msg;  
    std_msgs::Header hdr;

    hdr.frame_id = "era_target";

    cv::Mat viewer = recognizer.CreateAndGetPicture(_image, viewer_width, viewer_height);
    out_msg = cv_bridge::CvImage(hdr, "bgr8", viewer).toImageMsg();
    _image_pub.publish (out_msg);*/
}

void prepare (ros::NodeHandle nh)
{
    int viewer_width, viewer_height;
    int capture_width, capture_height;
    double CenterToCenter, HeightAbove, Diameter;
    double FocalLength;
    double PixelWidth, PixelHeight;
    int threshold;
    bool SendImage, SendContours, SendCircles, SendTriangles;

    nh.param (ros::this_node::getName() + "/InImageWidth", capture_width, 3264);
    nh.param (ros::this_node::getName() + "/InImageHeight", capture_height, 2464);
    nh.param (ros::this_node::getName() + "/OutImageWidth", viewer_width, 1280);
    nh.param (ros::this_node::getName() + "/OutImageHeight", viewer_height, 966);
    nh.param (ros::this_node::getName() + "/TargetCenterToCenter", CenterToCenter, 0.07);
    nh.param (ros::this_node::getName() + "/TargetHeightAbove", HeightAbove, 0.032);
    nh.param (ros::this_node::getName() + "/TargetDiameter", Diameter, 0.022);
    nh.param (ros::this_node::getName() + "/FocalLenght", FocalLength, 0.00296);
    nh.param (ros::this_node::getName() + "/PixelWidth", PixelWidth, 0.00000112);
    nh.param (ros::this_node::getName() + "/PixelHeight", PixelHeight, 0.00000112);
    nh.param (ros::this_node::getName() + "/SendImage", SendImage, false);
    nh.param (ros::this_node::getName() + "/SendContours", SendContours, false);
    nh.param (ros::this_node::getName() + "/SendCircles", SendCircles, false);
    nh.param (ros::this_node::getName() + "/SendTriangles", SendTriangles, false);
    nh.param (ros::this_node::getName() + "/Threshold", threshold, 50);

    _target.CenterToCenter = CenterToCenter;
    _target.HeightAbove = HeightAbove;
    _target.Diameter = Diameter;

    recognizer.set_SizeOfImage (cv::Size2d(capture_width, capture_height));
    recognizer.set_SizeOfViewer (cv::Size2d(viewer_width, viewer_height));
    recognizer.set_FocalLength (FocalLength);
    recognizer.set_SizeOfPixel (cv::Size2f(PixelWidth, PixelHeight));
    recognizer.set_Target (_target);
    recognizer.set_is_image (SendImage);
    recognizer.set_is_contours (SendContours);
    recognizer.set_is_circles (SendCircles);
    recognizer.set_is_triangles (SendTriangles);
    recognizer.set_thresold (threshold);

    image_transport::ImageTransport it(nh);
    _image_pub = it.advertise(TOPIC_NAME_IMAGE, 100);   
    _cartesian_pub = nh.advertise<geometry_msgs::Pose>(TOPIC_NAME_CARTESIAN, 100);
    _finded_pub = nh.advertise<std_msgs::Bool>(TOPIC_NAME_FINDED, 100);
}

int main(int argc, char ** argv)
{
    int framerate;
    int capture_width;
    int capture_height;
    int display_width;
    int display_height;
    int flip_method = 0;

    try
    {
        std::cout << "Started" << std::endl;
        // if (argv[1] == NULL) { throw err_vcap_argument; }
        ros::init(argc, argv, "camera_image_publisher");
        ros::NodeHandle nh;
        std::string s_video_source;
        if (!nh.param (ros::this_node::getName() + "/video_source", s_video_source, std::string("")))
            { throw err_vcap_argument; }
        // if (!nh.getParam (ros::this_node::getName() + "/video_source", s_video_source))
        //     { throw err_vcap_argument; }
        // std::istringstream video_sourceCmd(argv[1]);
        std::istringstream video_sourceCmd(s_video_source);
        int video_source;

        nh.param (ros::this_node::getName() + "/InImageWidth", capture_width, 3264);
        nh.param (ros::this_node::getName() + "/InImageHeight", capture_height, 2464);
        nh.param (ros::this_node::getName() + "/FrameRate", framerate, 21);
        display_width = capture_width;
        display_height = capture_height;

        if (!(video_sourceCmd >> video_source)) { throw err_vcap_number; }
        std::string pipeline = gstreamer_pipeline(
            video_source,
            capture_width,
            capture_height,
            display_width,
            display_height,
            framerate,
            flip_method);

        std::cout << "Using pipeline: \n\t" << pipeline << "\n";

        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
        if (!cap.isOpened()) { throw err_vcap_opening; }
        
        cv::Mat frame;
        std_msgs::Header hdr;
        sensor_msgs::ImagePtr msg;   
        
        // image_transport::ImageTransport it(nh);

        // image_transport::Publisher image_pub = it.advertise(TOPIC_NAME_IMAGE, 20);   
        // ros::Publisher cartesian_pub = nh.advertise<geometry_msgs::Pose>(TOPIC_NAME_CARTESIAN, 20);
        // ros::Publisher finded_pub = nh.advertise<bool>(TOPIC_NAME_FINDED, 20);

        ros::Rate loop_rate(framerate);
        std::cout << "Main loop... " << std::endl;
        prepare (nh);
        while (nh.ok())
        {
            cap >> frame;
            // Check if grabbed frame is actually full with some content
            if (!frame.empty()) 
            {
                // std::cout << "Sent" << std::endl;
                // ROS_INFO ("Sent");
                // msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
                // pub.publish(msg);
                // handler (frame, image_pub, cartesian_pub, finded_pub, nh);
                handler (frame);
                cv::waitKey(1);
            }
            loop_rate.sleep();
        }
        std::cout << "... is finished " << std::endl;
    }
    catch (ERRORS e)
    {
        switch (e)
        {
            case err_vcap_argument:
                std::cout << "Check if video source has been passed as a parameter" << std::endl;
            break;
            case err_vcap_number:
                std::cout << "Check if videosource is indeed a number" << std::endl;
            break;
            case err_vcap_opening:
                std::cout << "Check if video device can be opened with the given index" << std::endl;
            break;
            default: std::cout << "What the fuck happened" << std::endl;
        }
        return e;
    }
}