/*!
\file
\brief Заголовочный файл с описанием классов распознавания мишени ERA  
*/
#ifndef ERA_TARGET_RECOGN_HPP
#define ERA_TARGET_RECOGN_HPP

#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include "circle_calculating.hpp"

typedef struct 
{
  float X;
  float Y;
  cv::Point Point;
  float Radius;
  uint32_t Mark;
  uint32_t contour_id;
} V_CIRCLE;

typedef struct {
  uint32_t Mark;
  std::vector<V_CIRCLE> circles;
} MARKGROUP;

typedef struct
{
  float CenterToCenter; //were TargetP
  float HeightAbove; //were TargetH
  float Diameter;
  float key_feature; //Ratio of distance beetween left and right disks of target and their radius. Must be calculated
  float tolerance = 0.1; //Tolerance gets from cosinus of maximum tilt. It (mtilt) is 20 deegrees for ERA, then tolerance is (1 - cos(mtilt)) rounded to tenth
} TARGET;

/*!	
\brief Класс определения центра и радиуса окружности по трём точкам
*/

/*!	
\brief Класс нахождения мишени на изображении
*/
class class_ERA_target_recognizer
{
  public:
    class_ERA_target_recognizer();
    cv::Point3f get_position () {return position;}
    cv::Point3f get_rotation () {return rotation;}
    cv::Point3f to_degrees(cv::Point3f _rotation);
    bool FindTarget (cv::Mat _image);
    cv::Mat CreateAndGetPicture(cv::Mat _image);
    cv::Mat CreateAndGetPicture(cv::Mat _image, int _width, int _height);
    void set_viewer_params (bool _is_image, bool _is_contours, bool _is_circles, bool _is_triangles);
    void setup (cv::Size2d _SizeOfImage, float _FocalLength, cv::Size2f _SizeOfPixel, TARGET _Target);
    void set_SizeOfImage(cv::Size2d _SizeOfImage);
    void set_SizeOfViewer(cv::Size2d _SizeOfViewer);
    void set_FocalLength(float _FocalLength);
    void set_SizeOfPixel(cv::Size2f _SizeOfPixel);
    void set_Target(TARGET _Target);
    void set_is_image(bool _is_image);
    void set_is_contours(bool _is_contours);
    void set_is_circles(bool _is_circles);
    void set_is_triangles(bool _is_triangles);
    void set_thresold(int _threshold);
private:
    const uint16_t PointsInCircleEquation = 3; 
    cv::Point3f position;
    cv::Point3f rotation;
    bool is_image, is_contours, is_circles, is_triangles;
    int threshold;
    V_CIRCLE v_circle;
    std::vector<V_CIRCLE> v_circles;

    MARKGROUP MarkGroup;
    std::vector<MARKGROUP> v_MarkGroups;
    
    std::vector<std::vector<cv::Point> > contours;
    std::vector< cv::Vec4i > hierarchy;

    cv::Size2d SizeOfImage; //were ImageHSize - width, ImageVSize - height;
    cv::Size2d SizeOfViewer;
    float FocalLength; 
    cv::Size2f SizeOfPixel; // were PixelX - width, PixelH - height
    TARGET Target; 
    
    uint32_t l_circle, c_circle, r_circle;
    uint32_t MarkedCluster;

    float tA, tB, tG;
    float MaxTilt, MaxAngSum;
    cv::Mat gray;

    //camera_frame, dst, output, aux1, src, aux2;
    std::tuple<cv::Point3f, cv::Point3f> Calc6TargetCoord(cv::Point Point1, cv::Point Point2, cv::Point Point3);
    float LineLength (float x1, float y1, float x2, float y2);
    uint32_t FindClaster();
    std::tuple<float, float, float, bool> SolveTriangle (float a, float b, float c);    
    void find_circles (int _CentresForFind, int Accuracy, int MinRadius);
    
};
#endif //ERA_TARGET_RECOGN_HPP class_ERA_target_recognizer::