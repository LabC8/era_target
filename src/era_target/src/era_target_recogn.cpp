/*!
\file
\brief Файл с текстами функций классов распознавания мишени ERA  
*/
#include "era_target_recogn.hpp"

class_ERA_target_recognizer::class_ERA_target_recognizer()
{
  is_image = false, is_contours = false, is_circles = false, is_triangles = false; 
  MaxTilt = 20. / (180 / M_PI); //FOM 14. Figure 2.5.8.2-2  Location of Target in FoV of the EE CLU for Proximity Operations
  MaxAngSum = 22. / (180 / M_PI);
}

void class_ERA_target_recognizer::setup(cv::Size2d _SizeOfImage,
  float _FocalLength,
  cv::Size2f _SizeOfPixel,
  TARGET _Target
)
{
  SizeOfImage = _SizeOfImage; 
  FocalLength = _FocalLength; 
  SizeOfPixel.width = _SizeOfPixel.width;
  SizeOfPixel.height = _SizeOfPixel.height;  
  Target = _Target; 
// int SizeOfImage.width = 3264, SizeOfImage.height = 2464;
// float FocalLength = 0.00296; //From documentation
// float SizeOfPixel.width = 0.00000112, SizeOfPixel.height = 0.00000112;
  // float Target.CenterToCenter = 0.07, Target.HeightAbove = 0.032;
}

void class_ERA_target_recognizer::set_SizeOfImage(cv::Size2d _SizeOfImage)
{
  SizeOfImage = _SizeOfImage; 
}

void class_ERA_target_recognizer::set_SizeOfViewer(cv::Size2d _SizeOfViewer)
{
  SizeOfViewer = _SizeOfViewer; 
}

void class_ERA_target_recognizer::set_FocalLength(float _FocalLength)
{
  FocalLength = _FocalLength; 
}

void class_ERA_target_recognizer::set_SizeOfPixel(cv::Size2f _SizeOfPixel)
{
  SizeOfPixel.width = _SizeOfPixel.width;
  SizeOfPixel.height = _SizeOfPixel.height;  
}

void class_ERA_target_recognizer::set_Target(TARGET _Target)
{
  Target = _Target; 
  Target.key_feature = (2. * Target.CenterToCenter) / (Target.Diameter / 2.);
}

void class_ERA_target_recognizer::set_is_image(bool _is_image)
{
  is_image = _is_image;
}

void class_ERA_target_recognizer::set_is_contours(bool _is_contours)
{
  is_contours = _is_contours;
}

void class_ERA_target_recognizer::set_is_circles(bool _is_circles)
{
  is_circles = _is_circles;
}

void class_ERA_target_recognizer::set_is_triangles(bool _is_triangles)
{
  is_triangles = _is_triangles;
}

void class_ERA_target_recognizer::set_thresold(int _threshold)
{
  threshold = _threshold;
}

void class_ERA_target_recognizer::set_viewer_params(bool _is_image, bool _is_contours, bool _is_circles, bool _is_triangles)
{
  is_image = _is_image;
  is_contours = _is_contours;
  is_circles = _is_circles;
  is_triangles = _is_triangles;
}

cv::Point3f class_ERA_target_recognizer::to_degrees(cv::Point3f _rotation)
{
  return cv::Point3f(_rotation.x * (180. / M_PI), _rotation.y * (180. / M_PI), _rotation.z * (180. / M_PI));
}

std::tuple<cv::Point3f, cv::Point3f> class_ERA_target_recognizer::Calc6TargetCoord(cv::Point PLeft, cv::Point PCentral, cv::Point PRight)
{
  float ImageH = SizeOfPixel.width * SizeOfImage.width, ImageV = SizeOfPixel.height * SizeOfImage.height;
  float ImageCh = ImageH / 2, ImageCv = ImageV / 2;
  cv::Point3f position = {0,0,0}, rotation = {0,0,0};
  Eigen::MatrixXd Acal (6, 6);
  Eigen::VectorXd Bcal (6);
  Eigen::VectorXd Xcal (6);
  float Im1x, Im2x, Im3x, Im1y, Im2y, Im3y;

//  Acal.setZero();
  Im1x = - (PCentral.x * SizeOfPixel.width - ImageCh);
  Im2x = - (PLeft.x * SizeOfPixel.width - ImageCh);
  Im3x = - (PRight.x * SizeOfPixel.width - ImageCh);
  Im1y = - (PCentral.y * SizeOfPixel.height - ImageCv);
  Im2y = - (PLeft.y * SizeOfPixel.height - ImageCv);
  Im3y = - (PRight.y * SizeOfPixel.height - ImageCv);

  Acal <<	
  FocalLength, 	0, 	FocalLength, 0, FocalLength, 0, 
	0, FocalLength, 0, FocalLength, 0, FocalLength, 
	Im1x, Im1y, Im2x, Im2y, Im3x, Im3y, 
  0, -FocalLength * Target.HeightAbove, 0, 0, 0,	0, 
	FocalLength * Target.HeightAbove, 0, -Im2x * Target.CenterToCenter, -Im2y * Target.CenterToCenter, Im3x * Target.CenterToCenter, Im3y * Target.CenterToCenter, 
	0, 0, 0, - FocalLength * Target.CenterToCenter, 0, FocalLength * Target.CenterToCenter;

  Bcal << 
  Im1x * (FocalLength + Target.HeightAbove),
  Im1y * (FocalLength + Target.HeightAbove),
  FocalLength * (Im2x + Target.CenterToCenter),
  FocalLength * Im2y,
  - FocalLength * (Target.CenterToCenter - Im3x),
  FocalLength * Im3y;

  Xcal = (Acal.transpose()).colPivHouseholderQr().solve(Bcal);
  position.x = Xcal[0];
  position.y = Xcal[1];
  position.z = Xcal[2];
  rotation.x = Xcal[3];
  rotation.y = Xcal[4];
  rotation.z = Xcal[5];

  return {position, rotation};
}

std::tuple<float, float, float, bool> class_ERA_target_recognizer::SolveTriangle(float a, float b, float c)
{
  float p = (a + b + c) / 2;
  if (p <= 0) return {0, 0, 0, true};
  float r = (p - a) * (p - b) * (p - c) / p;
  if (r < 0) return {0, 0, 0, true};
  r = sqrt (r);
  return {2 * atan2 (r, p - a), 
          2 * atan2 (r, p - b),
          2 * atan2 (r, p - c), 
          false};
}

float class_ERA_target_recognizer::LineLength(float x1, float y1, float x2, float y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

void class_ERA_target_recognizer::find_circles (int CentresForFind, int Accuracy, int MinRadius) 
{
  int increm;
  int coinside;
  double C_X, C_Y;
  double SRadius = 0;
  std::vector<CvPoint2D32f> Centres;
  CvPoint P1, P2, P3;
  CvPoint2D32f Center;

  v_circles.clear();
  if (CentresForFind <= 0) return;
  Circle * crcl = new Circle ();
  // ROS_INFO ("Try to find circles in %ld contours", contours.size());
  for (uint32_t i = 0; i < contours.size(); i++) 
  {
    //ROS_INFO ("The contour has %d points", contours[i].size());
    if(hierarchy[i][2] < 0) /// Skip non closed contour
      continue;
    if (contours[i].size() / PointsInCircleEquation < CentresForFind) /// For every center cirle needs so many points
    {
      //ROS_INFO ("but we need more then %d points", CentresForFind * PointsInCircleEquation);
      continue;
    }
    coinside = 0;
    Centres.clear();
    // ROS_INFO ("Lets find circles");

    SRadius = 0;
    C_X = 0;
    C_Y = 0;

    increm = (contours[i].size() / PointsInCircleEquation) / CentresForFind;
    for (int j = 0; j < CentresForFind; j++) {
      P1 = contours[i][j * increm];
      P2 = contours[i][j * increm + contours[i].size() / PointsInCircleEquation];
      P3 = contours[i][j * increm + 2 * contours[i].size() / PointsInCircleEquation];
      crcl->SetCircle (&P1, &P2, &P3);
      Center = crcl->GetCenter();
      if (crcl->GetRadius() >= 0)
        SRadius += crcl->GetRadius();
      else
      {
        SRadius = -1;
        break;
      }
      Centres.push_back(Center);
    }
    if (SRadius < 0) 
    {
      Centres.clear();
      continue;
    }
    for (int j = 0; j < CentresForFind; j++) 
    {
      if (fabs(Centres[j].x - Centres[0].x) < Accuracy && fabs(Centres[j].y - Centres[0].y) < Accuracy) {
        C_X += Centres[j].x;
        C_Y += Centres[j].y;
        coinside++;
      } else break;
    }
    // ROS_INFO ("In contour number %d %d centers coinside", i, coinside + 1);
    if (coinside >= CentresForFind) {
      v_circle.Mark = 0;
      v_circle.Point.x = v_circle.X = C_X / CentresForFind;
      v_circle.Point.y = v_circle.Y = C_Y / CentresForFind;
      v_circle.Radius = SRadius / CentresForFind;//(int)crcl->GetRadius();
      v_circle.contour_id = i;
      if (v_circle.Radius >= MinRadius) 
      {
        // ROS_INFO("Entitle it the circle with the radius %f = (%f/%d)", v_circle.Radius, SRadius, CentresForFind);
        v_circles.push_back(v_circle);
      }
      else
      {
        // ROS_INFO("But it's radius is %f = (%f/%d)", v_circle.Radius, SRadius, CentresForFind);
      }

    }
  }
//  for (int i = 0; i < v_circles.size(); i++)
//    Form1->FStatus1->Memo1->Lines->Add (Format ("(%d,%d,%d)", ARRAYOFCONST((v_circles[i].X, v_circles[i].Y, v_circles[i].Radius))));
  delete crcl;
}

unsigned class_ERA_target_recognizer::FindClaster()
{
uint32_t Mark = 0;
float R;
float K;
  v_MarkGroups.clear();
  for (uint32_t i = 0; i < v_circles.size(); i++) 
  {
    if (v_circles[i].Mark) continue;
    v_circles[i].Mark = ++Mark;
    MarkGroup.circles.clear();
    MarkGroup.Mark = Mark;
    MarkGroup.circles.push_back(v_circles[i]);
    for (uint32_t j = i + 1; j < v_circles.size(); j++) {
      //if (j == i) continue;
      if (v_circles[i].Radius / v_circles[j].Radius >= 0.85 &&
          v_circles[i].Radius / v_circles[j].Radius <= 1.15) {
        v_circles[j].Mark = Mark;
        MarkGroup.circles.push_back(v_circles[j]);
      }
    }
    if (MarkGroup.circles.size() >= 3) v_MarkGroups.push_back(MarkGroup);
  }

  Mark = 0;
  float Tilt;

  for (uint32_t i = 0; i < v_MarkGroups.size(); i++) 
  {
    for (uint32_t i0 = 0; i0 < v_MarkGroups[i].circles.size(); i0++) 
    {
      for (uint32_t i1 = i0 + 1; i1 < v_MarkGroups[i].circles.size(); i1++) 
      {
        for (uint32_t i2 = i1 + 1; i2 < v_MarkGroups[i].circles.size(); i2++) 
        {
          if ((v_MarkGroups[i].circles[i0].X < v_MarkGroups[i].circles[i1].X) &&
              (v_MarkGroups[i].circles[i0].X < v_MarkGroups[i].circles[i2].X)) 
          {
              l_circle = i0;
              if (v_MarkGroups[i].circles[i1].X < v_MarkGroups[i].circles[i2].X) 
              {
                c_circle = i1;
                r_circle = i2;
              } else {
                c_circle = i2;
                r_circle = i1;
              }
          }
          if ((v_MarkGroups[i].circles[i1].X < v_MarkGroups[i].circles[i0].X) &&
              (v_MarkGroups[i].circles[i1].X < v_MarkGroups[i].circles[i2].X)) 
          {
              l_circle = i1;
              if (v_MarkGroups[i].circles[i0].X < v_MarkGroups[i].circles[i2].X) 
              {
                c_circle = i0;
                r_circle = i2;
              } else {
                c_circle = i2;
                r_circle = i0;
              }
          }
          if ((v_MarkGroups[i].circles[i2].X < v_MarkGroups[i].circles[i0].X) &&
              (v_MarkGroups[i].circles[i2].X < v_MarkGroups[i].circles[i1].X)) 
          {
              l_circle = i2;
              if (v_MarkGroups[i].circles[i0].X < v_MarkGroups[i].circles[i1].X) 
              {
                c_circle = i0;
                r_circle = i1;
              } else {
                c_circle = i1;
                r_circle = i0;
              }
          }
          R = v_MarkGroups[i].circles[l_circle].Radius +
              v_MarkGroups[i].circles[c_circle].Radius +
              v_MarkGroups[i].circles[r_circle].Radius;
          K = sqrt((v_MarkGroups[i].circles[l_circle].X - v_MarkGroups[i].circles[r_circle].X) *
                   (v_MarkGroups[i].circles[l_circle].X - v_MarkGroups[i].circles[r_circle].X) +
                   (v_MarkGroups[i].circles[l_circle].Y - v_MarkGroups[i].circles[r_circle].Y) *
                   (v_MarkGroups[i].circles[l_circle].Y - v_MarkGroups[i].circles[r_circle].Y)) / (R / 3.);

          // if (K > 12.5 && K < 15.5) 
          if ( (K > (Target.key_feature * (1 - Target.tolerance))) && (K < Target.key_feature * (1 + Target.tolerance)) ) 
          {
            float tA, tB, tG, error;
            std::tie(tA, tB, tG, error) = 
            SolveTriangle (LineLength(v_MarkGroups[i].circles[l_circle].X,
                                      v_MarkGroups[i].circles[l_circle].Y,
                                      v_MarkGroups[i].circles[c_circle].X,
                                      v_MarkGroups[i].circles[c_circle].Y),
                           LineLength(v_MarkGroups[i].circles[c_circle].X,
                                      v_MarkGroups[i].circles[c_circle].Y,
                                      v_MarkGroups[i].circles[r_circle].X,
                                      v_MarkGroups[i].circles[r_circle].Y),
                           LineLength(v_MarkGroups[i].circles[r_circle].X,
                                      v_MarkGroups[i].circles[r_circle].Y,
                                      v_MarkGroups[i].circles[l_circle].X,
                                      v_MarkGroups[i].circles[l_circle].Y));
            if (error) continue;
            if ((tA + tB < MaxAngSum) || (tB + tG < MaxAngSum) || (tA + tG < MaxAngSum)) 
            {
              Tilt = atan2 (fabs(v_MarkGroups[i].circles[l_circle].Y - v_MarkGroups[i].circles[r_circle].Y),
                              fabs(v_MarkGroups[i].circles[l_circle].X - v_MarkGroups[i].circles[r_circle].X));
              if (fabs(Tilt) < MaxTilt) 
              {
                Mark = v_MarkGroups[i].Mark;
                return Mark;
              }
            }
          }
        }
      }
    }
  }
  return Mark;
}

cv::Mat class_ERA_target_recognizer::CreateAndGetPicture(cv::Mat _image)
{
  return CreateAndGetPicture (_image, SizeOfViewer.width, SizeOfViewer.height);
}

cv::Mat class_ERA_target_recognizer::CreateAndGetPicture(cv::Mat _image, int _width, int _height)
{
  cv::Mat drawing;
  if (is_image) drawing = _image;
  else drawing = cv::Mat::zeros(_image.size(), CV_8UC3 );

  if (is_contours)
  {
    cv::RNG rng;
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color_cl = cv::Scalar( 0, 255, 255 );
        cv::Scalar color_noncl = cv::Scalar( 139, 219, 255);
        if(hierarchy[i][2]<0)
          drawContours( drawing, contours, (int)i, color_noncl, 1);
        else
          drawContours( drawing, contours, (int)i, color_cl, 2);
    }
  }
  if (is_circles)
  {
    cv::Scalar color = cv::Scalar( 0, 0, 255 );      
    for( size_t i = 0; i < v_circles.size(); i++ )
      cv::circle (drawing, v_circles[i].Point, v_circles[i].Radius, color, 2);
  }
  if (is_triangles)
  {
    if (MarkedCluster) 
    {
      for (unsigned i = 0; i < v_MarkGroups.size(); i++) 
      {
        if (v_MarkGroups[i].Mark == MarkedCluster) 
        {
          cv::line ( drawing,
                    cvPoint(cvRound(v_MarkGroups[i].circles[l_circle].X), cvRound(v_MarkGroups[i].circles[l_circle].Y)),
                    cvPoint(cvRound(v_MarkGroups[i].circles[c_circle].X), cvRound(v_MarkGroups[i].circles[c_circle].Y)),
                    CV_RGB(0,255,0),
                    1, 8, 0 );
          cv::line ( drawing,
                    cvPoint(cvRound(v_MarkGroups[i].circles[c_circle].X), cvRound(v_MarkGroups[i].circles[c_circle].Y)),
                    cvPoint(cvRound(v_MarkGroups[i].circles[r_circle].X), cvRound(v_MarkGroups[i].circles[r_circle].Y)),
                    CV_RGB(0,255,0),
                    1, 8, 0 );
          cv::line ( drawing,
                    cvPoint(cvRound(v_MarkGroups[i].circles[r_circle].X), cvRound(v_MarkGroups[i].circles[r_circle].Y)),
                    cvPoint(cvRound(v_MarkGroups[i].circles[l_circle].X), cvRound(v_MarkGroups[i].circles[l_circle].Y)),
                    CV_RGB(0,255,0),
                    1, 8, 0 );

          cv::circle( drawing, cvPoint(cvRound(v_MarkGroups[i].circles[l_circle].X), cvRound(v_MarkGroups[i].circles[l_circle].Y)), 3, CV_RGB(0,255,0), -1, 8, 0 );
          cv::circle( drawing, cvPoint(cvRound(v_MarkGroups[i].circles[l_circle].X), cvRound(v_MarkGroups[i].circles[l_circle].Y)), cvRound(v_MarkGroups[i].circles[0].Radius), CV_RGB(255,0,0), 1, 8, 0 );
          cv::circle( drawing, cvPoint(cvRound(v_MarkGroups[i].circles[c_circle].X), cvRound(v_MarkGroups[i].circles[c_circle].Y)), 3, CV_RGB(0,255,0), -1, 8, 0 );
          cv::circle( drawing, cvPoint(cvRound(v_MarkGroups[i].circles[c_circle].X), cvRound(v_MarkGroups[i].circles[c_circle].Y)), cvRound(v_MarkGroups[i].circles[1].Radius), CV_RGB(255,0,0), 1, 8, 0 );
          cv::circle( drawing, cvPoint(cvRound(v_MarkGroups[i].circles[r_circle].X), cvRound(v_MarkGroups[i].circles[r_circle].Y)), 3, CV_RGB(0,255,0), -1, 8, 0 );
          cv::circle( drawing, cvPoint(cvRound(v_MarkGroups[i].circles[r_circle].X), cvRound(v_MarkGroups[i].circles[r_circle].Y)), cvRound(v_MarkGroups[i].circles[2].Radius), CV_RGB(255,0,0), 1, 8, 0 );
        }
      }
    }
  }
  if (_width != drawing.cols || _height != drawing.rows)
  {
    cv::Mat drawing_resized;
    cv::resize (drawing, drawing_resized, cvSize(_width, _height));
    return drawing_resized;
  }
  else return drawing;
}

bool class_ERA_target_recognizer::FindTarget(cv::Mat _image)
{
  cv::Mat after_canny;
  try
  {
    if (SizeOfImage.width != _image.cols || SizeOfImage.height != _image.rows)
      return false; //Incorrect source image size
    cv::blur(_image, _image, cv::Size(3,3) );
    cv::Canny (_image, after_canny, threshold, threshold * 2);
    findContours(after_canny, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    find_circles (9, 5, 2);
    MarkedCluster = 0;
    if (v_circles.size() > 2) 
    {
      MarkedCluster = FindClaster();
      if (MarkedCluster) 
      {
        for (unsigned i = 0; i < v_MarkGroups.size(); i++) 
        {
          if (v_MarkGroups[i].Mark == MarkedCluster) 
          {
            std::tie (position, rotation) = Calc6TargetCoord (v_MarkGroups[i].circles[l_circle].Point, v_MarkGroups[i].circles[c_circle].Point, v_MarkGroups[i].circles[r_circle].Point);
            break;
          }  
        }
      }
      else return false;
    }
    else return false;
    return true;
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return false;
  }
}
