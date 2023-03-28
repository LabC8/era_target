#include <cv_bridge/cv_bridge.h>

class Circle
{
public:
  double GetRadius();
  CvPoint2D32f GetCenter();
  void SetCircle (CvPoint *p1, CvPoint *p2, CvPoint *p3);
  //Circle(CvPoint *p1, CvPoint *p2, CvPoint *p3);	// p1, p2, p3 are co-planar
  Circle();
  Circle(uint32_t _max_width, uint32_t _max_height);
  virtual ~Circle();

private:
  uint32_t max_width, max_height;
  void CalcCircle(CvPoint *pt1, CvPoint *pt2, CvPoint *pt3);
  double CalcRadius(CvPoint2D32f * Center, CvPoint * Pt);
  bool IsPerpendicular(CvPoint *pt1, CvPoint *pt2, CvPoint *pt3);
  double m_dRadius;
  CvPoint2D32f m_Center;
};
