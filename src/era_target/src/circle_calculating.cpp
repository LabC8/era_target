#include "circle_calculating.hpp"
Circle::Circle()
{
  Circle (0, 0);
}

Circle::Circle(uint32_t _max_width, uint32_t _max_height)
{
  max_width = _max_width;
  max_height = _max_height;
  this->m_dRadius = -1;		// признак ошибки
}

Circle::~Circle()
{

}

//Circle::Circle(CvPoint *V1, CvPoint *V2, CvPoint *V3)
void Circle::SetCircle(CvPoint *V1, CvPoint *V2, CvPoint *V3)
{
  this->m_dRadius = -1;		// признак ошибки

  CvPoint *pt1 = new CvPoint;
  CvPoint *pt2 = new CvPoint;
  CvPoint *pt3 = new CvPoint;     

  std::memcpy (pt1, V1, sizeof(CvPoint));
  std::memcpy (pt2, V2, sizeof(CvPoint));
  std::memcpy (pt3, V3, sizeof(CvPoint));

  if (!this->IsPerpendicular(pt1, pt2, pt3)) this->CalcCircle(pt1, pt2, pt3);
  else if (!this->IsPerpendicular(pt1, pt3, pt2)) this->CalcCircle(pt1, pt3, pt2);
       else if (!this->IsPerpendicular(pt2, pt1, pt3)) this->CalcCircle(pt2, pt1, pt3);
            else if (!this->IsPerpendicular(pt2, pt3, pt1)) this->CalcCircle(pt2, pt3, pt1);
                 else if (!this->IsPerpendicular(pt3, pt2, pt1)) this->CalcCircle(pt3, pt2, pt1);
                      else if (!this->IsPerpendicular(pt3, pt1, pt2)) this->CalcCircle(pt3, pt1, pt2);
                           else {
                             //The three pts are perpendicular to axis
                             delete pt1;
                             delete pt2;
                             delete pt3;
                             this->m_dRadius = -1;
                             return ;
                           }
  delete pt1;
  delete pt2;
  delete pt3;
}

bool Circle::IsPerpendicular(CvPoint *pt1, CvPoint *pt2, CvPoint *pt3)
// Check the given point are perpendicular to x or y axis
{
  double yDelta_a = pt2->y - pt1->y;
  double xDelta_a = pt2->x - pt1->x;
  double yDelta_b = pt3->y - pt2->y;
  double xDelta_b = pt3->x - pt2->x;

  // checking whether the line of the two pts are vertical
  if (fabs(xDelta_a) <= 0.0000001 && fabs(yDelta_b) <= 0.0000001) return false;

  if (fabs(yDelta_a) <= 0.0000001) return true;
  else if (fabs(yDelta_b) <= 0.0000001) return true;
       else if (fabs(xDelta_a) <= 0.0000001) return true;
            else if (fabs(xDelta_b) <= 0.0000001) return true;
                 else return false;
}

double Circle::CalcRadius(CvPoint2D32f * Center, CvPoint * Pt)
{
  return sqrt((Pt->x - Center->x) * (Pt->x - Center->x) + (Pt->y - Center->y) * (Pt->y - Center->y));
}

void Circle::CalcCircle(CvPoint *pt1, CvPoint *pt2, CvPoint *pt3)
{
  double yDelta_a = pt2->y - pt1->y;
  double xDelta_a = pt2->x - pt1->x;
  double yDelta_b = pt3->y - pt2->y;
  double xDelta_b = pt3->x - pt2->x;

  if (fabs(xDelta_a) <= 0.0000001 && fabs(yDelta_b) <= 0.0000001) {
    this->m_Center.x = 0.5 * (pt2->x + pt3->x);
    this->m_Center.y = 0.5 * (pt1->y + pt2->y);
    // this->m_dRadius = clength (&m_Center, pt1);		// calc. radius
    this->m_dRadius = CalcRadius (&m_Center, pt1);
    return;
  }

  // IsPerpendicular() assure that xDelta(s) are not zero
  double aSlope = yDelta_a / xDelta_a;
  double bSlope = yDelta_b / xDelta_b;
  // проверка на колинеарность
  if (fabs(aSlope - bSlope) <= 0.0000001) {
    this->m_dRadius = -1;
    return;
  }

  // calc center
  this->m_Center.x = (aSlope * bSlope * (pt1->y - pt3->y) + bSlope * (pt1->x + pt2->x) -
                     aSlope*(pt2->x+pt3->x) )/(2* (bSlope-aSlope) );
  this->m_Center.y = -1 * (m_Center.x - (pt1->x + pt2->x) / 2) /
                     aSlope + (pt1->y + pt2->y) / 2;
  
  if (max_width > 0 && max_width > 0)
    if (this->m_Center.x < 0 || this->m_Center.x > max_width || // руги с центром за пределами
        this->m_Center.y < 0 || this->m_Center.y > max_height) { //матрицы не рассматриваем
      this->m_dRadius = -1;
      return;
    }

  //this->m_dRadius = length(&m_Center,pt1);		// calc. radius
  this->m_dRadius = CalcRadius (&m_Center, pt1);
  return;
}

CvPoint2D32f Circle::GetCenter()
{
  return this->m_Center;
}

double Circle::GetRadius()
{
  return this->m_dRadius;
}
