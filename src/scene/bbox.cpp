#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>
#include "triangle.h"

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.


  // there are 6 faces, just have to call intersect triangle twice on each.

  vector<double> tsMax = vector<double>();
  vector<double> tsMin = vector<double>();
  // faces parallel with x axis
  if (r.d.x != 0) {
      double tx1 = (min.x - r.o.x) / r.d.x;
      double tx2 = (max.x - r.o.x) / r.d.x;
      tsMin.push_back(std::min(tx1, tx2));
      tsMax.push_back(std::max(tx1, tx2));
  }
    if (r.d.y != 0) {
        double ty1 = (min.y - r.o.y) / r.d.y;
        double ty2 = (max.y - r.o.y) / r.d.y;
        tsMin.push_back(std::min(ty1, ty2));
        tsMax.push_back(std::max(ty1, ty2));
    }
    if (r.d.z != 0) {
        double tz1 = (min.z - r.o.z) / r.d.z;
        double tz2 = (max.z - r.o.z) / r.d.z;
        tsMin.push_back(std::min(tz1, tz2));
        tsMax.push_back(std::max(tz1, tz2));
    }
    if (tsMax.size() == 0 || tsMin.size() == 0) {
        return false;
    } else {
        sort(tsMin.begin(), tsMin.end());
        t0 = tsMin.at(tsMin.size() - 1);
        sort(tsMax.begin(), tsMax.end());
        t1 = tsMax.at(0);

    }
    if (t0 > r.max_t || t1 < r.min_t) {
        return false;
    }

    if (t0 > t1) {
        return false;
    }



    return true;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
