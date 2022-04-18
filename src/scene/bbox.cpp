#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
	float x1,x2,y1, y2,z1,z2;
	if (r.d[0] == 0) {
		x1 = std::numeric_limits<float>::min();
		x2 = std::numeric_limits<float>::max();
	}
	else {
		x1 = (min[0] - r.o[0]) / r.d[0];
		x2 = (max[0] - r.o[0]) / r.d[0];
	}
	if (r.d[1] == 0) {
		y1 = std::numeric_limits<float>::min();
		y2 = std::numeric_limits<float>::max();
	}
	else {
		y1 = (min[1] - r.o[1]) / r.d[1];
		y2 = (max[1] - r.o[1]) / r.d[1];
	}
	if (r.d[2] == 0) {
		z1 = std::numeric_limits<float>::min();
		z2 = std::numeric_limits<float>::max();
	}
	else {
		z1 = (min[2] - r.o[2]) / r.d[2];
		z2 = (max[2] - r.o[2]) / r.d[2];
	}
	if (r.d[0] < 0) {
		float temp = x1;
		x1 = x2;
		x2 = temp;
	}
	if (r.d[1] < 0) {
		float temp = y1;
		y1 = y2;
		y2 = temp;
	}
	if (r.d[2] < 0) {
		float temp = z1;
		z1 = z2;
		z2 = temp;
	}
	float time0 = std::max(x1, std::max(y1, z1));
	float time1 = std::min(x2, std::min(y2, z2));
	if (time0 > time1 || time1 < 0) {
		return false;
	}
	if (time0 < 0) {
		t0 = 0.f;
		t1 = time1;
		return true;
	}
	t0 = time0;
	t1 = time1;
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
