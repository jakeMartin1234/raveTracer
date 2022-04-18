#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.


  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
	float a = dot(r.d, r.d);
	if (a == 0) return false;
	float b = dot(2 * (r.o - o), r.d);
	float c = dot(r.o - o, r.o - 1) - r2;
	float rad = b * b - 4 * a * c;
	if (rad < 0) return false;
	float t = (-b - sqrt(rad)) / (2 * a);
	if (t < r.min_t) t = (-b + sqrt(rad)) / (2 * a);
	return t > r.min_t && t < r.max_t;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
	float a = dot(r.d, r.d);
	if (a == 0) return false;
	float b = dot(2 * (r.o - o), r.d);
	float c = dot(r.o - o, r.o - o) - r2;
	float rad = b * b - 4 * a * c;
	if (rad < 0) return false;
	float t = (-b - sqrt(rad)) / (2 * a);
	if (t < r.min_t) {
		t = (-b + sqrt(rad)) / (2 * a);
	}
	if (t < r.min_t || t > r.max_t) return false;
	Vector3D point = r.o + r.d * t;
	Vector3D normal = point - o;
	normal.normalize();
	r.max_t = t;
	i->bsdf = get_bsdf();
	i->primitive = this;
	i->t = t;
	i->n = normal;
	return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
