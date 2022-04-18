#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
	Vector3D ab = p1 - p2;
	Vector3D bc = p2 - p3;
    Vector3D ca = p3 - p1;
    if (dot(cross(ab, bc), r.d) < 0) {
        ab *= -1;
        bc *= -1;
        ca *= -1;
    }
	Vector3D oa = p1 - r.o;
	Vector3D ob = p2 - r.o;
	Vector3D oc = p3 - r.o;
	Vector3D n1 = cross(oa, ab);
	Vector3D n2 = cross(ob, bc);
	Vector3D n3 = cross(oc, ca);
	return
		dot(r.d, n1) <= 0
		&& dot(r.d, n2) <= 0
		&& dot(r.d, n3) <= 0; 
}

float epsalon = 0.0001;
bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
    if (has_intersection(r)) {
        Vector3D N = cross(p2 - p1, p3 - p1);
        //N = (dot(N, r.d) < 0) ? -N : N;//Maybe flip normal
        N.normalize();
        float NdotD = dot(r.d, N);
        //cout << 1 << endl;
        if (abs(NdotD) < epsalon) return false;
        //cout << 2 << endl;
        float t = dot(p1 - r.o, N) / NdotD;
        if (t < r.min_t || t > r.max_t) return false;
        //cout << 3 << endl;
        Vector3D p = r.o + r.d * t;
        Vector3D ap = p - p1;
        Vector3D bp = p - p2;
        Vector3D cp = p - p3;
        Vector3D ab = p2 - p1;
        Vector3D bc = p3 - p2;
        Vector3D ca = p1 - p3;
        float gamma = cross(ap, ab).norm() / cross(bc, ab).norm();
        float alpha = cross(bp, bc).norm() / cross(bc, ab).norm();
        float beta = 1 - alpha - gamma;
        Vector3D normal = alpha * n1 + beta * n2 + gamma * n3;
        r.max_t = t;
        isect->t = t;
        isect->bsdf = get_bsdf();
        isect->n = normal;
        isect->primitive = this;
        return true;
    }
    return false;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
