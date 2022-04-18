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

  // first step is to find the normal to the plane.

  Vector3D e1 = p2 - p1;
  Vector3D e2 = p3 - p1;
  Vector3D P = cross(r.d, e2);
  Vector3D rel = r.o - p1;
  Vector3D b = cross(rel, e1);
  double determinent = dot(e1, P);

  double alpha = dot(P, rel) / determinent;
  double beta = dot(b, r.d) / determinent;
  double t = dot(b, e2) / determinent;

  if (alpha < 0 || alpha > 1) {
      return false;
  }
  if (beta < 0 || beta > 1) {
      return false;
  }
  if (t < r.min_t || t > r.max_t) {
      return false;
  }

  r.max_t = t;
  return true;





}

bool Triangle::isInside(Vector3D point, Vector3D po1, Vector3D po2, Vector3D po3) const {
//    float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
//    float cCROSSap, bCROSScp, aCROSSbp;
//
//    ax = Cx - Bx;  ay = Cy - By;
//    bx = Ax - Cx;  by = Ay - Cy;
//    cx = Bx - Ax;  cy = By - Ay;
//    apx= Px - Ax;  apy= Py - Ay;
//    bpx= Px - Bx;  bpy= Py - By;
//    cpx= Px - Cx;  cpy= Py - Cy;
//
//    aCROSSbp = ax*bpy - ay*bpx;
//    cCROSSap = cx*apy - cy*apx;
//    bCROSScp = bx*cpy - by*cpx;
//
//    return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
//    Vector3D v1 = po2 - po1;
//    Vector3D v11 = point - po1;
//    Vector3D v2 = po3 - po2;
//    Vector3D v22 = point - po2;
//    Vector3D v3 = po1 - po3;
//    Vector3D v33 = point - po3;
//
//    if (cross(v1, v11) >= Vector3D(0, 0, 0) && cross(v2, v22) >= Vector3D && cross(v3, v33) >= 0) {
//        return true;
//    }
//    return false;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
    Vector3D e1 = p2 - p1;
    Vector3D e2 = p3 - p1;
    Vector3D P = cross(r.d, e2);
    double determinent = dot(e1, P);
    Vector3D T = r.o - p1;
    Vector3D Q = cross(T, e1);

    double alpha = dot(T, P) / determinent;
    double beta = dot(r.d, Q) / determinent;
    double gamma = 1 - alpha - beta;
    double t = dot(e2, Q) / determinent;

    if (alpha < 0 || alpha > 1) {
        return false;
    }
    if (beta < 0 || beta > 1) {
        return false;
    }
    if (alpha + beta < 0 || alpha + beta > 1) {
        return false;
    }
    if (t < r.min_t || t > r.max_t || t < 0) {
        return false;
    }
    r.max_t = t;

    isect->t = t;
    isect->n = gamma * n1 + alpha * n2 + beta * n3;
    isect->n.normalize();
    isect->primitive = this;
    isect->bsdf = this->get_bsdf();
    return true;

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
