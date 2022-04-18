#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.



  BBox bbox;
  int primCounter = 0;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
    primCounter += 1;
  }

  BVHNode *node = new BVHNode(bbox);
  node->start = start;
  node->end = end;
  if (primCounter > max_leaf_size) {
      // need to figure out which axis to split
      // thinking that I will choose for the shortest length of bounding box.
      std::vector<Primitive *> left = vector<Primitive *>();
      std::vector<Primitive *> right = vector<Primitive *>();
      if ((bbox.extent.x > bbox.extent.y) && (bbox.extent.x > bbox.extent.z)) {
          // x axis
          sort(start, end, compX);
      } else if (bbox.extent.y > bbox.extent.z) {
          // y axis
          sort(start, end, compY);
      } else {
          // z axis
          sort(start, end, compZ);
      }
      double lEnd = floor(primCounter / 2);
      node->l = construct_bvh(start, start + lEnd, max_leaf_size);
      node->r = construct_bvh(start + lEnd, end, max_leaf_size);
  }
  return node;


}

bool BVHAccel::compX(Primitive *p1, Primitive *p2) {
    return p1->get_bbox().centroid().x < p2->get_bbox().centroid().x;
}

bool BVHAccel::compY(Primitive *p1, Primitive *p2) {
    return p1->get_bbox().centroid().y < p2->get_bbox().centroid().y;
}

bool BVHAccel::compZ(Primitive *p1, Primitive *p2) {
    return p1->get_bbox().centroid().z < p2->get_bbox().centroid().z;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.


  if (node->l == nullptr && node->r == nullptr) {
      for (auto p = node->start; p != node->end; p++) {
          if ((*p)->has_intersection(ray)){
              return true;
          }

      }
  } else {
      if (has_intersection(ray, node->r)) {
          return true;
      }
      if (has_intersection(ray, node->l)) {
          return true;
      }
  }
  return true;

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
    bool hit = false;
    double t0;
    double t1;
    if (node->bb.intersect(ray, t0, t1)) {
        if (node->isLeaf()) {
            for (auto p = node->start; p != node->end; p++) {
                if ((*p)->intersect(ray, i)){
                    hit = true;
                }
            }
        } else {

            if (intersect(ray, i, node->l)) {
                hit = true;
            }
            double lt = i->t;
            if (intersect(ray, i, node->r)) {
                hit = true;
            }
        }
    }



    return hit;


}

} // namespace SceneObjects
} // namespace CGL
