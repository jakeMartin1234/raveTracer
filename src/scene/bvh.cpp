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
void split(const std::vector<Primitive*>::iterator& start,
           const std::vector<Primitive*>::iterator& end,
           std::vector<Primitive*>::iterator& middle) {
    int size = end - start;
    Vector3D center;
    for (auto i = start; i != end; i++) {
        center += (*i)->get_bbox().centroid();
    }
    center /= size;
    int a = 0;
    int b = 0;
    int c = 0;
    for (auto i = start; i != end; i++) {
        Vector3D v = (*i)->get_bbox().centroid();
        if (v[0] - center[0] > 0) a++;
        if (v[1] - center[1] > 0) b++;
        if (v[2] - center[2] > 0) c++;
    }
    int index = rand() % 3;
    int mid = 0;
    for (auto i = start; i != end; i++) {
        Vector3D v = (*i)->get_bbox().centroid();
        if (v[index] < center[index]) mid++;
    }
    middle = start + mid;
    auto fast = start;
    auto slow = start;
    while (fast != end) {
        if ((*fast)->get_bbox().centroid()[index] < center[index]) {
            auto t = *slow;
            *slow = *fast;
            *fast = t;
            slow++;
        }
        fast++;
    }
    if (middle == end || middle == start) middle = start + 1;
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
    
    if (end - start <= max_leaf_size) {
        BBox bbox;
        for (auto p = start; p != end; p++) {
            BBox bb = (*p)->get_bbox();
            bbox.expand(bb);
        }
        BVHNode* node = new BVHNode(bbox);
        node->start = start;
        node->end = end;
        return node;
    }
    else {
        std::vector<Primitive*>::iterator middle;
        split(start, end, middle);
        BBox bbox;
        BVHNode* node = new BVHNode(bbox);
        node->l = construct_bvh(start, middle, max_leaf_size);
        node->r = construct_bvh(middle, end, max_leaf_size);
        bbox.expand(node->l->bb);
        bbox.expand(node->r->bb);
        node->bb = bbox;
        return node;
    }
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
    double t0;
    double t1;
    if (!node->bb.intersect(ray, t0, t1)) return false;
    if (node->isLeaf()) {
        for (auto p = node->start; p != node->end; p++) {
            total_isects++;
            if ((*p)->has_intersection(ray)) return true;
        }
        return false;
    }
    else {
        return has_intersection(ray, node->l)
            || has_intersection(ray, node->r);
    }

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
    double t0;
    double t1;
    if (!node->bb.intersect(ray, t0, t1) 
        || t1 < ray.min_t
        || t0 > ray.max_t) return false;
    if (node->isLeaf()) {
        bool hit = false;
        for (auto p = node->start; p != node->end; p++) {
            total_isects++;
            if ((*p)->intersect(ray, i)) hit = true;
            /*Vector3D min = node->bb.min;
            Vector3D max = node->bb.max;
            Vector3D c = (*p)->get_bbox().centroid();
            if (c[0] < min[0] || c[0] > max[0]) {
                cout << "a" << endl;
            }
            if (c[1] < min[1] || c[1] > max[1]) {
                cout << "b" << endl;
            }
            if (c[2] < min[2] || c[2] > max[2]) {
                cout << "c" << endl;
            }
            cout << 1 << endl;*/
        }
        //cout << "hit is " << hit << endl;
        return hit;
    }
    else {
        bool left = intersect(ray, i, node->l);
        bool right = intersect(ray, i, node->r);
        return left || right;
    }
}

} // namespace SceneObjects
} // namespace CGL
