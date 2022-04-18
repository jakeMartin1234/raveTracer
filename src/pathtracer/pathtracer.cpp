#include "pathtracer.h"
#include "math.h"
#include <cmath>



#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out = Vector3D();

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading
  for (int i = 0; i < num_samples; i++) {
      Vector3D sampleDir = hemisphereSampler->get_sample();
      // divide by the pdf at the end
      //need to align directions with the object in the world
      Ray ray_in = Ray(hit_p + (EPS_F * (o2w * sampleDir)), o2w * sampleDir);
      Intersection isect1 = Intersection();
      if (bvh->intersect(ray_in, &isect1)) {
          Vector3D f = isect.bsdf->f(w_out, sampleDir);
          double prob = 1 / (2 * M_PI);
          Vector3D l = isect1.bsdf->get_emission();
          sampleDir.normalize();
          Vector3D currVal = f * l * dot(o2w * sampleDir, isect.n) / prob;
          L_out += currVal;
      }
  }
  L_out = L_out / num_samples;

  return L_out;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  Vector3D L_out = Vector3D();

  for (int i = 0; i < scene->lights.size(); i++) {
      SceneLight *light = scene->lights.at(i);
      Vector3D wi;
      double distToLight;
      double pdf;

      if (light->is_delta_light()) {
          Vector3D sample = light->sample_L(hit_p, &wi, &distToLight, &pdf);
          Ray ray_in = Ray(hit_p + (EPS_F * (o2w * wi)), o2w * wi);
          ray_in.min_t = 0.0;
          ray_in.max_t = distToLight - EPS_F;
          Intersection isect1 = Intersection();
          if (!bvh->intersect(ray_in, &isect1)) {
              L_out += isect.bsdf->f(w_out, wi) * sample * dot(o2w * wi, isect.n) / pdf;
          }
      } else {
          Vector3D local = Vector3D();
          for (int j = 0; j < ns_area_light; j++) {
              Vector3D sample = light->sample_L(hit_p, &wi, &distToLight, &pdf);
              if ((w2o * wi).z >= 0) {
                  Ray ray_in = Ray(hit_p, wi);
                  ray_in.min_t = EPS_F;
                  ray_in.max_t = distToLight - EPS_F;
                  Intersection isect1 = Intersection();
                  if (!bvh->intersect(ray_in, &isect1)) {
                      local += isect.bsdf->f(w_out, w2o * wi) * sample * dot(wi, isect.n) / pdf;
                  }
              }
          }
          L_out += local / ns_area_light;
      }
  }
  L_out = L_out / scene->lights.size();
  return L_out;

}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light


  return isect.bsdf->get_emission();


}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  if (direct_hemisphere_sample) {
      return estimate_direct_lighting_hemisphere(r, isect);
  } else {
      return estimate_direct_lighting_importance(r, isect);
  }



}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  L_out += one_bounce_radiance(r, isect);
  Vector3D w_in;
  double pdf;
  Vector3D sampleSample = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  Vector3D worldw_in = o2w * w_in;
  Ray ray2 = Ray(hit_p + EPS_F * isect.n, worldw_in);
  Intersection isect1 = Intersection();
  bool pPrime = bvh->intersect(ray2, &isect1);
  double cpdf = 0.7;
  if (pPrime && ((coin_flip(cpdf) && r.depth < max_ray_depth) || r.depth == 0)) {
      Vector3D indirectIllum = sampleSample * dot(worldw_in, isect.n) / pdf;
      ray2.depth = r.depth + 1;
      if (r.depth == 0) {
          L_out += at_least_one_bounce_radiance(ray2, isect1) * indirectIllum;
      } else {
          L_out += at_least_one_bounce_radiance(ray2, isect1) * indirectIllum / cpdf;
      }

  }
  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;

  if (max_ray_depth == 0) {
      L_out = zero_bounce_radiance(r, isect);
  } else if (max_ray_depth == 1) {
      L_out = zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);
  } else {
      L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
  }



  // TODO (Part 3): Return the direct illumination.

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.


  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  int num_samples = ns_aa;          // total samples to evaluate
  int num_samples_per_batch;
  float s1 = 0.0;
  float s2 = 0.0;
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Vector3D total = Vector3D();
  int sampleCount = 0;
  bool br = false;
  for (int i = 0; i < num_samples && !br; i++) {
      num_samples_per_batch = i % samplesPerBatch;

      Vector2D sampleLoc = gridSampler->get_sample();
      double xSample = (origin.x + sampleLoc.x) / sampleBuffer.w;
      double ySample = (origin.y + sampleLoc.y) / sampleBuffer.h;
      Ray sampleRay = camera->generate_ray(xSample, ySample);
      Vector3D sampleVal = est_radiance_global_illumination(sampleRay);
      s1 = s1 + sampleVal.illum();
      s2 = s2 + pow(sampleVal.illum(), 2.0);
      total += sampleVal;
      sampleCount++;
      if (num_samples_per_batch == 0 && i != 0) {
          float mu = s1 / ((float) i);
          float var = (1/(((float) i) - 1)) * (s2 - (pow(s1, 2.0) / ((float) i)));
          float conv = 1.96 * (sqrt(var) / sqrt(i));
          if (conv <= maxTolerance * mu) {
              br = true;
          }
      }

  }

  sampleCountBuffer[y * sampleBuffer.w + x] = sampleCount;
  total = total / ((double) sampleCount);

  sampleBuffer.update_pixel(total, x, y);


}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
