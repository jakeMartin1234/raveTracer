#include "pathtracer.h"

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

    void PathTracer::write_to_framebuffer(ImageBuffer& framebuffer, size_t x0,
        size_t y0, size_t x1, size_t y1) {
        sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
    }




    Vector3D
        PathTracer::estimate_direct_lighting_hemisphere(const Ray& r,
            const Intersection& isect) {
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
        Vector3D L_out;
        // TODO (Part 3): Write your sampling loop here
        // TODO BEFORE YOU BEGIN
        // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
        for (int i = 0; i < num_samples; i++) {
            double pdf;
            Vector3D wi;
            Vector3D reflectance = isect.bsdf->sample_f(w_out, &wi, &pdf);
            wi = o2w * wi;
            Ray in(hit_p, wi);
            in.min_t = EPS_F;
            Intersection temp;
            if (bvh->intersect(in, &temp)) {
                L_out += temp.bsdf->get_emission() * reflectance / pdf;

            }
        }
        L_out /= num_samples;
        return L_out;
    }

    Vector3D
        PathTracer::estimate_direct_lighting_importance(const Ray& r,
            const Intersection& isect) {
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
        Vector3D L_out;
        for (SceneLight* light : scene->lights) {
            if (light->is_delta_light()) {
                Vector3D wi;
                double dist, pdf;
                Vector3D emission = light->sample_L(hit_p, &wi, &dist, &pdf);
                Ray r(hit_p, wi);
                r.min_t = EPS_F;
                Intersection lightIntersection;
                if (!bvh->intersect(r, &lightIntersection) || lightIntersection.t + EPS_F > dist) {
                    L_out +=
                        emission
                        * isect.bsdf->f(w_out, w2o * wi)
                        / (pdf * PI);

                }
            }
            else {
                Vector3D temp;
                for (int i = 0; i < ns_area_light; i++) {
                    Vector3D wi;
                    double dist, pdf;
                    Vector3D emission = light->sample_L(hit_p, &wi, &dist, &pdf);
                    if (dot(wi, isect.n) < 0) continue;
                    Ray r(hit_p, wi);
                    r.min_t = EPS_F;
                    Intersection lightIntersection;
                    if (!bvh->intersect(r, &lightIntersection) || lightIntersection.t + EPS_F > dist) {
                        temp +=
                            emission
                            * isect.bsdf->f(w_out, w2o * wi)
                            / (pdf * PI);
                    }
                }
                //cout << endl;
                if (temp[0] >= 0) {
                    L_out += temp / ns_area_light;
                }
                else {
                    cout << isect.n << " " << ((AreaLight*)light)->direction << endl;
                    cout << hit_p << " " << ((AreaLight*)light)->position << endl;
                    cout << dot(isect.n, ((AreaLight*)light)->direction) << endl;
                    cout << temp << endl;
                    cout << endl;
                }
            }
        }
        return L_out;

    }

    Vector3D PathTracer::zero_bounce_radiance(const Ray& r,
        const Intersection& isect) {
        // TODO: Part 3, Task 2
        // Returns the light that results from no bounces of light
        if (dot(r.d, isect.n) < 0) return Vector3D();
        return isect.bsdf->get_emission() / (2 * PI);
    }

    Vector3D PathTracer::one_bounce_radiance(const Ray& r,
        const Intersection& isect) {
        // TODO: Part 3, Task 3
        // Returns either the direct illumination by hemisphere or importance sampling
        // depending on `direct_hemisphere_sample`
        if (direct_hemisphere_sample) {
            return estimate_direct_lighting_hemisphere(r, isect);
        }
        else {
            return estimate_direct_lighting_importance(r, isect);
        }
    }

    Vector3D PathTracer::at_least_one_bounce_radiance(const Ray& r,
        const Intersection& isect) {
        Matrix3x3 o2w;
        make_coord_space(o2w, isect.n);
        Matrix3x3 w2o = o2w.T();

        Vector3D hit_p = r.o + r.d * isect.t;
        Vector3D w_out = w2o * (-r.d);

        Vector3D L_out(0, 0, 0);

        // TODO: Part 4, Task 2
        // Returns the one bounce radiance + radiance from extra bounces at this point.
        // Should be called recursively to simulate extra bounces.
        double rMin = r.min_t;
        double rMax = r.max_t;
        L_out += zero_bounce_radiance(r, isect);
        r.min_t = rMin;
        r.max_t = rMax;
        L_out += one_bounce_radiance(r, isect);
        r.min_t = rMin;
        r.max_t = rMax;
        float cpdf = 0.7f;
        if (r.depth > 0 && coin_flip(cpdf)) {
            double pdf;
            Vector3D wi;
            Vector3D reflectance = isect.bsdf->sample_f(w_out, &wi, &pdf);
            wi = o2w * wi;
            Ray in(hit_p, wi);
            in.min_t = EPS_F;
            in.depth = r.depth - 1;
            Intersection temp;
            if (bvh->intersect(in, &temp)) {
                in.min_t = EPS_F;
                in.max_t = INF_D;
                Vector3D samp =
                    at_least_one_bounce_radiance(in, temp)
                    * reflectance
                    / (pdf)
                    / cpdf;
                L_out += samp;
            }
        }
        return L_out;
    }

    Vector3D PathTracer::est_radiance_global_illumination(const Ray& r) {
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


        //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
        //return L_out;

        // TODO (Part 3): Return the direct illumination.
        if (max_ray_depth == 0) {
            L_out += zero_bounce_radiance(r, isect);
        }
        else if (max_ray_depth == 1) {
            L_out += zero_bounce_radiance(r, isect);
            L_out += one_bounce_radiance(r, isect);
        }
        else {
            L_out += at_least_one_bounce_radiance(r, isect);
        }
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

        int num_samples = 0;          // total samples to evaluate
        float s1 = 0;
        float s2 = 0;
        // x and y are bottom left corner of the pixel
        Vector3D result;
        float i = 0;
        while (num_samples < ns_aa) {
            num_samples = min(num_samples + samplesPerBatch, ns_aa);
            while (i < num_samples) {
                float xr = (float)rand() / RAND_MAX;
                float yr = (float)rand() / RAND_MAX;
                Ray r = camera->generate_ray(
                    ((float)x + xr) / sampleBuffer.w,
                    ((float)y + yr) / sampleBuffer.h);
                r.depth = max_ray_depth;
                Vector3D sample = est_radiance_global_illumination(r);
                s1 += sample.illum();
                s2 += sample.illum() * sample.illum();
                result += sample;
                i++;
            }
            float mean = s1 / i;
            float stdev = sqrt((1.f / (i - 1.f)) * (s2 - s1 * s1 / i));
            float I = 1.96f * stdev / sqrt(i);
            if (I <= maxTolerance * mean) {
                break;
            }
        }
        result /= i;
        sampleBuffer.update_pixel(result, x, y);
        sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
    }

    void PathTracer::autofocus(Vector2D loc) {
        Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
        Intersection isect;

        bvh->intersect(r, &isect);

        camera->focalDistance = isect.t;
    }

} // namespace CGL
