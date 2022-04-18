#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

Ray Camera::generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const {

  // TODO Project 3-2: Part 4
  // compute position and direction of ray from the input sensor sample coordinate.
  // Note: use rndR and rndTheta to uniformly sample a unit disk.
    Vector3D pLens(
        lensRadius * sqrt(rndR) * cos(rndTheta),
        lensRadius * sqrt(rndR) * sin(rndTheta),
        0
    );

    float vFovRad = vFov * (PI / 180.f);
    float hFovRad = vFovRad * aspect_ratio();
    Vector3D forward = targetPos - pos;
    forward.normalize();
    Vector3D right = cross(forward, up_dir());
    double width = 2 * tan(hFovRad / 2.f);
    double height = 2 * tan(vFovRad / 2.f);
    Vector3D screenDir = Vector3D(
        width * (x - 0.5),
        height * (y - 0.5),
        -1
    );
    screenDir.normalize();
    double t = (focalDistance + 1) / - screenDir.z;
    Vector3D focalPoint = screenDir * t;
    Vector3D toPLense = focalPoint - pLens;
    toPLense.normalize();
    return Ray(c2w * pLens + pos, c2w * toPLense);
}


} // namespace CGL
