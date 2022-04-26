# Monte Carlo Ray Tracer

This is a physically based renderer with Path Tracing and Photon Mapping.

![Path traced render of spaceship, 457 200 triangles. Original scene by thecali.](https://i.imgur.com/rSVyvl0.jpg "Path traced render of spaceship, 457 200 triangles. Original scene by thecali.")
![Photon mapped render of caustics, 6.9 million triangles and 347 million photon particles. Original scene by Benedikt Bitterli.](https://i.imgur.com/BsagYAi.jpg "Photon mapped render of caustics, 6.9 million triangles and 347 million photon particles. Original scene by Benedikt Bitterli.")

This renderer was originally developed for the course [Advanced Global Illumination and Rendering](https://liu.se/studieinfo/en/kurs/tncg15) (TNCG15) at Linköpings universitet, but I've continued to add features and improvements since then.

The program is written in C++ and requires a compiler with C++17 support. The only dependencies are the header-only libraries [GLM](https://glm.g-truc.net/) and [nlohmann::json](https://github.com/nlohmann/json), which are included in the repository.

## Building

Install [git](https://git-scm.com/), [CMake](https://cmake.org/download/) and a modern compiler for your platform and run the following commands:
```sh
git clone https://github.com/linusmossberg/monte-carlo-ray-tracer
cd monte-carlo-ray-tracer
cmake .
```
This will generate build files in the root folder of the cloned repository, which can be used to build the program.

## Usage

For basic use, just run the program in the directory that contains the *scenes* directory, i.e. the root folder of this repository. The program will then parse all scene files and create several rendering options to choose from in the terminal. It is also possible to supply a command line argument with the path to the scenes directory.

## Scene Format

I created a scene file format for this project to simplify scene creation. The format is defined using JSON and I used the library [nlohmann::json](https://github.com/nlohmann/json) for JSON parsing. Complete scene file examples can be found in the scenes directory.

The basic outline of the scene format is the following JSON object:

```json
{
  "num_render_threads": -1,
  "ior": 1.75,

  "photon_map": { },
  "bvh": { },
  "cameras": [ ],
  "materials":  { },
  "vertices": { },
  "surfaces": [ ]
}
```

The `num_render_threads` field specifies the number of rendering threads to use. This is limited between 1 and the number of concurrent threads available on the system. All concurrent threads are used if the specified value is outside of this range.

The `ior` field specifies the scene index of refraction. This can be used to simulate different types of environment mediums to see the effects this has on the angle of refraction and the Fresnel factor.

The `photon_map`, `bvh`, `cameras`, `materials`, `vertices`, and `surfaces` objects specifies different render settings and scene contents. I go through each of these in the following sections. Click the summaries for more details.

### Photon Map

<details><summary>The <code>photon_map</code> object is optional and it specifies the photon map properties.</summary><br>

Example:
```json
"photon_map": {
  "emissions": 1e6,
  "caustic_factor": 100.0,
  "k_nearest_photons": 50,
  "max_photons_per_octree_leaf": 200,
  "direct_visualization": false
}
```

The `emissions` field determines the base number of rays that should be emitted from light sources. More emissions will result in more spawned photons. 

The `caustic_factor` determines how many times more caustic photons should be generated relative to other photon types. 1 is the "natural" factor, but this results in blurry caustics since the caustic photon map is visualized directly.

The `k_nearest_photons` field specifies the number of nearest photons to search for and use in the radiance estimate each time a photon map is evaluated at a point. Larger values create better but less localized (blurrier) estimates since the search sphere is expanded to cover the target number of photons.

The `max_photons_per_octree_leaf` field affects both the octree search performance and memory usage of the application. This value can probably be left at ~200 in most cases.

The `direct_visualization` field can be used to visualize the photon maps directly. Setting this to true will make the program evaluate the global radiance at the first diffuse reflection.
</details>

___

### BVH

<details><summary>The <code>bvh</code> object is optional and it specifies the Bounding Volume Hierarchy acceleration structure properties.</summary><br>

Example:
```json
"bvh": {
    "type": "quaternary_sah",
    "bins_per_axis": 16
}
```

Normal naive scene intersection is used if this object is not specified. The `type` field specifies the hierarchy method to use when constructing the tree.

| `type`  | Method | 
| ------- | ------ | 
| `octree` | First creates an octree by iterative insertion of the primitive centroids, and then transforms this tree into a BVH by just transferring the octree node hierarchy and computing the bounding boxes. | 
| `binary_sah` | Creates a binary-tree BVH by recursively splitting the primitives into two groups. The split occurs along the axis with the largest primitive centroid extent, and the split position is determined by the Surface Area Heuristic (SAH). Binning is performed to reduce the number of evaluated split coordinates along the axis, and the number of bins is determined by the `bins_per_axis` field. | 
| `quaternary_sah` | Creates a quaternary-tree BVH by recursively splitting the primitives into the four groups that results in the lowest SAH-cost. This is similar to the binary version, but the split now occurs along two axes. The bins form a regular 2D grid and (`bins_per_axis`-1)<sup>2</sup> possible split coordinates are evaluated. |

I've also tried splitting along all three axes each recursion to create octonary-trees. This produces good results but there's not much of an improvement compared to the quaternary version and the construction time becomes much longer due to the dimensionality curse when using 3D bins.

`quaternary_sah` takes the longest to construct but tends to produce the best results. `octree` and `binary_sah` are faster to construct which is useful for quick renders. This is especially the case for the octree method, which surprisingly seems to be both faster to construct and create higher quality trees than the binary-tree SAH method.
</details>

___

### Cameras

<details><summary>The <code>cameras</code> object contains an array of different cameras</summary><br>

Example:
```json
"cameras": [
  {
    "focal_length": 23,
    "sensor_width": 35,
    "f_stop": 1.8,
    "eye": [ -2, 0, 0 ],
    "look_at": [ 13, -0.55, 0 ],
    "image": { 
      "width": 960, 
      "height": 720, 
      "exposure_compensation": -1, 
      "gain_compensation": 0.5 
    },
    "sqrtspp": 4,
    "savename": "c1b"
  },
  {
    "focal_length": 50,
    "sensor_width": 35,
    "f_stop": 5.6,
    "focus_distance": 3,
    "eye": [ -1, 0, 0 ],
    "forward": [ 1, 0, 0 ],
    "up": [ 0, 1, 0 ],
    "image": { 
      "width": 960, 
      "height": 540,
      "tonemapper": "ACES"
    },
    "sqrtspp": 1,
    "savename": "c2"
  }
]
```

The `focal_length` and `sensor_width` fields are defined in millimeters. A sensor width of 35mm (full frame) is most often useful since focal lengths normally are defined in terms of 35mm-equivalent focal lengths.

The `eye` field defines the position of the camera, and the `up` and `forward` fields defines the orientation vectors of the camera. The up and forward vectors can be replaced with the `look_at` field, which defines the coordinate that the camera should look at instead.

The `f_stop` and `focus_distance` fields defines the depth of field properties of the camera and are optional. The distance from the camera to the `look_at` coordinate is used as focus distance if this coordinate is specified and if no valid focus distance is specified.

The `sqrtspp` field defines the square-rooted number of ray paths that should be sampled from each pixel in the camera.

The `savename` property defines the name of the resulting saved image file. Images are saved in TGA format.

#### Image

The `image` object specifies the image properties of the camera. The `width` and `height` fields specifies the image resolution in pixels.

The `tonemapper` field specifies which tonemapper to use. The available ones are `Hable` ([filmic tonemapper by John Hable](http://filmicworlds.com/blog/filmic-tonemapping-operators/)) and `ACES` ([fitted by Stephen Hill](https://twitter.com/self_shadow)).

The program has histogram-based auto-exposure which centers the histogram around the 0.5 intensity level before applying tone mapping (corresponding to controlling the amount of light that reaches the film/sensor). This can be offset with the optional `exposure_compensation` field, which specifies the [exposure compensation](https://en.wikipedia.org/wiki/Exposure_compensation) in EV units (stops). 

The program also has a histogram-based auto-gain method which is applied after auto-exposure and tone-mapping, which instead tries to position the histogram of the resulting image to the right. This can similarly be offset with the optional `gain_compensation` field, which is also specified in EV units.

The reason for separating these steps is that the tone-mapping/camera response is non-linear, and as a result `exposure_compensation` mostly controls the camera response (contrast, dynamic range etc.) while `gain_compensation` controls the overall image intensity.
</details>

___

### Materials

<details><summary>The <code>materials</code> object contains a map of different materials.</summary><br>

Example:
```json
"materials": {
  "default": {
      "reflectance": 0.73,
      "roughness": 10.0
  },
  "iron": {
    "ior": "data/spectral-distributions/iron.csv"
  },
  "silver": {
    "specular_roughness": 0.06,
    "ior": {
      "real": [0.03122206, 0.02993163, 0.03752037],
      "imaginary": [4.52084303, 3.61703254, 2.59526494]
    }
  },
  "crystal": {
    "ior": 2.0,
    "transparency":  1.0,
    "transmittance": [ 0.5, 1.0, 0.9 ],
    "specular_roughness": 0.1
  },
  "one_sheet_hyperboloid": {
    "specular_reflectance": 0.5,
    "ior": 1.333,
    "reflectance": "#80B1D3"
  },
  "light": {
    "reflectance": 0.9,
    "emittance": [ 1000, 1000, 1000 ]
  },
  "horizon-light": {
    "emittance": { "illuminant": "D50", "scale": 1000 }
  },
  "3000-kelvin-blackbody-radiator": {
    "emittance": { "temperature": 3000, "scale": 1000 }
  }
}
```

The key string is used later when assigning a material to a surface. The material with the `default` key string is used for all surfaces that hasn't specified a material.

The material fields are:

| field                  | type        | default | interval    |
| ---------------------- | ----------- | ------- | ----------- |
| `reflectance`          | RGB         | 1       | [0, 1]      |
| `specular_reflectance` | RGB         | 1       | [0, 1]      |
| `transmittance`        | RGB         | 1       | [0, 1]      |
| `emittance`            | RGB         | 0       | [0, ∞)      |
| `roughness`            | scalar      | 0       | [0, ∞)      |
| `specular_roughness`   | scalar      | 0       | [0, 1]      |
| `transparency`         | scalar      | 0       | [0, 1]      |
| `perfect_mirror`       | bool        | false   | {f, t}      |
| `ior`                  | [IOR](#ior) | 0       | [IOR](#ior) |

These fields are all optional and any combination of fields can be used. A material can for example be a combination of diffusely reflecting, specularly reflecting, emissive, transmissive (specularly refracting) and rough. If set to true, the `perfect_mirror` field overrides most other fields to simulate a perfect mirror with infinite IOR.

The `reflectance`, `specular_reflectance` and `transmittance` fields specifies the amount of radiance that should be diffusely reflected and specularly reflected/transmitted for each RGB channel. This is a simplification since these are spectral properties that varies with wavelength and not by the resulting tristimulus values of the virtual camera, but this is computationally cheaper and simpler. These properties now take gamma-corrected values and linearizes them internally to make it easier to pick colors via color pickers.

The `emittance` field defines the radiant flux of each RGB channel in watts. This means that surfaces with different surface areas will emit the same amount of radiant energy if they are assigned the same emissive material. It's also possible to set the emittance by specifying an object with either an `illuminant` or a `temperature` field, along with a `scale` field. The `illuminant` field is used to specify a [CIE standard illuminant](https://en.wikipedia.org/wiki/Standard_illuminant), while the `temperature` field is used to specify a blackbody radiator in kelvin.

#### IOR

For dielectric materials such as glass and plastic, the `ior` field is specified as a scalar value in the range [1, ∞). If this value is less than 1, then the material will only produce diffuse reflections regardless of scene IOR. For conductive materials such as metals, the `ior` field is instead specified as a complex-valued IOR object with a `real` and an `imaginary` field specified as RGB vectors.

The `real` part is often called *n* and it represents the usual index of refraction that is also present in dielectrics, but the spectral dependence is now considered as well. The real part varies over the visible spectrum for dielectrics also (e.g. `[1.521, 1.525, 1.533]` for soda-lime glass), but refraction is difficult for spectrally varying IOR.

The `imaginary` part is often called *k* and it represents the absorption coefficient. The imaginary part is non-zero for conductives and zero for dielectrics, which means that conductives rapidly absorbs the transmitted radiance while dielectrics let it pass through.

Spectral distributions of these values are available at [refractiveindex.info](https://refractiveindex.info/). These spectral distributions can be reduced to linear sRGB by integrating the product of the spectral distributions and each of the CIE color matching functions over the visible spectrum, and then converting the resulting XYZ tristimulus values to linear sRGB. The program does this automatically if a path to a downloaded CSV file with spectral data is provided for the `ior` field, but I also wrote the following MATLAB script to get the values directly:

```matlab
% Read CIE cmfs, http://cvrl.ioo.ucl.ac.uk/cmfs.htm
xyz_cmfs = readmatrix('ciexyz31_1.csv');
xyz_w = xyz_cmfs(:,1); xyz = xyz_cmfs(:,2:4);

% Read complex IOR spectral distribution for iron
data = readtable('Johnson.csv');
[~,index] = ismember("wl",data.wl); % Find start position of k data

n_sRGB = integrate(data(1:index-1, :), xyz, xyz_w);
k_sRGB = integrate(data(index+1:end, :), xyz, xyz_w);

fprintf('"real": [%.8f, %.8f, %.8f],\n', n_sRGB)
fprintf('"imaginary": [%.8f, %.8f, %.8f]\n', k_sRGB)

function sRGB = integrate(data, xyz, xyz_w)
    spd = str2double(data.n);
    spd_w = str2double(data.wl) * 1000; % micro- to nanometers;
    
    % Average duplicate wavelengths
    [spd_w, ~, idx] = unique(spd_w); spd = accumarray(idx, spd, [], @mean);

    % Interpolate to align the spectral data wavelengths with the CMF's
    spd_interp = interp1(spd_w, spd, xyz_w, 'pchip');

    % Integrate using Riemann sum
    XYZ = (xyz' * spd_interp)' / sum(xyz(:,2));

    % Convert to linear sRGB
    sRGB = xyz2rgb(XYZ, 'colorspace', 'linear-rgb', 'WhitePoint', 'e');
end
```

Note that I implicitly use a constant illuminant `I(λ)` and stepsize `Δλ`, which results in:

<pre><code>X = ∫(S(λ)x(λ)I(λ)dλ) / ∫(y(λ)I(λ)dλ) ≈
  ≈ Σ(S(λ)x(λ)I(λ)Δλ) / Σ(y(λ)I(λ)Δλ) =
  = (<del>I(λ)Δλ</del> · Σ(S(λ)x(λ))) / (<del>I(λ)Δλ</del> · Σ(y(λ))) =
  = Σ(S(λ)x(λ)) / Σ(y(λ))</code></pre>

and the same for `Y` and `Z`. The constant illuminant is also the reason why the equal energy white point is used for `xyz2rgb`. A few metal materials based on measured data are available in *scenes/metals.json*.

![Metals with complex IOR based on measured data. Au, Ag, Cu, Fe, Al, Hg, Ni, Pd.](https://user-images.githubusercontent.com/15798094/104471328-010ff380-55bb-11eb-8f04-1f550129c28f.jpg "Metals with complex IOR based on measured data. Au, Ag, Cu, Fe, Al, Hg, Ni, Pd.")

</details>

___

### Vertices

<details><summary>The <code>vertices</code> object contains a map of vertex sets.</summary><br>

Example:
```json
"vertices": {
  "light": [
    [ 8, 4.9, -2.5 ],
    [ 9, 4.9, -2.5 ],
    [ 9, 4.9, -1.5 ],
    [ 8, 4.9, -1.5 ]
  ],
  "crystal": [
    [ 8.28362, -5.0, -4.78046 ],
    [ 6.47867, -0.90516, -3.67389 ],
    [ 7.97071, -0.85108, -2.79588 ],
    [ 7.93553, -0.41379, -4.47145 ],
    [ 6.63966, 3.55331, -2.51368 ]
  ]
}
```

Each vertex set contains an array of vertices specified as xyz-coordinates. The vertex set key string is used later to specify which set of vertices to build the surface from when creating surfaces of `object` type.
</details>

___

### Surfaces

<details><summary>The <code>surfaces</code> object contains an array of surfaces.</summary><br>

Example:
```json
"surfaces": [
  {
    "type": "object",
    "smooth": true,
    "file": "data/stanford_dragon.obj"
  },
  {
    "type": "object",
    "material": "light",
    "vertex_set": "light",
    "triangles": [
      [ 0, 1, 2 ],
      [ 0, 2, 3 ]
    ]
  },
  {
    "type": "object",
    "material": "crystal",
    "vertex_set": "crystal",
    "triangles": [
      [ 0, 2, 1 ],
      [ 0, 3, 2 ],
      [ 0, 1, 3 ],
      [ 2, 4, 1 ],
      [ 1, 4, 3 ],
      [ 3, 4, 2 ]
    ]
  },
  {
    "type": "sphere",
    "position": [ 9.25261, -3.70517, -0.58328 ],
    "radius": 1.15485
  },
  {
    "type": "triangle",
    "material":  "silver",
    "vertices": [ 
      [ 9, 4.9, -2.5 ],
      [ 9, 4.9, -1.5 ],
      [ 8, 4.9, -1.5 ]
    ]
  },
  {
    "type": "quadric",
    "material": "one_sheet_hyperboloid",
    "XX": -1, "YY": 1, "ZZ": 1, "R": -1,
    "bound_dimensions": [1.0, 0.2, 0.2],
    "position": [0.3, 0.3, 0.125],
    "scale": 0.025,
    "rotation": [45, 0, 0]
  }
]
```

Each surface has a `type` field which can be either `sphere`, `triangle`, `object` or `quadric`. All surfaces also has an optional `material` field, which specifies the material that the surface should use by material key string. 

All surface types can also be transformed using the optional `position`, `rotation` (degrees) and `scale` fields specified as xyz-vectors. The remaining fields are type specific.

#### Sphere
The sphere radius is defined by the `radius` field.

#### Triangle
The triangle is simply defined by its vertices, which is defined by the 3 vertices in the vertex array `vertices` in xyz-coordinates. The order of the vertices defines the normal direction.

#### Object
The object surface type defines a triangle mesh object that consists of multiple triangles. The `vertex_set` field can be used to specify the key string of the vertex set to pull vertices from, and the `triangles` field then specifies the array of triangles of the object. Each triangle of the array consists of 3 indices that references the corresponding vertex index in the vertex set. Alternatively, the `file` field can be used to specify a path to an OBJ-file to load instead. The path should be relative to the scenes directory. 

The program uses normal interpolation for smooth shading if the `smooth` field is set to true. This will either compute area+angle weighted vertex normals or use the vertex normals from the OBJ file if they exist.

#### Quadric
A quadric surface consists of all points `(x,y,z)` that satisfies the quadric equation<sup>1</sup>:

<pre><code>Ax<sup>2</sup> + Bxy + Cxz + Dx + Ey<sup>2</sup> + Fyz + Gy + Hz<sup>2</sup> + Iz + J = 0</code></pre>

where `A`, `B`, `C` etc. are real constants. A sphere with radius 1 can for example be defined by:

<pre><code>x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup> - 1 = 0</code></pre>

with constants `J=-1`, `A=E=H=1` and the rest 0. This is achieved in the program by specifying the following fields for a quadric surface:
```json
"XX": 1, "YY": 1, "ZZ": 1, "R": -1,
```
Instead of the usual constant names, I've opted for more descriptive field names that correspond to the expression that the field value is multiplied with in the quadric equation. The `R` field corresponds to `J` in the quadric equation, i.e. the scalar constant added at the end. The value of unspecified constants are set to 0.

The `bound_dimensions` field specifies the dimensions of the axis-aligned bounding box that the quadric surface is confined to.

Quadric surfaces currently do not support emissive materials (the emissive part is simply ignored).

___
<sup>1</sup> The usual quadric equation looks slightly different when it's derived from the quadric matrix representation *p<sup>T</sup>Qp* since this results in some constants being doubled. The program uses this representation internally, but I've eliminated this in the scene format since it's easier to not have to think about whether or not some constants will be doubled when creating a surface.
</details>

___

## Renders

![Path traced render of shells, 7 million triangles.](https://imgur.com/9Im3WBW.jpg "Path traced render of shells, 7 million triangles.")
![Path traced render of lego bulldozer, 2 million triangles. Original scene by Heinzelnisse.](https://imgur.com/zmBM0gP.jpg "Path traced render of lego bulldozer, 2 million triangles. Original scene by Heinzelnisse.")
![Path traced render of coffee maker, 235 049 triangles. Original scene by cekuhnen.](https://imgur.com/38jhuBX.jpg "Path traced render of coffee maker, 235 049 triangles. Original scene by cekuhnen.")
![Path traced render of baroque table, 3.8 million triangles. Original scene by 1DInc.](https://imgur.com/N3cM7Hl.jpg "Path traced render of baroque table, 3.8 million triangles. Original scene by 1DInc.")
![Path traced render of a scene containing only quadric surfaces.](https://user-images.githubusercontent.com/15798094/104470916-847d1500-55ba-11eb-99df-e600d248f495.jpg "Path traced render of a scene containing only quadric surfaces.")
![Path traced render of Veach MIS scene. Original scene by Benedikt Bitterli.](https://imgur.com/oNbmpir.jpg "Path traced render of Veach MIS scene. Original scene by Benedikt Bitterli.")
![Path traced render of piping, 2.4 million triangles. Original scene by seeker47.](https://imgur.com/AGnRbfX.jpg "Path traced render of piping, 2.4 million triangles. Original scene by seeker47.")

## Resources

The following resources have been useful for the project:
* [Physically Based Rendering](http://www.pbr-book.org/) - Matt Pharr, Wenzel Jakob and Greg Humphreys
* [Global Illumination using Photon Maps](http://graphics.stanford.edu/~henrik/papers/ewr7/ewr7.html) - Henrik Wann Jensen
* [Practical Hash-based Owen Scrambling](https://jcgt.org/published/0009/04/01/) - Brent Burley
* [Sobol sequence generator](https://web.maths.unsw.edu.au/~fkuo/sobol/) - S. Joe and F. Y. Kuo
* [Sampling the GGX Distribution of Visible Normals](http://jcgt.org/published/0007/04/01/) - Eric Heitz
* [Importance Sampling techniques for GGX with Smith Masking-Shadowing](https://schuttejoe.github.io/post/ggximportancesamplingpart2/) - Joe Schutte
* [PBR Diffuse Lighting for GGX+Smith Microsurfaces](https://twvideo01.ubm-us.net/o1/vault/gdc2017/Presentations/Hammon_Earl_PBR_Diffuse_Lighting.pdf) - Earl Hammon
* [Memo on Fresnel Equations](https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/) - Sébastien Lagarde
* [Useful Color Equations](http://www.brucelindbloom.com/) - Bruce Lindbloom
* [Filmic Tonemapping Operators](http://filmicworlds.com/blog/filmic-tonemapping-operators/) - John Hable
* [Automatic Exposure](https://knarkowicz.wordpress.com/2016/01/09/automatic-exposure/) - Krzysztof Narkowicz
* [Better Sampling](http://www.rorydriscoll.com/2009/01/07/better-sampling/) - Rory Driscoll
* [Introduction to Acceleration Structures](https://www.scratchapixel.com/lessons/advanced-rendering/introduction-acceleration-structure/bounding-volume-hierarchy-BVH-part1) - Scratchapixel
* [Scenes and assets](scenes/README.md)
