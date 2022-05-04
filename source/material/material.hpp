#pragma once

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <nlohmann/json.hpp>

struct ComplexIOR;

class Material
{
public:
    Material()
    {
        roughness = 0.0;
        specular_roughness = 0.0;
        ior = -1.0;
        complex_ior = nullptr;
        transparency = 0.0;
        difractivity = 0.0;
        perfect_mirror = false;
        isDifractive = false;
        reflectance = glm::dvec3(1.0);
        specular_reflectance = glm::dvec3(1.0);
        emittance = glm::dvec3(0.0);
        transmittance = glm::dvec3(1.0);

        computeProperties();
    }

    glm::dvec3 diffuseReflection(const glm::dvec3& i, const glm::dvec3& o, double& PDF) const;
    glm::dvec3 specularReflection(const glm::dvec3& wi, const glm::dvec3& wo, double& PDF) const;
    glm::dvec3 specularReflectionCustom(const glm::dvec3& wi, const glm::dvec3& wo, double& PDF, glm::dvec3& specReflectance) const;
    glm::dvec3 specularTransmission(const glm::dvec3& wi, const glm::dvec3& wo, double n1, 
                                    double n2, double& PDF, bool inside, bool flux) const;

    glm::dvec3 specularTransmissionCustom(const glm::dvec3& wi, const glm::dvec3& wo, double n1,
                                    double n2, double& PDF, bool inside, bool flux,
                                    glm::dvec3& transmittanceCustom) const;

    glm::dvec3 specularReflectionCustonIOR(const glm::dvec3& wi, const glm::dvec3& wo, double& PDF, double iorCustom) const;

    glm::dvec3 visibleMicrofacet(double u, double v, const glm::dvec3& o) const;

    void computeProperties();

    glm::dvec3 reflectance, specular_reflectance, transmittance, emittance;
    double roughness, specular_roughness, ior, transparency, difractivity;
    std::shared_ptr<ComplexIOR> complex_ior;

    bool rough, rough_specular, opaque, emissive, dirac_delta, isDifractive;

    // Represents ior = infinity -> fresnel factor = 1.0 -> all rays specularly reflected
    bool perfect_mirror;

private:
    glm::dvec3 lambertian() const;
    glm::dvec3 OrenNayar(const glm::dvec3& wi, const glm::dvec3& wo) const;

    // Pre-computed Oren-Nayar variables.
    double A, B;

    // Specular roughness
    glm::dvec2 a;
};

void from_json(const nlohmann::json &j, Material &m);

namespace std
{
    void from_json(const nlohmann::json &j, shared_ptr<Material> &m);
}