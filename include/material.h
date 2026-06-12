#ifndef MATERIAL_H
#define MATERIAL_H

#include "hittable.h"
#include "texture.h"

class material {
  public:
    virtual ~material() = default;

    virtual bool scatter(
        const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered
    ) const {
        return false;
    }

    virtual color get_albedo(const hit_record &rec) 
    { return color(1.0);}
};

class lambertian : public material {
  public:
    lambertian(const color& albedo) { mat_texture = texture(albedo);}
    lambertian(const std::string &file_path) {
      mat_texture = texture(file_path);
    }

    bool scatter(const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered)
    const override {
        auto scatter_direction = rec.normal + random_unit_vector();

        // Catch degenerate scatter direction incase random unit is the exact oposite
        if (scatter_direction.near_zero())
            scatter_direction = rec.normal;


        scattered = ray(rec.p, scatter_direction);
        attenuation = mat_texture.get_color_at_coordinate(rec.texture_sample_point.e[0], rec.texture_sample_point.e[1]) * 0.8;
        return true;
    }

    color get_albedo(const hit_record &rec) override {
      return mat_texture.get_color_at_coordinate(rec.texture_sample_point.e[0], rec.texture_sample_point.e[1]);
    }

  private:
    texture mat_texture;
    //color albedo;
};

class metal : public material {
  public:
    metal(const color& albedo, double fuzz) : fuzz(fuzz < 1 ? fuzz : 1) {mat_texture = texture(albedo);}
    metal(const std::string& file_path, double fuzz) : fuzz(fuzz < 1 ? fuzz : 1) {mat_texture = texture(file_path);}

    bool scatter(const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered)
    const override {
        vec3 reflected = reflect(r_in.direction(), rec.normal);
        reflected = unit_vector(reflected) + (fuzz * random_unit_vector());
        scattered = ray(rec.p, reflected);
        attenuation = mat_texture.get_color_at_coordinate(rec.texture_sample_point.e[0], rec.texture_sample_point.e[1]) * 0.8;
        return (dot(scattered.direction(), rec.normal) > 0);
    }

    color get_albedo(const hit_record &rec) override {
      //return albedo;
      return mat_texture.get_color_at_coordinate(rec.texture_sample_point.e[0], rec.texture_sample_point.e[1]);
    }

  private:
    texture mat_texture;
    double fuzz;
};

class dielectric : public material {
  public:
    dielectric(double refraction_index) : refraction_index(refraction_index) {}

    bool scatter(const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered)
    const override {
        attenuation = color(1.0, 1.0, 1.0);
        double ri = rec.front_face ? (1.0/refraction_index) : refraction_index;

        vec3 unit_direction = unit_vector(r_in.direction());
        double cos_theta = std::fmin(dot(-unit_direction, rec.normal), 1.0);
        double sin_theta = std::sqrt(1.0 - cos_theta*cos_theta);

        bool cannot_refract = ri * sin_theta > 1.0;
        vec3 direction;

         if (cannot_refract || reflectance(cos_theta, ri) > random_double())
            direction = reflect(unit_direction, rec.normal);
        else
            direction = refract(unit_direction, rec.normal, ri);

        scattered = ray(rec.p, direction);
        return true;
    }

  private:
    // Refractive index in vacuum or air, or the ratio of the material's refractive index over
    // the refractive index of the enclosing media
    double refraction_index;

    static double reflectance(double cosine, double refraction_index) {
        // Use Schlick's approximation for reflectance.
        auto r0 = (1 - refraction_index) / (1 + refraction_index);
        r0 = r0*r0;
        return r0 + (1-r0)*std::pow((1 - cosine),5);
    }
};

#endif