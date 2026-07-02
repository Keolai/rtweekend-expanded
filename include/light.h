#ifndef LIGHT_H
#define LIGHT_H

#include "vec3.h"
#include "color.h"
#include "hittable.h"
#include "ray.h"

class light
{
public:
    light() {}
    double power = 75;

    light(const point3 &origin, const color light_color) : orig(origin), light_color(light_color) { radius = 1.; }
    light(const point3 &origin, const color light_color, const double power) : orig(origin), light_color(light_color), power(power) { radius = 1.; }

    virtual const point3 &origin() const { return orig; }
    virtual const vec3 &get_color() const { return light_color; }
    virtual const float get_radius() const { return radius; }

    color light_hit(const hittable &world, const ray &r, hit_record &rec, int shadow_samples) const
    {
        hit_record shadow_rec;
        color lighting = color(0.);
        for (int j = 0; j < shadow_samples; j++)
        {
            lighting += light_sample(world, r, rec) / shadow_samples;
        }
        return lighting;
    }

    virtual color light_sample(const hittable &world, const ray &r, hit_record &rec) const
    {
        hit_record shadow_rec;
        color lighting = color(0.);
        vec3 jitter = random_in_unit_sphere() * this->get_radius();
        vec3 to_light = ((this->origin()) + jitter) - rec.p;
        ray light_direction = ray(rec.p + 0.001 * rec.geometry_normal, unit_vector(to_light));

        if (!world.hit(light_direction, interval(0.001, to_light.length()), shadow_rec)) // not blocked
        {
            double NdotL =
                std::max(0.,
                         dot(rec.geometry_normal,
                             light_direction.direction()));

            lighting =
                rec.mat->get_albedo(rec) * this->get_color() * this->power *
                NdotL /
                (to_light.length() * to_light.length());
        }
        return lighting;
    }

protected:
    point3 orig;
    color light_color;
    float radius;
};

class spot_light : public light
{
public:
    spot_light() = default;

    spot_light(const point3 &origin, const color light_color, double power)
        : light(origin, light_color, power)
    {
        inner_cone = 0.8;
        outer_cone = 1.13;
        direction = vec3(0, -1, 0);
        cos_inner = std::cos(inner_cone);
        cos_outer = std::cos(outer_cone);
    }

    spot_light(const point3 &origin, const color light_color, double power, double inner_cone, double outer_cone, const vec3 &direction)
        : light(origin, light_color, power),
          inner_cone(inner_cone),
          outer_cone(outer_cone), direction(direction)
    {
        cos_inner = std::cos(inner_cone);
        cos_outer = std::cos(outer_cone);
    }

    color light_sample(const hittable &world, const ray &r, hit_record &rec) const override
    {
        hit_record shadow_rec;
        color lighting = color(0.);
        vec3 jitter = random_in_unit_sphere() * this->get_radius();
        vec3 to_light = ((this->origin()) + jitter) - rec.p;
        double distance = to_light.length();
        ray light_direction = ray(rec.p + 0.001 * rec.geometry_normal, unit_vector(to_light));

        if (!world.hit(light_direction, interval(0.001, distance), shadow_rec)) // not blocked
        {
            double NdotL =
                std::max(0.,
                         dot(rec.geometry_normal,
                             light_direction.direction()));

            vec3 wi = to_light / distance;

            double true_falloff = (dot(-1 * wi, unit_vector(direction)) - cos_outer) / (cos_inner - cos_outer);

            double falloff = std::max(0., std::min(1., true_falloff));

            lighting =
                rec.mat->get_albedo(rec) * this->get_color() * this->power * NdotL * falloff /
                ((distance * distance));
        }

        return lighting;
    }

protected:
    double inner_cone; // in radians
    double outer_cone;
    double cos_inner;
    double cos_outer;
    vec3 direction;
};

#endif