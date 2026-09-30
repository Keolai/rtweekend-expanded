#ifndef METABALL_H
#define METABALL_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "hittable.h"
#include "physics/objects/particle.h"

class metaball : public hittable
{
public:
    std::vector<particle> *particles;

    // radius: how big a lone, isolated particle appears
    // threshold: iso-level of the surface, in (0, 1), smaller == gooier
    metaball(std::vector<particle> &particles, shared_ptr<material> mat, double radius, double threshold = 0.4)
        : particles(&particles), mat(mat), particle_radius(radius), threshold(threshold)
    {
        // Choose the influence radius R so that a single particle's surface lands at radius
        // (1 - r^2/R^2)^3 = threshold   =>   R = r / sqrt(1 - cbrt(threshold))
        influence_radius = radius / std::sqrt(1.0 - std::cbrt(threshold));
        R2 = influence_radius * influence_radius;
        inv_R2 = 1.0 / R2;
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        const vec3 O = r.origin();
        const vec3 D = r.direction();
        const double a = D.length_squared();
        const double dlen = std::sqrt(a);

        // Gather particles whose influence sphere the ray passes through.
        // Only these can contribute to the field along this ray.
        //(thread_local avoids a heap allocation per ray)
        thread_local std::vector<vec3> nearby;
        nearby.clear();
        double t_lo = std::numeric_limits<double>::infinity();
        double t_hi = -t_lo;

        for (const particle &p : *particles)
        {
            if (!p.alive)
            {
                continue; // skip if dead
            }

            vec3 oc = p.position - O;
            double h = dot(D, oc);
            double c = oc.length_squared() - R2;
            double disc = h * h - a * c;
            if (disc < 0)
            {
                continue;
            }

            double sq = std::sqrt(disc);
            double t0 = std::max((h - sq) / a, ray_t.min);
            double t1 = std::min((h + sq) / a, ray_t.max);
            if (t0 >= t1)
            {
                continue;
            }

            nearby.push_back(p.position);
            t_lo = std::min(t_lo, t0);
            t_hi = std::max(t_hi, t1);
        }
        if (nearby.empty())
        {
            return false; // no particles near
        }

        // March along [t_lo, t_hi] until the field crosses the threshold
        // A "crossing" is in either direction, so a ray that starts inside a blob
        // reports the point where it exits
        const double dt = (influence_radius * march_step) / dlen; // world-space step of R * march_step
        double t_prev = t_lo;
        const bool inside_prev = field_at(nearby, r.at(t_prev)) >= threshold;
        double t_hit = 0.0;
        bool found = false;

        while (t_prev < t_hi)
        {
            double t = std::min(t_prev + dt, t_hi);
            bool inside = field_at(nearby, r.at(t)) >= threshold;

            if (inside != inside_prev)
            {
                // Refine the crossing with bisection.
                double lo = t_prev, hi = t;
                for (int k = 0; k < 10; k++)
                {
                    double mid = 0.5 * (lo + hi);
                    if ((field_at(nearby, r.at(mid)) >= threshold) == inside_prev)
                    {
                        lo = mid;
                    }
                    else
                    {
                        hi = mid;
                    }
                }
                t_hit = 0.5 * (lo + hi);
                found = true;
                break;
            }
            t_prev = t;
        }
        if (!found)
        {
            return false;
        }

        // Normal = direction of steepest field decrease (gradient of F, negated), out of the object
        // d/dp f = -6 q^2 (p - c) / R^2 ; the positive constants vanish after normalizing.
        const vec3 P = r.at(t_hit);
        vec3 n(0, 0, 0);
        for (const vec3 &c : nearby)
        {
            vec3 d = P - c;
            double r2 = d.length_squared();
            if (r2 >= R2)
            {
                continue;
            }
            double q = 1.0 - r2 * inv_R2;
            n = n + d * (q * q);
        }
        double nl = n.length();
        if (nl < 1e-12)
        {
            return false; // gradient vanishes (measure-zero saddle point), let the ray through
        }
        vec3 outward_normal = n / nl;

        rec.t = t_hit;
        rec.p = P;
        rec.set_face_normal(r, outward_normal);
        rec.set_geometry_normal(r, outward_normal);
        rec.mat = mat;

        // Metaballs have no natural parametrization, so reuse the sphere mapping I guess
        double u = 0.5 + std::atan2(outward_normal.z(), outward_normal.x()) / (2.0 * pi);
        double v = 0.5 - std::asin(std::max(-1.0, std::min(1.0, outward_normal.y()))) / pi;
        rec.texture_sample_point = vec3(u, v, 0.);

        vec3 T(-outward_normal.z(), 0, outward_normal.x());
        if (T.length_squared() < 1e-12)
        {
            T = vec3(1, 0, 0);
        }
        else
        {
            T = unit_vector(T);
        }
        rec.tangent = T;
        rec.bitangent = unit_vector(cross(outward_normal, T));

        return true;
    }

    aabb bounding_box() const override
    {
        aabb box;
        bool first = true;
        vec3 rvec(influence_radius, influence_radius, influence_radius);

        for (const particle &p : *particles)
        {
            if (!p.alive)
            {
                continue;
            }

            aabb particle_box(p.position - rvec, p.position + rvec);
            if (first)
            {
                box = particle_box;
                first = false;
            }
            else
            {
                box = surrounding_box(box, particle_box);
            }
        }
        return box;
    }

    void position(vec3 &pos) override
    {
        // do nothing
    }

private:
    shared_ptr<material> mat;
    double particle_radius;
    double threshold;
    double influence_radius = 0.0;
    double R2 = 0.0;
    double inv_R2 = 0.0;
    double march_step = 0.125; // step size as a fraction of R

    double field_at(const std::vector<vec3> &centers, const vec3 &p) const
    {
        double sum = 0.0;
        for (const vec3 &c : centers)
        {
            double r2 = (p - c).length_squared();
            if (r2 < R2)
            {
                double q = 1.0 - r2 * inv_R2;
                sum += q * q * q;
            }
        }
        return sum;
    }
};

#endif