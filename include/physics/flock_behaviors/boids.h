#ifndef BOIDS_H
#define BOIDS_H

#include <algorithm>
#include <cmath>
#include <vector>

#include "../utilities/particle_bvh.h"
#include "../hittable.h"

#define TOO_SMALL 1e-6

inline vec3 limit_length(const vec3 &v, double max_len)
{
    double l2 = v.length_squared();
    if (l2 > max_len * max_len) {
        return v * (max_len / std::sqrt(l2));
    }
    return v;
}

// Reynolds-style steering
inline vec3 steer_toward(const vec3 &desired_dir, const vec3 &velocity, const flock_params &fp)
{
    double l2 = desired_dir.length_squared();
    if (l2 < TOO_SMALL) {
        return vec3(0, 0, 0);
    }
    vec3 desired_velocity = desired_dir * (fp.max_speed / std::sqrt(l2));
    return limit_length(desired_velocity - velocity, fp.max_force);
}

// Soft obstacle avoidance, uses feeler ray
inline vec3 avoid_obstacles(const particle &p, phy_hittable &world, const flock_params &fp)
{
    double speed = p.velocity.length();
    if (speed < TOO_SMALL)
    { // not moving, just return
        return vec3(0, 0, 0);
    }

    vec3 dir = p.velocity / speed;
    double look = fp.avoid_lookahead + speed * fp.avoid_lookahead_time;

    phy_hit_record rec;
    if (!world.hit(ray(p.position, dir), interval(0.001, look), rec))
    {
        return vec3(0, 0, 0);
    }

    double urgency = 1.0 - rec.t / look; // 0 at the tip of the feeler  1 at contact

    vec3 n = rec.normal;
    if (dot(n, dir) > 0)
    {
        n = -n; // make sure the normal points back toward us, whatever the hit record's convention
    }

    // Slide along the surface instead of just braking: aim away from the surface while
    // keeping the sideways part of motion
    vec3 tangent = dir - n * dot(dir, n);
    if (tangent.length_squared() < TOO_SMALL)
    {
        // head on hit
        tangent = cross(n, std::fabs(n.y()) < 0.9 ? vec3(0, 1, 0) : vec3(1, 0, 0));
    }

    vec3 desired = n + unit_vector(tangent);
    return steer_toward(desired, p.velocity, fp) * urgency;
}

inline void brute_force_neighbors(const std::vector<particle> &particles, const vec3 &center,
                                  double radius, std::vector<int> &out)
{
    out.clear();
    const double r2 = radius * radius;
    for (size_t j = 0; j < particles.size(); j++)
        if (particles[j].alive && (particles[j].position - center).length_squared() <= r2)
            out.push_back(static_cast<int>(j));
}

// if world is nullptr, disables obstacle avoidance
inline void compute_flock_steering(const std::vector<particle> &particles,
                                   particle_bvh &bvh,
                                   const flock_params &fp,
                                   std::vector<vec3> &steering,
                                   phy_hittable *world = nullptr)
{
    steering.assign(particles.size(), vec3(0, 0, 0));
    if (fp.use_bvh){
    bvh.build(particles);
    }

    std::vector<int> neighbors;
    const double sep2 = fp.separation_radius * fp.separation_radius;

    for (size_t i = 0; i < particles.size(); i++)
    {
        const particle &p = particles[i];
        if (!p.alive) {
            continue;
        }

        //flocking forces
        vec3 flock(0, 0, 0);
        if(fp.use_bvh) {
            bvh.query_radius(p.position, fp.neighbor_radius, neighbors);
        } else {
            brute_force_neighbors(particles, p.position, fp.neighbor_radius, neighbors);
        }

        vec3 center(0, 0, 0);  // sum of neighbor positions
        vec3 avg_vel(0, 0, 0); // sum of neighbor velocities
        vec3 away(0, 0, 0);    // sum of "push away" vectors from too-close neighbors
        int count = 0;

        for (int j : neighbors)
        {
            if (j == static_cast<int>(i))
                continue; // skip self

            const particle &q = particles[j];
            center = center + q.position;
            avg_vel = avg_vel + q.velocity;
            count++;

            vec3 d = p.position - q.position;
            double d2 = d.length_squared();
            if (d2 < sep2 && d2 > TOO_SMALL)
                away = away + d / d2; // magnitude 1/dist: closer neighbors push harder
        }

        if (count > 0)
        {
            center = center / count;
            avg_vel = avg_vel / count;

            vec3 cohesion = steer_toward(center - p.position, p.velocity, fp);
            vec3 alignment = steer_toward(avg_vel, p.velocity, fp);
            vec3 separation = steer_toward(away, p.velocity, fp);

            flock = cohesion * fp.cohesion_w + alignment * fp.alignment_w + separation * fp.separation_w;
        }

        //obstacle avoidance with priority over flocking
        vec3 total(0, 0, 0);
        if (world) {
            total = limit_length(avoid_obstacles(p, *world, fp) * fp.avoid_w, fp.max_force);
        }

        double remaining = fp.max_force - total.length();
        if (remaining > 0) {
            total = total + limit_length(flock, remaining);
        }

        steering[i] = total;
    }
}

#endif