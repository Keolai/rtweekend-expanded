#ifndef GROSS_H
#define GROSS_H

#include <algorithm>
#include <cmath>
#include <vector>

#include "../utilities/particle_bvh.h"
#include "../hittable.h"
#include "boids.h"

// if world is nullptr, disables obstacle avoidance
inline void compute_gross_steering(const std::vector<particle> &particles,
                                   particle_bvh &bvh,
                                   const flock_params &fp,
                                   std::vector<vec3> &steering,
                                   phy_hittable *world = nullptr)
{
    steering.assign(particles.size(), vec3(0, 0, 0));
    if (fp.use_bvh)
    {
        bvh.build(particles);
    }

    std::vector<int> neighbors;
    std::vector<int> icky_neighbors;
    const double sep2 = fp.separation_radius * fp.separation_radius;

    for (size_t i = 0; i < particles.size(); i++)
    {
        const particle &p = particles[i];
        if (!p.alive)
        {
            continue;
        }

        // flocking forces
        vec3 flock(0, 0, 0);

        if (fp.use_bvh)
        {
            bvh.query_radius(p.position, std::max(fp.gross_avoidance, fp.neighbor_radius), neighbors);
        }
        else
        {
            brute_force_neighbors(particles, p.position, std::max(fp.gross_avoidance, fp.neighbor_radius), neighbors);
        }

        vec3 center(0, 0, 0);  // sum of neighbor positions
        vec3 avg_vel(0, 0, 0); // sum of neighbor velocities
        vec3 away(0, 0, 0);    // sum of "push away" vectors from too-close neighbors
        int count = 0;

        for (int j : neighbors)
        {
            if (j == static_cast<int>(i)) {
                continue; // skip self
            }

            const particle &q = particles[j];
            if (q.is_gross){
                icky_neighbors.push_back(j);
                continue;
            }
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

        away = vec3(0, 0, 0);    
        double gross2 = fp.gross_avoidance * fp.gross_avoidance;

        for (int j: icky_neighbors){
            const particle &q = particles[j];

            vec3 d = p.position - q.position;
            double d2 = d.length_squared();
            if (d2 < gross2 && d2 > TOO_SMALL) {
                away = away + d / d2; // magnitude 1/dist: closer neighbors push harder
            }
        }

        if (count > 0){
            vec3 separation = steer_toward(away, p.velocity, fp);

            flock = flock + (fp.gross_weight * separation);
        }

        icky_neighbors.clear(); //I guess

        // obstacle avoidance with priority over flocking
        vec3 total(0, 0, 0);
        if (world)
        {
            total = limit_length(avoid_obstacles(p, *world, fp) * fp.avoid_w, fp.max_force);
        }

        double remaining = fp.max_force - total.length();
        if (remaining > 0)
        {
            total = total + limit_length(flock, remaining);
        }

        steering[i] = total;
    }
}

#endif