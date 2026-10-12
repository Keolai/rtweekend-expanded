#ifndef SPH_H
#define SPH_H

#include <algorithm>
#include <cmath>
#include <vector>

#include "../utilities/particle_bvh.h"
#include "boids.h" // behavior param enum and limit_length

// working storage
struct sph_scratch
{
    std::vector<int> nbr_start; // size n+1; neighbors of i are nbr_index[nbr_start[i] .. nbr_start[i+1])
    std::vector<int> nbr_index; // flat cache of every particle's neighbor list (includes self)
    std::vector<double> density;
    std::vector<double> pressure;
    std::vector<int> tmp;
};

// kernel normalization constants for 3D smoothing radius h
// https://matthias-research.github.io/pages/publications/sca03.pdf "Particle-Based Fluid Simulation for Interactive Applications"
struct sph_kernels
{
    double h, h2;
    double poly6;
    double spiky_grad;
    double visc_lap; 

    explicit sph_kernels(double h) : h(h), h2(h * h)
    {
        const double PI = 3.14159265358979323846;
        poly6 = 315.0 / (64.0 * PI * std::pow(h, 9));
        spiky_grad = -45.0 / (PI * std::pow(h, 6));
        visc_lap = 45.0 / (PI * std::pow(h, 6));
    }
};

inline double estimate_rest_density(double h, double spacing, double mass)
{
    const sph_kernels K(h);
    const int N = static_cast<int>(std::ceil(h / spacing));
    double rho = 0.0;
    for (int ix = -N; ix <= N; ix++)
        for (int iy = -N; iy <= N; iy++)
            for (int iz = -N; iz <= N; iz++)
            {
                double r2 = spacing * spacing * (ix * ix + iy * iy + iz * iz);
                if (r2 < K.h2)
                {
                    double d = K.h2 - r2;
                    rho += mass * K.poly6 * d * d * d;
                }
            }
    return rho;
}

// one acceleration per particle (zero for dead particles), only reads pos and velocity
template <typename Params>
inline void compute_sph_accel(const std::vector<particle> &particles,
                              particle_bvh &bvh,
                              const Params &sp,
                              sph_scratch &s,
                              std::vector<vec3> &accel)
{
    const size_t n = particles.size();
    accel.assign(n, vec3(0, 0, 0));
    s.density.assign(n, 0.0);
    s.pressure.assign(n, 0.0);
    s.nbr_start.assign(n + 1, 0);
    s.nbr_index.clear();

    const sph_kernels K(sp.neighbor_radius);
    const double m = sp.particle_mass;

    bvh.build(particles);

    // cache neighbor list of each particle
    for (size_t i = 0; i < n; i++)
    {
        s.nbr_start[i] = static_cast<int>(s.nbr_index.size());
        if (!particles[i].alive)
        {
            continue; // empty range
        }
        bvh.query_radius(particles[i].position, K.h, s.tmp);
        s.nbr_index.insert(s.nbr_index.end(), s.tmp.begin(), s.tmp.end());
    }
    s.nbr_start[n] = static_cast<int>(s.nbr_index.size());

    // density and pressure
    for (size_t i = 0; i < n; i++)
    {
        if (!particles[i].alive)
        {
            continue;
        }

        double rho = 0.0;
        for (int k = s.nbr_start[i]; k < s.nbr_start[i + 1]; k++)
        {
            const particle &q = particles[s.nbr_index[k]];
            double r2 = (particles[i].position - q.position).length_squared();
            if (r2 < K.h2)
            {
                double d = K.h2 - r2;
                rho += m * K.poly6 * d * d * d; // includes self (r = 0), as SPH requires
            }
        }
        s.density[i] = rho;
        // clamp at 0, negative pressure makes particles clump together instead of spreading out
        s.pressure[i] = std::max(0.0, sp.stiffness * (rho - sp.rest_density));
    }

    // pass 2: pressure and viscosity accelerations
    for (size_t i = 0; i < n; i++)
    {
        if (!particles[i].alive)
        {
            continue;
        }

        const particle &pi = particles[i];
        const double rho_i = s.density[i];
        const double p_i_term = s.pressure[i] / (rho_i * rho_i);

        vec3 a(0, 0, 0);
        for (int k = s.nbr_start[i]; k < s.nbr_start[i + 1]; k++)
        {
            const int j = s.nbr_index[k];
            if (j == static_cast<int>(i))
            {
                continue;
            }

            const particle &pj = particles[j];
            vec3 rij = pi.position - pj.position;
            double r2 = rij.length_squared();
            if (r2 >= K.h2 || r2 < 1e-12) {
                continue; // outside support, or exactly coincident (no direction)
            }

            const double r = std::sqrt(r2);
            const double rho_j = s.density[j];

            // pressure
            // a_i = -sum_j m * (p_i/rho_i^2 + p_j/rho_j^2) * gradW(r_ij)
            vec3 gradW = rij * (K.spiky_grad * (K.h - r) * (K.h - r) / r);
            double p_term = p_i_term + s.pressure[j] / (rho_j * rho_j);
            a = a - gradW * (m * p_term);

            // viscosity
            // a_i = (mu / rho_i) * sum_j m * (v_j - v_i) / rho_j * lapW(r)
            a = a + (pj.velocity - pi.velocity) * (sp.viscosity * m * K.visc_lap * (K.h - r) / (rho_i * rho_j));
        }

        accel[i] = limit_length(a, sp.max_accel);
    }
}

#endif