#ifndef PARTICLE_H
#define PARTICLE_H

#include <cmath>
#include <random>
#include <vector>

#define DEFAULT_MAX 1000

struct particle
{
    vec3 position;
    vec3 velocity;
    double lifespan; // remaining seconds, but can do ticks idk
    double age = 0.0;
    bool alive = false;
};

class particle_emitter
{
public:
    std::vector<particle> particles;

    particle_emitter(vec3 position, vec3 direction, double spread, double speed, double lifespan, double flow, int max) : position(position),
                                                                                                                          direction(direction), spread(spread), speed(speed), lifespan(lifespan), flow(flow), max(max) { particles.resize(max); }

    particle_emitter(vec3 position, vec3 direction, double spread, double speed, double lifespan, double flow) : position(position),
                                                                                                                 direction(direction), spread(spread), speed(speed), lifespan(lifespan), flow(flow), max(DEFAULT_MAX) { particles.resize(max); }
    void spawn(particle &p)
    {
        p.position = position;

        // Sample a direction within a cone of half-angle
        double cos_theta = 1.0 - random_double() * (1.0 - std::cos(spread)); // make sampling uniform
        double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);
        double phi = 2.0 * M_PI * random_double();

        vec3 local_dir(sin_theta * std::cos(phi),
                       sin_theta * std::sin(phi),
                       cos_theta);

        // Build an orthonormal basis (ONB)
        vec3 w = direction; // treat as the local z-axis
        vec3 a = (std::fabs(w.x()) > 0.9) ? vec3(0, 1, 0) : vec3(1, 0, 0);
        vec3 v = unit_vector(cross(w, a));
        vec3 u = cross(w, v);

        // local to world space
        vec3 world_dir = u * local_dir.x() + v * local_dir.y() + w * local_dir.z();

        p.velocity = world_dir * speed;
        p.lifespan = lifespan;
        p.age = 0.0;
        p.alive = true;
    }

    int get_max() { return max; }

    void step(double dt)
    {
        accumulator += flow * (dt / 1000);
        int to_spawn = static_cast<int>(accumulator);
        for (int i = 0; i < particles.size(); i++)
        { // update ages first
            particle &cur_particle = particles[i];
            if (cur_particle.alive)
            {
                cur_particle.age += dt / 1000; // ms per sec
                if (cur_particle.age > lifespan)
                {
                    cur_particle.age = 0.0;     // reset
                    cur_particle.alive = false; // KILL IT!!!
                }
            }
            else
            { // we can use a dead particle to spawn a new one in its place
                if (to_spawn > 0)
                {
                    spawn(cur_particle);
                    to_spawn--;
                    accumulator -= 1.0;
                }
            }
            // now we can check collisions
        }
          if (accumulator > 1.0) accumulator = 1.0;
    }

private:
    vec3 position;
    vec3 direction;
    double speed;
    double lifespan;
    double spread; // theta
    double flow;   // number of particles to come out per second
    double accumulator;
    int max;
};

#endif