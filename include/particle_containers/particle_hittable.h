#ifndef PARTICLE_HITTABLE_H
#define PARTICLE_HITTABLE_H

#include <vector>

#include "../hittable.h"
#include "../physics/utilities/particle_bvh.h"

// Base for renderable particle containers: hit() and bounding_box() only read snapshot,
// which refresh() copies from the live particles once per frame, so rendering never reads
// particles the physics thread is writing and the BVH always matches what is traced.
class particle_hittable : public hittable
{
public:
    std::vector<particle> *particles;

    explicit particle_hittable(std::vector<particle> &particles) : particles(&particles) {}

    void refresh() override
    {
        snapshot = *particles;
        bvh.build(snapshot);
    }

    aabb bounding_box() const override
    {
        vec3 lo, hi;
        if (!bvh.bounds(lo, hi))
        {
            return aabb();
        }
        double r = bound_radius();
        vec3 pad(r, r, r);
        return aabb(lo - pad, hi + pad);
    }

    void position(vec3 &pos) override {}

protected:
    std::vector<particle> snapshot;
    particle_bvh bvh;

    // world-space radius around a particle center that contains everything it renders
    virtual double bound_radius() const = 0;
};

#endif
