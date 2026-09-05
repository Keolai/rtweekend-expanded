#ifndef PHY_AABB_H
#define PHY_AABB_H

#include <cmath>

class phy_aabb
{
public:
    interval x, y, z;

    // Empty box
    phy_aabb() {}

    // Construct from intervals
    phy_aabb(const interval &x,
         const interval &y,
         const interval &z)
        : x(x), y(y), z(z) {}

    // Construct from two corner points
    phy_aabb(const point3 &a, const point3 &b)
    {
        x = interval(
            std::fmin(a.x(), b.x()),
            std::fmax(a.x(), b.x()));

        y = interval(
            std::fmin(a.y(), b.y()),
            std::fmax(a.y(), b.y()));

        z = interval(
            std::fmin(a.z(), b.z()),
            std::fmax(a.z(), b.z()));
    }

    // Access axis interval
    const interval &axis(int n) const
    {
        if (n == 0)
            return x;
        if (n == 1)
            return y;
        return z;
    }

    bool hit(const ray &r, interval ray_t) const
    {

        for (int a = 0; a < 3; a++)
        {

            const interval &ax = axis(a);

            double invD = 1.0 / r.direction()[a];
            double orig = r.origin()[a];

            double t0 = (ax.min - orig) * invD;
            double t1 = (ax.max - orig) * invD;

            if (invD < 0.0)
                std::swap(t0, t1);

            if (t0 > ray_t.min)
                ray_t.min = t0;

            if (t1 < ray_t.max)
                ray_t.max = t1;

            if (ray_t.max <= ray_t.min)
                return false;
        }

        return true;
    }
};

inline phy_aabb surrounding_box(const phy_aabb &box0,
                            const phy_aabb &box1)
{
    interval x(
        std::fmin(box0.x.min, box1.x.min),
        std::fmax(box0.x.max, box1.x.max));

    interval y(
        std::fmin(box0.y.min, box1.y.min),
        std::fmax(box0.y.max, box1.y.max));

    interval z(
        std::fmin(box0.z.min, box1.z.min),
        std::fmax(box0.z.max, box1.z.max));

    return phy_aabb(x, y, z);
}

#endif