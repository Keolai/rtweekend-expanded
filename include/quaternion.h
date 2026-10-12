#ifndef QUAT_H
#define QUAT_H

#include <cmath>
#include "vec3.h"

struct quat
{
    double w, x, y, z;

    quat(double w = 1, double x = 0, double y = 0, double z = 0)
        : w(w), x(x), y(y), z(z) {}

    static quat from_axis_angle(const vec3 &axis, double degrees)
    {
        vec3 a = unit_vector(axis);
        double half = degrees_to_radians(degrees) * 0.5;
        double s = std::sin(half);
        return quat(std::cos(half), a.x() * s, a.y() * s, a.z() * s);
    }

    // Euler degrees -> quaternion
    static quat from_euler(const vec3 &degrees)
    {
        quat qx = from_axis_angle(vec3(1, 0, 0), degrees.x());
        quat qy = from_axis_angle(vec3(0, 1, 0), degrees.y());
        quat qz = from_axis_angle(vec3(0, 0, 1), degrees.z());
        return qz * qy * qx; // rightmost is applied first
    }

    // Hamilton product: (a * b) applies b first, then a
    friend quat operator*(const quat &a, const quat &b)
    {
        return quat(
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w);
    }

    quat normalized() const
    {
        double n = std::sqrt(w * w + x * x + y * y + z * z);
        return quat(w / n, x / n, y / n, z / n);
    }

    // Rotate a vector: v' = v + 2w(u x v) + 2 u x (u x v)
    vec3 rotate(const vec3 &v) const
    {
        vec3 u(x, y, z);
        vec3 t = 2.0 * cross(u, v);
        return v + w * t + cross(u, t);
    }

    static quat from_to(const vec3 &from, const vec3 &to)
    {
        vec3 a = unit_vector(from);
        vec3 b = unit_vector(to);
        double d = dot(a, b);

        if (d > 0.999999) // already aligned
            return quat();

        if (d < -0.999999)
        {
            vec3 axis = cross(vec3(1, 0, 0), a);
            if (axis.length_squared() < 1e-6)
                axis = cross(vec3(0, 1, 0), a);
            return from_axis_angle(axis, 180.0);
        }

        vec3 axis = cross(a, b);
        return quat(1.0 + d, axis.x(), axis.y(), axis.z()).normalized();
    }
};

#endif