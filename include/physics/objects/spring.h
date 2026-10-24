#ifndef SPRING_H
#define SPRING_H

#include <vector>

// one vertex of a soft body
struct soft_node
{
    vec3 pos = vec3(0);
    vec3 vel = vec3(0);
    vec3 force = vec3(0);
    double mass = 1.0;
    bool pinned = false; //is static or not
};

enum class spring_type
{
    structural, // along a mesh edge
    bend        // across two triangles
};

struct spring
{
    int a;
    int b;
    double rest_length;
    double stiffness; // k
    double damping;   // kd
    spring_type type;
};

// hookes law and damping
inline vec3 spring_force(const spring &s, const std::vector<soft_node> &nodes)
{
    vec3 delta = nodes[s.a].pos - nodes[s.b].pos;
    double len = delta.length();
    if (len < 1e-12)
    {
        return vec3(0);
    }
    vec3 dir = delta / len;
    double stretch = len - s.rest_length;
    double rel_vel = dot(nodes[s.a].vel - nodes[s.b].vel, dir);
    return -(s.stiffness * stretch + s.damping * rel_vel) * dir;
}

inline double spring_energy(const spring &s, const std::vector<soft_node> &nodes)
{
    double stretch = (nodes[s.a].pos - nodes[s.b].pos).length() - s.rest_length;
    return 0.5 * s.stiffness * stretch * stretch;
}

#endif
