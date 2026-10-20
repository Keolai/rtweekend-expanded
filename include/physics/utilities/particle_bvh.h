#ifndef PARTICLE_BVH_H
#define PARTICLE_BVH_H

#include <algorithm>
#include <limits>
#include <vector>

#include "../objects/particle.h"

class particle_bvh
{
public:
    void build(const std::vector<particle> &particles)
    {
        refs.clear();
        nodes.clear();
 
        for (size_t i = 0; i < particles.size(); i++)
        {
            if (particles[i].alive)
                refs.push_back({particles[i].position, static_cast<int>(i)});
        }
        if (refs.empty())
            return;
 
        build_recursive(0, refs.size());
    }
 
    // Fills vector out with the indices of all alive particles within radius
    void query_radius(const vec3 &center, double radius, std::vector<int> &out) const
    {
        out.clear();
        if (nodes.empty()) {
            return;
        }
 
        const double r2 = radius * radius;
        //iterative traversal ok
        int stack[64];
        int top = 0;
        stack[top++] = 0; // root
 
        while (top > 0)
        {
            const node &nd = nodes[stack[--top]];
 
            if (box_dist_sq(center, nd) > r2){ continue; } //outside
 
            if (nd.left < 0)
            {
                // leaf
                for (int k = nd.start; k < nd.start + nd.count; k++)
                {
                    const ref &rf = refs[k];
                    if ((rf.position - center).length_squared() <= r2) {
                        out.push_back(rf.index);
                    }
                }
            }
            else
            {
                stack[top++] = nd.left;
                stack[top++] = nd.right;
            }
        }
    }
 
    // Calls visit(index, position) for particles in leaves whose box, padded by pad, the ray overlaps.
    // ray_t is re-read at every node, so visit may shrink the caller's ray_t.max to cull farther nodes.
    template <typename F>
    void query_ray(const ray &r, const interval &ray_t, double pad, F visit) const
    {
        if (nodes.empty())
        {
            return;
        }

        const vec3 orig = r.origin();
        const vec3 dir = r.direction();
        const vec3 inv_dir(1.0 / dir.x(), 1.0 / dir.y(), 1.0 / dir.z());
        const vec3 pad_vec(pad, pad, pad);

        int stack[64];
        int top = 0;
        stack[top++] = 0;

        while (top > 0)
        {
            const node &nd = nodes[stack[--top]];
            double t_enter;
            if (!ray_box(orig, inv_dir, nd.lo - pad_vec, nd.hi + pad_vec, ray_t, t_enter))
            {
                continue;
            }

            if (nd.left < 0)
            {
                for (int k = nd.start; k < nd.start + nd.count; k++)
                {
                    visit(refs[k].index, refs[k].position);
                }
                continue;
            }

            const node &l = nodes[nd.left];
            const node &rt = nodes[nd.right];
            double t_left, t_right;
            bool hit_left = ray_box(orig, inv_dir, l.lo - pad_vec, l.hi + pad_vec, ray_t, t_left);
            bool hit_right = ray_box(orig, inv_dir, rt.lo - pad_vec, rt.hi + pad_vec, ray_t, t_right);

            // push the far child first so the near one is visited first
            if (hit_left && hit_right)
            {
                if (t_left < t_right)
                {
                    stack[top++] = nd.right;
                    stack[top++] = nd.left;
                }
                else
                {
                    stack[top++] = nd.left;
                    stack[top++] = nd.right;
                }
            }
            else if (hit_left)
            {
                stack[top++] = nd.left;
            }
            else if (hit_right)
            {
                stack[top++] = nd.right;
            }
        }
    }

    // Bounds of all alive particle centers, false if the tree is empty
    bool bounds(vec3 &lo, vec3 &hi) const
    {
        if (nodes.empty())
        {
            return false;
        }
        lo = nodes[0].lo;
        hi = nodes[0].hi;
        return true;
    }

    size_t size() const { return refs.size(); } // number of alive particles in the tree
 
private:
    static constexpr int LEAF_SIZE = 4; // for brute force
 
    struct ref
    {
        vec3 position;
        int index;
    };
 
       struct node
    {
        vec3 lo; // bounding box min
        vec3 hi; // bounding box max
        int left = -1;  // child node indices, -1 for leaves
        int right = -1;
        int start = 0;  // leaf only
        int count = 0;
    };
 
    std::vector<ref> refs;
    std::vector<node> nodes;
 
    // Builds the subtree over refs start to end, returns index
    int build_recursive(size_t start, size_t end)
    {
        const int idx = static_cast<int>(nodes.size());
        nodes.emplace_back();
 
        // Bounding box over the points in this range.
        const double inf = std::numeric_limits<double>::infinity();
        vec3 lo(inf, inf, inf);
        vec3 hi(-inf, -inf, -inf);
        for (size_t i = start; i < end; i++)
        {
            for (int a = 0; a < 3; a++)
            {
                double c = refs[i].position[a];
                lo[a] = std::min(lo[a], c);
                hi[a] = std::max(hi[a], c);
            }
        }
        nodes[idx].lo = lo;
        nodes[idx].hi = hi;
 
        const size_t span = end - start;
        if (span <= static_cast<size_t>(LEAF_SIZE))
        {
            nodes[idx].start = static_cast<int>(start);
            nodes[idx].count = static_cast<int>(span);
            return idx; // left/right stay -1 => leaf
        }
 
        // Split along the longest axis at the median O(n)?
        int axis = 0;
        double best = hi[0] - lo[0];
        for (int a = 1; a < 3; a++)
        {
            if (hi[a] - lo[a] > best)
            {
                best = hi[a] - lo[a];
                axis = a;
            }
        }
 
        const size_t mid = start + span / 2;
        std::nth_element(refs.begin() + start, refs.begin() + mid, refs.begin() + end,
                         [axis](const ref &a, const ref &b)
                         { return a.position[axis] < b.position[axis]; });

        //nodes may reallocate after recursing, dont hold reference or pointers to them!
        const int l = build_recursive(start, mid);
        const int r = build_recursive(mid, end);
        nodes[idx].left = l;
        nodes[idx].right = r;
        return idx;
    }
 
    static bool ray_box(const vec3 &orig, const vec3 &inv_dir, const vec3 &lo, const vec3 &hi,
                        const interval &ray_t, double &t_enter)
    {
        double t_min = ray_t.min;
        double t_max = ray_t.max;
        for (int a = 0; a < 3; a++)
        {
            double t0 = (lo[a] - orig[a]) * inv_dir[a];
            double t1 = (hi[a] - orig[a]) * inv_dir[a];
            if (t0 > t1)
            {
                std::swap(t0, t1);
            }
            if (t0 > t_min)
            {
                t_min = t0;
            }
            if (t1 < t_max)
            {
                t_max = t1;
            }
            if (t_max < t_min)
            {
                return false;
            }
        }
        t_enter = t_min;
        return true;
    }

    // Squared distance from p to node's box, 0 if inside
    static double box_dist_sq(const vec3 &p, const node &nd)
    {
        double d2 = 0.0;
        for (int a = 0; a < 3; a++)
        {
            double v = p[a];
            if (v < nd.lo[a])
                d2 += (nd.lo[a] - v) * (nd.lo[a] - v);
            else if (v > nd.hi[a])
                d2 += (v - nd.hi[a]) * (v - nd.hi[a]);
        }
        return d2;
    }
};


#endif