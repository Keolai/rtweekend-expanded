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
        //iterative traversal
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

        //nodes may reallocate after recursing, dont hold reference or pointers to them
        const int l = build_recursive(start, mid);
        const int r = build_recursive(mid, end);
        nodes[idx].left = l;
        nodes[idx].right = r;
        return idx;
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