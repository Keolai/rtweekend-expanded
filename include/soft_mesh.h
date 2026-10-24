#ifndef SOFT_MESH_H
#define SOFT_MESH_H

#include "tri.h"
#include "bvh.h"
#include "material.h"
#include "physics/objects/spring.h"

#include <array>
#include <vector>
#include <memory>

// refresh() rebuilds the triangles from that copy so rendering never reads live nodes.
class soft_mesh : public hittable
{
public:
    soft_mesh(const std::vector<std::array<int, 3>> &faces, const std::vector<std::array<vec3, 3>> &face_uvs,
              const std::vector<soft_node> &nodes, std::shared_ptr<material> mat)
        : faces(faces), face_uvs(face_uvs), mat(mat)
    {
        set_positions(nodes);
        refresh();
    }

    void set_positions(const std::vector<soft_node> &nodes)
    {
        positions.resize(nodes.size());
        for (size_t i = 0; i < nodes.size(); i++)
        {
            positions[i] = nodes[i].pos;
        }
    }

    void refresh() override
    {
        // smooth normals
        std::vector<vec3> normals(positions.size(), vec3(0));
        for (auto &f : faces)
        {
            vec3 n = cross(positions[f[1]] - positions[f[0]], positions[f[2]] - positions[f[0]]);
            for (int i = 0; i < 3; i++)
            {
                normals[f[i]] += n;
            }
        }
        for (auto &n : normals)
        {
            n = (n.length_squared() > 1e-16) ? unit_vector(n) : vec3(0, 1, 0);
        }

        triangles.clear();
        for (size_t i = 0; i < faces.size(); i++)
        {
            const std::array<int, 3> &f = faces[i];
            std::array<point3, 3> verts = {{positions[f[0]], positions[f[1]], positions[f[2]]}};
            std::array<vec3, 3> norms = {{normals[f[0]], normals[f[1]], normals[f[2]]}};

            // placeholder UVs when the model has none
            std::array<vec3, 3> uvs = face_uvs.empty()
                                          ? std::array<vec3, 3>{{vec3(0, 0, 0), vec3(1, 0, 0), vec3(0, 1, 0)}}
                                          : face_uvs[i];
            triangles.push_back(std::make_shared<tri>(verts, norms, uvs, mat));
        }
        bvh.rebuild(triangles);
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        return bvh.hit(r, ray_t, rec);
    }

    aabb bounding_box() const override
    {
        return bvh.bounding_box();
    }

    void position(vec3 &pos) override {} // driven by set_positions do nothing!!

private:
    std::vector<std::array<int, 3>> faces;
    std::vector<std::array<vec3, 3>> face_uvs;
    std::vector<point3> positions;
    std::vector<std::shared_ptr<hittable>> triangles;
    std::shared_ptr<material> mat;
    bvh_node bvh;
};

#endif
