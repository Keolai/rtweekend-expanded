#ifndef MODEL_H
#define MODEL_H

#include "tri.h"

#include "../hittable.h"
#include "../utilities/phy_bvh.h"

#include <set>

struct phy_face_vertex
{
    int v = -1;
    int vn = -1;
};

class model : public phy_hittable
{
public:
    std::vector<std::shared_ptr<phy_hittable>> triangles;

    int model_id = -1;

    model(const std::string &file_path) : file_path(file_path) { load_model(); }
    model(int id) : model_id(id) {}
    model(std::vector<std::shared_ptr<phy_hittable>> &tris, int id) : model_id(id) { triangles = tris; }

    void add_tri(vec3 v1, vec3 v2, vec3 v3)
    {
        std::shared_ptr<phy_tri> new_tri = std::make_shared<phy_tri>(v1, v2, v3);
        triangles.push_back(new_tri);
    }

    bool hit(const ray &r, interval ray_t, phy_hit_record &rec) override
    {
        if (model_node) // in model bvh reduces up rendering by about 40% with simple models
        {
            return model_node->hit(r, ray_t, rec);
        }
        else
        {
            create_bvh();
            return model_node->hit(r, ray_t, rec);
        }
        return false;
    }

    bool load_model()
    {

        std::ifstream file(file_path);

        if (!file.is_open())
        {
            return false;
        }

        std::string line;
        std::set<std::pair<int, int>> edge_pairs; // (low index, high index) so shared edges are only added once

        while (std::getline(file, line))
        {
            if (line.empty())
            {
                continue;
            }

            std::stringstream ss(line);

            std::string prefix;
            ss >> prefix;

            //
            // Vertex position
            //
            if (prefix == "v")
            {

                double x, y, z;
                ss >> x >> y >> z;

                vertices.push_back(vec3(x, y, z));
            }

            //
            // Face
            //
            else if (prefix == "f")
            {

                std::vector<int> face_indices;

                std::string token;

                // Read every face token
                while (ss >> token)
                {

                    int vertex_index = parse_face_index(token);

                    // OBJ indices are 1-based
                    face_indices.push_back(vertex_index - 1);
                }

                // polygon outline, so the diagonals from the fan triangulation below are left out
                for (size_t i = 0; i < face_indices.size(); i++)
                {
                    int a = face_indices[i];
                    int b = face_indices[(i + 1) % face_indices.size()];
                    edge_pairs.insert(std::make_pair(std::min(a, b), std::max(a, b)));
                }

                //
                // Triangulate polygon faces
                // Supports:
                //
                // f v1 v2 v3
                // f v1/vt1 v2/vt2 v3/vt3
                // f v1//vn1 v2//vn2 v3//vn3
                // f v1/vt1/vn1 ...
                // quads/ngons
                //
                if (face_indices.size() >= 3)
                {

                    vec3 v0 = vertices[face_indices[0]];

                    // Triangle fan triangulation
                    for (size_t i = 1; i + 1 < face_indices.size(); i++)
                    {

                        vec3 v1 = vertices[face_indices[i]];
                        vec3 v2 = vertices[face_indices[i + 1]];

                        triangles.push_back(
                            std::make_shared<phy_tri>(v0, v1, v2));
                    }
                }
            }
        } // DONE READING LINES

        // vertices is done growing here, so the edge pointers stay valid
        for (auto &ab : edge_pairs)
        {
            edge e;
            e.u = &vertices[ab.first];
            e.v = &vertices[ab.second];
            edges.push_back(e);
        }

        return true;
    }

    phy_aabb bounding_box() const override // need to add
    {
        if (triangles.empty())
            return phy_aabb();

        phy_aabb box = triangles[0]->bounding_box();

        for (size_t i = 1; i < triangles.size(); i++)
        {
            box = surrounding_box(
                box,
                triangles[i]->bounding_box());
        }
        return box;
    }

    void create_bvh()
    {
        model_node = std::make_shared<phy_bvh_node>(
            triangles,
            0,
            triangles.size());
    }

    void position(vec3 &newPos) override
    {
        pos = newPos;
        for (int i = 0; i < triangles.size(); i++)
        {
            triangles[i]->position(newPos); // maybe
        }
        if (model_node)
        {
            model_node->refit(); // the triangles moved, so the boxes have to follow
        }
    }

    void set_pivot_to_center()
    {
        point3 sum(0, 0, 0);
        for (auto &t : triangles)
        {
            sum += std::static_pointer_cast<phy_tri>(t)->original_centroid();
        }
        pivot = sum / triangles.size();

        for (auto &t : triangles)
        {
            std::static_pointer_cast<phy_tri>(t)->set_pivot(pivot);
        }
    }

    void rotate(const vec3 &degrees)
    {
        for (auto &t : triangles)
        {
            std::static_pointer_cast<phy_tri>(t)->rotate(degrees);
        }
    }

    // RIGID BODY STUFF =========================

    // center of mass is vertex average
    //inertia = solid box from the vertex aabb (diagonal in body space)
    void set_mass(double m)
    {
        mass = m;
        if (vertices.empty())
        {
            return;
        }

        point3 sum(0, 0, 0);
        vec3 lo = vertices[0];
        vec3 hi = vertices[0];
        for (const vec3 &v : vertices)
        {
            sum += v;
            lo = vec3(std::min(lo.x(), v.x()), std::min(lo.y(), v.y()), std::min(lo.z(), v.z()));
            hi = vec3(std::max(hi.x(), v.x()), std::max(hi.y(), v.y()), std::max(hi.z(), v.z()));
        }
        pivot = sum / vertices.size();

        vec3 size = hi - lo;
        double x2 = size.x() * size.x();
        double y2 = size.y() * size.y();
        double z2 = size.z() * size.z();
        vec3 inertia = (m / 12.0) * vec3(y2 + z2, x2 + z2, x2 + y2);
        inv_inertia_body = vec3(safe_inverse(inertia.x()), safe_inverse(inertia.y()), safe_inverse(inertia.z()));
    }

    point3 com_local() const
    {
        return pivot;
    }

    point3 com_world(const state &s) const
    {
        return pivot + s.position;
    }

    // world space inverse inertia applied to v: R * I_body^-1 * R^T * v
    vec3 apply_inv_inertia(const state &s, const vec3 &v) const
    {
        vec3 body_v = s.orientation.conjugate().rotate(v);
        return s.orientation.rotate(inv_inertia_body * body_v);
    }

    // obj space to world space
    vec3 world_point(const state &s, const vec3 &local) const
    {
        return pivot + s.orientation.rotate(local - pivot) + s.position;
    }

    // world space to obj space
    vec3 local_point(const state &s, const vec3 &world) const
    {
        return pivot + s.orientation.conjugate().rotate(world - s.position - pivot);
    }

    std::vector<vec3> world_vertices(const state &s) const
    {
        std::vector<vec3> out;
        out.reserve(vertices.size());
        for (const vec3 &v : vertices)
        {
            out.push_back(world_point(s, v));
        }
        return out;
    }

    // moves the collision triangles to match s
    void set_transform(const state &s)
    {
        pos = s.position;
        for (auto &t : triangles)
        {
            auto tri = std::static_pointer_cast<phy_tri>(t);
            tri->set_pivot(pivot);
            tri->set_orientation(s.orientation);
            tri->position(pos);
        }
        if (model_node)
        {
            model_node->refit();
        }
    }
    // END OF RIGID BODY STUFF ======================
    
    void update_state() override
    {
        copy(next_state, current_state); // copy the next predicted state to the new state;
    }

    double hit_adjuster() const override
    {
        return 0.01;
    }

    bool is_inside(const ray &r) const override
    {
        return false;
    }

    vec3 closest_point_on_surface(const vec3 &pos) const override
    {

        return vec3(0);
    }

    void update_tri_id(int id)
    {
        for (int i = 0; i < triangles.size(); i++)
        {
            auto cur_tri = triangles[i];

            cur_tri->id = id;
        }
        return;
    }

    std::vector<vec3> get_vertices() const override
    {
        return vertices;
    }

    // model space, add current_state.position to get world positions (same as get_vertices)
    std::vector<edge> get_edges() override
    {
        return edges;
    }

    // e is expected in world space, the triangles already are (rotation included)
    bool edge_hit(edge &e, phy_hit_record &rec) override
    {
        if (!model_node)
        {
            create_bvh();
        }
        return model_node->edge_hit(e, rec);
    }

private:
    point3 pos = vec3(0);
    std::string file_path;
    std::vector<vec3> vertices; // original vertices
    std::shared_ptr<phy_bvh_node> model_node;
    std::vector<edge> edges; // original edges

    point3 pivot;                          // also the center of mass once set_mass is called
    vec3 inv_inertia_body = vec3(0, 0, 0); // diagonal, 0 on an axis means it can't spin that way

    static double safe_inverse(double v)
    {
        if (v < 1e-12)
        {
            return 0.0;
        }
        return 1.0 / v;
    }

    int parse_face_index(const std::string &token)
    {

        std::stringstream ss(token);

        std::string index_string;

        std::getline(ss, index_string, '/');

        return std::stoi(index_string);
    }
};

#endif