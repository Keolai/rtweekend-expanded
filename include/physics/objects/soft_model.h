#ifndef SOFT_MODEL_H
#define SOFT_MODEL_H

#include "spring.h"
#include "soft_tri.h"

#include "../hittable.h"
#include "../utilities/phy_bvh.h"

#include <array>
#include <map>
#include <set>

// every vertex is a point mass (soft_node), every edge is a spring,
// and the triangles only reference node indices so they follow the nodes around ohh
class soft_model : public phy_hittable
{
public:
    std::vector<soft_node> nodes;
    std::vector<std::array<int, 3>> faces;
    std::vector<std::array<vec3, 3>> face_uvs; // empty when the source has no UVs
    std::vector<spring> springs;
    std::vector<std::shared_ptr<phy_hittable>> triangles;

    double stretch_k = 200.0;
    double bend_k = 20.0;
    double spring_damping = 0.5;
    double pressure = 0.0; // only for closed meshes, explodes open ones LOL
    double rest_volume = 0.0;

    soft_model(const std::string &file_path, double node_mass = 0.05) : file_path(file_path)
    {
        load_soft_model(node_mass);
        finalize();
    }

    // flat grid in the xz plane centered on the origin
    soft_model(double width, double depth, int res_x, int res_z, double node_mass = 0.05)
    {
        make_cloth(width, depth, res_x, res_z, node_mass);
        finalize();
    }

    bool hit(const ray &r, interval ray_t, phy_hit_record &rec) override
    {
        return soft_model_node->hit(r, ray_t, rec);
    }

    phy_aabb bounding_box() const override
    {
        return soft_model_node->bounding_box();
    }

    void set_stiffness(double stretch, double bend, double damping)
    {
        stretch_k = stretch;
        bend_k = bend;
        spring_damping = damping;
        for (auto &s : springs)
        {
            s.stiffness = (s.type == spring_type::structural) ? stretch_k : bend_k;
            s.damping = spring_damping;
        }
    }

    void pin(int node)
    {
        nodes[node].pinned = true;
    }

    template <typename F>
    void pin_where(F predicate)
    {
        for (auto &n : nodes)
        {
            if (predicate(n.pos))
            {
                n.pinned = true;
            }
        }
    }

    // translate the rest shape, only meant for scene setup
    void position(vec3 &newPos) override
    {
        vec3 offset = newPos - pos;
        pos = newPos;
        for (auto &n : nodes)
        {
            n.pos += offset;
        }
        update_centroid();
        soft_model_node->refit();
    }

    // rotate the rest shape about its centroid, only meant for scene setup
    void rotate(const vec3 &degrees)
    {
        quat q = quat::from_euler(degrees);
        point3 center = centroid();
        for (auto &n : nodes)
        {
            n.pos = center + q.rotate(n.pos - center);
        }
        soft_model_node->refit();
    }

    // signed, so a mesh with inward winding gives a negative volume
    double volume() const
    {
        point3 c = centroid();
        double v = 0.0;
        for (auto &f : faces)
        {
            v += dot(nodes[f[0]].pos - c, cross(nodes[f[1]].pos - c, nodes[f[2]].pos - c)) / 6.0;
        }
        return v;
    }

    void add_pressure_forces()
    {
        double cur_volume = volume();
        double sign = (rest_volume < 0) ? -1.0 : 1.0;
        double scale = sign * pressure * (rest_volume / cur_volume - 1.0);
        for (auto &f : faces)
        {
            // cross / 2 is the area weighted normal, split over the 3 corners
            vec3 face_force = scale * cross(nodes[f[1]].pos - nodes[f[0]].pos, nodes[f[2]].pos - nodes[f[0]].pos) / 6.0;
            for (int i = 0; i < 3; i++)
            {
                nodes[f[i]].force += face_force;
            }
        }
    }

    point3 centroid() const
    {
        point3 sum(0, 0, 0);
        for (auto &n : nodes)
        {
            sum += n.pos;
        }
        return sum / nodes.size();
    }

    void update_centroid()
    {
        current_state.position = centroid();
        next_state.position = current_state.position;
    }

    void refit()
    {
        soft_model_node->refit();
    }

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

    std::vector<vec3> get_vertices() const override
    {
        std::vector<vec3> tmp;
        for (auto &n : nodes)
        {
            tmp.push_back(n.pos);
        }
        return tmp;
    }

    std::vector<edge> get_edges() override {
        return edges;
    }

    bool edge_hit(edge &e, phy_hit_record &rec) override
    {
        return soft_model_node->edge_hit(e, rec);
    }

private:
    point3 pos = vec3(0);
    std::string file_path;
    std::shared_ptr<phy_bvh_node> soft_model_node;
    std::vector<edge> edges;

    bool load_soft_model(double node_mass)
    {
        std::ifstream file(file_path);

        if (!file.is_open())
        {
            return false;
        }

        std::vector<vec3> texcoords;
        bool all_faces_have_uvs = true;
        std::string line;

        while (std::getline(file, line))
        {
            if (line.empty())
            {
                continue;
            }

            std::stringstream ss(line);

            std::string prefix;
            ss >> prefix;

            if (prefix == "v")
            {
                double x, y, z;
                ss >> x >> y >> z;

                soft_node n;
                n.pos = vec3(x, y, z);
                n.mass = node_mass;
                nodes.push_back(n);
            }
            else if (prefix == "vt")
            {
                double u, v;
                ss >> u >> v;
                texcoords.push_back(vec3(u, v, 0));
            }
            else if (prefix == "f")
            {
                std::vector<int> face_indices;
                std::vector<int> uv_indices;

                std::string token;
                while (ss >> token)
                {
                    int uv_index = -1;
                    face_indices.push_back(parse_face_index(token, uv_index));
                    uv_indices.push_back(uv_index);
                }

                // triangle fan, obj v indices are shared between faces so the mesh comes out welded
                for (size_t i = 1; i + 1 < face_indices.size(); i++)
                {
                    std::array<int, 3> face = {{face_indices[0], face_indices[i], face_indices[i + 1]}};
                    faces.push_back(face);

                    std::array<int, 3> uv = {{uv_indices[0], uv_indices[i], uv_indices[i + 1]}};
                    if (uv[0] < 0 || uv[1] < 0 || uv[2] < 0)
                    {
                        all_faces_have_uvs = false;
                        face_uvs.push_back(std::array<vec3, 3>());
                    }
                    else
                    {
                        std::array<vec3, 3> face_uv = {{texcoords[uv[0]], texcoords[uv[1]], texcoords[uv[2]]}};
                        face_uvs.push_back(face_uv);
                    }
                }
            }
        }

        if (!all_faces_have_uvs)
        {
            face_uvs.clear();
        }
        return true;
    }

    void make_cloth(double width, double depth, int res_x, int res_z, double node_mass)
    {
        for (int z = 0; z <= res_z; z++)
        {
            for (int x = 0; x <= res_x; x++)
            {
                soft_node n;
                n.pos = vec3(width * ((double)x / res_x - 0.5), 0, depth * ((double)z / res_z - 0.5));
                n.mass = node_mass;
                nodes.push_back(n);
            }
        }

        for (int z = 0; z < res_z; z++)
        {
            for (int x = 0; x < res_x; x++)
            {
                int i00 = z * (res_x + 1) + x;
                int i10 = i00 + 1;
                int i01 = i00 + res_x + 1;
                int i11 = i01 + 1;

                vec3 uv00((double)x / res_x, (double)z / res_z, 0);
                vec3 uv10((double)(x + 1) / res_x, (double)z / res_z, 0);
                vec3 uv01((double)x / res_x, (double)(z + 1) / res_z, 0);
                vec3 uv11((double)(x + 1) / res_x, (double)(z + 1) / res_z, 0);

                std::array<int, 3> f0 = {{i00, i01, i10}};
                std::array<int, 3> f1 = {{i10, i01, i11}};
                std::array<vec3, 3> uv0 = {{uv00, uv01, uv10}};
                std::array<vec3, 3> uv1 = {{uv10, uv01, uv11}};
                faces.push_back(f0);
                faces.push_back(f1);
                face_uvs.push_back(uv0);
                face_uvs.push_back(uv1);
            }
        }
    }

    void finalize()
    {
        build_springs();
        rest_volume = volume();

        // nodes must not be resized after this
        for (auto &f : faces)
        {
            triangles.push_back(std::make_shared<phy_soft_tri>(&nodes, f[0], f[1], f[2]));
        }
        soft_model_node = std::make_shared<phy_bvh_node>(triangles, 0, triangles.size());

        // structural springs are exactly the unique mesh edges, pointing at the nodes keeps the edges live
        for (auto &s : springs)
        {
            if (s.type == spring_type::structural)
            {
                edge e;
                e.u = &nodes[s.a].pos;
                e.v = &nodes[s.b].pos;
                edges.push_back(e);
            }
        }
        update_centroid();
        is_static = false;
    }

    void add_spring(int a, int b, spring_type type)
    {
        spring s;
        s.a = a;
        s.b = b;
        s.rest_length = (nodes[a].pos - nodes[b].pos).length();
        s.stiffness = (type == spring_type::structural) ? stretch_k : bend_k;
        s.damping = spring_damping;
        s.type = type;
        springs.push_back(s);
    }

    void build_springs()
    {
        // edge (low index, high index) to vertex opposite that edge in each face using it
        std::map<std::pair<int, int>, std::vector<int>> edge_opposites;
        for (auto &f : faces)
        {
            for (int i = 0; i < 3; i++)
            {
                int a = f[i];
                int b = f[(i + 1) % 3];
                std::pair<int, int> edge(std::min(a, b), std::max(a, b));
                edge_opposites[edge].push_back(f[(i + 2) % 3]);
            }
        }

        std::set<std::pair<int, int>> bend_pairs;
        for (auto &entry : edge_opposites)
        {
            add_spring(entry.first.first, entry.first.second, spring_type::structural);

            const std::vector<int> &opposite = entry.second;
            for (size_t i = 0; i < opposite.size(); i++)
            {
                for (size_t j = i + 1; j < opposite.size(); j++)
                {
                    int a = std::min(opposite[i], opposite[j]);
                    int b = std::max(opposite[i], opposite[j]);
                    // skip pairs that are already an edge or else it would form incorrectly
                    if (a != b && !edge_opposites.count(std::make_pair(a, b)) && bend_pairs.insert(std::make_pair(a, b)).second)
                    {
                        add_spring(a, b, spring_type::bend);
                    }
                }
            }
        }
    }

    int parse_face_index(const std::string &token, int &uv_index)
    {
        std::stringstream ss(token);
        std::string index_string;

        std::getline(ss, index_string, '/');
        int v = std::stoi(index_string) - 1; // OBJ indices are 1-based

        std::string uv_string;
        if (std::getline(ss, uv_string, '/') && !uv_string.empty())
        {
            uv_index = std::stoi(uv_string) - 1;
        }
        return v;
    }
};

#endif
