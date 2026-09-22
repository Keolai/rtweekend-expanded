#ifndef MODEL_H
#define MODEL_H

#include "tri.h"

#include "../hittable.h"
#include "../utilities/phy_bvh.h"

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
        if (model_node) //in model bvh reduces up rendering by about 40% with simple models
        {
            return model_node->hit(r, ray_t, rec);
        } else {
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

    void create_bvh(){
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

private:
    point3 pos = vec3(0);
    std::string file_path;
    std::vector<vec3> vertices; // original vertices
    std::shared_ptr<phy_bvh_node>  model_node;

    int parse_face_index(const std::string &token)
    {

        std::stringstream ss(token);

        std::string index_string;

        std::getline(ss, index_string, '/');

        return std::stoi(index_string);
    }
};

#endif