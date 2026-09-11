#ifndef MODEL_H
#define MODEL_H

#include "tri.h"

struct face_vertex
{
    int v = -1;
    int vn = -1;
};

class model : public phy_hittable
{
public:
    std::vector<std::shared_ptr<phy_tri>> triangles;

    int model_id = -1;

    model(const std::string &file_path) : file_path(file_path) {load_model();}

    model(int id) : model_id(id) {}
    model(std::vector<std::shared_ptr<phy_tri>> &tris, int id) : model_id(id) { triangles = tris; }

    void add_tri(vec3 v1, vec3 v2, vec3 v3)
    {
        std::shared_ptr<phy_tri> new_tri = std::make_shared<phy_tri>(v1, v2, v3);
        triangles.push_back(new_tri);
    }

    bool hit(const ray &r, interval ray_t, phy_hit_record &rec) const override
    {
        phy_hit_record rec;
        for (int i = 0; i < triangles.size(); i++)
        {
            phy_hit_record rec;
            phy_hit_record temp_rec;
            auto current_tri = triangles[i];
            bool hit_anything = false;
            std::shared_ptr<phy_tri> tri_hit; //maybe need to store idk

            auto closest_so_far = ray_t.max;
            if (current_tri->hit(r, interval(0.001, closest_so_far), temp_rec)) // ray hit something
            {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
                tri_hit = current_tri;
            }


            return hit_anything;
        }
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
                            std::make_shared<phy_tri>(
                                std::array<point3, 3>{
                                    v0,
                                    v1,
                                    v2}
                                ));
                    }
                }
            }
        } //DONE READING LINES

        return true;
    }


    phy_aabb bounding_box() const override // need to add
    {
        return phy_aabb();
    }

    void position(vec3 &newPos)
    {
        pos = newPos;
    }

    void update_state() override
    {
        copy(next_state, current_state); // copy the next predicted state to the new state;
    }

    double hit_adjuster() const override
    {
        return 0;
    }

    bool is_inside(const ray &r) const override
    {
        return false;
    }

    vec3 closest_point_on_surface(const vec3 &pos) const override
    {

        return vec3(0);
    }

private:
    point3 pos = vec3(0);
    std::string file_path;
    
};

#endif