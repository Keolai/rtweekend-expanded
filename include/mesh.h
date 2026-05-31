#ifndef MESH_H
#define MESH_H

#include "tri.h"
#include "hittable_list.h"
#include "material.h"

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <memory>

class mesh
{
public:
    mesh() {}

    mesh(const std::string &file_path,
         std::shared_ptr<material> mat)
        : file_path(file_path), mat(mat) {}

    bool load_model(hittable_list &world)
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

                        world.add(
                            std::make_shared<tri>(
                                std::array<point3, 3>{
                                    v0,
                                    v1,
                                    v2},
                                mat));
                    }
                }
            }
        } //DONE READING LINES

        return true;
    }

private:
   
    // Extract vertex index from OBJ face token
    int parse_face_index(const std::string &token)
    {

        std::stringstream ss(token);

        std::string index_string;

        std::getline(ss, index_string, '/');

        return std::stoi(index_string);
    }

private:
    std::string file_path;

    std::shared_ptr<material> mat;

    std::vector<vec3> vertices;
};

#endif