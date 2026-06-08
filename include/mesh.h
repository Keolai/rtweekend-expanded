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

struct face_vertex
{
    int v = -1;
    int vn = -1;
};

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
            return false;

        std::vector<vec3> vertices;
        std::vector<vec3> normals;

        std::string line;

        while (std::getline(file, line))
        {
            if (line.empty())
                continue;

            std::stringstream ss(line);

            std::string prefix;
            ss >> prefix;

            //vertex pos
            if (prefix == "v")
            {
                double x, y, z;
                ss >> x >> y >> z;

                vertices.emplace_back(x, y, z);
            }

           //normal
            else if (prefix == "vn")
            {
                double x, y, z;
                ss >> x >> y >> z;

                normals.push_back(
                    unit_vector(vec3(x, y, z)));
            }

        
            //face
            else if (prefix == "f")
            {
                std::vector<face_vertex> face;

                std::string token;

                while (ss >> token)
                {
                    face.push_back(
                        parse_face_index(token,vertices.size(),normals.size()));
                }

                if (face.size() < 3)
                    continue;

                face_vertex fv0 = face[0];

                for (size_t i = 1; i + 1 < face.size(); ++i)
                {
                    face_vertex fv1 = face[i];
                    face_vertex fv2 = face[i + 1];

                    point3 v0 = vertices[fv0.v];
                    point3 v1 = vertices[fv1.v];
                    point3 v2 = vertices[fv2.v];

                    bool has_normals =
                        fv0.vn >= 0 &&
                        fv1.vn >= 0 &&
                        fv2.vn >= 0;

                    if (has_normals)
                    {
                        vec3 n0 = normals[fv0.vn];
                        vec3 n1 = normals[fv1.vn];
                        vec3 n2 = normals[fv2.vn];

                        world.add(
                            std::make_shared<tri>(
                                std::array<point3, 3>{
                                    v0, v1, v2},
                                std::array<vec3, 3>{
                                    n0, n1, n2},
                                mat));
                    }
                    else
                    {
                        world.add(
                            std::make_shared<tri>(
                                std::array<point3, 3>{
                                    v0, v1, v2},
                                mat));
                    }
                }
            }
        }

        return true;
    }

private:

    face_vertex parse_face_index(const std::string &token,size_t vertex_count,
    size_t normal_count)
    {

        face_vertex result;

        std::stringstream ss(token);
        std::string part;

        std::vector<std::string> fields;

        while (std::getline(ss, part, '/'))
            fields.push_back(part);

        // vertex index
        if (!fields.empty() && !fields[0].empty())
            result.v = std::stoi(fields[0]) - 1;

        // normal index
        if (fields.size() >= 3 && !fields[2].empty())
            result.vn = std::stoi(fields[2]) - 1;

        return result;
    }

    int resolve_index(int idx, int count)
    {
        if (idx > 0)
            return idx - 1;

        return count + idx;
    }

private:
    std::string file_path;

    std::shared_ptr<material> mat;

    std::vector<vec3> vertices;
};

#endif