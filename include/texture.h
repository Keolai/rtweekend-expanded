#ifndef TEXTURE_H
#define TEXTURE_H

#include "color.h"
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

class texture
{
public:
    texture() : width(0), height(0) {}

    texture(const std::string& file_path)
        : file_path(file_path), width(0), height(0)
    {
        load_texture();
    }

    texture(const color &albedo) : width(1), height(1) {
        fake_texture(albedo);
    }

    color get_color_at_coordinate(double u, double v) const
    {
        if (color_buffer.empty())
            return color(1, 0, 1); 

        u = u - std::floor(u);
        v = v - std::floor(v);
        u = fmod(u,1.);
        v = fmod(v,1.);

        v = 1.0 - v;

        int x = static_cast<int>(u * (width - 1));
        int y = static_cast<int>(v * (height - 1));

        return color_buffer[y * width + x];
        //return color(u, 0, 0);
    }

private:
    std::string file_path;
    std::vector<color> color_buffer;
    int width;
    int height;

    void fake_texture(color albedo){
        color_buffer.push_back(albedo);
    }

    bool load_texture()
    {
        if (file_path.empty())
            return false;

        std::ifstream file(file_path, std::ios::binary);
        if (!file.is_open())
            return false;

        std::string magic;
        file >> magic;

        if (magic != "P3" && magic != "P6")
            return false;

        // Skip whitespace + comments
        auto skip_comments = [&]() {
            while (file >> std::ws && file.peek() == '#')
            {
                std::string tmp;
                std::getline(file, tmp);
            }
        };

        skip_comments();
        file >> width >> height;

        skip_comments();
        int max_value;
        file >> max_value;

        file.get(); // consume single whitespace after header (VERY important for P6)

        color_buffer.resize(width * height);

        if (magic == "P3")
        {
            // ASCII format
            for (int i = 0; i < width * height; i++)
            {
                int r, g, b;
                file >> r >> g >> b;

                color_buffer[i] = color(
                    r / double(max_value),
                    g / double(max_value),
                    b / double(max_value)
                );
            }
        }
        else
        {
            // P6 binary format
            for (int i = 0; i < width * height; i++)
            {
                unsigned char rgb[3];
                file.read(reinterpret_cast<char*>(rgb), 3);

                color_buffer[i] = color(
                    rgb[0] / double(max_value),
                    rgb[1] / double(max_value),
                    rgb[2] / double(max_value)
                );
            }
        }

        return true;
    }
};

#endif