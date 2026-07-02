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

        v = 1.0 - v;
        int x = static_cast<int>(u * (width - 1)); //it gets rounded down
        int y = static_cast<int>(v * (height - 1));

        double d_x = u * (width - 1) - (double) x;
        double d_y = v * (height - 1) - (double) y;
        double d_z = std::sqrt((d_x * d_x) + (d_y * d_y)); //distances

        color color_x_one = color_buffer[y * width + x];
        color color_x_two = color_buffer[y * width + ((x + 1) % width)];

        color color_y_one = color_buffer[((y + 1) % height) * width + x];
        color color_y_two = color_buffer[((y + 1) % height) * width + ((x + 1) % width)];
        return mix(mix(color_x_one,color_x_two,d_x),mix(color_y_one,color_y_two,d_x),d_y);
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