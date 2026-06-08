#include "include/rtweekend.h"
#include "include/camera.h"
#include "include/hittable.h"
#include "include/hittable_list.h"
#include "include/material.h"
#include "include/sphere.h"
#include "include/tri.h"
#include "include/mesh.h"
#include "include/bvh.h"
#include "include/window.h"

#include <iostream>
#include <fstream>

std::vector<color> color_buffer;
std::string labels[2] = {"position:", "look at:"};
std::string axis[3] = {"x", "y", "z"};

std::string file_name = "out";

std::vector<std::shared_ptr<text_box>> inputs;

void write_to_file(int image_width, int image_height)
{
    file_name = inputs[6]->get_text();
    if (file_name.length() == 0)
    {
        file_name = "out";
    }
    std::ofstream myFile(file_name + ".ppm");
    if (myFile.is_open())
    {
        myFile << "P3\n"
               << image_width << ' ' << image_height << "\n255\n";
        for (int i = 0; i < color_buffer.size(); i++)
        {
            write_color(myFile, color_buffer[i]);
        }
    }
    myFile.close();
}

void gui_setup(window &win, int height)
{
    win.initialize();
    int x_padding = 0;
    int y_padding = 0;
    int y_start = 0;

    win.create_panel(0, 0, 120, 100);
    win.create_panel(0, height - 55, 110, 50);
    for (int i = 0; i < 2; i++)
    {
        std::string cur_label = labels[i];
        win.create_text(x_padding, y_start, cur_label);
        for (int j = 0; j < 3; j++)
        {
            std::string cur_axis = axis[j];
            win.create_text(x_padding + (j * 40), (y_start + 10), cur_axis);
            inputs.push_back(win.create_fixed_width_text_box(10 + (j * 40), y_start + 10, "0", 3));
        }

        y_start += 24;
    }
    win.create_text(0, height - 50, "file name:");
    inputs.push_back(win.create_fixed_width_text_box(5, height - 40, "out", 8));
    win.create_text(72, height - 30, ".ppm");
}

void render(camera &cam, hittable &world, window &win, int samples, int shadow_samples)
{
    float input_floats[6];

    for (int i = 0; i < inputs.size() - 1; i++) //last input should be file name!
    {
        try
        {
            input_floats[i] = std::stof(inputs[i]->get_text());
        }
        catch (const std::invalid_argument &e)
        {
            std::cout << "Invalid input: " << e.what() << std::endl;
            input_floats[i] = 0;
        }
        catch (const std::out_of_range &e)
        {
            std::cout << "Number out of range: " << e.what() << std::endl;
            input_floats[i] = 0;
        }
    }

    cam.samples_per_pixel = samples;
    cam.shadow_samples = shadow_samples;
    cam.lookfrom = vec3(input_floats[0], input_floats[1], input_floats[2]);
    cam.lookat = vec3(input_floats[3], input_floats[4], input_floats[5]);
    cam.render(world, color_buffer, win);
}

void populate_gui_start_state(camera &cam)
{
    vec3 arr[2] = {cam.lookfrom,
                   cam.lookat};

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            std::string raw_str = std::to_string(arr[i].e[j]);
            int index = raw_str.find(".");
            if (index != -1)
            {
                inputs[j + (i * 3)]->set_text(raw_str.substr(0, index));
            }
        }
    }
}

int main()
{
    //* DEFINE WORLD HERE *//
    hittable_list world;
    auto met = make_shared<metal>(color(0.8, 0.8, 0.8), 0.4); // teapot material
    auto mat = make_shared<lambertian>(color(0.8, 0.8, 0.0)); // world material
    mesh Model = mesh("models/utah_teapot(2).obj", met);
    Model.load_model(world);
    //world.add(make_shared<sphere>(point3(0,2,0), 2.0, met));
    world.add(make_shared<sphere>(point3(0.0, -100.5, -1.0), 100.0, mat));
    // //* END OF WORLD DEFINITION *//

    auto world_bvh = make_shared<bvh_node>(
        world.objects,
        0,
        world.objects.size());

    camera cam;
    cam.aspect_ratio = 16.0 / 9.0;
    cam.image_width = 400;
    cam.samples_per_pixel = 1;
    cam.max_depth = 25;

    cam.vfov = 70;
    cam.lookfrom = point3(1, 3, 5);
    cam.lookat = point3(0, 0, -2);
    cam.vup = vec3(0, 1, 0);

    cam.add_light(light(point3(0,10,-4), color(0.7,0.7,0.4), 50));
    //cam.add_light(light(point3(3,5,0), color(1)));

    // window stuff
    window win = window(cam.get_height(), cam.image_width);
    gui_setup(win, cam.get_height());
    color_buffer.resize(cam.get_height() * cam.image_width);

    // buttons
    win.create_button(5, 50, 50, 20, "Render", [&]()
                      { render(cam, *world_bvh, win, 1,3); });
    win.create_button(5, 75, 75, 20, "HD Render", [&]()
                      { render(cam, *world_bvh, win, 25,8); });
    win.create_button(5, cam.get_height() - 25, 50, 20, "Save!", [&]()
                      { write_to_file(cam.image_width, cam.get_height()); });

    populate_gui_start_state(cam);
   // printf("meow meow moew\n");
    cam.render(*world_bvh, color_buffer, win);
    while (!win.poll_for_event())
    {
        win.update();
    }
}