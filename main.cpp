#include "include/rtweekend.h"
#include "include/camera.h"
#include "include/hittable.h"
#include "include/hittable_list.h"
#include "include/material.h"
#include "include/sphere.h"
#include "include/plane.h"
#include "include/tri.h"
#include "include/mesh.h"
#include "include/bvh.h"
#include "include/window.h"

#include "include/physics_layer.h"

#include <iostream>
#include <fstream>

#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>

#define SIM_RATE_MS 30

#define RECORDING false
#define MAX_RECORDING_STEPS 80

std::atomic<bool> running{true};
std::mutex state_mutex;

std::vector<color> color_buffer;
std::string labels[2] = {"position:", "look at:"};
std::string axis[3] = {"x", "y", "z"};

std::string file_name = "out";

std::vector<std::shared_ptr<text_box>> inputs;

std::atomic<bool> run_sim{false};

std::string pad_integer(int num, int width) {
    std::string tmp = std::to_string(num);

    if (tmp.length() < width) {
        tmp = std::string(width - tmp.length(), '0') + tmp;
    }

    return tmp;
}

void write_to_file(int image_width, int image_height, int step)
{
    file_name = inputs[6]->get_text();
    if (file_name.length() == 0 && !RECORDING)
    {
        file_name = "out";
    }
    else if (RECORDING) //override it
    {
        file_name = "recording/out" + pad_integer(step,4);
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
    printf("%s\n",file_name.c_str());
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

void run_sim_toggle()
{
    run_sim = !run_sim;
}

void render(camera &cam, hittable &world, window &win, int samples, int shadow_samples, int image_res)
{
    float input_floats[6];

    for (int i = 0; i < inputs.size() - 1; i++) // last input should be file name!
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
    cam.image_resolution = image_res;
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

void physics_thread_func(physics_layer &sim, std::chrono::milliseconds interval)
{
    auto next_tick = std::chrono::steady_clock::now();
    while (running)
    {
        next_tick += interval;
        if (run_sim)
        {
            sim.step(interval.count()); // locking now happens inside step()
        }
        std::this_thread::sleep_until(next_tick);
    }
}

int main()
{
    //* DEFINE WORLD HERE *//
    hittable_list world;
    auto met = make_shared<metal>(color(0.8, 0.8, 0.8), 0.1);      // teapot material
    auto mat = make_shared<lambertian>(color(0.8, 0.8, 0.0));      // world material
    auto cube_mat = make_shared<lambertian>(color(0.5, 0.5, 0.5)); // world material
    auto em = make_shared<emmissive>(color(0.5));
    auto tex_mat = make_shared<metal>("models/textures/checkered.ppm", 0.1);
    // mesh teapot_Model = mesh("models/solid_teapot.obj", tex_mat, world); this is to check stuff

    physics_layer sim;

    sim.add_force(vec3(0, -1, 0), 5.0); // gravity
    sim.add_wind_resistance(0.1, 1);    // wind resistance
    // sim.add_point_force(vec3(0,0,0),2);

    // START OF BASIC DEMO
    // int sphere_physics_1 = sim.add_sphere_to_world(vec3(0, 2.5, 0), vec3(0, 4, 0), 0.3);
    // auto mySphere1 = std::make_shared<sphere>(point3(0, 2.5, 0), 0.3, mat);
    // world.add(mySphere1);

    // int sphere_physics_2 = sim.add_sphere_to_world(vec3(1.2, 2.5, 0), vec3(4, 0, 0), 0.3);
    // auto mySphere2 = std::make_shared<sphere>(point3(1.2, 2.5, 0), 0.3, mat);
    // world.add(mySphere2);

    // int sphere_physics_3 = sim.add_sphere_to_world(vec3(-1.2, 2.5, 0), vec3(-4, 0, 0), 0.3);
    // auto mySphere3 = std::make_shared<sphere>(point3(-1.2, 2.5, 0), 0.3, mat);
    // world.add(mySphere3);

    // int sphere_physics_4 = sim.add_sphere_to_world(vec3(0, 2.5, -1.2), vec3(0, 0, -4), 0.3);
    // auto mySphere4 = std::make_shared<sphere>(point3(0, 2.5, -1.2), 0.3, mat);
    // world.add(mySphere4);

    // int sphere_physics_5 = sim.add_sphere_to_world(vec3(0, 2.5, 1.2), vec3(0, 0, 4), 0.3);
    // auto mySphere5 = std::make_shared<sphere>(point3(0, 2.5, 1.2), 0.3, mat);
    // world.add(mySphere5);

    // int model = sim.add_mesh_to_world("models/closed_cube.obj", true);
    // auto my_model = std::make_shared<mesh>("models/open_cube.obj", cube_mat, world);

    // sim.connect_objects(mySphere1->id, sphere_physics_1);
    // sim.connect_objects(mySphere2->id, sphere_physics_2);
    // sim.connect_objects(mySphere3->id, sphere_physics_3);
    // sim.connect_objects(mySphere4->id, sphere_physics_4);
    // sim.connect_objects(mySphere5->id, sphere_physics_5);
    // sim.connect_objects(my_model->id, model);
    //END OF BASIC DEMO

    // START OF TWO BALLS HITTING DEMO
    // int model = sim.add_mesh_to_world("models/closed_cube.obj",true);
    // auto my_model = std::make_shared<mesh>("models/open_cube.obj",cube_mat,world);

    // int sphere_physics_id = sim.add_sphere_to_world(vec3(-1,2,0),vec3(2,0,0),0.5);
    // auto mySphere1 = std::make_shared<sphere>(point3(-1,2,0), 0.5, mat);
    // world.add(mySphere1);

    // int sphere_physics_id2 = sim.add_sphere_to_world(vec3(1,2,0),vec3(-2,0,0),0.5);
    // auto mySphere2 = std::make_shared<sphere>(point3(1,2,0), 0.5, mat);
    // world.add(mySphere2);

    // sim.connect_objects(mySphere1->id,sphere_physics_id);
    // sim.connect_objects(mySphere2->id,sphere_physics_id2);
    // sim.connect_objects(my_model->id, model);
    // END OF TWO BALLS HITTING DEMO

    //CUBE FALLING DEMO
    int model = sim.add_mesh_to_world("models/closed_cube.obj",true);
    auto my_model = std::make_shared<mesh>("models/open_cube.obj",cube_mat,world);

    int cube_physics_id = sim.add_mesh_to_world("models/cube.obj",false);
    auto mySphere1 = std::make_shared<mesh>("models/cube.obj", mat, world);

    sim.connect_objects(mySphere1->id,cube_physics_id);
    sim.connect_objects(my_model->id, model);
    //END OF CUBE FALLING

    // sim.connect_objects(mySphere2->id,sphere_physics_id_rigid);

    sim.set_render_objects(world);

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
    cam.lookfrom = point3(0, 3, 5); // 0 5 10
    cam.lookat = point3(0, 0, -1);
    cam.vup = vec3(0, 1, 0);
    cam.ambient = color(0.00);
    cam.image_resolution = 4;
    cam.shadow_samples = 1; // 0 == no shadows

    // cam.add_light(std::make_shared<spot_light>(point3(1,3,5), color(0.7,0.7,0.4), 150, 0.22,0.3,vec3(0,0,-2)));
    cam.add_light(std::make_shared<light>(point3(2, 10, 0), vec3(1.), 100));
    // window stuff
    window win = window(cam.get_height(), cam.image_width);
    gui_setup(win, cam.get_height());
    color_buffer.resize(cam.get_height() * cam.image_width);

    // buttons
    win.create_button(5, 50, 50, 20, "Render", [&]()
                      { render(cam, world, win, 1, 3, 4); });
    // win.create_button(5, 75, 75, 20, "HD Render", [&]()
    //                   { render(cam, world, win, 25,8,1); });
    // win.create_button(5, cam.get_height() - 25, 50, 20, "Save!", [&]()
    //                   { write_to_file(cam.image_width, cam.get_height(), 0); });
    win.create_button(5, 75, 90, 20, "Stop/Start", [&]()
                      { run_sim_toggle(); });

    populate_gui_start_state(cam);

    if (!RECORDING) //run normally
    {

        std::thread physics_thread(physics_thread_func, std::ref(sim),
                                   std::chrono::milliseconds(SIM_RATE_MS));
        cam.render(world, color_buffer, win);
        while (!win.poll_for_event())
        {
            // printf("running!\n");
            sim.update_render();
            win.update();
            cam.render(world, color_buffer, win); // TODO: FIX THIS
        }

        running = false;
        physics_thread.join();
    }
    else
    {
        cam.image_resolution = 1;
        int cur_step = 0;
        while (!win.poll_for_event() && cur_step <= MAX_RECORDING_STEPS)
        {
            // printf("running!\n");
            sim.step(SIM_RATE_MS);
            sim.update_render();
            cam.render(world, color_buffer, win);
            write_to_file(cam.image_width, cam.get_height(), sim.get_step());   
            cur_step++;
        }
    }
}