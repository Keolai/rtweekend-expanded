#ifndef CAMERA_H
#define CAMERA_H

#include "hittable.h"
#include "material.h"
#include "window.h"
#include "light.h"
#include <vector>

using std::shared_ptr;

class camera
{
public:
    /* Public Camera Parameters Here */

    double aspect_ratio = 1.0;  // Ratio of image width over height
    int image_width = 100;      // Rendered image width in pixel count
    int samples_per_pixel = 10; // Count of random samples for each pixel
    int max_depth = 10;         // Maximum number of ray bounces into scene
    int shadow_samples = 5;
    std::vector<std::shared_ptr<light>> lights;
    color ambient_light = color(0.3);
    // light world_light = light(point3(0, 10, 0), color(1, 1, 1)); //default light

    double vfov = 90;                  // Vertical view angle (field of view)
    point3 lookfrom = point3(0, 0, 0); // Point camera is looking from
    point3 lookat = point3(0, 0, -1);  // Point camera is looking at
    vec3 vup = vec3(0, 1, 0);          // Camera-relative "up" direction
    int image_resolution = 2;

    double defocus_angle = 0; // Variation angle of rays through each pixel
    double focus_dist = 10;   // Distance from camera lookfrom point to plane of perfect focus
    color ambient = color(0.1);

    void render(const hittable &world, std::vector<color> &color_buffer, window &win)
    {
        initialize();

        for (int j = 0; j < image_height; j+=image_resolution)
        {
            std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
            for (int i = 0; i < image_width; i+=image_resolution)
            {
                color pixel_color(0, 0, 0);
                for (int sample = 0; sample < samples_per_pixel; sample++)
                {
                    ray r = get_ray(i, j);
                    pixel_color += ray_color(r, max_depth, world);
                }
                for (int k = 0; k < image_resolution; k++){
                    for (int g = 0; g < image_resolution; g++){
                        if((j+k) < image_height && (i + g) < image_width){
                        color_buffer[((j+k) * image_width) + (i+g)] = (pixel_samples_scale * pixel_color);
                        }
                    }
                }
                //color_buffer[(j * image_width) + i] = (pixel_samples_scale * pixel_color);
            }
            if (j % 10 == 0)
            {
                if (win.display_color_array(color_buffer))
                { // should return if done
                    return;
                }
            }
        }
        std::clog << "\rDone.                 \n";
        win.display_color_array(color_buffer);
    }

    int get_height()
    {
        int tmp_image_height = int(image_width / aspect_ratio);
        tmp_image_height = (tmp_image_height < 1) ? 1 : tmp_image_height;
        return tmp_image_height;
    }

    void add_light(std::shared_ptr<light> new_light)
    {
        lights.push_back(new_light);
    }

private:
    /* Private Camera Variables Here */

    int image_height; // Rendered image height
    double pixel_samples_scale;
    point3 center;       // Camera center
    point3 pixel00_loc;  // Location of pixel 0, 0
    vec3 pixel_delta_u;  // Offset to pixel to the right
    vec3 pixel_delta_v;  // Offset to pixel below
    vec3 u, v, w;        // Camera frame basis vectors
    vec3 defocus_disk_u; // Defocus disk horizontal radius
    vec3 defocus_disk_v; // Defocus disk vertical radius

    void initialize()
    {
        image_height = int(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        pixel_samples_scale = 1.0 / samples_per_pixel;

        center = lookfrom;

        // Determine viewport dimensions. //THIS IS FOV
        auto theta = degrees_to_radians(vfov);
        auto h = std::tan(theta / 2);
        auto viewport_height = 2 * h * focus_dist;
        auto viewport_width = viewport_height * (double(image_width) / image_height);

        // Calculate the u,v,w unit basis vectors for the camera coordinate frame.
        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);

        // Calculate the vectors across the horizontal and down the vertical viewport edges.
        vec3 viewport_u = viewport_width * u;   // Vector across viewport horizontal edge
        vec3 viewport_v = viewport_height * -v; // Vector down viewport vertical edge

        // Calculate the horizontal and vertical delta vectors from pixel to pixel.
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // Calculate the location of the upper left pixel.
        auto viewport_upper_left = center - (focus_dist * w) - viewport_u / 2 - viewport_v / 2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        // Calculate the camera defocus disk basis vectors.
        auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }

    ray get_ray(int i, int j) const
    {
        // Construct a camera ray originating from the defocus disk and directed at a randomly
        // sampled point around the pixel location i, j.

        auto offset = sample_square();
        auto pixel_sample = pixel00_loc + ((i + offset.x()) * pixel_delta_u) + ((j + offset.y()) * pixel_delta_v);

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;

        return ray(ray_origin, ray_direction);
    }

    vec3 sample_square() const
    {
        // Returns the vector to a random point in the [-.5,-.5]-[+.5,+.5] unit square.
        return vec3(random_double() - 0.5, random_double() - 0.5, 0);
    }

    point3 defocus_disk_sample() const
    {
        // Returns a random point in the camera defocus disk.
        auto p = random_in_unit_disk();
        return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    color ray_color(const ray &r, int depth, const hittable &world) const
    {
        // If we've exceeded the ray bounce limit, no more light is gathered.
        if (depth <= 0)
            return color(0, 0, 0);
        hit_record rec;

        if (world.hit(r, interval(0.001, infinity), rec)) // ray hit something
        {
            ray scattered;
            color attenuation;
            color direct =
                ray_light(world, r, rec);

            color indirect(0);

            if (rec.mat->scatter(r, rec, attenuation, scattered))
            {
                indirect = attenuation * mix(ray_color(scattered, depth - 1, world),color(1.), rec.mat->min_brightness) * ambient;
                direct = mix(direct,rec.mat->get_albedo(rec),rec.mat->min_brightness); //turn off shadows on glowy stuff
                // return clamp(direct + indirect,0,1);
            }
            return direct + indirect;
        }

        vec3 unit_direction = unit_vector(r.direction());
        auto a = 0.5 * (unit_direction.y() + 1.0);

        return (1.0 - a) * color(1.0, 1.0, 1.0) + a * color(0.5, 0.7, 1.0); // this is the sky/light color/ray did not hit
    }

    
    color ray_light(const hittable &world, const ray &r, hit_record &rec) const
    {
        if (lights.empty())
        {
            return color(0.0);
        }

        hit_record shadow_rec;
        color lighting = color(0.);

        for (int i = 0; i < lights.size(); i++)
        {

            lighting += lights[i]->light_hit(world, r, rec, shadow_samples);
        }
        return lighting;
    }
};

#endif