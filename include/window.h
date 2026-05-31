#ifndef WINDOW_H
#define WINDOW_H

#include "vec3.h"
#include "gui/gui.h"
#include "SDL3/SDL.h"
#include <functional>
#include <string>

class window
{
public:
    window() {}

    window(int height, int width) : height(height), width(width) {}

    int initialize()
    {
        pixels.resize(width * height);
        int WINDOW_HEIGHT = height;
        int WINDOW_WIDTH = width;

        // SDL3 returns bool
        if (!SDL_Init(SDL_INIT_VIDEO))
        {
            SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
            return -1;
        }

        // SDL3 returns bool
        if (!SDL_CreateWindowAndRenderer(
                "raytracer",
                WINDOW_WIDTH,
                WINDOW_HEIGHT,
                SDL_WINDOW_RESIZABLE,
                &sdlwindow,
                &renderer))
        {
            SDL_Log("Couldn't create window/renderer: %s", SDL_GetError());
            return -1;
        }

        texture = SDL_CreateTexture(
            renderer,
            SDL_PIXELFORMAT_RGBA8888,
            SDL_TEXTUREACCESS_STREAMING,
            width,
            height);

        if (!texture)
        {
            SDL_Log("Couldn't create texture: %s", SDL_GetError());
            return -1;
        }

        SDL_SetRenderLogicalPresentation(
            renderer,
            WINDOW_WIDTH,
            WINDOW_HEIGHT,
            SDL_LOGICAL_PRESENTATION_LETTERBOX);

        return 0;
    }

    void loop(bool done)
    {
        while (!app_done)
        {
            SDL_Event event;

            while (SDL_PollEvent(&event))
            {
                if (event.type == SDL_EVENT_QUIT)
                {
                    // app_done = true;
                    app_done = true;
                    quit_window();
                }
            }
        }
        return;
    }

    bool poll_for_event()
    {
        SDL_Event event;

        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_EVENT_QUIT)
            {
                // app_done = true;
                app_done = true;
                quit_window();
                return true;
            }

            if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
            {
                // printf("%f, %f\n",event.button.x, event.button.y);
                win_gui.check_for_clicks(event.button.x, event.button.y);
            }

            if(event.type == SDL_EVENT_KEY_DOWN){
                win_gui.process_text_input(event.key.key);
            }
        }
        return app_done;
    }

    bool display_color_array(std::vector<color> &color_buffer)
    {

        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width; i++)
            {
                int index = i + width * j;
                if (index < color_buffer.size())
                {
                    color pixel = norm_to_rgb(color_buffer.at(index));

                    uint8_t red = uint8_t(pixel.x());
                    uint8_t green = uint8_t(pixel.y());
                    uint8_t blue = uint8_t(pixel.z());
                    uint8_t alpha = 255;

                    pixels[index] = SDL_MapRGBA(
                        SDL_GetPixelFormatDetails(SDL_PIXELFORMAT_RGBA8888),
                        nullptr,
                        red,
                        green,
                        blue,
                        alpha);
                }
            }
            SDL_Event event;

            if (poll_for_event() == true)
            {
                return true;
            }
        }

        SDL_UpdateTexture(
            texture,
            nullptr,
            pixels.data(),
            width * sizeof(uint32_t));

        SDL_RenderClear(renderer);

        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        win_gui.render(renderer);

        SDL_RenderPresent(renderer);
        return false;
    }

    void update()
    {
        // SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        win_gui.render(renderer);
        win_gui.update_render_state();
        SDL_RenderPresent(renderer);
    }

    void create_button(float x, float y, float width, float height, std::string text, std::function<void()> callback)
    {
        button button = ::button(x, y, width, height, text, callback);
        win_gui.add(button);
        return;
    }

    text_t create_text(float x, float y, std::string text){
        text_t text_line = ::text_t(x,y,text);
        win_gui.add(text_line);
        return text_line;
    }

    void create_panel(float x, float y, float width, float height)
    {
        panel panel = ::panel(x, y, width, height);
        win_gui.add(panel);
        return;
    }

    std::shared_ptr<text_box> create_text_box(float x, float y, std::string text){
        auto box =
        std::make_shared<text_box>(
            x,
            y,
            text
        );
        win_gui.add(box);
        return box;
    }

    std::shared_ptr<text_box> create_fixed_width_text_box(float x, float y, std::string text, int length){
        auto box =
        std::make_shared<fixed_width_text_box>(
            x,
            y,
            length,
            text
        );
        win_gui.add(box);
        return box;
    }

private:
    std::vector<uint32_t> pixels;
    int height;
    int width;
    static SDL_Window *sdlwindow;
    static SDL_Renderer *renderer;
    static SDL_Surface *surface;
    static SDL_Texture *texture;
    static gui win_gui;
    bool app_done;

    int quit_window()
    {
        SDL_DestroyWindow(sdlwindow);
        SDL_Quit();
        printf("SDL quitting...\n");
        return 0;
    }
};

SDL_Window *window::sdlwindow = nullptr;
SDL_Renderer *window::renderer = nullptr;
SDL_Surface *window::surface = nullptr;
SDL_Texture *window::texture = nullptr;
gui window::win_gui;

#endif