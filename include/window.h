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

        pixel_format = SDL_GetPixelFormatDetails(SDL_PIXELFORMAT_RGBA8888);
        if (!pixel_format)
        {
            SDL_Log("Couldn't get pixel format details: %s", SDL_GetError());
            return -1;
        }

        SDL_SetRenderLogicalPresentation(
            renderer,
            WINDOW_WIDTH,
            WINDOW_HEIGHT,
            SDL_LOGICAL_PRESENTATION_LETTERBOX);

        SDL_StartTextInput(sdlwindow);

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
                // window coords -> logical coords, otherwise clicks miss after a resize
                SDL_ConvertEventToRenderCoordinates(renderer, &event);
                win_gui.check_for_clicks(event.button.x, event.button.y);
            }

            if (event.type == SDL_EVENT_TEXT_INPUT)
            {
                win_gui.process_text_input(event.text.text);
            }

            if (event.type == SDL_EVENT_KEY_DOWN)
            {
                win_gui.process_key(event.key.key);
            }
        }
        return app_done;
    }

    // Converts and uploads rows [row_start, row_end) (row_end < 0 means the last row), then presents.
    // Returns true if the app should quit.
    bool display_color_array(const std::vector<color> &color_buffer, int row_start = 0, int row_end = -1)
    {
        if (poll_for_event())
        {
            return true;
        }

        int buffer_rows = int(color_buffer.size()) / width;
        if (row_end < 0 || row_end > buffer_rows)
        {
            row_end = buffer_rows;
        }
        if (row_end > height)
        {
            row_end = height;
        }
        if (row_start < 0)
        {
            row_start = 0;
        }

        if (row_start < row_end)
        {
            for (int index = row_start * width; index < row_end * width; index++)
            {
                color pixel = norm_to_rgb(color_buffer[index]);
                pixels[index] = SDL_MapRGBA(pixel_format, nullptr, uint8_t(pixel.x()), uint8_t(pixel.y()), uint8_t(pixel.z()), 255);
            }

            SDL_Rect rows = {0, row_start, width, row_end - row_start};
            SDL_UpdateTexture(
                texture,
                &rows,
                pixels.data() + row_start * width,
                width * sizeof(uint32_t));
        }
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

    text_t create_text(float x, float y, std::string text)
    {
        text_t text_line = ::text_t(x, y, text);
        win_gui.add(text_line);
        return text_line;
    }

    void create_panel(float x, float y, float width, float height)
    {
        panel panel = ::panel(x, y, width, height);
        win_gui.add(panel);
        return;
    }

    std::shared_ptr<text_box> create_text_box(float x, float y, std::string text)
    {
        auto box =
            std::make_shared<text_box>(
                x,
                y,
                text);
        win_gui.add(box);
        return box;
    }

    std::shared_ptr<text_box> create_fixed_width_text_box(float x, float y, std::string text, int length)
    {
        auto box =
            std::make_shared<fixed_width_text_box>(
                x,
                y,
                length,
                text);
        win_gui.add(box);
        return box;
    }

private:
    std::vector<uint32_t> pixels;
    const SDL_PixelFormatDetails *pixel_format = nullptr;
    int height;
    int width;
    static SDL_Window *sdlwindow;
    static SDL_Renderer *renderer;
    static SDL_Surface *surface;
    static SDL_Texture *texture;
    static gui win_gui;
    bool app_done = false;

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