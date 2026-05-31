#ifndef GUI_H
#define GUI_H

#include <vector>
#include "panel.h"
#include "button.h"
#include "text.h"
#include "text_box.h"
#include "SDL3/SDL.h"

using std::shared_ptr;

struct text_render_data
{
    std::string text;
    float x;
    float y;
};

class gui
{
public:
    std::vector<panel> panels;
    std::vector<button> buttons;
    std::vector<text_t> text_lines;
    std::vector<std::shared_ptr<text_box>> text_boxes;

    int cur_text_index = -1;

    gui() {}
    //gui(panel object) { add(object); }

    void clear() { panels.clear(); }

    void add(panel object)
    {
        panels.push_back(object);
    }

    void add(button button)
    {
        buttons.push_back(button);
    }

    void add(text_t text)
    {
        text_lines.push_back(text);
    }

    void add(std::shared_ptr<text_box> box)
    {
        text_boxes.push_back(box);
    }

    void update_render_state()
    {
        for (int i = 0; i < buttons.size(); i++)
        {
            buttons[i].advance_pressed();
        }
    }

    void render(SDL_Renderer *renderer)
    {
        std::vector<SDL_FRect> rects;
        std::vector<SDL_FRect> active_buttons;
        std::vector<SDL_FRect> inactive_buttons;
        std::vector<text_render_data> text;
        // render to screen, screen should be passed in.
        for (int i = 0; i < panels.size(); i++)
        {
            rects.push_back({panels[i].get_x(), panels[i].get_y(), panels[i].get_width(), panels[i].get_height()});
        }
        SDL_SetRenderDrawColor(renderer, 64, 64, 64, SDL_ALPHA_OPAQUE); /* white, full alpha */
        SDL_RenderFillRects(renderer, rects.data(), (int)rects.size());

        for (int i = 0; i < buttons.size(); i++)
        {
            if (buttons[i].is_clicked())
            {
                active_buttons.push_back({buttons[i].get_x(), buttons[i].get_y(), buttons[i].get_width(), buttons[i].get_height()});
            }
            else
            {
                inactive_buttons.push_back({buttons[i].get_x(), buttons[i].get_y(), buttons[i].get_width(), buttons[i].get_height()});
            }
            text.push_back({buttons[i].get_text(), buttons[i].get_text_x(), buttons[i].get_text_y()});
        }

        for (int i = 0; i < text_boxes.size(); i++)
        {
            text.push_back({text_boxes[i]->get_show_text(), text_boxes[i]->get_x(), text_boxes[i]->get_y()});

            if (text_boxes[i]->is_being_edited())
            {
                active_buttons.push_back({text_boxes[i]->get_x(), text_boxes[i]->get_y(), text_boxes[i]->get_width(), text_boxes[i]->get_height()});
            }
            else
            {
                inactive_buttons.push_back({text_boxes[i]->get_x(), text_boxes[i]->get_y(), text_boxes[i]->get_width(), text_boxes[i]->get_height()});
            }
        }

        for (int i = 0; i < text_lines.size(); i++)
        {
            text.push_back({text_lines[i].get_text(), text_lines[i].get_x(), text_lines[i].get_y()});
        }

        if (!active_buttons.empty())
        {
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE); /* grey */
            SDL_RenderFillRects(renderer, active_buttons.data(), (int)active_buttons.size());
        }
        if (!inactive_buttons.empty())
        {
            SDL_SetRenderDrawColor(renderer, 128, 128, 128, SDL_ALPHA_OPAQUE); /* grey */
            SDL_RenderFillRects(renderer, inactive_buttons.data(), (int)inactive_buttons.size());
        }

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
        for (int i = 0; i < text.size(); i++)
        {
            if (text[i].text.length() > 0)
            {
                SDL_RenderDebugText(renderer, text[i].x, text[i].y, text[i].text.c_str());
            }
        }
    }

    void check_for_clicks(float x, float y)
    {
        cur_text_index = -1;
        for (int i = 0; i < text_boxes.size(); i++)
        {
            if (text_boxes[i]->check_if_clicked(x, y))
            {
                text_boxes[i]->set_editing(true); // we are editing this box!
                cur_text_index = i;
            }
            else
            {
                text_boxes[i]->set_editing(false);
            }
        }
        for (int i = 0; i < buttons.size(); i++)
        {
            if (buttons[i].check_if_clicked(x, y))
            {
                // do whatever
            }
        }
    }

    void process_text_input(SDL_Keycode key)
    { // take in char or something and process it
        if (cur_text_index == -1 || !key_is_valid(key))
        { // no text box is active
            return;
        }

        std::string cur_str = text_boxes[cur_text_index]->get_text();
        cur_str.pop_back();
        // printf("%c\n", (char)key);
        if (key == SDLK_ESCAPE)
        {
            text_boxes[cur_text_index]->set_editing(false);
            return;
        }
        if (key != SDLK_BACKSPACE)
        {
            cur_str += (char)key;
        }
        else
        {
            cur_str.pop_back();
        }
        cur_str += "|";
        text_boxes[cur_text_index]->set_text(cur_str);
        return;
    }

    bool key_is_valid(SDL_Keycode key)
    {
        // switch (key)
        // {
        // case SDLK_0:
        // case SDLK_1:
        // case SDLK_2:
        // case SDLK_3:
        // case SDLK_4:
        // case SDLK_5:
        // case SDLK_6:
        // case SDLK_7:
        // case SDLK_8:
        // case SDLK_9:
        // case SDLK_BACKSPACE:
        // case SDLK_PERIOD:
        // case SDLK_ESCAPE:
        // case SDLK_MINUS:
        //     return true;
        //     break;
        // default:
        //     return false;
        //     break;
        // }
        return (int) key < 178; //IDK !!
    }
};

#endif