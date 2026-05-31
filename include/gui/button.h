#ifndef BUTTON_H
#define BUTTON_H

#include "panel.h"
#include <functional>
#include <string>

class button : public panel
{
public:
    // using panel::panel; // just use the panel constructor
    button(
        float x,
        float y,
        float width,
        float height,
        std::string text,
        std::function<void()> callback)
        : panel(x, y, width, height),
          on_click(callback), text(text)
    {
        text_x = x;
        text_y = y + (height/2.) - 4;
    }

    void click()
    {
        cur_tick = 0;
        clicked = true;

        if (on_click)
        {
            on_click();
        }
        else
        {
            printf("no function bound to this button!\n");
        }
        return;
    }

    bool advance_pressed()
    { // also returns if its no longer clicked
        if (clicked)
        {
            cur_tick++;
            if (cur_tick >= PRESSED_LENGTH)
            {
                cur_tick = 0;
                clicked = false;
                return true;
            }
        }
        return false;
    }

    bool check_if_clicked(float x, float y)
    {
        if (x > corner_x && x < (corner_x + width))
        {
            if (y > corner_y && y < (corner_y + height))
            {
                click();
                return true;
            }
        }
        return false;
    }

    bool is_clicked()
    {
        return clicked;
    }

    std::string get_text(){
        return text;
    }

    float get_text_x(){return text_x;}
    float get_text_y(){return text_y;}

private:
    int PRESSED_LENGTH = 10; // idk make it static
    int cur_tick = 0;
    bool clicked = false;
    float text_x;
    float text_y;
    std::string text;
    std::function<void()> on_click;
};

#endif