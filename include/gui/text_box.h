#ifndef TEXT_BOX_H
#define TEXT_BOX_H

#define CURSOR "|"

#include <string>

class text_box
{
public:
    bool has_cursor = false;
    text_box() {}

    text_box(const float corner_x, const float corner_y, std::string str) : corner_x(corner_x),
                                                                            corner_y(corner_y), str(str) { width = calculate_width(str.length()); }

    float get_x() { return corner_x; }
    float get_y() { return corner_y; }
    float get_width() { return width; }
    float get_height() { return height; }
    virtual std::string get_text() { return str; }
    virtual std::string get_show_text() { return str;}
    virtual void set_text(std::string new_str)
    {
        str = new_str;
        width = calculate_width(new_str.length());
    }

    bool check_if_clicked(float x, float y)
    {
        if (x > corner_x && x < (corner_x + width))
        {
            if (y > corner_y && y < (corner_y + height))
            {
                return true;
            }
        }
        return false;
    }

    virtual void set_editing(bool new_state)
    {
        activated = new_state;
        if (activated && !has_cursor)
        {
            set_text(str + CURSOR);
            has_cursor = true;
            return;
        }

        if (!activated && has_cursor)
        {
            str.pop_back();
            set_text(str);
            has_cursor = false;
            return;
        }
    }

    bool is_being_edited() { return activated; }

protected:
    float corner_x;
    float corner_y;
    float height = 10;
    float width;
    bool activated = false;
    std::string str;

    float calculate_width(int length)
    {
        return length * 8. + 2.;
    }
};

//width should be string length
class fixed_width_text_box : public text_box
{
    public:
    fixed_width_text_box(const float corner_x, const float corner_y, const float length, std::string str) : text_box(corner_x,corner_y,str), full_str(str), max_length(length){ 
        width = calculate_width();
    }

    void set_text(std::string new_str) override
    {
        full_str = new_str;
        if (new_str.length() <= max_length){
            //printf("new string is less than max length\n");
            str = new_str;
            return;
        }

        //printf("running new set_text!\n");

        str = new_str.substr(new_str.length() - max_length,new_str.length());
        //printf("%s\n",str.c_str());
        
    }

    void set_editing(bool new_state) override
    {
        activated = new_state;
        if (activated && !has_cursor)
        {
            set_text(full_str + CURSOR);
            has_cursor = true;
            return;
        }

        if (!activated && has_cursor)
        {
            full_str.pop_back();
            set_text(full_str);
            has_cursor = false;
            return;
        }
    }

    
    std::string get_show_text() override { return str;}
    std::string get_text() override { return full_str; } //only ever returns full string, not string to show, need to edit parent class
    
    protected:
    std::string full_str;
    int max_length;

     float calculate_width()
    {
        return max_length * 8. + 2.;
    }
    
};

#endif