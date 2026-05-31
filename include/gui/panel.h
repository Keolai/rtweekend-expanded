#ifndef PANEL_H
#define PANEL_H

class panel {
  public:
    panel() {}

    panel(const float corner_x, const float corner_y, const float width, const float height) :  corner_x(corner_x),
     corner_y(corner_y), width(width), height(height){} //maybe need vec2 class...

    float get_x() {return corner_x;}
    float get_y() {return corner_y;}
    float get_width() {return width;}
    float get_height() {return height;}
    
  protected:
    float corner_x;
    float corner_y;
    float width;
    float height;
};

#endif