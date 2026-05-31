#ifndef TEXT_H
#define TEXT_H

#include <string>

class text_t {
  public:
    text_t() {}

    text_t(const float corner_x, const float corner_y, std::string str) :  corner_x(corner_x),
     corner_y(corner_y), str(str){} 

    float get_x() {return corner_x;}
    float get_y() {return corner_y;}
    std::string get_text() {return str;}
    void set_text(std::string new_str){str = new_str;}
    
  protected:
    float corner_x;
    float corner_y;
    std::string str;

};

#endif