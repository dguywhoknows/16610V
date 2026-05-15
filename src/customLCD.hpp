#ifndef CUSTOMLCD_HPP
#define CUSTOMLCD_HPP

#pragma once
#include "main.h"
#include <string>
#include <vector>

class Button {
public:
    int x;
    int y;
    int width;
    int height;
    std::string text;
    uint32_t fillColor;
    uint32_t textColor;

    Button(
        int x,
        int y,
        int width,
        int height,
        std::string text,
        uint32_t fillColor = pros::c::COLOR_BLUE,
        uint32_t textColor = pros::c::COLOR_WHITE
    );

    bool isPressed(int touchX, int touchY);
    
    bool isPressed();
};

void initUI();
void updateUI();

#endif