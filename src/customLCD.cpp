#include "customLCD.hpp"
#include "globals.hpp"
#include "pros/colors.hpp"
#include "pros/screen.hpp"
#include "pros/rtos.hpp"
#include <string>
#include <vector>
#include <cmath>

struct HomeButtons {
    Button autons;
    Button motors;
    Button controls;
    Button sensors;
    Button tunePID;
    Button alliance;
};

static std::vector<Button> autonSelectorButtons;
static double pathProgress = 0.0;
static int pendingAutonIndex = -1;
static bool showingAutonPopup = false;
static int popupStartTime = 0;
static bool wasScreenTouched = false;

static HomeButtons getHomeButtons();
static void drawBackground();
static void drawHomePage(const HomeButtons& btns);
static void drawAutonPage();
static void drawAutonPopup();
static void drawPath(int startingPos, int offsetX, int offsetY);
static void drawButton(const Button& button);
static void processTouches(const pros::screen_touch_status_s_t& touch, const HomeButtons& homeBtns);

Button::Button(int x, int y, int width, int height, std::string text, uint32_t fillColor, uint32_t textColor)
    : x(x), y(y), width(width), height(height), text(text), fillColor(fillColor), textColor(textColor) {}

bool Button::isPressed(int touchX, int touchY) {
    return touchX >= x && touchX <= x + width &&
           touchY >= y && touchY <= y + height;
}

bool Button::isPressed() {
    auto touch = pros::screen::touch_status();
    if (touch.touch_status != pros::E_TOUCH_PRESSED) return false;
    return isPressed(touch.x, touch.y);
}

static HomeButtons getHomeButtons() {
    return {
        {40, 55, 180, 60, "Autons"},
        {260, 55, 180, 60, "Motors"},
        {40, 130, 180, 60, "Controls"},
        {260, 130, 180, 60, "Sensors"},
        {40, 205, 180, 60, "Tune PID"},
        {
            260, 205, 180, 60,
            allianceColor == "RED" ? "Alliance: RED" : "Alliance: BLUE",
            allianceColor == "RED" ? pros::c::COLOR_RED : pros::c::COLOR_BLUE
        }
    };
}

void initUI() {
    autonSelectorButtons.clear();
    for (int i = 0; i < 8; i++) {
        autonSelectorButtons.push_back({
            300,            // x
            50 + i * 20,    // y
            160,            // width
            18,             // height
            "Auton " + std::to_string(i)
        });
    }
}

static void drawButton(const Button& button) {
    int radius = 10;
    bool isSmallButton = button.height < 25;
    if (isSmallButton) radius = button.height / 2 - 1;

    pros::screen::set_pen(button.fillColor);

    pros::screen::fill_rect(button.x + radius, button.y,
                            button.x + button.width - radius,
                            button.y + button.height);

    pros::screen::fill_rect(button.x, button.y + radius,
                            button.x + button.width,
                            button.y + button.height - radius);

    pros::screen::fill_circle(button.x + radius, button.y + radius, radius);
    pros::screen::fill_circle(button.x + button.width - radius, button.y + radius, radius);
    pros::screen::fill_circle(button.x + radius, button.y + button.height - radius, radius);
    pros::screen::fill_circle(button.x + button.width - radius, button.y + button.height - radius, radius);

    pros::screen::set_pen(button.textColor);

    int lineOffset = (button.height / 2) - 8;
    if (isSmallButton) lineOffset = 0;

    pros::screen::print(
        isSmallButton ? pros::E_TEXT_SMALL : pros::E_TEXT_MEDIUM,
        (button.y + lineOffset) / 20 + 1,
        button.text.c_str()
    );
}

static void drawBackground() {
    pros::screen::set_pen(pros::c::COLOR_LIGHT_GRAY);
    pros::screen::fill_rect(0, 0, 480, 35);

    pros::screen::set_pen(pros::c::COLOR_BLACK);
    std::string batteryText = "Batt%: " + std::to_string((int)pros::battery::get_capacity());
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, batteryText.c_str());
}

static void drawHomePage(const HomeButtons& btns) {
    drawButton(btns.autons);
    drawButton(btns.motors);
    drawButton(btns.controls);
    drawButton(btns.sensors);
    drawButton(btns.tunePID);
    drawButton(btns.alliance);
}

static void drawPath(int startingPos, int offsetX, int offsetY) {
    if (startingPos < 0 || startingPos >= autonPaths.size()) return;
    auto &path = autonPaths[startingPos];
    if (path.size() < 2) return;

    pros::screen::set_pen(pros::c::COLOR_WHITE);
    for (size_t i = 1; i < path.size(); i++) {
        pros::screen::draw_line(
            offsetX + (int)path[i-1][0], offsetY + (int)path[i-1][1],
            offsetX + (int)path[i][0], offsetY + (int)path[i][1]
        );
    }

    int seg = (int)pathProgress;
    double t = pathProgress - seg;
    seg = seg % (path.size() - 1);

    double x = path[seg][0] + (path[seg+1][0] - path[seg][0]) * t;
    double y = path[seg][1] + (path[seg+1][1] - path[seg][1]) * t;

    pros::screen::set_pen(pros::c::COLOR_RED);
    pros::screen::fill_circle(offsetX + (int)x, offsetY + (int)y, 3);

    pathProgress += 0.02; 
    if (pathProgress >= path.size() - 1) pathProgress = 0;
}

static void drawAutonPage() {
    pros::screen::set_pen(pros::c::COLOR_DARK_GRAY);
    pros::screen::fill_rect(20, 50, 140, 170);

    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, 3, "FIELD");

    drawPath(currentStartingPos, 20, 50);

    for (int i = 0; i < 8; i++) {
        drawButton(autonSelectorButtons[i]);
        if (currentStartingPos == i) {
            pros::screen::set_pen(pros::c::COLOR_WHITE);
            int checkY = autonSelectorButtons[i].y / 20 + 1;
            pros::screen::print(pros::E_TEXT_SMALL, checkY, "✓");
        }
    }
}

static void drawAutonPopup() {
    pros::screen::set_pen(pros::c::COLOR_LIGHT_GRAY);
    pros::screen::fill_rect(90, 70, 390, 180);

    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::print(pros::E_TEXT_LARGE, 4, "ARE YOU SURE?");

    // Blue Yes/No Buttons
    Button yes = {140, 120, 100, 40, "YES"};
    Button no  = {260, 120, 100, 40, "NO"};

    drawButton(yes);
    drawButton(no);
}

static void processTouches(const pros::screen_touch_status_s_t& touch, const HomeButtons& homeBtns) {
    bool isTouchActive = (touch.touch_status == pros::E_TOUCH_PRESSED);
    bool isNewClick = isTouchActive && !wasScreenTouched;
    wasScreenTouched = isTouchActive;

    if (!isNewClick) return;

    if (showingAutonPopup) {
        bool yesCoords = touch.x > 140 && touch.x < 240 && touch.y > 120 && touch.y < 160;
        bool noCoords = touch.x > 260 && touch.x < 360 && touch.y > 120 && touch.y < 160;

        if (yesCoords) {
            currentStartingPos = pendingAutonIndex;
            showingAutonPopup = false;
            pathProgress = 0;
        }
        if (noCoords) {
            showingAutonPopup = false;
            pendingAutonIndex = -1;
        }
        return;
    }

    if (currentPage == 0) {
        if (const_cast<HomeButtons&>(homeBtns).autons.isPressed(touch.x, touch.y)) {
            currentPage = 1;
        }

        if (const_cast<HomeButtons&>(homeBtns).alliance.isPressed(touch.x, touch.y)) {
            allianceColor = (allianceColor == "RED") ? "BLUE" : "RED";
        }
    }
    else if (currentPage == 1) {
        for (int i = 0; i < 8; i++) {
            if (autonSelectorButtons[i].isPressed(touch.x, touch.y)) {
                pendingAutonIndex = i;
                showingAutonPopup = true;
                popupStartTime = pros::millis();
            }
        }
    }
}

void updateUI() {
    pros::screen::set_eraser(pros::c::COLOR_WHITE);
    pros::screen::erase();

    drawBackground();

    HomeButtons homeBtns = getHomeButtons();

    if (currentPage == 0) {
        drawHomePage(homeBtns);
    } else if (currentPage == 1) {
        drawAutonPage();
    }

    if (showingAutonPopup) {
        if (pros::millis() - popupStartTime > 5000) {
            showingAutonPopup = false;
            pendingAutonIndex = -1;
        } else {
            drawAutonPopup();
        }
    }

    processTouches(pros::screen::touch_status(), homeBtns);
}