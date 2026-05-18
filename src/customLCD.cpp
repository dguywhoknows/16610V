#include "customLCD.hpp"
#include "globals.hpp"
#include "pros/colors.hpp"
#include "pros/screen.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

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
static int selectedMotorIndex = -1;
static int selectedPistonIndex = -1;
static int motorVoltage = 0;
static bool motorBrake = false;
static bool keepMotorChanges = false;
static HomeButtons getHomeButtons();
static void drawBackground();
static void drawHomePage(const HomeButtons& btns);
static void drawAutonPage();
static void drawMotorsPage();
static void drawSensorsPage();
static void drawAutonPopup();
static void drawPath(int startingPos, int offsetX, int offsetY);
static void drawButton(const Button& button);
static void processTouches(const pros::screen_touch_status_s_t& touch, const HomeButtons& homeBtns);
static void applySelectedMotorVoltage();
static double getMotorTemperature(int index);
static uint32_t getTemperatureColor(double temp);
static bool keepLateralPID = false;
static bool keepAngularPID = false;
static double originalLateralkP = 0;
static double originalLateralkI = 0;
static double originalLateralkD = 0;
static double originalLateralSlew = 0;
static double originalAngularkP = 0;
static double originalAngularkI = 0;
static double originalAngularkD = 0;
static double originalAngularSlew = 0;

Button::Button(
    int x,
    int y,
    int width,
    int height,
    std::string text,
    uint32_t fillColor,
    uint32_t textColor
)
    : x(x),
      y(y),
      width(width),
      height(height),
      text(text),
      fillColor(fillColor),
      textColor(textColor) {}

bool Button::isPressed(int touchX, int touchY) const {
    return touchX >= x &&
           touchX <= x + width &&
           touchY >= y &&
           touchY <= y + height;
}

bool Button::isPressed() {
    auto touch = pros::screen::touch_status();
    if (touch.touch_status != pros::E_TOUCH_PRESSED) {
        return false;
    }
    return isPressed(touch.x, touch.y);
}

static HomeButtons getHomeButtons() {
    return {{40, 55, 180, 60, "Autons"}, {260, 55, 180, 60, "Motors"}, {40, 130, 180, 60, "Controls"}, {260, 130, 180, 60, "Sensors"}, {40, 205, 180, 60, "Tune PID"}, {260, 205, 180, 60, allianceColor == "RED" ? "Alliance: RED" : "Alliance: BLUE", allianceColor == "RED" ? pros::c::COLOR_RED : pros::c::COLOR_BLUE}};
}

void initUI() {
    autonSelectorButtons.clear();
    for (int i = 0; i < 8; i++) {
        autonSelectorButtons.push_back({300, 50 + i * 20, 160, 18, "Auton " + std::to_string(i)});
    }
    originalLateralkP = lateralSettings.kP;
    originalLateralkI = lateralSettings.kI;
    originalLateralkD = lateralSettings.kD;
    originalLateralSlew = lateralSettings.slew;

    originalAngularkP = angularSettings.kP;
    originalAngularkI = angularSettings.kI;
    originalAngularkD = angularSettings.kD;
    originalAngularSlew = angularSettings.slew;
}

static void drawButton(const Button& button) {
    int radius = 10;
    if (button.height < 25) {
        radius = button.height / 2 - 1;
    }
    pros::screen::set_pen(button.fillColor);
    pros::screen::fill_rect(button.x + radius, button.y, button.x + button.width - radius, button.y + button.height);
    pros::screen::fill_rect(button.x, button.y + radius, button.x + button.width, button.y + button.height - radius);
    pros::screen::fill_circle(button.x + radius, button.y + radius, radius);
    pros::screen::fill_circle(button.x + button.width - radius, button.y + radius, radius);
    pros::screen::fill_circle(button.x + radius, button.y + button.height - radius, radius);
    pros::screen::fill_circle(button.x + button.width - radius, button.y + button.height - radius, radius);
    pros::screen::set_pen(button.textColor);
    int lineOffset = button.height < 25 ? 0 : (button.height / 2) - 8;
    pros::screen::print(button.height < 25 ? pros::E_TEXT_SMALL : pros::E_TEXT_MEDIUM, (button.y + lineOffset) / 20 + 1, button.text.c_str());
}

static void drawBackground() {
    pros::screen::set_pen(pros::c::COLOR_LIGHT_GRAY);
    pros::screen::fill_rect(0, 0, 480, 35);
    pros::screen::set_pen(pros::c::COLOR_BLACK);

    std::string batteryText = "Batt%: " + std::to_string((int)pros::battery::get_capacity());
    pros::screen::print(pros::E_TEXT_MEDIUM, 1,batteryText.c_str());
    Button controllerButton = {250, 2, 170, 28, controllerEnabled ? "Controller: ON" : "Controller: OFF"};
    drawButton(controllerButton);
    if (currentPage != 0) {
        Button backButton = {425, 2, 50, 28, "BACK"};
        drawButton(backButton);
    }
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
    auto& path = autonPaths[startingPos];
    if (path.size() < 2) return;
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    for (size_t i = 1; i < path.size(); i++) {
        pros::screen::draw_line(offsetX + (int)path[i - 1][0], offsetY + (int)path[i - 1][1], offsetX + (int)path[i][0], offsetY + (int)path[i][1]);
    }

    int seg = (int)pathProgress;
    double t = pathProgress - seg;
    seg = seg % (path.size() - 1);
    double x = path[seg][0] + (path[seg + 1][0] - path[seg][0]) * t;
    double y = path[seg][1] + (path[seg + 1][1] - path[seg][1]) * t;
    pros::screen::set_pen(pros::c::COLOR_RED);
    pros::screen::fill_circle(offsetX + (int)x, offsetY + (int)y, 3);
    pathProgress += 0.02;
    if (pathProgress >= path.size() - 1) {
        pathProgress = 0;
    }
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

static double getMotorTemperature(int index) {
    switch (index) {
        case 0: return something1.get_temperature();
        case 1: return something2.get_temperature();
        case 2: return something3.get_temperature();
        case 3: return something4.get_temperature();
        case 4: return leftMotor1.get_temperature();
        case 5: return leftMotor2.get_temperature();
        case 6: return leftMotor3.get_temperature();
        case 7: return rightMotor1.get_temperature();
        case 8: return rightMotor2.get_temperature();
        case 9: return rightMotor3.get_temperature();
    }
    return 0;
}

static uint32_t getTemperatureColor(double temp) {
    if (temp < 35) {
        return pros::c::COLOR_GREEN;
    }
    if (temp < 50) {
        return pros::c::COLOR_YELLOW;
    }
    return pros::c::COLOR_RED;
}

static void applySelectedMotorVoltage() {
    switch (selectedMotorIndex) {
        case 0: something1.move(motorVoltage);
            break;
        case 1: something2.move(motorVoltage);
            break;
        case 2: something3.move(motorVoltage);
            break;
        case 3: something4.move(motorVoltage);
            break;
        case 4: leftMotor1.move(motorVoltage);
            break;
        case 5: leftMotor2.move(motorVoltage);
            break;
        case 6: leftMotor3.move(motorVoltage);
            break;
        case 7: rightMotor1.move(motorVoltage);
            break;
        case 8: rightMotor2.move(motorVoltage);
            break;
        case 9: rightMotor3.move(motorVoltage);
            break;
        case 10: driveLeftMotors.move(motorVoltage);
            break;
        case 11: driveRightMotors.move(motorVoltage);
            break;
        case 12: fullDrive.move(motorVoltage);
            break;
    }
}

static void drawMotorsPage() {
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    int x = 10;
    int y = 45;
    std::vector<Button> motorButtons = {
        {x,y+0,220,15,"Motor 1"},
        {x,y+15,220,15,"Motor 2"},
        {x,y+30,220,15,"Motor 3"},
        {x,y+45,220,15,"Motor 4"},
        {x,y+70,220,15,"Left 1"},
        {x,y+85,220,15,"Left 2"},
        {x,y+100,220,15,"Left 3"},
        {x,y+125,220,15,"Right 1"},
        {x,y+140,220,15,"Right 2"},
        {x,y+155,220,15,"Right 3"},
        {x,y+180,220,15,"Left Side"},
        {x,y+195,220,15,"Right Side"},
        {x,y+210,220,15,"Drivetrain"},
        {x,y+235,220,15,"Piston 1"},
        {x,y+250,220,15,"Piston 2"},
        {x,y+265,220,15,"Piston 3"}
    };

    for (size_t i = 0; i < motorButtons.size(); i++) {
        drawButton(motorButtons[i]);
        if ((int)i == selectedMotorIndex) {
            pros::screen::set_pen(pros::c::COLOR_GREEN);
            pros::screen::draw_rect(motorButtons[i].x - 2, motorButtons[i].y - 2, motorButtons[i].x + motorButtons[i].width + 2, motorButtons[i].y + motorButtons[i].height + 2);
        }
    }

    pros::screen::set_pen(pros::c::COLOR_DARK_GRAY);
    pros::screen::fill_rect(240, 40, 470, 230);
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "CONTROL PANEL");

    Button minus = {250, 60, 40, 30, "-"};
    Button plus = {420, 60, 40, 30, "+"};
    Button reset = {320, 40, 70, 20, "RESET"};
    Button brake = {250, 105, 150, 25, motorBrake ? "BRAKE: ON" : "BRAKE: OFF"};
    Button keep = {250, 200, 150, 25, keepMotorChanges ? "KEEP ✓" : "KEEP"};

    drawButton(minus);
    drawButton(plus);
    drawButton(reset);
    drawButton(brake);
    drawButton(keep);

    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, ("Voltage: " + std::to_string(motorVoltage)).c_str());
    if (selectedMotorIndex >= 0 && selectedMotorIndex <= 9) {
        double temp = getMotorTemperature(selectedMotorIndex);
        pros::screen::set_pen(getTemperatureColor(temp));
        pros::screen::print(pros::E_TEXT_MEDIUM, 9, ("Temp: " + std::to_string(temp)).c_str());
    }
}

static void drawSensorsPage() {
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    auto pose = chassis.getPose();

    pros::screen::print(pros::E_TEXT_MEDIUM, 3, ("IMU: " + std::to_string(imu.get_rotation())).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, ("Vert Rot: " + std::to_string(verticalRotation.get_position())).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, ("Horiz Rot: " + std::to_string(horizontalRotation.get_position())).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, ("Lift Sensor: " + std::to_string(liftSensor.get_position())).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, ("Dist1: " + std::to_string(distanceSensor1.get())).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, ("Dist2: " + std::to_string(distanceSensor2.get())).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, ("Dist3: " + std::to_string(distanceSensor3.get())).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, ("Dist4: " + std::to_string(distanceSensor4.get())).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 11, ("Dist5: " + std::to_string(distanceSensor5.get())).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 13, ("Opt1 Hue: " + std::to_string(opticalSensor1.get_hue())).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 14, ("Opt2 Hue: " + std::to_string(opticalSensor2.get_hue())).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, ("X: " + std::to_string(pose.x)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, ("Y: " + std::to_string(pose.y)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, ("Theta: " +std::to_string(pose.theta)).c_str());

    Button imuReset = {320, 50, 120, 20, "Reset IMU"};
    Button vertReset = {320, 80, 120, 20, "Reset Vert"};
    Button horizReset = {320, 110, 120, 20, "Reset Horiz"};
    Button liftReset = {320, 140, 120, 20, "Reset Lift"};
    drawButton(imuReset);
    drawButton(vertReset);
    drawButton(horizReset);
    drawButton(liftReset);
}

static void drawAutonPopup() {
    pros::screen::set_pen(pros::c::COLOR_LIGHT_GRAY);
    pros::screen::fill_rect(90, 70, 390, 180);
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::print(pros::E_TEXT_LARGE, 4, "ARE YOU SURE?");
    Button yes = {140, 120, 100, 40, "YES"};
    Button no = {260, 120, 100, 40, "NO"};
    drawButton(yes);
    drawButton(no);
}

static void processTouches(const pros::screen_touch_status_s_t& touch, const HomeButtons& homeBtns) {
    bool isNewClick = (touch.touch_status == pros::E_TOUCH_PRESSED) && !wasScreenTouched;
    wasScreenTouched = (touch.touch_status == pros::E_TOUCH_PRESSED);
    if (!isNewClick) {
        return;
    }
    if (showingAutonPopup) {
        if (touch.x > 140 && touch.x < 240 && touch.y > 120 && touch.y < 160) {
            currentStartingPos = pendingAutonIndex;
            showingAutonPopup = false;
            pathProgress = 0;
        }
        if (touch.x > 260 && touch.x < 360 && touch.y > 120 && touch.y < 160) {
            showingAutonPopup = false;
            pendingAutonIndex = -1;
        }
        return;
    }
    if (touch.x >= 250 && touch.x <= 420 && touch.y >= 2 && touch.y <= 30) {
        controllerEnabled = !controllerEnabled;
        return;
    }

    if (currentPage != 0 && touch.x >= 425 && touch.x <= 475 && touch.y >= 2 && touch.y <= 30) {
        if (currentPage == 5) {
            if (!keepLateralPID) {
                lateralSettings.kP = originalLateralkP;
                lateralSettings.kI = originalLateralkI;
                lateralSettings.kD = originalLateralkD;
                lateralSettings.slew = originalLateralSlew;
            }
            if (!keepAngularPID) {
                angularSettings.kP = originalAngularkP;
                angularSettings.kI = originalAngularkI;
                angularSettings.kD = originalAngularkD;
                angularSettings.slew = originalAngularSlew;
            }
        }
        currentPage = 0;
        return;
    }

    if (currentPage == 0) {
        if (const_cast<HomeButtons&>(homeBtns).autons.isPressed(touch.x,touch.y)) {
            currentPage = 1;
        }
        if (const_cast<HomeButtons&>(homeBtns).motors.isPressed(touch.x,touch.y)) {
            currentPage = 2;
        }
        if (const_cast<HomeButtons&>(homeBtns).controls.isPressed(touch.x,touch.y)) {
            currentPage = 3;
        }
        if (const_cast<HomeButtons&>(homeBtns).sensors.isPressed(touch.x,touch.y)) {
            currentPage = 4;
        }
        if (const_cast<HomeButtons&>(homeBtns).tunePID.isPressed(touch.x,touch.y)) {
            currentPage = 5;
            originalLateralkP = lateralSettings.kP;
            originalLateralkI = lateralSettings.kI;
            originalLateralkD = lateralSettings.kD;
            originalLateralSlew = lateralSettings.slew;

            originalAngularkP = angularSettings.kP;
            originalAngularkI = angularSettings.kI;
            originalAngularkD = angularSettings.kD;
            originalAngularSlew = angularSettings.slew;
        }
        if (const_cast<HomeButtons&>(homeBtns).alliance.isPressed(touch.x, touch.y)) {
            allianceColor = (allianceColor == "RED") ? "BLUE" : "RED";
        }
    }

    else if (currentPage == 1) {
        for (int i = 0; i < 8; i++) {
            if (autonSelectorButtons[i].isPressed(touch.x,touch.y)) {
                pendingAutonIndex = i;
                showingAutonPopup = true;
                popupStartTime = pros::millis();
            }
        }
    }

    else if (currentPage == 2) {
        if (touch.x >= 10 && touch.x <= 230) {
            int relativeY = touch.y - 45;
            if (relativeY >= 0 && relativeY < 15) selectedMotorIndex = 0;
            else if (relativeY >= 15 && relativeY < 30) selectedMotorIndex = 1;
            else if (relativeY >= 30 && relativeY < 45) selectedMotorIndex = 2;
            else if (relativeY >= 45 && relativeY < 60) selectedMotorIndex = 3;
            else if (relativeY >= 70 && relativeY < 85) selectedMotorIndex = 4;
            else if (relativeY >= 85 && relativeY < 100) selectedMotorIndex = 5;
            else if (relativeY >= 100 && relativeY < 115) selectedMotorIndex = 6;
            else if (relativeY >= 125 && relativeY < 140) selectedMotorIndex = 7;
            else if (relativeY >= 140 && relativeY < 155) selectedMotorIndex = 8;
            else if (relativeY >= 155 && relativeY < 170) selectedMotorIndex = 9;
            else if (relativeY >= 180 && relativeY < 195) selectedMotorIndex = 10;
            else if (relativeY >= 195 && relativeY < 210) selectedMotorIndex = 11;
            else if (relativeY >= 210 && relativeY < 225) selectedMotorIndex = 12;
        }
        if (touch.x > 250 && touch.x < 290 && touch.y > 60 && touch.y < 90) {
            motorVoltage = std::max(-127,motorVoltage - 5);
            applySelectedMotorVoltage();
        }
        if (touch.x > 420 && touch.x < 460 && touch.y > 60 &&touch.y < 90) {
            motorVoltage = std::min(127,motorVoltage + 5);
            applySelectedMotorVoltage();
        }
        if (touch.x > 320 && touch.x < 390 && touch.y > 40 && touch.y < 60) {
            motorVoltage = 0;
            applySelectedMotorVoltage();
        }
        if (touch.x > 250 && touch.x < 400 && touch.y > 105 && touch.y < 130) {
            motorBrake = !motorBrake;
        }
        if (touch.x > 250 && touch.x < 400 && touch.y > 200 && touch.y < 225) {
            keepMotorChanges = !keepMotorChanges;
        }
    }
    else if (currentPage == 5) {
        if (touch.x > 10 && touch.x < 80 && touch.y > 50 && touch.y < 75) {
            chassis.setPose(0,0,0,false);
            chassis.moveToPoint(24,0,3000,{.maxSpeed = 127},false);
        }
        if (touch.x > 250 && touch.x < 320 && touch.y > 50 && touch.y < 75) {
            chassis.setPose(0,0,0,false);
            chassis.moveToPoint(24,0,3000,{.maxSpeed = 127},false);
        }
        if (touch.x > 90 && touch.x < 180 && touch.y > 50 && touch.y < 75) {
            pros::screen::print(pros::E_TEXT_SMALL, 15, ("Lateral | Lift: " + std::to_string(liftSensor.get_position()) + " | kP:" + std::to_string(lateralSettings.kP) + " | kI:" + std::to_string(lateralSettings.kI) + " | kD:" + std::to_string(lateralSettings.kD) + " | Slew:" + std::to_string(lateralSettings.slew)).c_str());
        }
        if (touch.x > 330 && touch.x < 420 && touch.y > 50 && touch.y < 75) {
            pros::screen::print(pros::E_TEXT_SMALL, 16, ("Angular | Lift: " + std::to_string(liftSensor.get_position()) + " | kP:" + std::to_string(angularSettings.kP) + " | kI:" + std::to_string(angularSettings.kI) + " | kD:" + std::to_string(angularSettings.kD) + " | Slew:" + std::to_string(angularSettings.slew)).c_str());
        }
        if (touch.x > 10 && touch.x < 90 && touch.y > 80 && touch.y < 100) {
            lateralSettings.kP = originalLateralkP;
            lateralSettings.kI = originalLateralkI;
            lateralSettings.kD = originalLateralkD;
            lateralSettings.slew = originalLateralSlew;
        }
        if (touch.x > 250 && touch.x < 330 && touch.y > 80 && touch.y < 100) {
            angularSettings.kP = originalAngularkP;
            angularSettings.kI = originalAngularkI;
            angularSettings.kD = originalAngularkD;
            angularSettings.slew = originalAngularSlew;
        }
        if (touch.x > 100 && touch.x < 200 && touch.y > 80 && touch.y < 100) {
            keepLateralPID = !keepLateralPID;
        }
        if (touch.x > 340 && touch.x < 440 && touch.y > 80 && touch.y < 100) {
            keepAngularPID = !keepAngularPID;
        }
        if (touch.x > 160 && touch.x < 185 && touch.y > 120 && touch.y < 140) lateralSettings.kP += 0.1;
        if (touch.x > 10 && touch.x < 35 && touch.y > 120 && touch.y < 140) lateralSettings.kP = std::max(0.0, lateralSettings.kP - 0.1);
        if (touch.x > 160 && touch.x < 185 && touch.y > 160 && touch.y < 180) lateralSettings.kI += 0.01;
        if (touch.x > 10 && touch.x < 35 && touch.y > 160 && touch.y < 180) lateralSettings.kI = std::max(0.0, lateralSettings.kI - 0.01);
        if (touch.x > 160 && touch.x < 185 && touch.y > 200 && touch.y < 220) lateralSettings.kD += 0.1;
        if (touch.x > 10 && touch.x < 35 && touch.y > 200 && touch.y < 220) lateralSettings.kD = std::max(0.0, lateralSettings.kD - 0.1);
        if (touch.x > 160 && touch.x < 185 && touch.y > 240 && touch.y < 260) lateralSettings.slew += 1;
        if (touch.x > 10 && touch.x < 35 && touch.y > 240 && touch.y < 260) lateralSettings.slew = std::max(0.0, lateralSettings.slew - 1.0);
        if (touch.x > 400 && touch.x < 425 && touch.y > 120 && touch.y < 140) angularSettings.kP += 0.1;
        if (touch.x > 250 && touch.x < 275 && touch.y > 120 && touch.y < 140) angularSettings.kP = std::max(0.0, angularSettings.kP - 0.1);
        if (touch.x > 400 && touch.x < 425 && touch.y > 160 && touch.y < 180) angularSettings.kI += 0.01;
        if (touch.x > 250 && touch.x < 275 && touch.y > 160 && touch.y < 180) angularSettings.kI = std::max(0.0, angularSettings.kI - 0.01);
        if (touch.x > 400 && touch.x < 425 && touch.y > 200 && touch.y < 220) angularSettings.kD += 0.1;
        if (touch.x > 250 && touch.x < 275 && touch.y > 200 && touch.y < 220) angularSettings.kD = std::max(0.0, angularSettings.kD - 0.1);
        if (touch.x > 400 && touch.x < 425 && touch.y > 240 && touch.y < 260) angularSettings.slew += 1;
        if (touch.x > 250 && touch.x < 275 && touch.y > 240 && touch.y < 260) angularSettings.slew = std::max(0.0, angularSettings.slew - 1.0);
    }
    else if (currentPage == 4) {
        if (touch.x > 320 && touch.x < 440 && touch.y > 50 && touch.y < 70) {
            imu.reset();
        }
        if (touch.x > 320 && touch.x < 440 && touch.y > 80 && touch.y < 100) {
            verticalRotation.reset();
        }
        if (touch.x > 320 && touch.x < 440 && touch.y > 110 && touch.y < 130) {
            horizontalRotation.reset();
        }
        if (touch.x > 320 && touch.x < 440 && touch.y > 140 && touch.y < 160) {
            liftSensor.reset();
        }
    }
}

static void drawControlsPage() {
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "DRIVER CONTROLS");
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "A:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "B:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "X:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Y:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, "L1:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 11, "L2:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 12, "R1:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 13, "R2:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "                         UP:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "                       DOWN:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "                       LEFT:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, "                      RIGHT:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 15, "Left Joystick:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, "Right Joystick:");
}

static void drawPIDPage() {
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::draw_line(240, 40, 240, 240);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "LATERAL PID");
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "                         ANGULAR PID");
    Button runLat = {10, 50, 70, 25, "RUN"};
    Button logLat = {90, 50, 90, 25, "Log PID"};
    Button resetLat = {10, 80, 80, 20, "RESET"};
    Button keepLat = {100, 80, 100, 20, keepLateralPID ? "KEEP ✓" : "KEEP PID"};
    drawButton(runLat);
    drawButton(logLat);
    drawButton(resetLat);
    drawButton(keepLat);
    Button runAng = {250, 50, 70, 25, "RUN"};
    Button logAng = {330, 50, 90, 25, "Log PID"};
    Button resetAng = {250, 80, 80, 20, "RESET"};
    Button keepAng = {340, 80, 100, 20, keepAngularPID ? "KEEP ✓" : "KEEP PID"};
    drawButton(runAng);
    drawButton(logAng);
    drawButton(resetAng);
    drawButton(keepAng);
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, ("kP: " + std::to_string(lateralSettings.kP)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, ("kI: " + std::to_string(lateralSettings.kI)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 11, ("kD: " + std::to_string(lateralSettings.kD)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 13, ("Slew: " + std::to_string(lateralSettings.slew)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, ("                    kP: " + std::to_string(angularSettings.kP)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, ("                    kI: " + std::to_string(angularSettings.kI)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 11, ("                    kD: " + std::to_string(angularSettings.kD)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 13, ("                    Slew: " + std::to_string(angularSettings.slew)).c_str());
    std::vector<Button> pidButtons = {
        {160, 120, 25, 20, "+"},
        {10, 120, 25, 20, "-"},
        {160, 160, 25, 20, "+"},
        {10, 160, 25, 20, "-"},
        {160, 200, 25, 20, "+"},
        {10, 200, 25, 20, "-"},
        {160, 240, 25, 20, "+"},
        {10, 240, 25, 20, "-"},
        {400, 120, 25, 20, "+"},
        {250, 120, 25, 20, "-"},
        {400, 160, 25, 20, "+"},
        {250, 160, 25, 20, "-"},
        {400, 200, 25, 20, "+"},
        {250, 200, 25, 20, "-"},
        {400, 240, 25, 20, "+"},
        {250, 240, 25, 20, "-"}
    };
    for (auto& btn : pidButtons) {
        drawButton(btn);
    }
}

void updateUI() {
    pros::screen::set_eraser(pros::c::COLOR_WHITE);
    pros::screen::erase();
    drawBackground();
    HomeButtons homeBtns = getHomeButtons();
    if (currentPage == 0) {
        drawHomePage(homeBtns);
    }
    else if (currentPage == 1) {
        drawAutonPage();
    }
    else if (currentPage == 2) {
        drawMotorsPage();
    }
    else if (currentPage == 3) {
        drawControlsPage();
    }
    else if (currentPage == 4) {
        drawSensorsPage();
    }
    else if (currentPage == 5) {
        drawPIDPage();
    }
    if (showingAutonPopup) {
        if (pros::millis() - popupStartTime > 5000) {
            showingAutonPopup = false;
            pendingAutonIndex = -1;
        }
        else {
            drawAutonPopup();
        }
    }
    processTouches(pros::screen::touch_status(), homeBtns);
}