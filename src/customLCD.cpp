#include "customLCD.hpp"
#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/screen.hpp"
#include "pros/misc.hpp"
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace {

constexpr int screenW = 480;
constexpr int screenH = 240;

enum class Page { auton, motors, sensors };
Page currentPage = Page::auton;
int32_t lastPressCount = 0;

struct Btn {
    int x0, y0, x1, y1;
    bool contains(int px, int py) const { return px >= x0 && px <= x1 && py >= y0 && py <= y1; }
};

const Btn backBtn = {5, 2, 75, 22};

constexpr int charW = 8;

void drawCenteredLabel(const char* text, int cx, int cy) {
    int x = cx - (int)(std::strlen(text) * charW) / 2;
    if (x < 2) x = 2;
    pros::screen::print(pros::E_TEXT_MEDIUM, x, cy, "%s", text);
}

uint32_t tempGradient(double tempC, double coolC = 25.0, double hotC = 60.0) {
    double t = (tempC - coolC) / (hotC - coolC);
    t = std::clamp(t, 0.0, 1.0);
    uint32_t r = (uint32_t)(t * 255.0);
    uint32_t g = (uint32_t)((1.0 - t) * 255.0);
    return (r << 16) | (g << 8);
}

void drawTile(int x0, int y0, int x1, int y1, const char* label, const char* value, uint32_t valueColor) {
    pros::screen::set_pen(0x00303030);
    pros::screen::fill_rect(x0, y0, x1, y1);
    pros::screen::set_pen(0x00808080);
    pros::screen::draw_rect(x0, y0, x1, y1);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, x0 + 4, y0 + 2, "%s", label);
    pros::screen::set_pen(valueColor);
    pros::screen::print(pros::E_TEXT_MEDIUM, x0 + 4, y0 + 20, "%s", value);
}

void drawButton(int x0, int y0, int x1, int y1, const char* label, bool selected) {
    pros::screen::set_pen(selected ? 0x0000AA00 : 0x00303030);
    pros::screen::fill_rect(x0, y0, x1, y1);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::draw_rect(x0, y0, x1, y1);
    drawCenteredLabel(label, (x0 + x1) / 2, (y0 + y1) / 2 - 8);
}

void drawHeader(const char* title, bool showBack) {
    pros::screen::set_pen(0x00000000);
    pros::screen::fill_rect(0, 0, screenW - 1, 44);

    if (showBack) {
        pros::screen::set_pen(0x00552222);
        pros::screen::fill_rect(backBtn.x0, backBtn.y0, backBtn.x1, backBtn.y1);
        pros::screen::set_pen(0x00FFFFFF);
        pros::screen::draw_rect(backBtn.x0, backBtn.y0, backBtn.x1, backBtn.y1);
        pros::screen::print(pros::E_TEXT_MEDIUM, backBtn.x0 + 6, backBtn.y0 + 2, "Back");
    }

    pros::screen::set_pen(0x00FFFFFF);
    char battText[24];
    std::snprintf(battText, sizeof(battText), "Batt: %.0f%%", pros::battery::get_capacity());
    drawCenteredLabel(battText, screenW / 2, 4);

    drawCenteredLabel(title, screenW / 2, 26);

    pros::screen::set_pen(0x00808080);
    pros::screen::draw_line(0, 46, screenW - 1, 46);
}

const Btn autonBtn[7] = {
    {8, 54, 116, 108},  {128, 54, 236, 108},  {248, 54, 356, 108},  {368, 54, 476, 108},
    {8, 118, 116, 172}, {128, 118, 236, 172}, {248, 118, 356, 172}
};
const char* autonNames[7] = {"Line1", "Line2", "Wall1", "Wall2", "SAWP1", "SAWP2", "Skills"};
const Btn motorsNavBtn = {8, 182, 238, 228};
const Btn sensorsNavBtn = {248, 182, 476, 228};

bool confirmPopupActive = false;
int pendingAutonIndex = -1;
const Btn confirmPopupBox = {90, 60, 390, 190};
const Btn confirmYesBtn = {110, 140, 220, 178};
const Btn confirmNoBtn = {260, 140, 370, 178};

void drawConfirmPopup() {
    pros::screen::set_pen(0x00101010);
    pros::screen::fill_rect(confirmPopupBox.x0, confirmPopupBox.y0, confirmPopupBox.x1, confirmPopupBox.y1);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::draw_rect(confirmPopupBox.x0, confirmPopupBox.y0, confirmPopupBox.x1, confirmPopupBox.y1);

    char msg[32];
    std::snprintf(msg, sizeof(msg), "Run %s?", autonNames[pendingAutonIndex]);
    drawCenteredLabel(msg, (confirmPopupBox.x0 + confirmPopupBox.x1) / 2, confirmPopupBox.y0 + 16);

    drawButton(confirmYesBtn.x0, confirmYesBtn.y0, confirmYesBtn.x1, confirmYesBtn.y1, "Yes", false);
    drawButton(confirmNoBtn.x0, confirmNoBtn.y0, confirmNoBtn.x1, confirmNoBtn.y1, "Cancel", false);
}

void handleConfirmTouch(int x, int y) {
    if (confirmYesBtn.contains(x, y)) {
        currentStartingPos = pendingAutonIndex;
        confirmPopupActive = false;
        pendingAutonIndex = -1;
    } else if (confirmNoBtn.contains(x, y)) {
        confirmPopupActive = false;
        pendingAutonIndex = -1;
    }
}

void drawAutonPage() {
    drawHeader("Autonomous Select", false);
    for (int i = 0; i < 7; i++) {
        drawButton(autonBtn[i].x0, autonBtn[i].y0, autonBtn[i].x1, autonBtn[i].y1, autonNames[i],
                   currentStartingPos == i);
    }
    drawButton(motorsNavBtn.x0, motorsNavBtn.y0, motorsNavBtn.x1, motorsNavBtn.y1, "Motors", false);
    drawButton(sensorsNavBtn.x0, sensorsNavBtn.y0, sensorsNavBtn.x1, sensorsNavBtn.y1, "Sensors", false);
}

void handleAutonTouch(int x, int y) {
    for (int i = 0; i < 7; i++) {
        if (autonBtn[i].contains(x, y)) {
            pendingAutonIndex = i;
            confirmPopupActive = true;
            return;
        }
    }
    if (motorsNavBtn.contains(x, y)) currentPage = Page::motors;
    if (sensorsNavBtn.contains(x, y)) currentPage = Page::sensors;
}

struct MotorTile { const char* label; double temp; };

constexpr int motorTileW = 115, motorTileH = 42, motorTileGap = 5, motorStartX = 5, motorStartY = 50;

Btn motorTileRect(int i) {
    int col = i % 4;
    int row = i / 4;
    int x0 = motorStartX + col * (motorTileW + motorTileGap);
    int y0 = motorStartY + row * (motorTileH + motorTileGap);
    return {x0, y0, x0 + motorTileW, y0 + motorTileH};
}

const Btn liftGroupBtn = {5, 142, 237, 184};
const Btn driveGroupBtn = {243, 142, 475, 184};
const Btn clawBtn = {164, 190, 316, 228};

enum class MotorSel { none, lift1, lift2, lift3, lift4, driveL1, driveL2, driveR1, driveR2, liftGroup, driveGroup };
bool motorControlActive = false;
MotorSel selectedMotor = MotorSel::none;

constexpr int jogMinMillivolts = 3000;
constexpr int jogMaxMillivolts = 12000;
constexpr uint32_t jogRampMs = 1500;
int jogHoldDirection = 0;
uint32_t jogHoldStartTime = 0;

int rampedJogVoltage(uint32_t heldMs) {
    double t = std::clamp((double)heldMs / jogRampMs, 0.0, 1.0);
    return jogMinMillivolts + (int)(t * (jogMaxMillivolts - jogMinMillivolts));
}

const char* motorSelName(MotorSel sel) {
    switch (sel) {
        case MotorSel::lift1: return "Lift 1 (5)";
        case MotorSel::lift2: return "Lift 2 (8)";
        case MotorSel::lift3: return "Lift 3 (19)";
        case MotorSel::lift4: return "Lift 4 (13)";
        case MotorSel::driveL1: return "DriveL 1 (11)";
        case MotorSel::driveL2: return "DriveL 2 (16)";
        case MotorSel::driveR1: return "DriveR 1 (1)";
        case MotorSel::driveR2: return "DriveR 2 (7)";
        case MotorSel::liftGroup: return "Lift Group";
        case MotorSel::driveGroup: return "Drive Group";
        default: return "";
    }
}

void jogMotorSel(MotorSel sel, int millivolts) {
    switch (sel) {
        case MotorSel::lift1: { pros::Motor m(elevator.get_port(0)); m.move_voltage(millivolts); break; }
        case MotorSel::lift2: { pros::Motor m(elevator.get_port(1)); m.move_voltage(millivolts); break; }
        case MotorSel::lift3: { pros::Motor m(elevator.get_port(2)); m.move_voltage(millivolts); break; }
        case MotorSel::lift4: { pros::Motor m(elevator.get_port(3)); m.move_voltage(millivolts); break; }
        case MotorSel::driveL1: { pros::Motor m(driveLeftMotors.get_port(0)); m.move_voltage(millivolts); break; }
        case MotorSel::driveL2: { pros::Motor m(driveLeftMotors.get_port(1)); m.move_voltage(millivolts); break; }
        case MotorSel::driveR1: { pros::Motor m(driveRightMotors.get_port(0)); m.move_voltage(millivolts); break; }
        case MotorSel::driveR2: { pros::Motor m(driveRightMotors.get_port(1)); m.move_voltage(millivolts); break; }
        case MotorSel::liftGroup: elevator.move_voltage(millivolts); break;
        case MotorSel::driveGroup: fullDrive.move_voltage(millivolts); break;
        default: break;
    }
}

const Btn motorPopupBox = {55, 45, 425, 210};
const Btn jogFwdBtn = {65, 95, 175, 150};
const Btn jogStopBtn = {185, 95, 295, 150};
const Btn jogRevBtn = {305, 95, 415, 150};
const Btn jogCloseBtn = {180, 160, 300, 200};

void drawMotorControlPopup() {
    pros::screen::set_pen(0x00101010);
    pros::screen::fill_rect(motorPopupBox.x0, motorPopupBox.y0, motorPopupBox.x1, motorPopupBox.y1);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::draw_rect(motorPopupBox.x0, motorPopupBox.y0, motorPopupBox.x1, motorPopupBox.y1);

    drawCenteredLabel(motorSelName(selectedMotor), (motorPopupBox.x0 + motorPopupBox.x1) / 2, motorPopupBox.y0 + 12);

    drawButton(jogFwdBtn.x0, jogFwdBtn.y0, jogFwdBtn.x1, jogFwdBtn.y1, "FWD", false);
    drawButton(jogStopBtn.x0, jogStopBtn.y0, jogStopBtn.x1, jogStopBtn.y1, "STOP", false);
    drawButton(jogRevBtn.x0, jogRevBtn.y0, jogRevBtn.x1, jogRevBtn.y1, "REV", false);
    drawButton(jogCloseBtn.x0, jogCloseBtn.y0, jogCloseBtn.x1, jogCloseBtn.y1, "Close", false);
}

void handleMotorControlTouch(int x, int y) {
    if (jogStopBtn.contains(x, y)) {
        jogHoldDirection = 0;
        jogHoldStartTime = 0;
        jogMotorSel(selectedMotor, 0);
    } else if (jogCloseBtn.contains(x, y)) {
        jogHoldDirection = 0;
        jogHoldStartTime = 0;
        jogMotorSel(selectedMotor, 0);
        motorControlActive = false;
        lcdMotorTestActive = false;
        selectedMotor = MotorSel::none;
    }
}

void updateMotorJogHold(const pros::screen_touch_status_s_t& touch) {
    bool touching = (touch.touch_status == pros::E_TOUCH_PRESSED || touch.touch_status == pros::E_TOUCH_HELD);
    int dir = 0;
    if (touching) {
        if (jogFwdBtn.contains(touch.x, touch.y)) dir = 1;
        else if (jogRevBtn.contains(touch.x, touch.y)) dir = -1;
    }

    if (dir != 0) {
        if (jogHoldDirection != dir) {
            jogHoldDirection = dir;
            jogHoldStartTime = pros::millis();
        }
        jogMotorSel(selectedMotor, dir * rampedJogVoltage(pros::millis() - jogHoldStartTime));
    } else if (jogHoldDirection != 0) {
        jogHoldDirection = 0;
        jogHoldStartTime = 0;
        jogMotorSel(selectedMotor, 0);
    }
}

void drawMotorsPage() {
    drawHeader("Motor Status", true);

    MotorTile tiles[8] = {
        {"Lift 1 (5)", elevator.get_temperature(0)},
        {"Lift 2 (8)", elevator.get_temperature(1)},
        {"Lift 3 (19)", elevator.get_temperature(2)},
        {"Lift 4 (13)", elevator.get_temperature(3)},
        {"DriveL 1 (11)", driveLeftMotors.get_temperature(0)},
        {"DriveL 2 (16)", driveLeftMotors.get_temperature(1)},
        {"DriveR 1 (1)", driveRightMotors.get_temperature(0)},
        {"DriveR 2 (7)", driveRightMotors.get_temperature(1)},
    };

    for (int i = 0; i < 8; i++) {
        Btn tile = motorTileRect(i);
        char value[16];
        uint32_t color;
        if (tiles[i].temp > 0.0 && tiles[i].temp < 120.0) {
            std::snprintf(value, sizeof(value), "%.1f C", tiles[i].temp);
            color = tempGradient(tiles[i].temp);
        } else {
            std::snprintf(value, sizeof(value), "N/C");
            color = 0x00808080;
        }
        drawTile(tile.x0, tile.y0, tile.x1, tile.y1, tiles[i].label, value, color);
    }

    double liftGroupAvg = (elevator.get_temperature(0) + elevator.get_temperature(1) +
                            elevator.get_temperature(2) + elevator.get_temperature(3)) / 4.0;
    double driveGroupAvg = (driveLeftMotors.get_temperature(0) + driveLeftMotors.get_temperature(1) +
                             driveRightMotors.get_temperature(0) + driveRightMotors.get_temperature(1)) / 4.0;

    char liftVal[24], driveVal[24];
    std::snprintf(liftVal, sizeof(liftVal), "%.1f C avg", liftGroupAvg);
    std::snprintf(driveVal, sizeof(driveVal), "%.1f C avg", driveGroupAvg);
    drawTile(liftGroupBtn.x0, liftGroupBtn.y0, liftGroupBtn.x1, liftGroupBtn.y1, "Lift Group", liftVal, tempGradient(liftGroupAvg));
    drawTile(driveGroupBtn.x0, driveGroupBtn.y0, driveGroupBtn.x1, driveGroupBtn.y1, "Drivetrain Group", driveVal, tempGradient(driveGroupAvg));

    bool clawOn = pros::c::adi_digital_read('A');
    drawTile(clawBtn.x0, clawBtn.y0, clawBtn.x1, clawBtn.y1, "Claw", clawOn ? "ON" : "OFF",
              clawOn ? 0x0000CC00 : 0x00808080);
}

void handleMotorsTouch(int x, int y) {
    if (backBtn.contains(x, y)) {
        currentPage = Page::auton;
        return;
    }
    for (int i = 0; i < 8; i++) {
        if (motorTileRect(i).contains(x, y)) {
            selectedMotor = static_cast<MotorSel>(static_cast<int>(MotorSel::lift1) + i);
            motorControlActive = true;
            lcdMotorTestActive = true;
            return;
        }
    }
    if (liftGroupBtn.contains(x, y)) {
        selectedMotor = MotorSel::liftGroup;
        motorControlActive = true;
        lcdMotorTestActive = true;
        return;
    }
    if (driveGroupBtn.contains(x, y)) {
        selectedMotor = MotorSel::driveGroup;
        motorControlActive = true;
        lcdMotorTestActive = true;
        return;
    }
    if (clawBtn.contains(x, y)) {
        bool clawOn = pros::c::adi_digital_read('A');
        endEffectorPiston.set_value(!clawOn);
        return;
    }
}

void drawSensorsPage() {
    drawHeader("Sensor Readings", true);

    lemlib::Pose pose = chassis.getPose(false);
    char line[64];
    int y = 52;
    const int lineH = 20;

    pros::screen::set_pen(0x00FFFFFF);

    std::snprintf(line, sizeof(line), "IMU Heading: %.1f deg", imu.get_heading());
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, y, "%s", line);
    y += lineH;

    std::snprintf(line, sizeof(line), "Vertical Rot: %d   Horizontal Rot: %d",
                  verticalRotation.get_position(), horizontalRotation.get_position());
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, y, "%s", line);
    y += lineH;

    std::snprintf(line, sizeof(line), "Pose  X: %.2f  Y: %.2f  Theta: %.1f", pose.x, pose.y, pose.theta);
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, y, "%s", line);
    y += lineH;

    std::snprintf(line, sizeof(line), "Dist Back: %d mm", distBack.get());
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, y, "%s", line);
    y += lineH;

    std::snprintf(line, sizeof(line), "Dist Left: %d mm", distLeft.get());
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, y, "%s", line);
    y += lineH;

    std::snprintf(line, sizeof(line), "Dist Right: %d mm", distRight.get());
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, y, "%s", line);
    y += lineH;
}

void handleBackTouch(int x, int y) {
    if (backBtn.contains(x, y)) currentPage = Page::auton;
}

void draw() {
    switch (currentPage) {
        case Page::auton: drawAutonPage(); break;
        case Page::motors: drawMotorsPage(); break;
        case Page::sensors: drawSensorsPage(); break;
    }
    if (confirmPopupActive) {
        drawConfirmPopup();
    } else if (motorControlActive) {
        drawMotorControlPopup();
    }
}

void handleTouch(int x, int y) {
    if (confirmPopupActive) {
        handleConfirmTouch(x, y);
        return;
    }
    if (motorControlActive) {
        handleMotorControlTouch(x, y);
        return;
    }
    switch (currentPage) {
        case Page::auton: handleAutonTouch(x, y); break;
        case Page::motors: handleMotorsTouch(x, y); break;
        case Page::sensors: handleBackTouch(x, y); break;
    }
}

}

void initUI() {
    pros::screen::set_eraser(0x00000000);
    pros::screen::erase();
    draw();
}

void updateUI() {
    pros::screen_touch_status_s_t touch = pros::screen::touch_status();
    if (touch.press_count != lastPressCount && touch.press_count > 0) {
        lastPressCount = touch.press_count;
        handleTouch(touch.x, touch.y);
    }
    if (motorControlActive) {
        updateMotorJogHold(touch);
    }
    draw();
}
