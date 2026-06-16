#include "customLCD.hpp"   // header, declares Button class, initUI(), updateUI()
#include "globals.hpp"     // access to all hardware objects: chassis, motors, sensors, PID settings
#include "pros/colors.hpp" // color constants like COLOR_RED, COLOR_GREEN, COLOR_WHITE, etc.
#include "pros/screen.hpp" // V5 brain screen API
#include "pros/rtos.hpp"   // pros::millis(), used to time the auton confirmation popup
#include <cmath>           // math functions (not directly used here but available)
#include <string>          // std::string, used everywhere for button labels and print strings
#include <vector>          // std::vector, used for lists of buttons and motor button arrays
#include <algorithm>       // std::max(), std::min(), used to clamp motor voltage to [-127, 127]

// ---- Button struct to hold which buttons appear on the home screen ----
struct HomeButtons {
    Button autons;    // "Autons" button, navigates to autonomous selector page
    Button motors;    // "Motors" button, navigates to motor test/control page
    Button controls;  // "Controls" button, navigates to driver control reference page
    Button sensors;   // "Sensors" button, navigates to sensor readout page
    Button tunePID;   // "Tune PID" button, navigates to live PID tuning page
    Button alliance;  // "Alliance" toggle button, switches between RED and BLUE
};

// ---- Module-level state (private to this file) ----
static std::vector<Button> autonSelectorButtons; // list of 8 buttons on the auton selector page, one per auton path
static double pathProgress = 0.0;                // animation counter for the moving dot along the path preview (0 to path.size()-1)
static int pendingAutonIndex = -1;               // which auton the user tapped before confirming, held until YES/NO popup
static bool showingAutonPopup = false;           // whether the "ARE YOU SURE?" confirmation popup is currently visible
static int popupStartTime = 0;                   // millis() timestamp when the popup appeared: auto-dismisses after 5 seconds
static bool wasScreenTouched = false;            // tracks whether the screen was touched last frame: used to detect new presses (rising edge)
static int selectedMotorIndex = -1;              // which motor/group is currently selected on the motors page (-1 = none)
static int selectedPistonIndex = -1;             // which piston is selected (not fully implemented yet)
static int motorVoltage = 0;                     // current test voltage being sent to the selected motor (-127 to 127)
static bool motorBrake = false;                  // whether the selected motor should brake (true) or coast (false) when voltage = 0
static bool keepMotorChanges = false;            // if true, motor voltage changes persist when leaving the motors page

// ---- Forward declarations (defined later in this file) ----
static HomeButtons getHomeButtons();                                                        // builds and returns the HomeButtons struct
static void drawBackground();                                                               // draws the top bar (battery %, controller toggle, back button)
static void drawHomePage(const HomeButtons& btns);                                         // draws all 6 home page buttons
static void drawAutonPage();                                                                // draws the auton selector with field preview and 8 auton buttons
static void drawMotorsPage();                                                               // draws the motor list and control panel
static void drawSensorsPage();                                                              // draws all sensor readouts and reset buttons
static void drawAutonPopup();                                                               // draws the "ARE YOU SURE?" confirmation popup
static void drawPath(int startingPos, int offsetX, int offsetY);                           // draws an animated path preview on a mini field diagram
static void drawButton(const Button& button);                                              // draws a single rounded-rectangle button with text
static void processTouches(const pros::screen_touch_status_s_t& touch, const HomeButtons& homeBtns); // handles all touch input for whichever page is active
static void applySelectedMotorVoltage();                                                   // sends the current motorVoltage to whichever motor/group is selected
static double getMotorTemperature(int index);                                              // returns the temperature of a motor by index (0–9)
static uint32_t getTemperatureColor(double temp);                                          // returns green/yellow/red color based on temperature threshold

// ---- PID tuning state ----
static bool keepLateralPID = false;    // if true, lateral PID changes persist when leaving the Tune PID page
static bool keepAngularPID = false;    // if true, angular PID changes persist when leaving the Tune PID page
static double originalLateralkP = 0;  // snapshot of lateral kP when entering Tune PID: restored on exit if keepLateralPID is false
static double originalLateralkI = 0;  // snapshot of lateral kI
static double originalLateralkD = 0;  // snapshot of lateral kD
static double originalLateralSlew = 0;// snapshot of lateral slew
static double originalAngularkP = 0;  // snapshot of angular kP
static double originalAngularkI = 0;  // snapshot of angular kI
static double originalAngularkD = 0;  // snapshot of angular kD
static double originalAngularSlew = 0;// snapshot of angular slew

// ---- Button constructor ----
Button::Button(int x, int y, int width, int height, std::string text, uint32_t fillColor, uint32_t textColor)
    : x(x), y(y), width(width), height(height), text(text), fillColor(fillColor), textColor(textColor) {}
// stores all the visual properties of a button: position, size, label, and colors

bool Button::isPressed(int touchX, int touchY) const {
    return touchX >= x &&           // touch is to the right of the left edge
           touchX <= x + width &&   // touch is to the left of the right edge
           touchY >= y &&           // touch is below the top edge
           touchY <= y + height;    // touch is above the bottom edge
    // returns true if the (touchX, touchY) coordinate is inside this button's rectangle
}

bool Button::isPressed() {
    auto touch = pros::screen::touch_status(); // reads the current touch state from the V5 screen
    if (touch.touch_status != pros::E_TOUCH_PRESSED) {
        return false; // screen is not being touched right now
    }
    return isPressed(touch.x, touch.y); // check if the touch position falls inside this button
}

static HomeButtons getHomeButtons() {
    // builds all 6 home page buttons with their positions, sizes, and labels
    return {
        {40, 55, 180, 60, "Autons"},    // Autons button: top-left area
        {260, 55, 180, 60, "Motors"},   // Motors button: top-right area
        {40, 130, 180, 60, "Controls"}, // Controls button: middle-left
        {260, 130, 180, 60, "Sensors"}, // Sensors button: middle-right
        {40, 205, 180, 60, "Tune PID"}, // Tune PID button: bottom-left
        {260, 205, 180, 60,             // Alliance button: bottom-right
            allianceColor == "RED" ? "Alliance: RED" : "Alliance: BLUE",  // label changes with current alliance
            allianceColor == "RED" ? pros::c::COLOR_RED : pros::c::COLOR_BLUE} // fill color matches alliance color
    };
}

void initUI() {
    autonSelectorButtons.clear(); // remove any buttons from a previous init call

    for (int i = 0; i < 8; i++) { // create 8 auton selector buttons, stacked vertically
        autonSelectorButtons.push_back({300, 50 + i * 20, 160, 18, "Auton " + std::to_string(i)});
        // each button is 160px wide, 18px tall, starting at x=300, spaced 20px apart vertically
    }

    // snapshot current PID values so we can restore them if user exits without saving
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
    int radius = 10; // default corner radius for rounded rectangle
    if (button.height < 25) {
        radius = button.height / 2 - 1; // for small buttons, make radius proportional to height so it looks like a pill/capsule
    }

    pros::screen::set_pen(button.fillColor); // set drawing color to the button's fill color

    // draw the rounded rectangle using 3 overlapping shapes: a vertical rect, a horizontal rect, and 4 corner circles
    pros::screen::fill_rect(button.x + radius, button.y, button.x + button.width - radius, button.y + button.height); // vertical center strip
    pros::screen::fill_rect(button.x, button.y + radius, button.x + button.width, button.y + button.height - radius); // horizontal center strip
    pros::screen::fill_circle(button.x + radius,                button.y + radius,                radius); // top-left corner circle
    pros::screen::fill_circle(button.x + button.width - radius, button.y + radius,                radius); // top-right corner circle
    pros::screen::fill_circle(button.x + radius,                button.y + button.height - radius, radius); // bottom-left corner circle
    pros::screen::fill_circle(button.x + button.width - radius, button.y + button.height - radius, radius); // bottom-right corner circle

    pros::screen::set_pen(button.textColor); // switch to the button's text color for printing

    int lineOffset = button.height < 25 ? 0 : (button.height / 2) - 8; // vertical offset to center text inside the button
    pros::screen::print(
        button.height < 25 ? pros::E_TEXT_SMALL : pros::E_TEXT_MEDIUM, // small buttons use small text, larger buttons use medium text
        (button.y + lineOffset) / 20 + 1,  // convert pixel Y to the screen's line number (screen uses 20px line height)
        button.text.c_str()                 // the button's label string
    );
}

static void drawBackground() {
    pros::screen::set_pen(pros::c::COLOR_LIGHT_GRAY);
    pros::screen::fill_rect(0, 0, 480, 35); // draw the top status bar across the full 480px screen width, 35px tall

    pros::screen::set_pen(pros::c::COLOR_BLACK);

    std::string batteryText = "Batt%: " + std::to_string((int)pros::battery::get_capacity()); // read battery % and format as a string
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, batteryText.c_str()); // print battery % on line 1 of the top bar

    Button controllerButton = {250, 2, 170, 28, controllerEnabled ? "Controller: ON" : "Controller: OFF"};
    drawButton(controllerButton); // draws the controller enable/disable toggle button in the top bar

    if (currentPage != 0) {           // only show the BACK button when we're not on the home page
        Button backButton = {425, 2, 50, 28, "BACK"};
        drawButton(backButton);        // draws the BACK button in the top-right corner of the status bar
    }
}

static void drawHomePage(const HomeButtons& btns) {
    // draws all 6 navigation buttons on the home screen
    drawButton(btns.autons);   // "Autons" button
    drawButton(btns.motors);   // "Motors" button
    drawButton(btns.controls); // "Controls" button
    drawButton(btns.sensors);  // "Sensors" button
    drawButton(btns.tunePID);  // "Tune PID" button
    drawButton(btns.alliance); // "Alliance" toggle button
}

static void drawPath(int startingPos, int offsetX, int offsetY) {
    if (startingPos < 0 || startingPos >= autonPaths.size()) return; // safety check: don't draw if index is out of range
    auto& path = autonPaths[startingPos]; // get the waypoint list for the selected auton
    if (path.size() < 2) return; // need at least 2 points to draw a line

    pros::screen::set_pen(pros::c::COLOR_WHITE);
    for (size_t i = 1; i < path.size(); i++) { // draw a line between each consecutive pair of waypoints
        pros::screen::draw_line(
            offsetX + (int)path[i - 1][0], offsetY + (int)path[i - 1][1], // start point of this segment
            offsetX + (int)path[i][0],     offsetY + (int)path[i][1]       // end point of this segment
        );
    }

    // animate a red dot moving along the path
    int seg = (int)pathProgress;          // which segment (pair of waypoints) the dot is currently on
    double t = pathProgress - seg;        // how far along that segment (0.0 = start, 1.0 = end)
    seg = seg % (path.size() - 1);        // wrap segment index so it loops back to the beginning

    double x = path[seg][0] + (path[seg + 1][0] - path[seg][0]) * t; // interpolated X position of the dot
    double y = path[seg][1] + (path[seg + 1][1] - path[seg][1]) * t; // interpolated Y position of the dot

    pros::screen::set_pen(pros::c::COLOR_RED);
    pros::screen::fill_circle(offsetX + (int)x, offsetY + (int)y, 3); // draw the red dot at the interpolated position

    pathProgress += 0.02; // advance the animation counter each frame
    if (pathProgress >= path.size() - 1) {
        pathProgress = 0; // reset when the dot reaches the end of the path: loops the animation
    }
}

static void drawAutonPage() {
    pros::screen::set_pen(pros::c::COLOR_DARK_GRAY);
    pros::screen::fill_rect(20, 50, 140, 170); // draw the mini field preview rectangle (dark gray background)

    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, 3, "FIELD"); // label the field preview

    drawPath(currentStartingPos, 20, 50); // draw the animated path for whichever auton is currently selected

    for (int i = 0; i < 8; i++) { // draw all 8 auton selector buttons
        drawButton(autonSelectorButtons[i]);

        if (currentStartingPos == i) { // if this is the currently selected auton, draw a checkmark next to it
            pros::screen::set_pen(pros::c::COLOR_WHITE);
            int checkY = autonSelectorButtons[i].y / 20 + 1; // convert Y pixel to screen line number
            pros::screen::print(pros::E_TEXT_SMALL, checkY, "✓"); // print checkmark on the same line as the button
        }
    }
}

static double getMotorTemperature(int index) {
    // returns the temperature in °C for a motor by its list index
    switch (index) {
        case 0: return something1.get_temperature();  // placeholder motor 1
        case 1: return something2.get_temperature();  // placeholder motor 2
        case 2: return something3.get_temperature();  // placeholder motor 3
        case 3: return something4.get_temperature();  // placeholder motor 4
        case 4: return leftMotor1.get_temperature();  // front-left drive motor
        case 5: return leftMotor2.get_temperature();  // mid-left drive motor
        case 6: return leftMotor3.get_temperature();  // rear-left drive motor
        case 7: return rightMotor1.get_temperature(); // front-right drive motor
        case 8: return rightMotor2.get_temperature(); // mid-right drive motor
        case 9: return rightMotor3.get_temperature(); // rear-right drive motor
    }
    return 0; // unknown index: return 0
}

static uint32_t getTemperatureColor(double temp) {
    if (temp < 35) return pros::c::COLOR_GREEN;  // cool: safe operating temp
    if (temp < 50) return pros::c::COLOR_YELLOW; // warm: getting close to thermal throttle
    return pros::c::COLOR_RED;                   // hot: motor is overheating, reduce use
}

static void applySelectedMotorVoltage() {
    // sends the current motorVoltage value to whichever motor or group is selected
    switch (selectedMotorIndex) {
        case 0:  something1.move(motorVoltage);       break; // placeholder motor 1
        case 1:  something2.move(motorVoltage);       break; // placeholder motor 2
        case 2:  something3.move(motorVoltage);       break; // placeholder motor 3
        case 3:  something4.move(motorVoltage);       break; // placeholder motor 4
        case 4:  leftMotor1.move(motorVoltage);       break; // front-left drive
        case 5:  leftMotor2.move(motorVoltage);       break; // mid-left drive
        case 6:  leftMotor3.move(motorVoltage);       break; // rear-left drive
        case 7:  rightMotor1.move(motorVoltage);      break; // front-right drive
        case 8:  rightMotor2.move(motorVoltage);      break; // mid-right drive
        case 9:  rightMotor3.move(motorVoltage);      break; // rear-right drive
        case 10: driveLeftMotors.move(motorVoltage);  break; // entire left side motor group
        case 11: driveRightMotors.move(motorVoltage); break; // entire right side motor group
        case 12: fullDrive.move(motorVoltage);        break; // all 6 drive motors at once
    }
}

static void drawMotorsPage() {
    pros::screen::set_pen(pros::c::COLOR_BLACK);

    int x = 10, y = 45; // top-left starting position for the motor button list

    // build the list of motor/group buttons for the left panel
    std::vector<Button> motorButtons = {
        {x,y+0,  220,15,"Motor 1"},    // placeholder motor 1
        {x,y+15, 220,15,"Motor 2"},    // placeholder motor 2
        {x,y+30, 220,15,"Motor 3"},    // placeholder motor 3
        {x,y+45, 220,15,"Motor 4"},    // placeholder motor 4
        {x,y+70, 220,15,"Left 1"},     // front-left drive (note the 10px gap separating mechanisms from drive)
        {x,y+85, 220,15,"Left 2"},     // mid-left drive
        {x,y+100,220,15,"Left 3"},     // rear-left drive
        {x,y+125,220,15,"Right 1"},    // front-right drive (another 10px gap)
        {x,y+140,220,15,"Right 2"},    // mid-right drive
        {x,y+155,220,15,"Right 3"},    // rear-right drive
        {x,y+180,220,15,"Left Side"},  // left motor group (controls all 3 left motors at once)
        {x,y+195,220,15,"Right Side"}, // right motor group
        {x,y+210,220,15,"Drivetrain"}, // full drive group (all 6 motors)
        {x,y+235,220,15,"Piston 1"},   // pneumatic solenoid 1
        {x,y+250,220,15,"Piston 2"},   // pneumatic solenoid 2
        {x,y+265,220,15,"Piston 3"},   // pneumatic solenoid 3
    };

    for (size_t i = 0; i < motorButtons.size(); i++) {
        drawButton(motorButtons[i]); // draw each motor button

        if ((int)i == selectedMotorIndex) { // highlight the currently selected motor with a green outline
            pros::screen::set_pen(pros::c::COLOR_GREEN);
            pros::screen::draw_rect(
                motorButtons[i].x - 2, motorButtons[i].y - 2,
                motorButtons[i].x + motorButtons[i].width + 2,
                motorButtons[i].y + motorButtons[i].height + 2
            ); // draw a 2px border around the selected motor button
        }
    }

    // draw the right-side control panel background
    pros::screen::set_pen(pros::c::COLOR_DARK_GRAY);
    pros::screen::fill_rect(240, 40, 470, 230); // dark gray panel on the right half of the screen

    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "CONTROL PANEL"); // header for the right panel

    // control panel buttons
    Button minus  = {250, 60, 40, 30, "-"};                                           // decrease voltage by 5
    Button plus   = {420, 60, 40, 30, "+"};                                           // increase voltage by 5
    Button reset  = {320, 40, 70, 20, "RESET"};                                       // reset voltage to 0
    Button brake  = {250, 105, 150, 25, motorBrake ? "BRAKE: ON" : "BRAKE: OFF"};    // toggle brake mode
    Button keep   = {250, 200, 150, 25, keepMotorChanges ? "KEEP ✓" : "KEEP"};       // toggle whether changes persist

    drawButton(minus);
    drawButton(plus);
    drawButton(reset);
    drawButton(brake);
    drawButton(keep);

    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, ("Voltage: " + std::to_string(motorVoltage)).c_str()); // show current voltage value

    if (selectedMotorIndex >= 0 && selectedMotorIndex <= 9) { // only show temperature for individual motors (not groups)
        double temp = getMotorTemperature(selectedMotorIndex);
        pros::screen::set_pen(getTemperatureColor(temp)); // color-code the temp (green/yellow/red)
        pros::screen::print(pros::E_TEXT_MEDIUM, 9, ("Temp: " + std::to_string(temp)).c_str()); // print the temperature
    }
}

static void drawSensorsPage() {
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    auto pose = chassis.getPose(); // get current odometry position from LemLib

    // print all sensor values on the left half of the screen
    pros::screen::print(pros::E_TEXT_MEDIUM, 3,  ("IMU: "         + std::to_string(imu.get_rotation())).c_str());              // IMU cumulative rotation in degrees
    pros::screen::print(pros::E_TEXT_MEDIUM, 4,  ("Vert Rot: "    + std::to_string(verticalRotation.get_position())).c_str());  // vertical tracking wheel encoder position
    pros::screen::print(pros::E_TEXT_MEDIUM, 5,  ("Horiz Rot: "   + std::to_string(horizontalRotation.get_position())).c_str());// horizontal tracking wheel encoder position
    pros::screen::print(pros::E_TEXT_MEDIUM, 6,  ("Lift Sensor: " + std::to_string(liftSensor.get_position())).c_str());        // lift encoder position
    pros::screen::print(pros::E_TEXT_MEDIUM, 7,  ("Dist1: "       + std::to_string(distanceSensor1.get())).c_str());            // distance sensor 1 reading in mm
    pros::screen::print(pros::E_TEXT_MEDIUM, 8,  ("Dist2: "       + std::to_string(distanceSensor2.get())).c_str());            // distance sensor 2
    pros::screen::print(pros::E_TEXT_MEDIUM, 9,  ("Dist3: "       + std::to_string(distanceSensor3.get())).c_str());            // distance sensor 3
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, ("Dist4: "       + std::to_string(distanceSensor4.get())).c_str());            // distance sensor 4
    pros::screen::print(pros::E_TEXT_MEDIUM, 11, ("Dist5: "       + std::to_string(distanceSensor5.get())).c_str());            // distance sensor 5
    pros::screen::print(pros::E_TEXT_MEDIUM, 13, ("Opt1 Hue: "    + std::to_string(opticalSensor1.get_hue())).c_str());         // optical sensor 1 detected hue (0–360°)
    pros::screen::print(pros::E_TEXT_MEDIUM, 14, ("Opt2 Hue: "    + std::to_string(opticalSensor2.get_hue())).c_str());         // optical sensor 2 detected hue

    // NOTE: these three lines overwrite lines 3 to 5 with pose data, prints on top of the sensor values on the right half
    pros::screen::print(pros::E_TEXT_MEDIUM, 3,  ("X: "     + std::to_string(pose.x)).c_str());     // robot X position in inches
    pros::screen::print(pros::E_TEXT_MEDIUM, 4,  ("Y: "     + std::to_string(pose.y)).c_str());     // robot Y position in inches
    pros::screen::print(pros::E_TEXT_MEDIUM, 5,  ("Theta: " + std::to_string(pose.theta)).c_str()); // robot heading in degrees

    // reset buttons on the right side
    Button imuReset   = {320, 50,  120, 20, "Reset IMU"};   // resets IMU heading and rotation to 0
    Button vertReset  = {320, 80,  120, 20, "Reset Vert"};  // resets vertical tracking wheel encoder
    Button horizReset = {320, 110, 120, 20, "Reset Horiz"}; // resets horizontal tracking wheel encoder
    Button liftReset  = {320, 140, 120, 20, "Reset Lift"};  // resets lift encoder
    drawButton(imuReset);
    drawButton(vertReset);
    drawButton(horizReset);
    drawButton(liftReset);
}

static void drawAutonPopup() {
    pros::screen::set_pen(pros::c::COLOR_LIGHT_GRAY);
    pros::screen::fill_rect(90, 70, 390, 180); // draw a gray popup box in the center of the screen

    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::print(pros::E_TEXT_LARGE, 4, "ARE YOU SURE?"); // confirmation message in large text

    Button yes = {140, 120, 100, 40, "YES"}; // confirm button: sets currentStartingPos to pendingAutonIndex
    Button no  = {260, 120, 100, 40, "NO"};  // cancel button: dismisses popup without changing selection
    drawButton(yes);
    drawButton(no);
}

static void processTouches(const pros::screen_touch_status_s_t& touch, const HomeButtons& homeBtns) {

    bool isNewClick = (touch.touch_status == pros::E_TOUCH_PRESSED) && !wasScreenTouched;
    // only fire once per press, detects rising edge (was not touched last frame, is touched this frame)
    wasScreenTouched = (touch.touch_status == pros::E_TOUCH_PRESSED); // update state for next frame
    if (!isNewClick) return; // if this isn't a new press, ignore everything below

    // ---- Auton confirmation popup ----
    if (showingAutonPopup) {
        if (touch.x > 140 && touch.x < 240 && touch.y > 120 && touch.y < 160) {
            // user tapped YES - apply the pending auton selection
            currentStartingPos = pendingAutonIndex;
            showingAutonPopup = false;
            pathProgress = 0; // reset path animation so it starts from the beginning
        }
        if (touch.x > 260 && touch.x < 360 && touch.y > 120 && touch.y < 160) {
            // user tapped NO - cancel, close popup without changing selection
            showingAutonPopup = false;
            pendingAutonIndex = -1;
        }
        return; // block all other touch handling while popup is showing
    }

    // ---- Controller enable/disable toggle (top bar) ----
    if (touch.x >= 250 && touch.x <= 420 && touch.y >= 2 && touch.y <= 30) {
        controllerEnabled = !controllerEnabled; // flip the flag, main opcontrol() checks this before reading input
        return;
    }

    // ---- BACK button (top bar, only visible when not on home page) ----
    if (currentPage != 0 && touch.x >= 425 && touch.x <= 475 && touch.y >= 2 && touch.y <= 30) {
        if (currentPage == 5) { // leaving the Tune PID page, check if we should restore original values
            if (!keepLateralPID) { // restore lateral PID if user didn't click KEEP
                lateralSettings.kP = originalLateralkP;
                lateralSettings.kI = originalLateralkI;
                lateralSettings.kD = originalLateralkD;
                lateralSettings.slew = originalLateralSlew;
            }
            if (!keepAngularPID) { // restore angular PID if user didn't click KEEP
                angularSettings.kP = originalAngularkP;
                angularSettings.kI = originalAngularkI;
                angularSettings.kD = originalAngularkD;
                angularSettings.slew = originalAngularSlew;
            }
        }
        currentPage = 0; // go back to home page
        return;
    }

    // ---- Home page navigation ----
    if (currentPage == 0) {
        if (const_cast<HomeButtons&>(homeBtns).autons.isPressed(touch.x, touch.y))   currentPage = 1; // go to auton selector
        if (const_cast<HomeButtons&>(homeBtns).motors.isPressed(touch.x, touch.y))   currentPage = 2; // go to motors page
        if (const_cast<HomeButtons&>(homeBtns).controls.isPressed(touch.x, touch.y)) currentPage = 3; // go to controls reference
        if (const_cast<HomeButtons&>(homeBtns).sensors.isPressed(touch.x, touch.y))  currentPage = 4; // go to sensors page
        if (const_cast<HomeButtons&>(homeBtns).tunePID.isPressed(touch.x, touch.y)) { // go to PID tuning page
            currentPage = 5;
            // snapshot current PID values so we can restore them if user exits without saving
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
            allianceColor = (allianceColor == "RED") ? "BLUE" : "RED"; // toggle alliance between RED and BLUE
        }
    }

    // ---- Auton selector page ----
    else if (currentPage == 1) {
        for (int i = 0; i < 8; i++) {
            if (autonSelectorButtons[i].isPressed(touch.x, touch.y)) {
                pendingAutonIndex = i;       // store which auton was tapped
                showingAutonPopup = true;    // show the "ARE YOU SURE?" popup
                popupStartTime = pros::millis(); // record when popup appeared for auto-dismiss after 5 seconds
            }
        }
    }

    // ---- Motors page ----
    else if (currentPage == 2) {
        if (touch.x >= 10 && touch.x <= 230) { // tap is in the left motor list panel
            int relativeY = touch.y - 45; // convert touch Y to position relative to the first motor button

            // map Y position ranges to motor indices, each button is 15px tall, with gaps between groups
            if      (relativeY >= 0   && relativeY < 15)  selectedMotorIndex = 0;  // Motor 1
            else if (relativeY >= 15  && relativeY < 30)  selectedMotorIndex = 1;  // Motor 2
            else if (relativeY >= 30  && relativeY < 45)  selectedMotorIndex = 2;  // Motor 3
            else if (relativeY >= 45  && relativeY < 60)  selectedMotorIndex = 3;  // Motor 4
            else if (relativeY >= 70  && relativeY < 85)  selectedMotorIndex = 4;  // Left 1
            else if (relativeY >= 85  && relativeY < 100) selectedMotorIndex = 5;  // Left 2
            else if (relativeY >= 100 && relativeY < 115) selectedMotorIndex = 6;  // Left 3
            else if (relativeY >= 125 && relativeY < 140) selectedMotorIndex = 7;  // Right 1
            else if (relativeY >= 140 && relativeY < 155) selectedMotorIndex = 8;  // Right 2
            else if (relativeY >= 155 && relativeY < 170) selectedMotorIndex = 9;  // Right 3
            else if (relativeY >= 180 && relativeY < 195) selectedMotorIndex = 10; // Left Side group
            else if (relativeY >= 195 && relativeY < 210) selectedMotorIndex = 11; // Right Side group
            else if (relativeY >= 210 && relativeY < 225) selectedMotorIndex = 12; // Full Drivetrain group
        }

        if (touch.x > 250 && touch.x < 290 && touch.y > 60 && touch.y < 90) { // tapped the "-" button
            motorVoltage = std::max(-127, motorVoltage - 5); // decrease voltage by 5, clamped to -127
            applySelectedMotorVoltage(); // send new voltage immediately
        }
        if (touch.x > 420 && touch.x < 460 && touch.y > 60 && touch.y < 90) { // tapped the "+" button
            motorVoltage = std::min(127, motorVoltage + 5); // increase voltage by 5, clamped to +127
            applySelectedMotorVoltage();
        }
        if (touch.x > 320 && touch.x < 390 && touch.y > 40 && touch.y < 60) { // tapped RESET
            motorVoltage = 0;
            applySelectedMotorVoltage(); // immediately stop the motor
        }
        if (touch.x > 250 && touch.x < 400 && touch.y > 105 && touch.y < 130) { // tapped BRAKE toggle
            motorBrake = !motorBrake; // flip brake mode flag (actual brake mode application not shown here)
        }
        if (touch.x > 250 && touch.x < 400 && touch.y > 200 && touch.y < 225) { // tapped KEEP toggle
            keepMotorChanges = !keepMotorChanges; // toggle whether voltage persists on page exit
        }
    }

    // ---- PID Tuning page ----
    else if (currentPage == 5) {
        // RUN buttons, test lateral or angular PID with a 24" straight move
        if (touch.x > 10  && touch.x < 80  && touch.y > 50 && touch.y < 75) { // lateral RUN
            chassis.setPose(0, 0, 0, false);                                    // reset pose to origin
            chassis.moveToPoint(24, 0, 3000, {.maxSpeed = 127}, false);        // move 24" forward
        }
        if (touch.x > 250 && touch.x < 320 && touch.y > 50 && touch.y < 75) { // angular RUN (duplicate: likely meant to do a turn)
            chassis.setPose(0, 0, 0, false);
            chassis.moveToPoint(24, 0, 3000, {.maxSpeed = 127}, false);
        }

        // LOG buttons, print current PID values to the screen for debugging
        if (touch.x > 90 && touch.x < 180 && touch.y > 50 && touch.y < 75) { // log lateral PID
            pros::screen::print(pros::E_TEXT_SMALL, 15, ("Lateral | Lift: " + std::to_string(liftSensor.get_position()) + " | kP:" + std::to_string(lateralSettings.kP) + " | kI:" + std::to_string(lateralSettings.kI) + " | kD:" + std::to_string(lateralSettings.kD) + " | Slew:" + std::to_string(lateralSettings.slew)).c_str());
        }
        if (touch.x > 330 && touch.x < 420 && touch.y > 50 && touch.y < 75) { // log angular PID
            pros::screen::print(pros::E_TEXT_SMALL, 16, ("Angular | Lift: " + std::to_string(liftSensor.get_position()) + " | kP:" + std::to_string(angularSettings.kP) + " | kI:" + std::to_string(angularSettings.kI) + " | kD:" + std::to_string(angularSettings.kD) + " | Slew:" + std::to_string(angularSettings.slew)).c_str());
        }

        // RESET buttons, restore original PID values
        if (touch.x > 10  && touch.x < 90  && touch.y > 80 && touch.y < 100) { // reset lateral PID
            lateralSettings.kP   = originalLateralkP;
            lateralSettings.kI   = originalLateralkI;
            lateralSettings.kD   = originalLateralkD;
            lateralSettings.slew = originalLateralSlew;
        }
        if (touch.x > 250 && touch.x < 330 && touch.y > 80 && touch.y < 100) { // reset angular PID
            angularSettings.kP   = originalAngularkP;
            angularSettings.kI   = originalAngularkI;
            angularSettings.kD   = originalAngularkD;
            angularSettings.slew = originalAngularSlew;
        }

        // KEEP toggles, whether to save PID changes when leaving the page
        if (touch.x > 100 && touch.x < 200 && touch.y > 80 && touch.y < 100) keepLateralPID = !keepLateralPID; // toggle lateral keep
        if (touch.x > 340 && touch.x < 440 && touch.y > 80 && touch.y < 100) keepAngularPID = !keepAngularPID; // toggle angular keep

        // Lateral PID +/- buttons
        if (touch.x > 160 && touch.x < 185 && touch.y > 120 && touch.y < 140) lateralSettings.kP += 0.1;                         // kP +
        if (touch.x > 10  && touch.x < 35  && touch.y > 120 && touch.y < 140) lateralSettings.kP = std::max(0.0, lateralSettings.kP - 0.1); // kP -
        if (touch.x > 160 && touch.x < 185 && touch.y > 160 && touch.y < 180) lateralSettings.kI += 0.01;                        // kI +
        if (touch.x > 10  && touch.x < 35  && touch.y > 160 && touch.y < 180) lateralSettings.kI = std::max(0.0, lateralSettings.kI - 0.01); // kI -
        if (touch.x > 160 && touch.x < 185 && touch.y > 200 && touch.y < 220) lateralSettings.kD += 0.1;                         // kD +
        if (touch.x > 10  && touch.x < 35  && touch.y > 200 && touch.y < 220) lateralSettings.kD = std::max(0.0, lateralSettings.kD - 0.1); // kD -
        if (touch.x > 160 && touch.x < 185 && touch.y > 240 && touch.y < 260) lateralSettings.slew += 1;                         // slew +
        if (touch.x > 10  && touch.x < 35  && touch.y > 240 && touch.y < 260) lateralSettings.slew = std::max(0.0, lateralSettings.slew - 1.0); // slew -

        // Angular PID +/- buttons (same layout, right half of screen)
        if (touch.x > 400 && touch.x < 425 && touch.y > 120 && touch.y < 140) angularSettings.kP += 0.1;
        if (touch.x > 250 && touch.x < 275 && touch.y > 120 && touch.y < 140) angularSettings.kP = std::max(0.0, angularSettings.kP - 0.1);
        if (touch.x > 400 && touch.x < 425 && touch.y > 160 && touch.y < 180) angularSettings.kI += 0.01;
        if (touch.x > 250 && touch.x < 275 && touch.y > 160 && touch.y < 180) angularSettings.kI = std::max(0.0, angularSettings.kI - 0.01);
        if (touch.x > 400 && touch.x < 425 && touch.y > 200 && touch.y < 220) angularSettings.kD += 0.1;
        if (touch.x > 250 && touch.x < 275 && touch.y > 200 && touch.y < 220) angularSettings.kD = std::max(0.0, angularSettings.kD - 0.1);
        if (touch.x > 400 && touch.x < 425 && touch.y > 240 && touch.y < 260) angularSettings.slew += 1;
        if (touch.x > 250 && touch.x < 275 && touch.y > 240 && touch.y < 260) angularSettings.slew = std::max(0.0, angularSettings.slew - 1.0);
    }

    // ---- Sensors page reset buttons ----
    else if (currentPage == 4) {
        if (touch.x > 320 && touch.x < 440 && touch.y > 50  && touch.y < 70)  imu.reset();                // reset IMU heading and rotation
        if (touch.x > 320 && touch.x < 440 && touch.y > 80  && touch.y < 100) verticalRotation.reset();   // reset vertical tracking wheel
        if (touch.x > 320 && touch.x < 440 && touch.y > 110 && touch.y < 130) horizontalRotation.reset(); // reset horizontal tracking wheel
        if (touch.x > 320 && touch.x < 440 && touch.y > 140 && touch.y < 160) liftSensor.reset();         // reset lift encoder
    }
}

static void drawControlsPage() {
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3,  "DRIVER CONTROLS");   // page title
    pros::screen::print(pros::E_TEXT_MEDIUM, 5,  "A:");                // A button label (action not filled in yet)
    pros::screen::print(pros::E_TEXT_MEDIUM, 6,  "B:");                // B button label
    pros::screen::print(pros::E_TEXT_MEDIUM, 7,  "X:");                // X button label
    pros::screen::print(pros::E_TEXT_MEDIUM, 8,  "Y:");                // Y button label (re-run auton)
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, "L1:");               // L1 trigger label
    pros::screen::print(pros::E_TEXT_MEDIUM, 11, "L2:");               // L2 trigger label
    pros::screen::print(pros::E_TEXT_MEDIUM, 12, "R1:");               // R1 trigger label
    pros::screen::print(pros::E_TEXT_MEDIUM, 13, "R2:");               // R2 trigger label
    pros::screen::print(pros::E_TEXT_MEDIUM, 5,  "                         UP:");    // D-pad up label (printed on same line as A, right-side column)
    pros::screen::print(pros::E_TEXT_MEDIUM, 6,  "                       DOWN:");    // D-pad down
    pros::screen::print(pros::E_TEXT_MEDIUM, 7,  "                       LEFT:");    // D-pad left
    pros::screen::print(pros::E_TEXT_MEDIUM, 8,  "                      RIGHT:");    // D-pad right
    pros::screen::print(pros::E_TEXT_MEDIUM, 15, "Left Joystick:");    // left stick function
    pros::screen::print(pros::E_TEXT_MEDIUM, 16, "Right Joystick:");   // right stick function
}

static void drawPIDPage() {
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::draw_line(240, 40, 240, 240); // vertical divider line splitting lateral (left) and angular (right) sections

    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "LATERAL PID");                         // left section header
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "                         ANGULAR PID"); // right section header (padded to appear on the right)

    // lateral control buttons
    Button runLat   = {10,  50, 70,  25, "RUN"};                                      // run a test move with current lateral PID
    Button logLat   = {90,  50, 90,  25, "Log PID"};                                  // print lateral PID values to screen
    Button resetLat = {10,  80, 80,  20, "RESET"};                                    // restore original lateral PID values
    Button keepLat  = {100, 80, 100, 20, keepLateralPID ? "KEEP ✓" : "KEEP PID"};    // toggle whether changes persist on exit
    drawButton(runLat); drawButton(logLat); drawButton(resetLat); drawButton(keepLat);

    // angular control buttons (same layout, right half)
    Button runAng   = {250, 50, 70,  25, "RUN"};
    Button logAng   = {330, 50, 90,  25, "Log PID"};
    Button resetAng = {250, 80, 80,  20, "RESET"};
    Button keepAng  = {340, 80, 100, 20, keepAngularPID ? "KEEP ✓" : "KEEP PID"};
    drawButton(runAng); drawButton(logAng); drawButton(resetAng); drawButton(keepAng);

    // print live lateral PID values
    pros::screen::print(pros::E_TEXT_MEDIUM, 7,  ("kP: "   + std::to_string(lateralSettings.kP)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 9,  ("kI: "   + std::to_string(lateralSettings.kI)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 11, ("kD: "   + std::to_string(lateralSettings.kD)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 13, ("Slew: " + std::to_string(lateralSettings.slew)).c_str());

    // print live angular PID values (padded to appear in the right column)
    pros::screen::print(pros::E_TEXT_MEDIUM, 7,  ("                    kP: "   + std::to_string(angularSettings.kP)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 9,  ("                    kI: "   + std::to_string(angularSettings.kI)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 11, ("                    kD: "   + std::to_string(angularSettings.kD)).c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, 13, ("                    Slew: " + std::to_string(angularSettings.slew)).c_str());

    // +/- buttons for each PID value (lateral on left, angular on right)
    std::vector<Button> pidButtons = {
        {160, 120, 25, 20, "+"}, {10,  120, 25, 20, "-"}, // lateral kP + / -
        {160, 160, 25, 20, "+"}, {10,  160, 25, 20, "-"}, // lateral kI + / -
        {160, 200, 25, 20, "+"}, {10,  200, 25, 20, "-"}, // lateral kD + / -
        {160, 240, 25, 20, "+"}, {10,  240, 25, 20, "-"}, // lateral slew + / -
        {400, 120, 25, 20, "+"}, {250, 120, 25, 20, "-"}, // angular kP + / -
        {400, 160, 25, 20, "+"}, {250, 160, 25, 20, "-"}, // angular kI + / -
        {400, 200, 25, 20, "+"}, {250, 200, 25, 20, "-"}, // angular kD + / -
        {400, 240, 25, 20, "+"}, {250, 240, 25, 20, "-"}, // angular slew + / -
    };
    for (auto& btn : pidButtons) drawButton(btn); // draw all +/- buttons
}

void updateUI() {
    pros::screen::set_eraser(pros::c::COLOR_WHITE);
    pros::screen::erase(); // clear the entire screen to white before redrawing everything

    drawBackground(); // draw the top status bar (battery %, controller toggle, back button)

    HomeButtons homeBtns = getHomeButtons(); // build the home page buttons with current state (alliance color, etc.)

    // draw whichever page is currently active
    if      (currentPage == 0) drawHomePage(homeBtns);  // home: navigation buttons
    else if (currentPage == 1) drawAutonPage();          // auton selector with field preview
    else if (currentPage == 2) drawMotorsPage();         // motor test panel
    else if (currentPage == 3) drawControlsPage();       // driver controls reference
    else if (currentPage == 4) drawSensorsPage();        // sensor readouts + reset buttons
    else if (currentPage == 5) drawPIDPage();            // live PID tuning

    if (showingAutonPopup) {
        if (pros::millis() - popupStartTime > 5000) {   // if popup has been showing for more than 5 seconds
            showingAutonPopup = false;                   // auto-dismiss it
            pendingAutonIndex = -1;                      // clear the pending selection
        } else {
            drawAutonPopup();                            // draw the "ARE YOU SURE?" popup on top of everything
        }
    }

    processTouches(pros::screen::touch_status(), homeBtns); // handle any touch input for this frame
}