/**
 * File: 1D_control.cpp
 *
 * Controls the robot to maintain a given distance to the wall directly in
 * front of it.
 *
 * This code uses Bang-Bang control or P-control to maintain the setpoint
 * distance.
 */

#include <iostream>
#include <cmath>

#include <signal.h>

#include <follow_me/common/utils.h>
#include <mbot_bridge/robot.h>


bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


int main(int argc, const char *argv[])
{
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    // Initialize the robot.
    mbot_bridge::MBot robot;
    // We will store the Lidar scan data in these vectors.
    std::vector<float> ranges;
    std::vector<float> thetas;

    // *** Task 1: Adjust these values appropriately ***

    float setpoint = 0.5;  // The goal distance from the wall in meters
    float kp = 1.0;

    // *** End student code *** //

    while (true) {
        // This function gets the Lidar scan data.
        robot.readLidarScan(ranges, thetas);

        // Get the distance to the wall.
        float dist_to_wall = findFwdDist(ranges, thetas);
        if (dist_to_wall < 0) continue;

        // *** Task 2: Implement the Follow Me controller *** //

        float current_error = setpoint - dist_to_wall;
        float velocity = kp * current_error;
        
        robot.drive(velocity, 0, 0)

        // *** End Student Code *** //

        if (ctrl_c_pressed)
            break;
    }

    // Stop the robot.
    robot.stop();
    return 0;
}
