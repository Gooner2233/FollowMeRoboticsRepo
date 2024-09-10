/**
 * File: 2D_control_nearest.cpp
 *
 * Controls the robot to maintain a given distance to the nearest wall.
 *
 * This code finds the distance to the nearest wall in the Lidar scan. It
 * applies a control to the robot in the direction of the wall using the angle
 * to the scan.
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

/**
 * @brief findMinDist takes an vector of ranges, and returns the
 *        index of the smallest range in the vector
 *
 * @param ranges
 * @return int
 */
int findMinDist(const std::vector<float> &ranges)
{
    // *** Task 2: Implement findMinDist logic *** //
    // TODO: Add logic so that findMinDist returns the correct index.
    // HINT: Remember to ignore any ranges that are zero!

    int indx = -1;
    float minimum_rng = ranges[indx];

    for (int i = 0; i < ranges.size(); ++i)
    {
        if (ranges[i] != 0 && ranges[i] < minimum_rng)
        {
            indx = minimum_rng;
        }
    }

    return indx;

    // *** End Student Code *** //
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

    float setpoint = 0.5; // The goal distance from the wall in meters
    float kp = 1.0;
    // *** End student code *** //

    while (true)
    {
        // This function gets the Lidar scan data.
        robot.readLidarScan(ranges, thetas);

        // Get the distance to the wall.
        float dist_to_wall = findFwdDist(ranges, thetas);
        if (dist_to_wall < 0)
            continue;

        float min_idx = findMinDist(ranges);
        if (findMinDist(ranges) == -1) continue;
        float dist_to_wall = ranges[min_idx];
        float angle_to_wall = thetas[min_idx];

        // *** Task 3: Implement the 2D Follow Me controller ***
        // Hint: Look at your code from follow_1D
        // Hint: When you compute the velocity command, you might find the functions
        // sin(value) and cos(value) helpful!

        float current_error = setpoint - dist_to_wall;
        float velocity = kp * current_error;
        
        float vx = velocity * cos(thetas[min_idx]);
        float vy = velocity * sin(thetas[min_idx]);
        robot.drive(vx, vy, 0);
        // *** End Student Code ***

        if (ctrl_c_pressed)
            break;
    }

    // Stop the robot.
    robot.stop();
    return 0;
}
