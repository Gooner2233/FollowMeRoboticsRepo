#ifndef FOLLOW_ME_COMMON_UTILS_H
#define FOLLOW_ME_COMMON_UTILS_H

#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

#define HIGH 1e6


/**
 * Gets the current time in microseconds.
 */
static inline int getTimeMicro()
{
    auto now = std::chrono::system_clock::now();
    return now.time_since_epoch().count();
}

/**
 * Sleeps for the provided number of seconds.
 */
static inline void sleepFor(const double secs)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(secs * 1000)));
}

/**
 * Normalizes angle to range [-PI, PI].
 * @param  angle Angle to normalize.
 * @return       Normalized angle.
 */
static inline double normalizeAngle(const double angle)
{
    const double result = fmod(angle + M_PI, 2.0 * M_PI);
    if(result <= 0) return result + M_PI;
    return result - M_PI;
}

/**
 * Finds the distance to the nearest obstacle directly in front of the robot.
 */
float findFwdDist(const std::vector<float>& ranges, const std::vector<float>& thetas);

#endif  // FOLLOW_ME_COMMON_UTILS_H
