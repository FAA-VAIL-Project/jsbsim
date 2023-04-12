#ifndef TU_CONSTANTS_HPP
#define TU_CONSTANTS_HPP

#include <iostream>
#include <string>

/* flight constants */
#define IDLE "Idle"
#define LEFT_MANEUVER "Left"
#define RIGHT_MANEUVER "Right"
#define STRAIGHT_MANEUVER "Straight"

namespace tulsa
{
    /* program constants */
    const int MAGNETO_ON_CMD = 3;
    const int ENGINE_START_CMD = 1;
    const int ENGINE_RUNNING_CMD  = -1; // -1 refers to "All Engines"
    const int ENGINE_CUTOFF_CMD = 0;
    const double C172_BEST_RATE = 75.0;

    /* physical constants */
    const double RAD2DEG = 57.295779;
    const double FPS2KNOT = 0.592484;
    const double KNOT2FPS = 1 / FPS2KNOT;
    const double FOOT2NMI = 0.000165;
    const double EARADNMI = 3443.930886; // Earth radius in nmi
    const double DEG2RAD = 1 / RAD2DEG;

    /* C172 constants https://www.globalair.com/aircraft-for-sale/specifications?specid=131 */
    const double C172_MAX_TAKEOFF_WEIGHT_LBS = 2400.0;
    const double C172_MIN_EMPTY_WEIGHT_LBS = 1454.0;
}

#endif // TU_CONSTANTS_HPP