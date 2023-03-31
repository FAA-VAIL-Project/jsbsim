#ifndef TU_PID_HPP
#define TU_PID_HPP

#include <math.h>
#include "Constants.tu.hpp"
#include "AircraftState.tu.hpp"

namespace tulsa
{

// PID is used to keep track of PID values for each controller
class pid
{
    public:
    double kp, ki, kd;
    double pG = 0;               // Used for G-Limiting, will only be needed for longitudinal controller (at what level of control though?)
    double overSpd = 0;          // Keep track of overspeed ONLY FOR LONGITUDINAL CONTROLLER
    double min, max;             // Saturation values for controller. i.e. saturation for phi controller would be the min/max values we want the aircraft to be able to roll to.
    double integ = 0, deriv = 0; // Reset integrator and deriv values or they'll carry over to the next run
    double integMinMax;
    double error;
    double previousError = std::numeric_limits<double>::quiet_NaN(); // Set the previous error value to NaN so we can reset it in the "PID_calculate" function.
    double kpO = 0, kiO = 0, kdO = 0;
    bool isBestRateController = false;

    void configure(double kp, double ki, double kd, double min, double max, double integMinMax)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->min = min;
        this->max = max;
        this->integMinMax = integMinMax;
    }

    void init_integrator(AircraftState* aircraftstate, std::map<std::string, int>* maneuverConfiguration)
    {
        if(this->isBestRateController)
        {
            // constants found by finding alpha bias for aircraft at certain weights maintaining 75kts full throttle, then interpalated
            if(aircraftstate->apMode == maneuverConfiguration->at(LEFT_MANEUVER) || aircraftstate->apMode == maneuverConfiguration->at(RIGHT_MANEUVER))
            {
                this->integ = ((0.00224 * aircraftstate->weight) - 1.783) / this->ki;
            }
            else if (aircraftstate->apMode == maneuverConfiguration->at(STRAIGHT_MANEUVER))
            {
                this->integ = ((0.00193 * aircraftstate->weight - 1.942)) / this->ki;
            }
        
            return;
        }
        
        // non best-rate controllers
        this->integ = aircraftstate->initAlpha / this->ki;
    }

    // Perform pid calculations here
    double calculate(double cmdVal, double currVal, double dt, bool isBR)
    {
    double error = currVal - cmdVal;
    if (std::isnan(this->previousError))
    {
        this->previousError = error;
    }

    this->kpO = this->kp * error;

    this->integ += error * dt;

    if (this->integ >= this->integMinMax) // handle integrator windup by capping max/min integrator
    {
        this->integ = this->integMinMax;
    }
    else if (this->integ <= (-this->integMinMax))
    {
        this->integ = -this->integMinMax;
    }
    this->kiO = this->ki * this->integ;

    this->deriv = (error - this->previousError) / dt;
    this->kdO = this->kd * this->deriv;

    double totalO = this->kpO + this->kiO + this->kdO;

    if (totalO > this->max) // saturation max
    {
        totalO = this->max;
    }
    else if (totalO < this->min) // saturation min
    {
        totalO = this->min;
    }
    this->previousError = error;
    return totalO;
    }

}; // class pid

} // namespace tulsa

#endif // TU_PID_HPP