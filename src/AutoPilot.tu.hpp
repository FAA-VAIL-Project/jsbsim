#ifndef JSB_INTERFACE_H
#define JSB_INTERFACE_H

#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <math.h>
#include <fstream>
#include "pathing.h"
#include <string>
#include <map>

/* JSBSim includes */
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include "json/json.hpp"

/* tu includes */
#include "PID.tu.hpp"
#include "AircraftState.tu.hpp"

#ifdef __linux__
#include <unistd.h> // for real time functionality ( sleep() )
#elif _WIN32
#include <windows.h> // for real time functionality ( Sleep() )
#endif

/* beta controller */
#define BETA_KP -0.1
#define BETA_KI 0.0
#define BETA_KD -0.09
#define BETA_MIN -1.0
#define BETA_MAX 1.0
#define BETA_INTEGMINMAX 20.0

/* crs controller */
#define CRS_KP -2.0
#define CRS_KI -0.5
#define CRS_KD -0.05
#define CRS_MIN -25.0
#define CRS_MAX 25.0
#define CRS_INTEGMINMAX 20.0

/* phi controller */
#define PHI_KP -0.015
#define PHI_KI -0.0045
#define PHI_KD -0.027
#define PHI_MIN -1.0
#define PHI_MAX 1.0
#define PHI_INTEGMINMAX 20.0

/* alpha controller */
#define ALPHA_KP 0.15
#define ALPHA_KI 0.035
#define ALPHA_KD 0.005
#define ALPHA_MIN -1.0
#define ALPHA_MAX 1.0
#define ALPHA_INTEGMINMAX 36.0

/* gamma controller */
#define GAMMA_KP -0.6
#define GAMMA_KI -0.7
#define GAMMA_KD 0.0
#define GAMMA_MIN -2.0
#define GAMMA_MAX 12.0
#define GAMMA_INTEGMINMAX 5.0

/* bestRateAlpha controller */
#define BESTRATEALPHA_KP 0.01
#define BESTRATEALPHA_KI 0.0055
#define BESTRATEALPHA_KD 0.4
#define BESTRATEALPHA_MIN -5.0
#define BESTRATEALPHA_MAX 12.0
#define BESTRATEALPHA_INTEGMINMAX 550.0

/* namespace handling */
namespace py = pybind11;
using json = nlohmann::json;

namespace tulsa
{

/** @brief This class is the autopilot, but it also exposed interface to python
 */
class AutoPilot
{
    private:
    void get_inertial_vector();
    void updateAircraftStateParameters(JSBSim::FGFDMExec*);
    void initializePIDControllers(void);
    double unwrap(double);

    /* control */
    double G_limit(void);
    double get_alpha_cmd(double);
    double blend_alpha_cmd(double, bool);
    void set_throttle_pos(JSBSim::FGFDMExec*);

    AircraftState aircraftstate; // the global aircraft state that is referenced throughout the autopilot.

    bool gammaSwitch;               // gamma switch for best rate or 0 degree flight path angle
    uint16_t simStep;               // Current iteration of the simulation.
    double simTime;                 // simulation time, in seconds.
    float sleep_nseconds;           // real-time sleep option, in seconds.
    bool mission;                   // 
    bool throttleOn;
    double prevAngleWrap;
    double thetaI, psiI, chi, gamma; //"B" suffixes denote body frame, "I" suffixes denote inertial frame. i.e. psiI is the direction the aircraft is in relative to the origin.
    double crsCmd, phiCmd, deltRCmd, alphaCmd, deltACmd, deltECmd, gamCmd, betaCmd;
    double phiB, thetaB, psiB; //"B" suffixes denote body frame, "I" suffixes denote inertial frame. i.e. psiI is the direction the aircraft is in relative to the origin.

    // intertial vector
    std::vector<double> inertialVector; // Vector that holds inertial frame data. Was set up as a vector for passing back to main, but it doesn't get passed anymore. Not really necessary to have it as a vector.
    double pn, pe, pd, horDist;

    /* wrapping */
    int tWrapped;

    /* controllers */
    pid betaController;          // takes in commanded beta, actual beta, and returns a rudder command.
    pid crsController;           // takes in commanded course, actual course, and returns a roll command
    pid phiController;           // takes in commanded roll, actual roll, and returns an aileron deflection angle command
    pid alphaController;         // takes in commanded alpha, actual alpha, and returns an elevator deflection angle command - Q: do we want outer-loop to be alpha control so we can limit AoA? Or just assume we'll never send an over-crit AoA command?
    pid nzGController;           // takes in commanded G (nz in body frame), actual G, and returns an elevator deflection angle command.
    pid gammaController;         // takes in commanded gamma (vertical velocity vector of aircraft), actual gamma, and returns a commanded angle of attack in degrees AND nZ in Gs.
    pid bestRateControllerAlpha; // takes in commanded airspeed (this will be preset), actual airspeed, and returns a commanded angle of attack in degrees.

    // these variables keeps track of the current maneuver modes. (IDLE, STRAIGHT, LEFT, RIGHT).
    std::map<std::string, int> maneuverConfig;
    int currentManeuver;

    public:
    void initialize(JSBSim::FGFDMExec*);
    void run(JSBSim::FGFDMExec*, int);
    void writeManeuverConfig(std::string); // custom configuration from Python
    void writeManeuverConfig(void); // default configuration
    void setCommandedManeuver(int);
    void readAircraftState(json&);
    
};

} // namespace tulsa

#endif // JSB_INTERFACE_H