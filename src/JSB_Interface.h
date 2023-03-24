#ifndef JSB_INTERFACE_H
#define JSB_INTERFACE_H

#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <math.h>
#include <fstream>
#include "pathing.h"

/* JSBSim includes */
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include "json/json.hpp"

#ifdef __linux__
#include <unistd.h> // for real time functionality ( sleep() )
#elif _WIN32
#include <windows.h> // for real time functionality ( Sleep() )
#endif

/* program constants */
#define MODE_1 1 // TODO figure out what the parameter (int) mode doees for JSB
#define MAGNETO_ON_CMD 3
#define ENGINE_START_CMD 1
#define ENGINE_RUNNING_CMD -1
#define ENGINE_CUTOFF_CMD 0
#define STANDARD_RUN_ITERATIONS 5 // ::run() was previously a static 5 iterations. This is here for executable debugging if necessary.
#define C172_BEST_RATE 75.0

/* physical constants */
#define RAD2DEG 57.295779
#define FPS2KNOT 0.592484
#define KNOT2FPS 1 / FPS2KNOT
#define FOOT2NMI 0.000165
#define EARADNMI 3443.930886 // Earth radius in nmi
#define DEG2RAD 1 / RAD2DEG

/* flight constants */
#define IDLE "Idle"
#define LEFT_MANEUVER "Left"
#define RIGHT_MANEUVER "Right"
#define STRAIGHT_MANEUVER "Straight"

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

/* function prototypes */
void sim_nsleep(long);
int isNeg(double num);
bool set_winds(vector<vector<double>> &windVals, vector<int> &windTiming);
bool set_tuning_cmd(vector<double> &cmdVals, vector<int> &cmdTiming, double &tuningVar);
double unwrap(double);

/* GLOBAL SCOPED VARIABLES */
JSBSim::FGFDMExec FDM;          // Flight Executive. Read more information here. https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGFDMExec.html
// json maneuverConfig;            // JSON object of enumerated maneuvers. This determines key value pairs between -1: "Idle", 0: "Left", etc. JSON because it is also used in python.
std::map<std::string, int> maneuverConfig;
bool gammaSwitch;               // gamma switch for best rate or 0 degree flight path angle
uint16_t simStep;               // Current iteration of the simulation.
double simTime;                 // simulation time, in seconds.
float sleep_nseconds;           // real-time sleep option, in seconds.
bool mission;                   // 
bool realtime;                  // Sets if FDM runs in realtime or not.

/* wrapping */
double prevAngleWrap;
int tWrapped;

/* winds */
int windState;                  // Disturbance winds change as sim time progresses - this keeps track of which wind values should be used
int cmdState;

// Class used to keep track of aircraft state. Member value reset is handled by calling the "update_readonly" function in main after resetting the simulation/
class JSBAircraftState
{
public:
    /* state values */
    double initLat, initLon, initAlt;
    double latPairs[2], lonPairs[2], altPairs[2]; // Pairs are needed to calculate XYZ euclidean distance from origin. First item in each pair is origin lat/lon/alt.
    double crsCmd, phiCmd, deltRCmd, alphaCmd, deltACmd, deltECmd, gamCmd, betaCmd;
    double lat, lon, alt;
    double pn, pe, pd, horDist;
    double phiB, thetaB, psiB, thetaI, psiI, chi, gamma; //"B" suffixes denote body frame, "I" suffixes denote inertial frame. i.e. psiI is the direction the aircraft is in relative to the origin.
    double uB, vB, wB;
    double vc, vt_kts;                               // Body frame velocities
    double p, q, r;
    double alpha, initAlpha, beta;
    double weight;
    double vA, bestRate; // max maneuvering speed, function of weight and best rate of climb speed
    double simDt;
    double simT;
    int apMode;
    bool isSteady = false;
    double nzG;
    int throttleOn = 0;
    
    vector<double> inertialVector; // Vector that holds inertial frame data. Was set up as a vector for passing back to main, but it doesn't get passed anymore. Not really necessary to have it as a vector.

    json jsonObject;

    void setModeFromPython(py::object);
    void setMode(std::string);

	/* Python -> JSB */
    void writeWithJSONFromPython(py::object);
    void writeWithJSON(std::string);

	/* Python <- JSB */
    py::object getJSONFromPython(void);
    std::string getJSON(void);

    // Updates the current state (globalAircraftState) of the aircraft
    void update_readonly_properties(void);

    void get_inertial_vector(double (&lat)[2], double (&lon)[2], double (&alt)[2], vector<double> &vectorFromOrigin);

    py::object getExtendedJSBSimStateFromPython(void);
    std::string getExtendedJSBSimState(void);

};

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

    void configure(double, double, double, double, double, double);
    double calculate(double, double, double, bool);
    void init_integrator(bool);

    // this is literally a constructor
};


/** @brief This class is the autopilot, but it also exposed interface to python
 */
class AutoPilot
{
private:
    /* data */
public:

    /* controllers */
    pid betaController;          // takes in commanded beta, actual beta, and returns a rudder command.
    pid crsController;           // takes in commanded course, actual course, and returns a roll command
    pid phiController;           // takes in commanded roll, actual roll, and returns an aileron deflection angle command
    pid alphaController;         // takes in commanded alpha, actual alpha, and returns an elevator deflection angle command - Q: do we want outer-loop to be alpha control so we can limit AoA? Or just assume we'll never send an over-crit AoA command?
    pid nzGController;           // takes in commanded G (nz in body frame), actual G, and returns an elevator deflection angle command.
    pid gammaController;         // takes in commanded gamma (vertical velocity vector of aircraft), actual gamma, and returns a commanded angle of attack in degrees AND nZ in Gs.
    pid bestRateControllerAlpha; // takes in commanded airspeed (this will be preset), actual airspeed, and returns a commanded angle of attack in degrees.
    pid bestRateControllernZG;   // takes in commanded airspeed (this will be preset), actual airspeed, and returns a commanded nZ in Gs.

    /* flight dynamics model initialization */
    void initializeFDMFromPython();
    void initializeFDM();

    /* maneuver configuration */
    void writeManeuverConfigFromPython(py::object);
    void writeManeuverConfig(std::string);

    /* core */
    void runFromPython(py::object);
    void run(int);

    /* i/o */
    py::object getAircraftStateJSONFromPython(void);
    std::string getAircraftStateJSON(void);

    /* control */
    double G_limit(void);
    double get_alpha_cmd(double);
    double blend_alpha_cmd(double alphaCmd, bool gammaToBR);
    void set_throttle_pos();

    // returns the global aircraft state internal to JSB side
    JSBAircraftState* getAircraftStatePointerFromPython(void);

    void ResetToInitialConditionsFromPython(int);
    
};

JSBAircraftState globalAircraftState; // the global aircraft state for the JSB half of this project.

#endif // JSB_INTERFACE_H