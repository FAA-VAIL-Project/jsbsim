#include "JSB_Interface.h"

/* functions */
/* ****************************************** */

void sim_nsleep(long millisec)
{
#ifdef __linux__
  sleep(millisec);
#elif _WIN32
  Sleep((DWORD)(millisec));
#endif
}

// using this when limiting control surface commands to keep polarity
int isNeg(double num)
{
  if (num >= 0)
  {
    return 1;
  }
  else
  {
    return -1;
  }
}

bool sameSign(float n1, float n2)
{
  return n1 * n2 > (float)0.0;
}

bool sameSign(double n1, double n2)
{
  return n1 * n2 > (double)0.0;
}

// Sets winds for simulation
bool set_winds(vector<vector<double>> &windVals, vector<int> &windTiming, double simTime)
{
  try
  {
    double windTimingVal = windTiming[windState]; // trying to do this to catch vector out of range exception - not working
    if ((simTime > windTimingVal) && windState < (int)(windTiming.size() - 1))
    {
      windState++;
    }
  }
  catch (const std::exception &e)
  {
    std::cout << "error occurred: " << e.what() << std::endl;
    return 0;
  }

  FDM.SetPropertyValue("atmosphere/wind-north-fps", (windVals[windState][0]) * 1 / FPS2KNOT); // convert knot input to fps since jsbsim only takes fps for wind
  FDM.SetPropertyValue("atmosphere/wind-east-fps", (windVals[windState][1]) * 1 / FPS2KNOT);
  return 1;
}

// sets value for given tuning command variable - function varies value over time
bool set_tuning_cmd(vector<double> &cmdVals, vector<int> &cmdTiming, double simTime, double &tuningVar)
{
  try
  {
    double cmdTimingVal = cmdTiming[cmdState]; // trying to do this to catch vector out of range exception - not working
    if ((simTime > cmdTimingVal) && cmdState < (int)(cmdTiming.size() - 1))
    {
      cmdState++;
    }
  }
  catch (const std::exception &e)
  {
    std::cout << "error occurred: " << e.what() << std::endl;
    return 0;
  }
  tuningVar = cmdVals[cmdState];
  return 1;
}

/* autopilot */
/* ****************************************** */

void AutoPilot::writeManeuverConfigFromPython(py::object pyObj_maneuverConfigPayload)
{
  std::string maneuverConfigPayload = pyObj_maneuverConfigPayload.cast<std::string>();
  writeManeuverConfig(maneuverConfigPayload);
  return;
}

void AutoPilot::writeManeuverConfig(std::string maneuverConfigPayload)
{
  maneuverConfig = json::parse(maneuverConfigPayload);
  return;
}

/** @brief Run the Autopilot from Python
 *
 *
 * @param[in] None
 * @return
 */
void AutoPilot::runFromPython(py::object pyObj_iterations)
{
  int iterations = pyObj_iterations.cast<int>();
  run(iterations);
  return;
}

/* changed this method from its previous title "newMain" */

/** @brief Main entry to run the AutoPilot
 * This method was changed from its previous title of "newMain"
 *
 * @param[in] None
 * @return JSONPaylod JSON payload of the updated aircraft state
 */
void AutoPilot::run(int iterations = 5)
{
  for (int iter = 0; iter < iterations; iter++)
  {
    // run FDM in AP mode

    if (realtime)
    {
      sim_nsleep(sleep_nseconds); // Allows visualization in FG to run in realtime.
    }
    simTime = FDM.GetSimTime();
    globalAircraftState.simT = simTime;
    // winds removed for now, will need to add in new functions on the AutoPilot.cpp file for python manipulation
    /*
    tryWinds = set_winds(FDM, windVals, windTiming, simTime); //Sets wind values of FDM
    tryCmdTune = set_tuning_cmd(FDM, cmdVals, cmdTiming, simTime, phiCmdTuning); //updates command for specified controller  **MODIFY TUNING VARIABLE HERE**
    if ((tryWinds == 0) || (tryCmdTune == 0))
    {
        return -1; //catches if vectors are not properly sized for set_winds() or set_tuning_cmd()
    }
    */

    globalAircraftState.update_readonly_properties(); // Updates elements of AircraftState instance.

    // waypoint follow
    if (globalAircraftState.apMode == maneuverConfig.at(IDLE).get<int>())
    {
      mission = true;
      crsCmd = globalAircraftState.chi; // hold current course
      phiCmd = crsController.calculate(crsCmd, globalAircraftState.chi, globalAircraftState.simDt, false);
      gamCmd = 0.0;
    }
    else if (globalAircraftState.apMode == maneuverConfig.at(LEFT_MANEUVER).get<int>())
    {
      mission = false;
      phiCmd = -35.0;
      gamCmd = 0;
    }
    else if (globalAircraftState.apMode == maneuverConfig.at(RIGHT_MANEUVER).get<int>())
    {
      mission = false;
      phiCmd = 35.0;
      gamCmd = 0;
    }
    else if (globalAircraftState.apMode == maneuverConfig.at(STRAIGHT_MANEUVER).get<int>())
    {
      mission = false;
      phiCmd = 0.0;
      gamCmd = 0;
    }

    betaCmd = 0.0;
    deltRCmd = betaController.calculate(betaCmd, globalAircraftState.beta, globalAircraftState.simDt, false);

    set_throttle_pos();

    deltACmd = phiController.calculate(phiCmd, globalAircraftState.phiB, globalAircraftState.simDt, false);

    alphaCmd = get_alpha_cmd(gamCmd, mission, gammaSwitch);
    deltECmd = alphaController.calculate(alphaCmd, globalAircraftState.alpha, globalAircraftState.simDt, false);

    alphaController.overSpd = G_limit();
    if (globalAircraftState.vA < globalAircraftState.vc) // if KCAS greater than maneuvering speed, limit control surface command
    {
      // double test = globalAircraftState.vc - globalAircraftState.vA; unused
      double clampG = abs((-0.000082 * pow((globalAircraftState.vc - globalAircraftState.vA), 2)) + (0.01071 * (globalAircraftState.vc - globalAircraftState.vA)) - 0.35); // double clampG = abs(0.133267 * log(1200 * (globalAircraftState.vc - globalAircraftState.vA) + 20) - 1.51);
      deltECmd = isNeg(deltECmd) * std::min(clampG, abs(deltECmd));
    }

    //****STORING EXTERNAL VALUES IN IC REGISTERS - NEED TO CHANGE THIS WHEN WE QUIT INSTANTIATING NEW JSBSIM EXEC EACH RUN!!****

    FDM.SetPropertyValue("fcs/rudder-cmd-norm", deltRCmd);
    FDM.SetPropertyValue("fcs/aileron-cmd-norm", deltACmd);
    FDM.SetPropertyValue("fcs/elevator-cmd-norm", deltECmd);

    // double kpoWatch = bestRateControllerAlpha.kpO; unused

    assert(FDM.Run());
    if (simStep > 6400)
    {
      globalAircraftState.isSteady = true;
    }

    simStep++;
  }
}

double AutoPilot::get_alpha_cmd(double gamCmd, bool mission, bool gammaSwitch)
{
  double alphaCmd;

  if (gammaSwitch || mission)
  {
    alphaCmd = gammaController.calculate(gamCmd, globalAircraftState.gamma, globalAircraftState.simDt, false);
  }
  else if (!gammaSwitch)
  {
    alphaCmd = bestRateControllerAlpha.calculate(C172_BEST_RATE, globalAircraftState.vc, globalAircraftState.simDt, true);
  }

  if (globalAircraftState.vc < (C172_BEST_RATE - 20) && !gammaSwitch) // going from best rate to gamma controller
  {
    gammaSwitch = true;
    // gammaController.integ = blend_alpha_cmd(bestRateControllerAlpha, gammaController, alphaCmd, 0);
  }
  else if (globalAircraftState.vc > C172_BEST_RATE && gammaSwitch) // going from gamma to best rate controller
  {
    gammaSwitch = false;
    bestRateControllerAlpha.integ = blend_alpha_cmd(bestRateControllerAlpha, gammaController, globalAircraftState.apMode, 1);
  }
  return alphaCmd;
}

void AutoPilot::initializeFDMFromPython()
{
  initializeFDM();
  return;
}

void AutoPilot::initializeFDM()
{

  // assign the proper working path to the FDM
  FDM.SetRootDir(SGPath::fromUtf8(tulsa::dirPath.path));
  FDM.SetAircraftPath(SGPath::fromUtf8(tulsa::dirPath.path + "/jsbsim/aircraft/"));
  FDM.SetEnginePath(SGPath::fromUtf8(tulsa::dirPath.path + "/jsbsim/engine/"));
  FDM.SetSystemsPath(SGPath::fromUtf8(tulsa::dirPath.path + "/jsbsim/systems/"));

  // load the model, now that the correct path has been established
  assert(FDM.LoadModel("c172x"));
  configure_jsbsim();

  //================ Set ICs =====================

  FDM.SetPropertyValue("ic/lat-gc-deg", globalAircraftState.initLat);
  FDM.SetPropertyValue("ic/long-gc-deg", globalAircraftState.initLon);
  FDM.SetPropertyValue("ic/h-agl-ft", globalAircraftState.alt);
  FDM.SetPropertyValue("ic/vc-kts", globalAircraftState.vc);
  FDM.SetPropertyValue("ic/phi-deg", globalAircraftState.phiB);
  FDM.SetPropertyValue("ic/theta-deg", globalAircraftState.thetaB);
  FDM.SetPropertyValue("ic/psi-true-deg", globalAircraftState.psiB);
  FDM.SetPropertyValue("flight-path/psi-gt-rad", globalAircraftState.chi * DEG2RAD);
  FDM.SetPropertyValue("ic/p-rad_sec", globalAircraftState.p);
  FDM.SetPropertyValue("ic/q-rad_sec", globalAircraftState.q);
  FDM.SetPropertyValue("ic/r-rad_sec", globalAircraftState.r);
  FDM.SetPropertyValue("ic/alpha-deg", globalAircraftState.alpha);
  FDM.SetPropertyValue("ic/gamma-deg", globalAircraftState.gamma);

  // why are these commented?
  // FDM.SetPropertyValue("ic/u-fps", globalAircraftState.uB);
  // FDM.SetPropertyValue("ic/v-fps", globalAircraftState.vB);
  // FDM.SetPropertyValue("ic/w-fps", globalAircraftState.wB);
  // FDM.SetPropertyValue("ic/beta-deg", globalAircraftState.beta);

  // ------constants----------
  FDM.SetPropertyValue("propulsion/magneto_cmd", MAGNETO_ON_CMD);
  FDM.SetPropertyValue("propulsion/starter_cmd", ENGINE_START_CMD);

  // configure the aircraft's weight distribution
  int initFuel = (FDM.GetPropertyValue("propulsion/tank[0]/contents-lbs") + FDM.GetPropertyValue("propulsion/tank[1]/contents-lbs"));
  int desWeight = globalAircraftState.weight;
  int neededWeight = desWeight - FDM.GetPropertyValue("inertia/weight-lbs") - initFuel; // Weight is not properly set until first RunIC call - fuel is not included in weight calculation here, need to add it in.
  FDM.SetPropertyValue("inertia/pointmass-weight-lbs[0]", neededWeight / 2);
  FDM.SetPropertyValue("inertia/pointmass-weight-lbs[1]", neededWeight / 2);

  // Run initial condtions (usually set in reset file, can also set manually by setting property values)
  assert(FDM.RunIC());

  // Initial run, sets stepResult to true if successful. Main while loop will not run if it remains false.
  assert(FDM.Run());

  simStep = 0; // reset simStep
  globalAircraftState.update_readonly_properties();
  altSetpoint = globalAircraftState.alt;
  simTime = FDM.GetSimTime();
  globalAircraftState.simT = simTime;

  /* CONTROLLER CONFIGURATION */

  betaController.configure(BETA_KP, BETA_KI, BETA_KD, BETA_MIN, BETA_MAX, BETA_INTEGMINMAX); // set up beta controller. Rudder commands are normalized to [-1, 1]. Actual deflection maximums are +- 15 degrees
  crsController.configure(CRS_KP, CRS_KI, CRS_KD, CRS_MIN, CRS_MAX, CRS_INTEGMINMAX);        // LIMITS HERE ARE FOR ROLL ANGLES! Overshoots significantly, but any more kD causes bad oscillations on rudder and ailerons.
  phiController.configure(PHI_KP, PHI_KI, PHI_KD, PHI_MIN, PHI_MAX, PHI_INTEGMINMAX);        // set up roll controller //Aileron commands are normalized to [-1, 1]. Actual deflection maximums are [-20, 15] degrees.

  alphaController.configure(ALPHA_KP, ALPHA_KI, ALPHA_KD, ALPHA_MIN, ALPHA_MAX, ALPHA_INTEGMINMAX);
  alphaController.pG = 0.15;
  alphaController.overSpd = G_limit();

  gammaController.configure(GAMMA_KP, GAMMA_KI, GAMMA_KD, GAMMA_MIN, GAMMA_MAX, GAMMA_INTEGMINMAX); // ADD HYSTERESIS FOR GAMMA-BEST RATE CONTROLLER SWITCHING - Switch to gamma when vc < 65, switch back to best rate when vc > 75
  gammaController.init_integrator(false);

  bestRateControllerAlpha.configure(BESTRATEALPHA_KP, BESTRATEALPHA_KI, BESTRATEALPHA_KD, BESTRATEALPHA_MIN, BESTRATEALPHA_MAX, BESTRATEALPHA_INTEGMINMAX); //(0.01, 0.0055, 0.02, 450), (0.01, 0.0055, 0.4 , 450)
  bestRateControllerAlpha.init_integrator(true);

  set_throttle_pos();

  if (globalAircraftState.vc >= C172_BEST_RATE)
  {
    gammaSwitch = false;
  }
  else
  {
    gammaSwitch = true;
  }

  return;
}

py::object AutoPilot::getAircraftStateJSONFromPython()
{
  return py::cast(getAircraftStateJSON());
}

std::string AutoPilot::getAircraftStateJSON()
{
  return globalAircraftState.getJSON();
}

double AutoPilot::blend_alpha_cmd(pid &bestRatePID, pid &gammaPID, double alphaCmd, bool gammaToBR) // Call this right before switch-off - calculate new integral for BR controller
{
  double integral;

  if (gammaToBR && (int(alphaCmd) == maneuverConfig.at(LEFT_MANEUVER).get<int>() || int(alphaCmd) == maneuverConfig.at(RIGHT_MANEUVER).get<int>()))
  {
    integral = 2.8 / bestRatePID.ki;
  }
  else if (gammaToBR && (int(alphaCmd) == maneuverConfig.at(STRAIGHT_MANEUVER).get<int>()))
  {
    integral = 2.05 / bestRatePID.ki; // for straight at 75 knots only
  }
  else
  {
    double integral = (bestRatePID.integ - integral) / 2;
  }

  return integral;
}

void AutoPilot::set_throttle_pos()
{
  int maxRateSpeed = C172_BEST_RATE; // knots
  // int maxAirspeed = 125;                           // knots usused
  int throttleOffset = 15; // knots above bestRate where the throttle will go to 100%
  int throttleHyster = 5;  // To go 100%, vc <= bestRate + throttleOffset - throttleHyster. For 0%, vc >= bestRate + throttleOffset + throttleHyster

  bool throttleOn = 0;

  if ((globalAircraftState.vc <= maxRateSpeed + throttleOffset - throttleHyster) && !throttleOn)
  {
    throttleOn = 1;
  }
  else if ((globalAircraftState.vc >= maxRateSpeed + throttleOffset + throttleHyster) && throttleOn)
  {
    throttleOn = 0;
  }
  FDM.SetPropertyValue("fcs/throttle-cmd-norm", throttleOn);
}

void AutoPilot::configure_jsbsim() // Loads scripts/aircraft/reset files (if any passed)
{
  if (!ScriptName.isNull())
  {
    FDM.LoadScript(ScriptName); // retrieved by parsing options in options()
  }
  else if (!AircraftName.empty())
  {
    FDM.LoadModel(SGPath("aircraft"), SGPath("engine"), SGPath("systems"), AircraftName);

    if (!ResetName.isNull())
    {
      auto IC = FDM.GetIC(); // GetIC() returns a pointer to the FGInitialConditions object.
      IC->Load(ResetName);
    }
  }
  for (unsigned oDirChar = 0; oDirChar < LogDirectiveName.size(); oDirChar++) // Loads output directive file(s)
  {
    if (!LogDirectiveName[oDirChar].isNull())
    {
      FDM.SetOutputDirectives(LogDirectiveName[oDirChar]);
    }
  }

  frame_duration = FDM.GetDeltaT();
  if (realtime)
  {
    sleep_nseconds = (long)(frame_duration * 1e3);
  }
  else
  {
    sleep_nseconds = (sleep_period)*1e3;
  }
}

double AutoPilot::G_limit()
{
  double overManSpeed = globalAircraftState.vc - globalAircraftState.vA;
  if (overManSpeed > 0)
  {
    return overManSpeed;
  }
  else
  {
    return 0;
  }
}

JSBAircraftState* AutoPilot::getAircraftStatePointerFromPython()
{
  return &globalAircraftState;
}

/* PID */
/* ****************************************** */

void pid::configure(double kp, double ki, double kd, double min, double max, double integMinMax)
{
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->min = min;
  this->max = max;
  this->integMinMax = integMinMax;
}

void pid::init_integrator(bool isBR)
{
  // constants found by finding alpha bias for aircraft at certain weights maintaining 75kts full throttle, then interpalated
  if (isBR && (globalAircraftState.apMode == maneuverConfig.at(LEFT_MANEUVER).get<int>() || globalAircraftState.apMode == maneuverConfig.at(RIGHT_MANEUVER).get<int>()))
  {
    this->integ = ((0.00224 * globalAircraftState.weight) - 1.783) / this->ki;
  }
  else if (isBR && globalAircraftState.apMode == maneuverConfig.at(STRAIGHT_MANEUVER).get<int>())
  {
    this->integ = ((0.00193 * globalAircraftState.weight - 1.942)) / this->ki;
  }
  else
  {
    this->integ = globalAircraftState.initAlpha / this->ki;
  }
}

// Perform pid calculations here
double pid::calculate(double cmdVal, double currVal, double dt, bool isBR)
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

/* AircraftState */
/* ****************************************** */

void JSBAircraftState::get_inertial_vector(double (&lat)[2], double (&lon)[2], double (&alt)[2], vector<double> &vectorFromOrigin)                                  // takes in lat, lon, alt and returns...
{                                                                                                                                                                   //...[2dEucFromOrigin, 3dEucFromOrigin, psiFromOrigin, thetaFromOrigin, pn, pe, pd]
  double yDist = (cos(lat[1] * DEG2RAD) * sin((lon[1] - lon[0]) * DEG2RAD)) * EARADNMI;                                                                             // cos and sin in math.h take in argument as radians
  double xDist = ((cos(lat[0] * DEG2RAD) * sin(lat[1] * DEG2RAD)) - (sin(lat[0] * DEG2RAD) * cos(lat[1] * DEG2RAD) * cos((lon[1] - lon[0]) * DEG2RAD))) * EARADNMI; // xDist and yDist calculated using Haversine formula.
  double hDist = (alt[0] - alt[1]);                                                                                                                                 // lat/lon/alt[1] is current lat/lon/alt. lat/lon/alt[0] is lat/lon/alt of origin.

  double flatEucDist = sqrt(pow(yDist, 2) + pow(xDist, 2));
  double eucDist = sqrt(pow(yDist, 2) + pow(xDist, 2) + pow(hDist * FOOT2NMI, 2));
  double psiFromOrigin = atan2(yDist, xDist);         // Heading of aircraft in relation to the origin (inertial frame)
  double thetaFromOrigin = atan2(hDist, flatEucDist); // pitch of aircraft in relation to origin (inertial frame)

  vectorFromOrigin = {flatEucDist, eucDist, thetaFromOrigin, psiFromOrigin, xDist, yDist, hDist}; // Was originally going to pass this back, but now it just updates the vector in the ...
}

void JSBAircraftState::update_readonly_properties()
{
  // make sure we're referencing the globalAircraftState
  assert(this == &globalAircraftState);

  if (simStep == 0)
  {
    initLat = FDM.GetPropertyValue("position/lat-gc-deg");
    initLon = FDM.GetPropertyValue("position/long-gc-deg");
    initAlt = FDM.GetPropertyValue("position/h-sl-ft");

    latPairs[0] = initLat;
    lonPairs[0] = initLon;
    altPairs[0] = initAlt;

    initAlpha = FDM.GetPropertyValue("aero/alpha-deg"); // trimmed alpha

    simDt = FDM.GetDeltaT();
    bestRate = 75;
  }

  latPairs[1] = FDM.GetPropertyValue("position/lat-gc-deg");
  lonPairs[1] = FDM.GetPropertyValue("position/long-gc-deg");
  altPairs[1] = FDM.GetPropertyValue("position/h-sl-ft");

  lat = FDM.GetPropertyValue("position/lat-gc-deg");
  lon = FDM.GetPropertyValue("position/long-gc-deg");
  alt = FDM.GetPropertyValue("position/h-sl-ft");

  get_inertial_vector(latPairs, lonPairs, altPairs, inertialVector);

  pn = inertialVector[4]; // euclidean distances from the origin (origin is starting point of aircraft)
  pe = inertialVector[5];
  pd = inertialVector[6];
  horDist = pow((pow(pn, 2) + pow(pe, 2)), 0.5);

  thetaI = inertialVector[2] * RAD2DEG; // theta and psi in intertial frame
  psiI = inertialVector[3] * RAD2DEG;

  phiB = FDM.GetPropertyValue("attitude/phi-rad") * RAD2DEG; // phi, theta, chi, and psi in body frame
  thetaB = FDM.GetPropertyValue("attitude/theta-rad") * RAD2DEG;
  psiB = FDM.GetPropertyValue("attitude/psi-rad") * RAD2DEG; // unwrap(FDM.GetPropertyValue("attitude/psi-rad") * RAD2DEG);
  chi = unwrap(FDM.GetPropertyValue("flight-path/psi-gt-rad") * RAD2DEG);
  gamma = FDM.GetPropertyValue("flight-path/gamma-rad") * RAD2DEG;

  uB = FDM.GetPropertyValue("velocities/u-fps") * FPS2KNOT; // body velocities
  vB = FDM.GetPropertyValue("velocities/v-fps") * FPS2KNOT;
  wB = FDM.GetPropertyValue("velocities/w-fps") * FPS2KNOT;
  vt_kts = FDM.GetPropertyValue("velocities/vt-fps") * FPS2KNOT;
  vc = FDM.GetPropertyValue("velocities/vc-kts");

  p = FDM.GetPropertyValue("velocities/p-rad_sec") * RAD2DEG;
  q = FDM.GetPropertyValue("velocities/q-rad_sec") * RAD2DEG;
  r = FDM.GetPropertyValue("velocities/r-rad_sec") * RAD2DEG;

  alpha = FDM.GetPropertyValue("aero/alpha-deg");
  beta = FDM.GetPropertyValue("aero/beta-deg");
  nzG = FDM.GetPropertyValue("accelerations/Nz");

  vA = (0.0215 * FDM.GetPropertyValue("inertia/weight-lbs")) + 47.35;

}

double JSBAircraftState::unwrap(double newAng) // PID controller for course freaks out when angle gets wrapped. Need to unwrap angle for course controller ONLY. JSBSim needs it to be wrapped to work properly
{
  if (std::isnan(prevAngleWrap)) // catch first run when prev angle is uninstantiated
  {
    prevAngleWrap = newAng;
  }
  double angDiff = (prevAngleWrap - newAng);
  double wrappedAng = newAng;

  if (angDiff < 0 && abs(angDiff) > 350) // we wrapped from 0 to 360
  {
    tWrapped--;
  }
  else if (angDiff > 0 && abs(angDiff) > 350) // we wrapped from 360 to 0
  {
    tWrapped++;
  }

  if (tWrapped > 0)
  {
    wrappedAng = newAng + (360 * tWrapped);
  }
  else if (tWrapped < 0)
  {
    wrappedAng = -(360 - newAng) + (360 * (tWrapped + 1));
  }
  prevAngleWrap = newAng;

  return wrappedAng;
}

void JSBAircraftState::writeWithJSONFromPython(py::object pyStr_JSONPayload)
{
  // convert the python string into valid json
  std::string JSONPayload = pyStr_JSONPayload.cast<std::string>();
  writeWithJSON(JSONPayload);
  return;
}

/** @brief Accepts a JSON payload of the aircraft state and
 * - updates the JSON object of the aircraftState
 * - updates all members of the aircraftState object
 * - updates all applicable FDM properties
 *
 * @param[in]  JSONPayload JSON Payload of the aircraftState
 * @return  None
 */
void JSBAircraftState::writeWithJSON(std::string JSONPayload)
{

  // overwrite the object's JSON object from the JSON payload
  jsonObject = json::parse(JSONPayload);

  initLat = jsonObject.at("initLat");
  initLon = jsonObject.at("initLon");
  initAlt = jsonObject.at("initAlt");

  /* position */
  lat = jsonObject.at("lat");
  lon = jsonObject.at("lon");
  alt = jsonObject.at("alt");

  /* velocites */
  vc = jsonObject.at("vc");
  uB = jsonObject.at("uB");
  vB = jsonObject.at("vB");
  wB = jsonObject.at("wB");

  /* attitude and orientation */
  phiB = jsonObject.at("phiB");
  thetaB = jsonObject.at("thetaB");
  psiB = jsonObject.at("psiB");

  chi = jsonObject.at("chi");
  alpha = jsonObject.at("alpha");
  beta = jsonObject.at("beta");
  gamma = jsonObject.at("gamma");

  /* orientation rates */
  p = jsonObject.at("p");
  q = jsonObject.at("q");
  r = jsonObject.at("r");

  /* misc */
  weight = jsonObject.at("weight");
  isSteady = jsonObject.at("isSteady");

  nzG = jsonObject.at("nzG");
  vt_kts = jsonObject.at("vt");

  /* overwrite JSB's values */
  FDM.Hold(); // pause the physics engine

  FDM.SetPropertyValue("ic/lat-gc-deg", initLat);
  FDM.SetPropertyValue("ic/long-gc-deg", initLon);
  FDM.SetPropertyValue("ic/h-agl-ft", alt);
  FDM.SetPropertyValue("ic/vc-kts", vc);
  FDM.SetPropertyValue("ic/phi-deg", phiB);
  FDM.SetPropertyValue("ic/theta-deg", thetaB);
  FDM.SetPropertyValue("ic/psi-true-deg", psiB);
  FDM.SetPropertyValue("flight-path/psi-gt-rad", chi * DEG2RAD);
  FDM.SetPropertyValue("ic/p-rad_sec", p);
  FDM.SetPropertyValue("ic/q-rad_sec", q);
  FDM.SetPropertyValue("ic/r-rad_sec", r);
  FDM.SetPropertyValue("ic/alpha-deg", alpha);
  FDM.SetPropertyValue("ic/gamma-deg", gamma);

  FDM.SetPropertyValue("position/lat-gc-deg", lat);
  FDM.SetPropertyValue("position/long-gc-deg", lon);
  FDM.SetPropertyValue("position/h-agl-ft", alt);
  FDM.SetPropertyValue("velocities/vc-kts", vc);
  FDM.SetPropertyValue("velocities/u-fps", uB);
  FDM.SetPropertyValue("velocities/v-fps", vB);
  FDM.SetPropertyValue("velocities/w-fps", wB);
  FDM.SetPropertyValue("attitude/phi-rad", phiB * DEG2RAD);
  FDM.SetPropertyValue("attitude/theta-rad", thetaB * DEG2RAD);
  FDM.SetPropertyValue("attitude/psi-true-rad", psiB * DEG2RAD);
  FDM.SetPropertyValue("aero/alpha-rad", alpha * DEG2RAD);
  FDM.SetPropertyValue("aero/beta-rad", beta * DEG2RAD);
  FDM.SetPropertyValue("flight-path/gamma-rad", gamma * DEG2RAD);
  FDM.SetPropertyValue("velocities/p-rad_sec", p);
  FDM.SetPropertyValue("velocities/q-rad_sec", q);
  FDM.SetPropertyValue("velocities/r-rad_sec", r);
  FDM.SetPropertyValue("inertia/weight-lbs", weight);
  FDM.SetPropertyValue("accelerations/Nz", nzG);
  FDM.SetPropertyValue("velocities/vt-fps", vt_kts * KNOT2FPS);


  /* resume the physics engine */
  FDM.ResetToInitialConditions(MODE_1);

  /* reenable engine if it is disabled */
  if (FDM.GetPropertyValue("propulsion/active_engine") == -1)
  {
    std::cout << "engine is off, turning on\n";
    FDM.SetPropertyValue("propulsion/starter_cmd", ENGINE_START_CMD);
  }

  assert(FDM.RunIC());
  assert(FDM.Run());
  FDM.Resume();
}

py::object JSBAircraftState::getJSONFromPython()
{
  return py::cast<std::string>(this->getJSON());
}

std::string JSBAircraftState::getJSON()
{
  assert(this == &globalAircraftState); // this line is to assure that we're referencing the correct aircraft state, and not an unconstructed object.

  // update the json object and send the string to python

  /* locally scoped variables */
  jsonObject["initLat"] = initLat;
  jsonObject["initLon"] = initLon;
  jsonObject["initAlt"] = initAlt;
  jsonObject["lat"] = lat;
  jsonObject["lon"] = lon;
  jsonObject["alt"] = alt;
  jsonObject["vc"] = vc;
  jsonObject["vt"] = vt_kts;
  jsonObject["phiB"] = phiB;
  jsonObject["thetaB"] = thetaB;
  jsonObject["psiB"] = psiB;
  jsonObject["chi"] = chi;
  jsonObject["uB"] = uB;
  jsonObject["vB"] = vB;
  jsonObject["wB"] = wB;
  jsonObject["p"] = p;
  jsonObject["q"] = q;
  jsonObject["r"] = r;
  jsonObject["alpha"] = alpha;
  jsonObject["beta"] = beta;
  jsonObject["gamma"] = gamma;
  jsonObject["weight"] = weight;
  jsonObject["isSteady"] = isSteady;
  jsonObject["nzG"] = nzG;

  return jsonObject.dump();
}

void JSBAircraftState::setModeFromPython(py::object pyObj_commandedMode)
{
  std::string commandedMode = pyObj_commandedMode.cast<std::string>();
  setMode(commandedMode);
  return;
}

void JSBAircraftState::setMode(std::string commandedMode)
{
  this->apMode = maneuverConfig.at(commandedMode).get<int>();
}

py::object JSBAircraftState::getExtendedJSBSimStateFromPython()
{
  return py::cast<std::string>(getExtendedJSBSimState());
}

std::string JSBAircraftState::getExtendedJSBSimState()
{
  json jsonObject;
  jsonObject["accelerations/Nz"] = FDM.GetPropertyValue("accelerations/Nz");
  jsonObject["accelerations/a-pilot-x-ft_sec2"] = FDM.GetPropertyValue("accelerations/a-pilot-x-ft_sec2");
  jsonObject["accelerations/a-pilot-y-ft_sec2"] = FDM.GetPropertyValue("accelerations/a-pilot-y-ft_sec2");
  jsonObject["accelerations/a-pilot-z-ft_sec2"] = FDM.GetPropertyValue("accelerations/a-pilot-z-ft_sec2");
  jsonObject["accelerations/n-pilot-x-norm"] = FDM.GetPropertyValue("accelerations/n-pilot-x-norm");
  jsonObject["accelerations/n-pilot-y-norm"] = FDM.GetPropertyValue("accelerations/n-pilot-y-norm");
  jsonObject["accelerations/n-pilot-z-norm"] = FDM.GetPropertyValue("accelerations/n-pilot-z-norm");
  jsonObject["accelerations/pdot-rad_sec2"] = FDM.GetPropertyValue("accelerations/pdot-rad_sec2");
  jsonObject["accelerations/qdot-rad_sec2"] = FDM.GetPropertyValue("accelerations/qdot-rad_sec2");
  jsonObject["accelerations/rdot-rad_sec2"] = FDM.GetPropertyValue("accelerations/rdot-rad_sec2");
  jsonObject["accelerations/udot-ft_sec2"] = FDM.GetPropertyValue("accelerations/udot-ft_sec2");
  jsonObject["accelerations/vdot-ft_sec2"] = FDM.GetPropertyValue("accelerations/vdot-ft_sec2");
  jsonObject["accelerations/wdot-ft_sec2"] = FDM.GetPropertyValue("accelerations/wdot-ft_sec2");
  jsonObject["aero/alpha-deg"] = FDM.GetPropertyValue("aero/alpha-deg");
  jsonObject["aero/alpha-max-rad"] = FDM.GetPropertyValue("aero/alpha-max-rad");
  jsonObject["aero/alpha-min-rad"] = FDM.GetPropertyValue("aero/alpha-min-rad");
  jsonObject["aero/alpha-rad"] = FDM.GetPropertyValue("aero/alpha-rad");
  jsonObject["aero/alpha-wing-rad"] = FDM.GetPropertyValue("aero/alpha-wing-rad");
  jsonObject["aero/alphadot-deg_sec"] = FDM.GetPropertyValue("aero/alphadot-deg_sec");
  jsonObject["aero/alphadot-rad_sec"] = FDM.GetPropertyValue("aero/alphadot-rad_sec");
  jsonObject["aero/beta-deg"] = FDM.GetPropertyValue("aero/beta-deg");
  jsonObject["aero/beta-rad"] = FDM.GetPropertyValue("aero/beta-rad");
  jsonObject["aero/betadot-deg_sec"] = FDM.GetPropertyValue("aero/betadot-deg_sec");
  jsonObject["aero/betadot-rad_sec"] = FDM.GetPropertyValue("aero/betadot-rad_sec");
  jsonObject["aero/bi2vel"] = FDM.GetPropertyValue("aero/bi2vel");
  jsonObject["aero/ci2vel"] = FDM.GetPropertyValue("aero/ci2vel");
  jsonObject["aero/cl-squared"] = FDM.GetPropertyValue("aero/cl-squared");
  jsonObject["aero/h_b-cg-ft"] = FDM.GetPropertyValue("aero/h_b-cg-ft");
  jsonObject["aero/h_b-mac-ft"] = FDM.GetPropertyValue("aero/h_b-mac-ft");
  jsonObject["aero/mag-beta-deg"] = FDM.GetPropertyValue("aero/mag-beta-deg");
  jsonObject["aero/mag-beta-rad"] = FDM.GetPropertyValue("aero/mag-beta-rad");
  jsonObject["aero/qbar-area"] = FDM.GetPropertyValue("aero/qbar-area");
  jsonObject["aero/qbar-psf"] = FDM.GetPropertyValue("aero/qbar-psf");
  jsonObject["aero/qbarUV-psf"] = FDM.GetPropertyValue("aero/qbarUV-psf");
  jsonObject["aero/qbarUW-psf"] = FDM.GetPropertyValue("aero/qbarUW-psf");
  jsonObject["aero/stall-hyst-norm"] = FDM.GetPropertyValue("aero/stall-hyst-norm");
  jsonObject["atmosphere/P-psf"] = FDM.GetPropertyValue("atmosphere/P-psf");
  jsonObject["atmosphere/P-sl-psf"] = FDM.GetPropertyValue("atmosphere/P-sl-psf");
  jsonObject["atmosphere/T-R"] = FDM.GetPropertyValue("atmosphere/T-R");
  jsonObject["atmosphere/T-sl-R"] = FDM.GetPropertyValue("atmosphere/T-sl-R");
  jsonObject["atmosphere/T-sl-dev-F"] = FDM.GetPropertyValue("atmosphere/T-sl-dev-F");
  jsonObject["atmosphere/a-fps"] = FDM.GetPropertyValue("atmosphere/a-fps");
  jsonObject["atmosphere/a-ratio"] = FDM.GetPropertyValue("atmosphere/a-ratio");
  jsonObject["atmosphere/a-sl-fps"] = FDM.GetPropertyValue("atmosphere/a-sl-fps");
  jsonObject["atmosphere/crosswind-fps"] = FDM.GetPropertyValue("atmosphere/crosswind-fps");
  jsonObject["atmosphere/delta"] = FDM.GetPropertyValue("atmosphere/delta");
  jsonObject["atmosphere/delta-T"] = FDM.GetPropertyValue("atmosphere/delta-T");
  jsonObject["atmosphere/density-altitude"] = FDM.GetPropertyValue("atmosphere/density-altitude");
  jsonObject["atmosphere/gust-down-fps"] = FDM.GetPropertyValue("atmosphere/gust-down-fps");
  jsonObject["atmosphere/gust-east-fps"] = FDM.GetPropertyValue("atmosphere/gust-east-fps");
  jsonObject["atmosphere/gust-north-fps"] = FDM.GetPropertyValue("atmosphere/gust-north-fps");
  jsonObject["atmosphere/headwind-fps"] = FDM.GetPropertyValue("atmosphere/headwind-fps");
  jsonObject["atmosphere/p-turb-rad_sec"] = FDM.GetPropertyValue("atmosphere/p-turb-rad_sec");
  jsonObject["atmosphere/psiw-rad"] = FDM.GetPropertyValue("atmosphere/psiw-rad");
  jsonObject["atmosphere/q-turb-rad_sec"] = FDM.GetPropertyValue("atmosphere/q-turb-rad_sec");
  jsonObject["atmosphere/r-turb-rad_sec"] = FDM.GetPropertyValue("atmosphere/r-turb-rad_sec");
  jsonObject["atmosphere/rho-sl-slugs_ft3"] = FDM.GetPropertyValue("atmosphere/rho-sl-slugs_ft3");
  jsonObject["atmosphere/rho-slugs_ft3"] = FDM.GetPropertyValue("atmosphere/rho-slugs_ft3");
  jsonObject["atmosphere/sigma"] = FDM.GetPropertyValue("atmosphere/sigma");
  jsonObject["atmosphere/theta"] = FDM.GetPropertyValue("atmosphere/theta");
  jsonObject["atmosphere/turb-gain"] = FDM.GetPropertyValue("atmosphere/turb-gain");
  jsonObject["atmosphere/turb-rate"] = FDM.GetPropertyValue("atmosphere/turb-rate");
  jsonObject["atmosphere/turb-rhythmicity"] = FDM.GetPropertyValue("atmosphere/turb-rhythmicity");
  jsonObject["atmosphere/wind-from-cw"] = FDM.GetPropertyValue("atmosphere/wind-from-cw");
  jsonObject["attitude/heading-true-rad"] = FDM.GetPropertyValue("attitude/heading-true-rad");
  jsonObject["attitude/phi-rad"] = FDM.GetPropertyValue("attitude/phi-rad");
  jsonObject["attitude/pitch-rad"] = FDM.GetPropertyValue("attitude/pitch-rad");
  jsonObject["attitude/psi-rad"] = FDM.GetPropertyValue("attitude/psi-rad");
  jsonObject["attitude/roll-rad"] = FDM.GetPropertyValue("attitude/roll-rad");
  jsonObject["attitude/theta-rad"] = FDM.GetPropertyValue("attitude/theta-rad");
  jsonObject["fcs/aileron-cmd-norm"] = FDM.GetPropertyValue("fcs/aileron-cmd-norm");
  jsonObject["fcs/center-brake-cmd-norm"] = FDM.GetPropertyValue("fcs/center-brake-cmd-norm");
  jsonObject["fcs/elevator-cmd-norm"] = FDM.GetPropertyValue("fcs/elevator-cmd-norm");
  jsonObject["fcs/elevator-pos-deg"] = FDM.GetPropertyValue("fcs/elevator-pos-deg");
  jsonObject["fcs/elevator-pos-norm"] = FDM.GetPropertyValue("fcs/elevator-pos-norm");
  jsonObject["fcs/elevator-pos-rad"] = FDM.GetPropertyValue("fcs/elevator-pos-rad");
  jsonObject["fcs/flap-cmd-norm"] = FDM.GetPropertyValue("fcs/flap-cmd-norm");
  jsonObject["fcs/flap-pos-deg"] = FDM.GetPropertyValue("fcs/flap-pos-deg");
  jsonObject["fcs/flap-pos-norm"] = FDM.GetPropertyValue("fcs/flap-pos-norm");
  jsonObject["fcs/flap-pos-rad"] = FDM.GetPropertyValue("fcs/flap-pos-rad");
  jsonObject["fcs/left-aileron-pos-deg"] = FDM.GetPropertyValue("fcs/left-aileron-pos-deg");
  jsonObject["fcs/left-aileron-pos-norm"] = FDM.GetPropertyValue("fcs/left-aileron-pos-norm");
  jsonObject["fcs/left-aileron-pos-rad"] = FDM.GetPropertyValue("fcs/left-aileron-pos-rad");
  jsonObject["fcs/left-brake-cmd-norm"] = FDM.GetPropertyValue("fcs/left-brake-cmd-norm");
  jsonObject["fcs/mag-elevator-pos-rad"] = FDM.GetPropertyValue("fcs/mag-elevator-pos-rad");
  jsonObject["fcs/mag-left-aileron-pos-rad"] = FDM.GetPropertyValue("fcs/mag-left-aileron-pos-rad");
  jsonObject["fcs/mag-right-aileron-pos-rad"] = FDM.GetPropertyValue("fcs/mag-right-aileron-pos-rad");
  jsonObject["fcs/mag-rudder-pos-rad"] = FDM.GetPropertyValue("fcs/mag-rudder-pos-rad");
  jsonObject["fcs/mag-speedbrake-pos-rad"] = FDM.GetPropertyValue("fcs/mag-speedbrake-pos-rad");
  jsonObject["fcs/mag-spoiler-pos-rad"] = FDM.GetPropertyValue("fcs/mag-spoiler-pos-rad");
  jsonObject["fcs/pitch-trim-cmd-norm"] = FDM.GetPropertyValue("fcs/pitch-trim-cmd-norm");
  jsonObject["fcs/right-aileron-pos-deg"] = FDM.GetPropertyValue("fcs/right-aileron-pos-deg");
  jsonObject["fcs/right-aileron-pos-norm"] = FDM.GetPropertyValue("fcs/right-aileron-pos-norm");
  jsonObject["fcs/right-aileron-pos-rad"] = FDM.GetPropertyValue("fcs/right-aileron-pos-rad");
  jsonObject["fcs/right-brake-cmd-norm"] = FDM.GetPropertyValue("fcs/right-brake-cmd-norm");
  jsonObject["fcs/roll-trim-cmd-norm"] = FDM.GetPropertyValue("fcs/roll-trim-cmd-norm");
  jsonObject["fcs/rudder-cmd-norm"] = FDM.GetPropertyValue("fcs/rudder-cmd-norm");
  jsonObject["fcs/rudder-pos-deg"] = FDM.GetPropertyValue("fcs/rudder-pos-deg");
  jsonObject["fcs/rudder-pos-norm"] = FDM.GetPropertyValue("fcs/rudder-pos-norm");
  jsonObject["fcs/rudder-pos-rad"] = FDM.GetPropertyValue("fcs/rudder-pos-rad");
  jsonObject["fcs/speedbrake-cmd-norm"] = FDM.GetPropertyValue("fcs/speedbrake-cmd-norm");
  jsonObject["fcs/speedbrake-pos-deg"] = FDM.GetPropertyValue("fcs/speedbrake-pos-deg");
  jsonObject["fcs/speedbrake-pos-norm"] = FDM.GetPropertyValue("fcs/speedbrake-pos-norm");
  jsonObject["fcs/speedbrake-pos-rad"] = FDM.GetPropertyValue("fcs/speedbrake-pos-rad");
  jsonObject["fcs/spoiler-cmd-norm"] = FDM.GetPropertyValue("fcs/spoiler-cmd-norm");
  jsonObject["fcs/spoiler-pos-deg"] = FDM.GetPropertyValue("fcs/spoiler-pos-deg");
  jsonObject["fcs/spoiler-pos-norm"] = FDM.GetPropertyValue("fcs/spoiler-pos-norm");
  jsonObject["fcs/spoiler-pos-rad"] = FDM.GetPropertyValue("fcs/spoiler-pos-rad");
  jsonObject["fcs/steer-cmd-norm"] = FDM.GetPropertyValue("fcs/steer-cmd-norm");
  jsonObject["fcs/yaw-trim-cmd-norm"] = FDM.GetPropertyValue("fcs/yaw-trim-cmd-norm");
  jsonObject["flight-path/gamma-rad"] = FDM.GetPropertyValue("flight-path/gamma-rad");
  jsonObject["flight-path/psi-gt-rad"] = FDM.GetPropertyValue("flight-path/psi-gt-rad");
  jsonObject["forces/fbx-aero-lbs"] = FDM.GetPropertyValue("forces/fbx-aero-lbs");
  jsonObject["forces/fbx-gear-lbs"] = FDM.GetPropertyValue("forces/fbx-gear-lbs");
  jsonObject["forces/fbx-prop-lbs"] = FDM.GetPropertyValue("forces/fbx-prop-lbs");
  jsonObject["forces/fbx-total-lbs"] = FDM.GetPropertyValue("forces/fbx-total-lbs");
  jsonObject["forces/fby-aero-lbs"] = FDM.GetPropertyValue("forces/fby-aero-lbs");
  jsonObject["forces/fby-gear-lbs"] = FDM.GetPropertyValue("forces/fby-gear-lbs");
  jsonObject["forces/fby-prop-lbs"] = FDM.GetPropertyValue("forces/fby-prop-lbs");
  jsonObject["forces/fby-total-lbs"] = FDM.GetPropertyValue("forces/fby-total-lbs");
  jsonObject["forces/fbz-aero-lbs"] = FDM.GetPropertyValue("forces/fbz-aero-lbs");
  jsonObject["forces/fbz-gear-lbs"] = FDM.GetPropertyValue("forces/fbz-gear-lbs");
  jsonObject["forces/fbz-prop-lbs"] = FDM.GetPropertyValue("forces/fbz-prop-lbs");
  jsonObject["forces/fbz-total-lbs"] = FDM.GetPropertyValue("forces/fbz-total-lbs");
  jsonObject["forces/fwx-aero-lbs"] = FDM.GetPropertyValue("forces/fwx-aero-lbs");
  jsonObject["forces/fwy-aero-lbs"] = FDM.GetPropertyValue("forces/fwy-aero-lbs");
  jsonObject["forces/fwz-aero-lbs"] = FDM.GetPropertyValue("forces/fwz-aero-lbs");
  jsonObject["forces/hold-down"] = FDM.GetPropertyValue("forces/hold-down");
  jsonObject["forces/lod-norm"] = FDM.GetPropertyValue("forces/lod-norm");
  jsonObject["gear/gear-cmd-norm"] = FDM.GetPropertyValue("gear/gear-cmd-norm");
  jsonObject["gear/gear-pos-norm"] = FDM.GetPropertyValue("gear/gear-pos-norm");
  jsonObject["gear/num-units"] = FDM.GetPropertyValue("gear/num-units");
  jsonObject["ic/alpha-deg"] = FDM.GetPropertyValue("ic/alpha-deg");
  jsonObject["ic/alpha-rad"] = FDM.GetPropertyValue("ic/alpha-rad");
  jsonObject["ic/beta-deg"] = FDM.GetPropertyValue("ic/beta-deg");
  jsonObject["ic/beta-rad"] = FDM.GetPropertyValue("ic/beta-rad");
  jsonObject["ic/gamma-deg"] = FDM.GetPropertyValue("ic/gamma-deg");
  jsonObject["ic/gamma-rad"] = FDM.GetPropertyValue("ic/gamma-rad");
  jsonObject["ic/h-agl-ft"] = FDM.GetPropertyValue("ic/h-agl-ft");
  jsonObject["ic/h-sl-ft"] = FDM.GetPropertyValue("ic/h-sl-ft");
  jsonObject["ic/lat-gc-deg"] = FDM.GetPropertyValue("ic/lat-gc-deg");
  jsonObject["ic/lat-gc-rad"] = FDM.GetPropertyValue("ic/lat-gc-rad");
  jsonObject["ic/long-gc-deg"] = FDM.GetPropertyValue("ic/long-gc-deg");
  jsonObject["ic/long-gc-rad"] = FDM.GetPropertyValue("ic/long-gc-rad");
  jsonObject["ic/mach"] = FDM.GetPropertyValue("ic/mach");
  jsonObject["ic/p-rad_sec"] = FDM.GetPropertyValue("ic/p-rad_sec");
  jsonObject["ic/phi-deg"] = FDM.GetPropertyValue("ic/phi-deg");
  jsonObject["ic/phi-rad"] = FDM.GetPropertyValue("ic/phi-rad");
  jsonObject["ic/psi-true-deg"] = FDM.GetPropertyValue("ic/psi-true-deg");
  jsonObject["ic/psi-true-rad"] = FDM.GetPropertyValue("ic/psi-true-rad");
  jsonObject["ic/q-rad_sec"] = FDM.GetPropertyValue("ic/q-rad_sec");
  jsonObject["ic/r-rad_sec"] = FDM.GetPropertyValue("ic/r-rad_sec");
  jsonObject["ic/roc-fpm"] = FDM.GetPropertyValue("ic/roc-fpm");
  jsonObject["ic/roc-fps"] = FDM.GetPropertyValue("ic/roc-fps");
  jsonObject["ic/sea-level-radius-ft"] = FDM.GetPropertyValue("ic/sea-level-radius-ft");
  jsonObject["ic/terrain-altitude-ft"] = FDM.GetPropertyValue("ic/terrain-altitude-ft");
  jsonObject["ic/theta-deg"] = FDM.GetPropertyValue("ic/theta-deg");
  jsonObject["ic/theta-rad"] = FDM.GetPropertyValue("ic/theta-rad");
  jsonObject["ic/u-fps"] = FDM.GetPropertyValue("ic/u-fps");
  jsonObject["ic/v-fps"] = FDM.GetPropertyValue("ic/v-fps");
  jsonObject["ic/vc-kts"] = FDM.GetPropertyValue("ic/vc-kts");
  jsonObject["ic/vd-fps"] = FDM.GetPropertyValue("ic/vd-fps");
  jsonObject["ic/ve-fps"] = FDM.GetPropertyValue("ic/ve-fps");
  jsonObject["ic/ve-kts"] = FDM.GetPropertyValue("ic/ve-kts");
  jsonObject["ic/vg-fps"] = FDM.GetPropertyValue("ic/vg-fps");
  jsonObject["ic/vg-kts"] = FDM.GetPropertyValue("ic/vg-kts");
  jsonObject["ic/vn-fps"] = FDM.GetPropertyValue("ic/vn-fps");
  jsonObject["ic/vt-fps"] = FDM.GetPropertyValue("ic/vt-fps");
  jsonObject["ic/vt-kts"] = FDM.GetPropertyValue("ic/vt-kts");
  jsonObject["ic/vw-bx-fps"] = FDM.GetPropertyValue("ic/vw-bx-fps");
  jsonObject["ic/vw-by-fps"] = FDM.GetPropertyValue("ic/vw-by-fps");
  jsonObject["ic/vw-bz-fps"] = FDM.GetPropertyValue("ic/vw-bz-fps");
  jsonObject["ic/vw-dir-deg"] = FDM.GetPropertyValue("ic/vw-dir-deg");
  jsonObject["ic/vw-down-fps"] = FDM.GetPropertyValue("ic/vw-down-fps");
  jsonObject["ic/vw-east-fps"] = FDM.GetPropertyValue("ic/vw-east-fps");
  jsonObject["ic/vw-mag-fps"] = FDM.GetPropertyValue("ic/vw-mag-fps");
  jsonObject["ic/vw-north-fps"] = FDM.GetPropertyValue("ic/vw-north-fps");
  jsonObject["ic/w-fps"] = FDM.GetPropertyValue("ic/w-fps");
  jsonObject["inertia/cg-x-in"] = FDM.GetPropertyValue("inertia/cg-x-in");
  jsonObject["inertia/cg-y-in"] = FDM.GetPropertyValue("inertia/cg-y-in");
  jsonObject["inertia/cg-z-in"] = FDM.GetPropertyValue("inertia/cg-z-in");
  jsonObject["inertia/empty-weight-lbs"] = FDM.GetPropertyValue("inertia/empty-weight-lbs");
  jsonObject["inertia/mass-slugs"] = FDM.GetPropertyValue("inertia/mass-slugs");
  jsonObject["inertia/weight-lbs"] = FDM.GetPropertyValue("inertia/weight-lbs");
  jsonObject["metrics/Sh-sqft"] = FDM.GetPropertyValue("metrics/Sh-sqft");
  jsonObject["metrics/Sv-sqft"] = FDM.GetPropertyValue("metrics/Sv-sqft");
  jsonObject["metrics/Sw-sqft"] = FDM.GetPropertyValue("metrics/Sw-sqft");
  jsonObject["metrics/aero-rp-x-in"] = FDM.GetPropertyValue("metrics/aero-rp-x-in");
  jsonObject["metrics/aero-rp-y-in"] = FDM.GetPropertyValue("metrics/aero-rp-y-in");
  jsonObject["metrics/aero-rp-z-in"] = FDM.GetPropertyValue("metrics/aero-rp-z-in");
  jsonObject["metrics/bw-ft"] = FDM.GetPropertyValue("metrics/bw-ft");
  jsonObject["metrics/cbarw-ft"] = FDM.GetPropertyValue("metrics/cbarw-ft");
  jsonObject["metrics/eyepoint-x-in"] = FDM.GetPropertyValue("metrics/eyepoint-x-in");
  jsonObject["metrics/eyepoint-y-in"] = FDM.GetPropertyValue("metrics/eyepoint-y-in");
  jsonObject["metrics/eyepoint-z-in"] = FDM.GetPropertyValue("metrics/eyepoint-z-in");
  jsonObject["metrics/iw-deg"] = FDM.GetPropertyValue("metrics/iw-deg");
  jsonObject["metrics/iw-rad"] = FDM.GetPropertyValue("metrics/iw-rad");
  jsonObject["metrics/lh-ft"] = FDM.GetPropertyValue("metrics/lh-ft");
  jsonObject["metrics/lh-norm"] = FDM.GetPropertyValue("metrics/lh-norm");
  jsonObject["metrics/lv-ft"] = FDM.GetPropertyValue("metrics/lv-ft");
  jsonObject["metrics/lv-norm"] = FDM.GetPropertyValue("metrics/lv-norm");
  jsonObject["metrics/runway-radius"] = FDM.GetPropertyValue("metrics/runway-radius");
  jsonObject["metrics/vbarh-norm"] = FDM.GetPropertyValue("metrics/vbarh-norm");
  jsonObject["metrics/vbarv-norm"] = FDM.GetPropertyValue("metrics/vbarv-norm");
  jsonObject["metrics/visualrefpoint-x-in"] = FDM.GetPropertyValue("metrics/visualrefpoint-x-in");
  jsonObject["metrics/visualrefpoint-y-in"] = FDM.GetPropertyValue("metrics/visualrefpoint-y-in");
  jsonObject["metrics/visualrefpoint-z-in"] = FDM.GetPropertyValue("metrics/visualrefpoint-z-in");
  jsonObject["moments/l-aero-lbsft"] = FDM.GetPropertyValue("moments/l-aero-lbsft");
  jsonObject["moments/l-gear-lbsft"] = FDM.GetPropertyValue("moments/l-gear-lbsft");
  jsonObject["moments/l-prop-lbsft"] = FDM.GetPropertyValue("moments/l-prop-lbsft");
  jsonObject["moments/l-total-lbsft"] = FDM.GetPropertyValue("moments/l-total-lbsft");
  jsonObject["moments/m-aero-lbsft"] = FDM.GetPropertyValue("moments/m-aero-lbsft");
  jsonObject["moments/m-gear-lbsft"] = FDM.GetPropertyValue("moments/m-gear-lbsft");
  jsonObject["moments/m-prop-lbsft"] = FDM.GetPropertyValue("moments/m-prop-lbsft");
  jsonObject["moments/m-total-lbsft"] = FDM.GetPropertyValue("moments/m-total-lbsft");
  jsonObject["moments/n-aero-lbsft"] = FDM.GetPropertyValue("moments/n-aero-lbsft");
  jsonObject["moments/n-gear-lbsft"] = FDM.GetPropertyValue("moments/n-gear-lbsft");
  jsonObject["moments/n-prop-lbsft"] = FDM.GetPropertyValue("moments/n-prop-lbsft");
  jsonObject["moments/n-total-lbsft"] = FDM.GetPropertyValue("moments/n-total-lbsft");
  jsonObject["output-norm"] = FDM.GetPropertyValue("output-norm");
  jsonObject["position/distance-from-start-lat-mt"] = FDM.GetPropertyValue("position/distance-from-start-lat-mt");
  jsonObject["position/distance-from-start-lon-mt"] = FDM.GetPropertyValue("position/distance-from-start-lon-mt");
  jsonObject["position/distance-from-start-mag-mt"] = FDM.GetPropertyValue("position/distance-from-start-mag-mt");
  jsonObject["position/epa-rad"] = FDM.GetPropertyValue("position/epa-rad");
  jsonObject["position/geod-alt-ft"] = FDM.GetPropertyValue("position/geod-alt-ft");
  jsonObject["position/h-agl-ft"] = FDM.GetPropertyValue("position/h-agl-ft");
  jsonObject["position/h-sl-ft"] = FDM.GetPropertyValue("position/h-sl-ft");
  jsonObject["position/h-sl-meters"] = FDM.GetPropertyValue("position/h-sl-meters");
  jsonObject["position/lat-gc-deg"] = FDM.GetPropertyValue("position/lat-gc-deg");
  jsonObject["position/lat-gc-rad"] = FDM.GetPropertyValue("position/lat-gc-rad");
  jsonObject["position/lat-geod-deg"] = FDM.GetPropertyValue("position/lat-geod-deg");
  jsonObject["position/lat-geod-rad"] = FDM.GetPropertyValue("position/lat-geod-rad");
  jsonObject["position/long-gc-deg"] = FDM.GetPropertyValue("position/long-gc-deg");
  jsonObject["position/long-gc-rad"] = FDM.GetPropertyValue("position/long-gc-rad");
  jsonObject["position/radius-to-vehicle-ft"] = FDM.GetPropertyValue("position/radius-to-vehicle-ft");
  jsonObject["position/terrain-elevation-asl-ft"] = FDM.GetPropertyValue("position/terrain-elevation-asl-ft");
  jsonObject["propulsion/active_engine"] = FDM.GetPropertyValue("propulsion/active_engine");
  jsonObject["propulsion/cutoff_cmd"] = FDM.GetPropertyValue("propulsion/cutoff_cmd");
  jsonObject["propulsion/fuel_dump"] = FDM.GetPropertyValue("propulsion/fuel_dump");
  jsonObject["propulsion/magneto_cmd"] = FDM.GetPropertyValue("propulsion/magneto_cmd");
  jsonObject["propulsion/pt-lbs_sqft"] = FDM.GetPropertyValue("propulsion/pt-lbs_sqft");
  jsonObject["propulsion/refuel"] = FDM.GetPropertyValue("propulsion/refuel");
  jsonObject["propulsion/set-running"] = FDM.GetPropertyValue("propulsion/set-running");
  jsonObject["propulsion/starter_cmd"] = FDM.GetPropertyValue("propulsion/starter_cmd");
  jsonObject["propulsion/starter_cmd"] = FDM.GetPropertyValue("propulsion/starter_cmd");
  jsonObject["propulsion/tat-c"] = FDM.GetPropertyValue("propulsion/tat-c");
  jsonObject["propulsion/tat-r"] = FDM.GetPropertyValue("propulsion/tat-r");
  jsonObject["propulsion/total-fuel-lbs"] = FDM.GetPropertyValue("propulsion/total-fuel-lbs");
  jsonObject["sim-time-sec"] = FDM.GetPropertyValue("sim-time-sec");
  jsonObject["simulation/cycle_duration"] = FDM.GetPropertyValue("simulation/cycle_duration");
  jsonObject["simulation/do_simple_trim"] = FDM.GetPropertyValue("simulation/do_simple_trim");
  jsonObject["simulation/do_trim_analysis"] = FDM.GetPropertyValue("simulation/do_trim_analysis");
  jsonObject["simulation/frame_start_time"] = FDM.GetPropertyValue("simulation/frame_start_time");
  jsonObject["simulation/integrator/position/rotational"] = FDM.GetPropertyValue("simulation/integrator/position/rotational");
  jsonObject["simulation/integrator/position/translational"] = FDM.GetPropertyValue("simulation/integrator/position/translational");
  jsonObject["simulation/integrator/rate/rotational"] = FDM.GetPropertyValue("simulation/integrator/rate/rotational");
  jsonObject["simulation/integrator/rate/translational"] = FDM.GetPropertyValue("simulation/integrator/rate/translational");
  jsonObject["simulation/terminate"] = FDM.GetPropertyValue("simulation/terminate");
  jsonObject["simulation/write-state-file"] = FDM.GetPropertyValue("simulation/write-state-file");
  jsonObject["systems/stall-warn-norm"] = FDM.GetPropertyValue("systems/stall-warn-norm");
  jsonObject["velocities/eci-velocity-mag-fps"] = FDM.GetPropertyValue("velocities/eci-velocity-mag-fps");
  jsonObject["velocities/h-dot-fps"] = FDM.GetPropertyValue("velocities/h-dot-fps");
  jsonObject["velocities/mach"] = FDM.GetPropertyValue("velocities/mach");
  jsonObject["velocities/machU"] = FDM.GetPropertyValue("velocities/machU");
  jsonObject["velocities/p-aero-rad_sec"] = FDM.GetPropertyValue("velocities/p-aero-rad_sec");
  jsonObject["velocities/p-rad_sec"] = FDM.GetPropertyValue("velocities/p-rad_sec");
  jsonObject["velocities/phidot-rad_sec"] = FDM.GetPropertyValue("velocities/phidot-rad_sec");
  jsonObject["velocities/psidot-rad_sec"] = FDM.GetPropertyValue("velocities/psidot-rad_sec");
  jsonObject["velocities/q-aero-rad_sec"] = FDM.GetPropertyValue("velocities/q-aero-rad_sec");
  jsonObject["velocities/q-rad_sec"] = FDM.GetPropertyValue("velocities/q-rad_sec");
  jsonObject["velocities/r-aero-rad_sec"] = FDM.GetPropertyValue("velocities/r-aero-rad_sec");
  jsonObject["velocities/r-rad_sec"] = FDM.GetPropertyValue("velocities/r-rad_sec");
  jsonObject["velocities/thetadot-rad_sec"] = FDM.GetPropertyValue("velocities/thetadot-rad_sec");
  jsonObject["velocities/u-aero-fps"] = FDM.GetPropertyValue("velocities/u-aero-fps");
  jsonObject["velocities/u-fps"] = FDM.GetPropertyValue("velocities/u-fps");
  jsonObject["velocities/v-aero-fps"] = FDM.GetPropertyValue("velocities/v-aero-fps");
  jsonObject["velocities/v-down-fps"] = FDM.GetPropertyValue("velocities/v-down-fps");
  jsonObject["velocities/v-east-fps"] = FDM.GetPropertyValue("velocities/v-east-fps");
  jsonObject["velocities/v-fps"] = FDM.GetPropertyValue("velocities/v-fps");
  jsonObject["velocities/v-north-fps"] = FDM.GetPropertyValue("velocities/v-north-fps");
  jsonObject["velocities/vc-fps"] = FDM.GetPropertyValue("velocities/vc-fps");
  jsonObject["velocities/vc-kts"] = FDM.GetPropertyValue("velocities/vc-kts");
  jsonObject["velocities/ve-fps"] = FDM.GetPropertyValue("velocities/ve-fps");
  jsonObject["velocities/ve-kts"] = FDM.GetPropertyValue("velocities/ve-kts");
  jsonObject["velocities/vg-fps"] = FDM.GetPropertyValue("velocities/vg-fps");
  jsonObject["velocities/vt-fps"] = FDM.GetPropertyValue("velocities/vt-fps");
  jsonObject["velocities/w-aero-fps"] = FDM.GetPropertyValue("velocities/w-aero-fps");
  jsonObject["velocities/w-fps"] = FDM.GetPropertyValue("velocities/w-fps");
  
  return  jsonObject.dump();

}

void configureManeuverConfig()
{
  maneuverConfig[IDLE] = -1;
  maneuverConfig[LEFT_MANEUVER] = 0;
  maneuverConfig[RIGHT_MANEUVER] = 1;
  maneuverConfig[STRAIGHT_MANEUVER] = 2;
}

/* executable entry */
int main()
{
  AutoPilot autopilot;
  configureManeuverConfig();

  std::ifstream aircraftStateFile(tulsa::dirPath.path + tulsa::AIRCRAFT_STATE_PATH);
  std::stringstream aircraftStateBuffer;

    if (!aircraftStateFile)
    {
        std::cout << "ERROR: INVALID JSON FILE PATH FOR AIRCRAFT INPUT STATE: " << tulsa::dirPath.path + tulsa::AIRCRAFT_STATE_PATH << std::endl;
        return EXIT_FAILURE;
    }

  aircraftStateBuffer << aircraftStateFile.rdbuf();
  json jsonObject = json::parse(aircraftStateBuffer.str());

  globalAircraftState.writeWithJSON(jsonObject["JSB"].dump());
  autopilot.initializeFDM();

  for (int i = 0; i < 10; i++)
  {
    autopilot.run();
    std::cout << "globalAircraftState.getJSON()\n\n" << globalAircraftState.getJSON() << std::endl;
  }

  std::cout << "JSBSim_Interface.cpp\nmain() complete.\n";

  return EXIT_SUCCESS;
}

/* shared library (python) entry */
PYBIND11_MODULE(libJSB_Interface, m)
{
  /* special structures */

  // autopilot structure
  py::class_<AutoPilot>(m, "AutoPilot")

      /* initialization */
      .def(py::init<>()) // constructor
      .def("getAircraftStatePointerFromPython", &AutoPilot::getAircraftStatePointerFromPython, "Return the pointer to the globalAircraftState to Python.")
      .def("initializeFDMFromPython", &AutoPilot::initializeFDMFromPython, "Start the FDM and the physics engine based on the current aircraft state.")
      .def("writeManeuverConfigFromPython", &AutoPilot::writeManeuverConfigFromPython, "Configure the maneuver configuration from EVAA.")

      /* FDM core run */
      .def("runFromPython", &AutoPilot::runFromPython, "Run n iterations from Python.");

  // AircraftState Structure
  py::class_<JSBAircraftState>(m, "JSBAircraftState")

      /* Python -> JSB */
      .def("writeWithJSONFromPython", &JSBAircraftState::writeWithJSONFromPython, "Pauses the physics engine, overwrites the aircraft State, and resumes the phyiscs engine.")
      .def("setModeFromPython", &JSBAircraftState::setModeFromPython, "Sets the commanded maneuver of the aircrafState (without overwriting any other parameters).")

      /* Python <- JSB */
      .def("getJSONFromPython", &JSBAircraftState::getJSONFromPython, "Returns a JSON Payload of the globalAircraftState")
      .def("getExtendedJSBSimStateFromPython", &JSBAircraftState::getExtendedJSBSimStateFromPython, "Returns a JSON Payload of the entre suite of JSBSim parameters.");
}