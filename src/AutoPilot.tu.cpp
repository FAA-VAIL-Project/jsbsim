#include "AutoPilot.tu.hpp"
#include "Constants.tu.hpp"

namespace tulsa
{

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

  void saturateNormalizedControlSurface(double *controlSurfaceCommand)
  {
    if (*controlSurfaceCommand > (double)1.0)
    {
      *controlSurfaceCommand = 1.0;
    }
    else if (*controlSurfaceCommand < (double)0.0)
    {
      *controlSurfaceCommand = 0.0;
    }
  }

  /* changed this method from its previous title "newMain" */

  /** @brief Main entry to run the AutoPilot
   * This method was changed from its previous title of "newMain"
   *
   * @param[in] None
   * @return JSONPaylod JSON payload of the updated aircraft state
   */
  void AutoPilot::run(JSBSim::FGFDMExec *FDM, int numIterations = 5)
  {
    // enable the engines if it off
    reenable_engine(FDM);

    for (int iter = 0; iter < numIterations; iter++)
    {
      simTime = FDM->GetSimTime();
      aircraftstate.simT = simTime;

      updateAircraftStateParameters(FDM); // Updates elements of AircraftState instance.

      // waypoint follow
      if (aircraftstate.apMode == maneuverConfig.at(IDLE))
      {
        mission = true;
        crsCmd = aircraftstate.chi; // hold current course
        phiCmd = crsController.calculate(crsCmd, aircraftstate.chi, aircraftstate.simDt, false);
        gamCmd = 0.0;
      }
      else if (aircraftstate.apMode == maneuverConfig.at(LEFT_MANEUVER))
      {
        mission = false;
        phiCmd = -35.0;
        gamCmd = 0;
      }
      else if (aircraftstate.apMode == maneuverConfig.at(RIGHT_MANEUVER))
      {
        mission = false;
        phiCmd = 35.0;
        gamCmd = 0;
      }
      else if (aircraftstate.apMode == maneuverConfig.at(STRAIGHT_MANEUVER))
      {
        mission = false;
        phiCmd = 0.0;
        gamCmd = 0;
      }

      betaCmd = 0.0;
      deltRCmd = betaController.calculate(betaCmd, aircraftstate.beta, aircraftstate.simDt, false);

      set_throttle_pos(FDM);

      deltACmd = phiController.calculate(phiCmd, aircraftstate.phiB, aircraftstate.simDt, false);

      alphaCmd = get_alpha_cmd(gamCmd);
      deltECmd = alphaController.calculate(alphaCmd, aircraftstate.alpha, aircraftstate.simDt, false);

      alphaController.overSpd = G_limit();
      if (aircraftstate.vA < aircraftstate.vc) // if KCAS greater than maneuvering speed, limit control surface command
      {
        // double test = aircraftstate.vc - aircraftstate.vA; unused
        double clampG = abs((-0.000082 * pow((aircraftstate.vc - aircraftstate.vA), 2)) + (0.01071 * (aircraftstate.vc - aircraftstate.vA)) - 0.35); // double clampG = abs(0.133267 * log(1200 * (aircraftstate.vc - aircraftstate.vA) + 20) - 1.51);
        deltECmd = isNeg(deltECmd) * std::min(clampG, abs(deltECmd));
      }

      FDM->SetPropertyValue("fcs/rudder-cmd-norm", deltRCmd);
      FDM->SetPropertyValue("fcs/aileron-cmd-norm", deltACmd);
      FDM->SetPropertyValue("fcs/elevator-cmd-norm", deltECmd);

      FDM->Run();
      if (simStep > 6400)
      {
        aircraftstate.isSteady = true;
      }

      simStep++;
    }
  }

  double AutoPilot::get_alpha_cmd(double gamCmd)
  {
    double alphaCmd;

    if (gammaSwitch || mission)
    {
      alphaCmd = gammaController.calculate(gamCmd, aircraftstate.gamma, aircraftstate.simDt, false);
    }
    else if (!gammaSwitch)
    {
      alphaCmd = bestRateControllerAlpha.calculate(C172_BEST_RATE, aircraftstate.vc, aircraftstate.simDt, true);
    }

    if (aircraftstate.vc < (C172_BEST_RATE - 20) && !gammaSwitch) // going from best rate to gamma controller
    {
      gammaSwitch = true;
      // gammaController.integ = blend_alpha_cmd(bestRateControllerAlpha, gammaController, alphaCmd, 0);
    }
    else if (aircraftstate.vc > C172_BEST_RATE && gammaSwitch) // going from gamma to best rate controller
    {
      gammaSwitch = false;
      bestRateControllerAlpha.integ = blend_alpha_cmd(alphaCmd, 1);
    }
    return alphaCmd;
  }

  void AutoPilot::initialize(JSBSim::FGFDMExec *FDM)
  {
    writeManeuverConfig();
    initializePIDControllers();
    updateAircraftStateParameters(FDM);
    set_throttle_pos(FDM);

    simStep = 0; // reset simStep
    simTime = FDM->GetSimTime();
    aircraftstate.simT = simTime;

    if (aircraftstate.vc >= C172_BEST_RATE)
    {
      gammaSwitch = false;
    }
    else
    {
      gammaSwitch = true;
    }
    return;
  }

  void AutoPilot::initializePIDControllers()
  {
    betaController.configure(BETA_KP, BETA_KI, BETA_KD, BETA_MIN, BETA_MAX, BETA_INTEGMINMAX); // set up beta controller. Rudder commands are normalized to [-1, 1]. Actual deflection maximums are +- 15 degrees
    crsController.configure(CRS_KP, CRS_KI, CRS_KD, CRS_MIN, CRS_MAX, CRS_INTEGMINMAX);        // LIMITS HERE ARE FOR ROLL ANGLES! Overshoots significantly, but any more kD causes bad oscillations on rudder and ailerons.
    phiController.configure(PHI_KP, PHI_KI, PHI_KD, PHI_MIN, PHI_MAX, PHI_INTEGMINMAX);        // set up roll controller //Aileron commands are normalized to [-1, 1]. Actual deflection maximums are [-20, 15] degrees.

    alphaController.configure(ALPHA_KP, ALPHA_KI, ALPHA_KD, ALPHA_MIN, ALPHA_MAX, ALPHA_INTEGMINMAX);
    alphaController.pG = 0.15;
    alphaController.overSpd = G_limit();

    gammaController.configure(GAMMA_KP, GAMMA_KI, GAMMA_KD, GAMMA_MIN, GAMMA_MAX, GAMMA_INTEGMINMAX); // ADD HYSTERESIS FOR GAMMA-BEST RATE CONTROLLER SWITCHING - Switch to gamma when vc < 65, switch back to best rate when vc > 75
    gammaController.init_integrator(&aircraftstate, &maneuverConfig);

    bestRateControllerAlpha.configure(BESTRATEALPHA_KP, BESTRATEALPHA_KI, BESTRATEALPHA_KD, BESTRATEALPHA_MIN, BESTRATEALPHA_MAX, BESTRATEALPHA_INTEGMINMAX); //(0.01, 0.0055, 0.02, 450), (0.01, 0.0055, 0.4 , 450)
    bestRateControllerAlpha.isBestRateController = true;
    bestRateControllerAlpha.init_integrator(&aircraftstate, &maneuverConfig);
  }

  double AutoPilot::blend_alpha_cmd(double alphaCmd, bool gammaToBR)
  {
    double integral;

    if (gammaToBR && (int(alphaCmd) == maneuverConfig.at(LEFT_MANEUVER) || int(alphaCmd) == maneuverConfig.at(RIGHT_MANEUVER)))
    {
      integral = 2.8 / bestRateControllerAlpha.ki;
    }
    else if (gammaToBR && (int(alphaCmd) == maneuverConfig.at(STRAIGHT_MANEUVER)))
    {
      integral = 2.05 / bestRateControllerAlpha.ki; // for straight at 75 knots only
    }
    else
    {
      double integral = (bestRateControllerAlpha.integ - integral) / 2;
    }

    return integral;
  }

  void AutoPilot::set_throttle_pos(JSBSim::FGFDMExec *FDM)
  {
    int maxRateSpeed = C172_BEST_RATE; // knots
    // int maxAirspeed = 125;                           // knots usused
    int throttleOffset = 15; // knots above bestRate where the throttle will go to 100%
    int throttleHyster = 5;  // To go 100%, vc <= bestRate + throttleOffset - throttleHyster. For 0%, vc >= bestRate + throttleOffset + throttleHyster

    // if the throttle is not on AND we're below best rate, enable the trottle
    if (!throttleOn && aircraftstate.vc <= maxRateSpeed + throttleOffset - throttleHyster)
    {
      throttleOn = true;
      FDM->SetPropertyValue("fcs/throttle-cmd-norm", (double)1.0);
    }
    // if the throttle is on AND we're above best rate, disable the trottle
    else if (throttleOn && aircraftstate.vc >= maxRateSpeed + throttleOffset + throttleHyster)
    {
      throttleOn = false;
      FDM->SetPropertyValue("fcs/throttle-cmd-norm", (double)0.0);
    }

    return;
  }

  double AutoPilot::G_limit()
  {
    double overManSpeed = aircraftstate.vc - aircraftstate.vA;
    if (overManSpeed > 0)
    {
      return overManSpeed;
    }
    else
    {
      return 0;
    }
  }

  /* AircraftState */
  /* ****************************************** */

  void AutoPilot::writeManeuverConfig(void)
  {
    maneuverConfig.emplace(std::make_pair<std::string, int>(std::string(IDLE), -1));
    maneuverConfig.emplace(std::make_pair<std::string, int>(std::string(LEFT_MANEUVER), 0));
    maneuverConfig.emplace(std::make_pair<std::string, int>(std::string(RIGHT_MANEUVER), 1));
    maneuverConfig.emplace(std::make_pair<std::string, int>(std::string(STRAIGHT_MANEUVER), 2));
    return;
  }

  void AutoPilot::writeManeuverConfig(std::string JSONPayload)
  {
    json JSONObject = json::parse(JSONPayload);
    maneuverConfig.emplace(std::make_pair<std::string, int>(std::string(IDLE), JSONObject.at(IDLE).get<int>()));
    maneuverConfig.emplace(std::make_pair<std::string, int>(std::string(LEFT_MANEUVER), JSONObject.at(LEFT_MANEUVER).get<int>()));
    maneuverConfig.emplace(std::make_pair<std::string, int>(std::string(RIGHT_MANEUVER), JSONObject.at(RIGHT_MANEUVER).get<int>()));
    maneuverConfig.emplace(std::make_pair<std::string, int>(std::string(STRAIGHT_MANEUVER), JSONObject.at(STRAIGHT_MANEUVER).get<int>()));
    return;
  }

  void AutoPilot::setCommandedManeuver(int desiredManeuver)
  {
    aircraftstate.apMode = desiredManeuver;
    currentManeuver = desiredManeuver;

    // assign the trim condition
    if (currentManeuver == maneuverConfig[LEFT_MANEUVER] || currentManeuver == maneuverConfig[RIGHT_MANEUVER])
    {
      this->trimCondition = "turn";
    }
    else if (currentManeuver == maneuverConfig[STRAIGHT_MANEUVER])
    {
      this->trimCondition = "pullup";
    }

    return;
  }

  void AutoPilot::readAircraftState(json &JSONObject)
  {
    aircraftstate.populateJSONObject(JSONObject);

    // populate values only internal to AutoPilot class here.

    // attitude commands
    JSONObject["phiCmd"] = phiCmd;
    JSONObject["crsCmd"] = crsCmd;
    JSONObject["alphaCmd"] = alphaCmd;
    JSONObject["betaCmd"] = betaCmd;
    JSONObject["gamCmd"] = gamCmd;

    // control surface commands
    JSONObject["deltRCmd"] = deltRCmd;
    JSONObject["deltACmd"] = deltACmd;
    JSONObject["deltECmd"] = deltECmd;
    
    throttleOn ? JSONObject["throttleOn"] = 1.0 :  JSONObject["throttleOn"] = 0.0;
    return;
  }

  void AutoPilot::updateAircraftStateParameters(JSBSim::FGFDMExec *FDM)
  {
    if (simStep == 0)
    {
      aircraftstate.initLat = FDM->GetPropertyValue("position/lat-gc-deg");
      aircraftstate.initLon = FDM->GetPropertyValue("position/long-gc-deg");
      aircraftstate.initAlt = FDM->GetPropertyValue("position/h-sl-ft");

      aircraftstate.latPairs[0] = aircraftstate.initLat;
      aircraftstate.lonPairs[0] = aircraftstate.initLon;
      aircraftstate.altPairs[0] = aircraftstate.initAlt;

      aircraftstate.initAlpha = FDM->GetPropertyValue("aero/alpha-deg"); // trimmed alpha

      aircraftstate.simDt = FDM->GetDeltaT();
      aircraftstate.bestRate = 75;
    }

    aircraftstate.latPairs[1] = FDM->GetPropertyValue("position/lat-gc-deg");
    aircraftstate.lonPairs[1] = FDM->GetPropertyValue("position/long-gc-deg");
    aircraftstate.altPairs[1] = FDM->GetPropertyValue("position/h-sl-ft");

    aircraftstate.lat = FDM->GetPropertyValue("position/lat-gc-deg");
    aircraftstate.lon = FDM->GetPropertyValue("position/long-gc-deg");
    aircraftstate.alt = FDM->GetPropertyValue("position/h-sl-ft");

    get_inertial_vector();

    pn = inertialVector[4]; // euclidean distances from the origin (origin is starting point of aircraft)
    pe = inertialVector[5];
    pd = inertialVector[6];

    horDist = pow((pow(pn, 2) + pow(pe, 2)), 0.5);
    thetaI = inertialVector[2] * RAD2DEG; // theta and psi in intertial frame
    psiI = inertialVector[3] * RAD2DEG;

    aircraftstate.phiB = FDM->GetPropertyValue("attitude/phi-rad") * RAD2DEG; // phi, theta, chi, and psi in body frame
    aircraftstate.thetaB = FDM->GetPropertyValue("attitude/theta-rad") * RAD2DEG;
    aircraftstate.psiB = FDM->GetPropertyValue("attitude/psi-rad") * RAD2DEG; // unwrap(FDM->GetPropertyValue("attitude/psi-rad") * RAD2DEG);

    aircraftstate.chi = unwrap(FDM->GetPropertyValue("flight-path/psi-gt-rad") * RAD2DEG);
    aircraftstate.gamma = FDM->GetPropertyValue("flight-path/gamma-rad") * RAD2DEG;
    aircraftstate.uB = FDM->GetPropertyValue("velocities/u-fps") * FPS2KNOT; // body velocities
    aircraftstate.vB = FDM->GetPropertyValue("velocities/v-fps") * FPS2KNOT;
    aircraftstate.wB = FDM->GetPropertyValue("velocities/w-fps") * FPS2KNOT;
    aircraftstate.vt_kts = FDM->GetPropertyValue("velocities/vt-fps") * FPS2KNOT;
    aircraftstate.vc = FDM->GetPropertyValue("velocities/vc-kts");

    aircraftstate.p = FDM->GetPropertyValue("velocities/p-rad_sec") * RAD2DEG;
    aircraftstate.q = FDM->GetPropertyValue("velocities/q-rad_sec") * RAD2DEG;
    aircraftstate.r = FDM->GetPropertyValue("velocities/r-rad_sec") * RAD2DEG;
    aircraftstate.alpha = FDM->GetPropertyValue("aero/alpha-deg");
    aircraftstate.beta = FDM->GetPropertyValue("aero/beta-deg");
    aircraftstate.nzG = FDM->GetPropertyValue("accelerations/Nz");
    aircraftstate.weight = FDM->GetPropertyValue("inertia/weight-lbs");

    aircraftstate.deltE = FDM->GetPropertyValue("fcs/elevator-cmd-norm");
    aircraftstate.deltA = FDM->GetPropertyValue("fcs/aileron-cmd-norm");
    aircraftstate.deltR = FDM->GetPropertyValue("fcs/rudder-cmd-norm");

    // no idea what these numbers are...
    aircraftstate.vA = (0.0215 * FDM->GetPropertyValue("inertia/weight-lbs")) + 47.35;

    return;
  }

  double AutoPilot::unwrap(double newAng) // PID controller for course freaks out when angle gets wrapped. Need to unwrap angle for course controller ONLY. JSBSim needs it to be wrapped to work properly
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

  void AutoPilot::get_inertial_vector()
  {                                                                                                                                                                                                                                                                                     //...[2dEucFromOrigin, 3dEucFromOrigin, psiFromOrigin, thetaFromOrigin, pn, pe, pd]
    double yDist = (cos(aircraftstate.latPairs[1] * DEG2RAD) * sin((aircraftstate.lonPairs[1] - aircraftstate.lonPairs[0]) * DEG2RAD)) * EARADNMI;                                                                                                                                      // cos and sin in math.h take in argument as radians
    double xDist = ((cos(aircraftstate.latPairs[0] * DEG2RAD) * sin(aircraftstate.latPairs[1] * DEG2RAD)) - (sin(aircraftstate.latPairs[0] * DEG2RAD) * cos(aircraftstate.latPairs[1] * DEG2RAD) * cos((aircraftstate.lonPairs[1] - aircraftstate.lonPairs[0]) * DEG2RAD))) * EARADNMI; // xDist and yDist calcuaircraftstate.latPairsed using Haversine formula.
    double hDist = (aircraftstate.altPairs[0] - aircraftstate.altPairs[1]);                                                                                                                                                                                                             // aircraftstate.latPairs/aircraftstate.lonPairs/aircraftstate.altPairs[1] is current aircraftstate.latPairs/aircraftstate.lonPairs/aircraftstate.altPairs. aircraftstate.latPairs/aircraftstate.lonPairs/aircraftstate.altPairs[0] is lat/aircraftstate.lonPairs/aircraftstate.altPairs of origin.

    double flatEucDist = sqrt(pow(yDist, 2) + pow(xDist, 2));
    double eucDist = sqrt(pow(yDist, 2) + pow(xDist, 2) + pow(hDist * FOOT2NMI, 2));
    double psiFromOrigin = atan2(yDist, xDist);         // Heading of aircraft in relation to the origin (inertial frame)
    double thetaFromOrigin = atan2(hDist, flatEucDist); // pitch of aircraft in relation to origin (inertial frame)

    inertialVector = {flatEucDist, eucDist, thetaFromOrigin, psiFromOrigin, xDist, yDist, hDist}; // Was originally going to pass this back, but now it just updates the vector in the ...
  }

  void AutoPilot::reenable_engine(JSBSim::FGFDMExec *FDM)
  {
    // reenable the engines if disabled
    if (FDM->GetPropertyValue("propulsion/starter_cmd") != 1)
    {
      FDM->SetPropertyValue("propulsion/starter_cmd", 1);
    }
    if (FDM->GetPropertyValue("propulsion/magneto_cmd") != 3)
    {
      FDM->SetPropertyValue("propulsion/magneto_cmd", 3);
    }
    if (FDM->GetPropertyValue("propulsion/set-running") != -1)
    {
      FDM->SetPropertyValue("propulsion/set-running", -1);
    }
    return;
  }

} // namespace tulsa