#include "JSBSim_Interface.tu.hpp"

#define MODE_1 1 // TODO figure out what the parameter (int) mode doees for JSB for ResetToInitialConditions()

namespace tulsa
{
    void JSBSim_Interface::initializeJSBSimFromPython(void)
    {
        initializeJSBSim();
    }

    void JSBSim_Interface::initializeJSBSim()
    {
        // assign the proper working path to the FDM
        FDM.SetRootDir(SGPath::fromUtf8(tulsa::dirPath.path));
        FDM.SetAircraftPath(SGPath::fromUtf8(tulsa::dirPath.path + "/tu_jsbsim/aircraft/"));
        FDM.SetEnginePath(SGPath::fromUtf8(tulsa::dirPath.path + "/tu_jsbsim/engine/"));
        FDM.SetSystemsPath(SGPath::fromUtf8(tulsa::dirPath.path + "/tu_jsbsim/systems/"));

        // load the model, now that the correct path has been established
        assert(FDM.LoadModel("c172x"));
        autopilot.initialize(&FDM);

        readAircraftState();

        // configure the aircraft's weight distribution
        int initFuel = (FDM.GetPropertyValue("propulsion/tank[0]/contents-lbs") + FDM.GetPropertyValue("propulsion/tank[1]/contents-lbs"));
        int desiredWeight = aircraftstateJSONObject.at("weight").get<int>();
        assert(aircraftstateJSONObject.at("weight").get<int>() > 1600);
        int neededWeight = desiredWeight - FDM.GetPropertyValue("inertia/weight-lbs") - initFuel; // Weight is not properly set until first RunIC call - fuel is not included in weight calculation here, need to add it in.
        FDM.SetPropertyValue("inertia/pointmass-weight-lbs[0]", neededWeight / 2);
        FDM.SetPropertyValue("inertia/pointmass-weight-lbs[1]", neededWeight / 2);

        if(FDM.GetPropertyValue("propulsion/magneto_cmd") != 1)
        {
            FDM.SetPropertyValue("propulsion/starter_cmd", 1);
        }

        autopilot.reenable_engine(&FDM);
        FDM.RunIC();
        FDM.Run();
        return;
    }

    void JSBSim_Interface::writeManeuverConfigurationFromPython(py::object pyObj_maneuverConfigurationPayload)
    {
        std::string maneuverConfigurationPayload = pyObj_maneuverConfigurationPayload.cast<std::string>();
        writeManeuverConfiguration(maneuverConfigurationPayload);
    }

    void JSBSim_Interface::writeManeuverConfiguration(std::string JSONPayload)
    {
        autopilot.writeManeuverConfig(JSONPayload);
        return;
    }

    void JSBSim_Interface::writeManeuverConfiguration()
    {
        autopilot.writeManeuverConfig();
        return;
    }
    
    void JSBSim_Interface::setCommandedManeuverFromPython(py::object pyObj_desiredManeuver)
    {
        int desiredManeuver = pyObj_desiredManeuver.cast<int>();
        setCommandedManeuver(desiredManeuver);
    }

    void JSBSim_Interface::setCommandedManeuver(int desiredManeuver)
    {
        autopilot.setCommandedManeuver(desiredManeuver);
    }

    void JSBSim_Interface::runAutoPilotFromPython(py::object pyObj_numIterations)
    {
        int numIterations = pyObj_numIterations.cast<int>();
        runAutoPilot(numIterations);
        return;
    }

    void JSBSim_Interface::runAutoPilot(int numIterations)
    {
        autopilot.run(&FDM, numIterations);
        return;
    }

    void JSBSim_Interface::resetAircraftStateFromPython(py::object pyObj_newAircraftStatePayload)
    {
        std::string newAircraftStatePayload = pyObj_newAircraftStatePayload.cast<std::string>();
        resetAircraftState(newAircraftStatePayload);
    }

    void JSBSim_Interface::resetAircraftState(std::string newAircraftStatePayload)
    {
        // overwrite the object's JSON object from the JSON payload
        json JSONObject = json::parse(newAircraftStatePayload);

        /* recreate functionality from FGInitialCondition::Load_v1() */

        std::shared_ptr<JSBSim::FGInitialCondition> fgic = this->FDM.GetIC();

        // TODO write logic to manually set individual parameters
        // if(existsInJSON) { setParameter(jsonObject.at("paramKey")) }
        // JSONObject.contains("key_value") -> true, false

        double true_airspeed = JSONObject.at("vt").get<double>();
        double heading = JSONObject.at("psiB").get<double>();

        /* position */
        fgic->SetLongitudeDegIC(JSONObject.at("lon").get<double>());
        fgic->SetTerrainElevationFtIC((double)0.0);
        fgic->SetAltitudeASLFtIC(JSONObject.at("alt").get<double>());
        fgic->SetLatitudeDegIC(JSONObject.at("lat").get<double>());

        /* orientation */
        JSBSim::FGColumnVector3 vOrient = fgic->GetOrientation().GetEuler();
        vOrient(FDM.ePhi) = JSONObject.at("phiB").get<double>() * DEG2RAD;
        vOrient(FDM.eTht) = JSONObject.at("thetaB").get<double>() * DEG2RAD;
        vOrient(FDM.ePsi) = heading * DEG2RAD;

        // TODO do I need to reset the orientation here?
        // orientation = FGQuaternion(vOrient);

        fgic->SetUBodyFpsIC(JSONObject.at("uB").get<double>());
        fgic->SetVBodyFpsIC(JSONObject.at("vB").get<double>());
        fgic->SetWBodyFpsIC(JSONObject.at("wB").get<double>());

        // TODO evaluate this math
        fgic->SetVNorthFpsIC(true_airspeed * cos(JSONObject.at("chi").get<double>() * DEG2RAD));
        fgic->SetVEastFpsIC(true_airspeed * sin(JSONObject.at("chi").get<double>() * DEG2RAD));
        fgic->SetVDownFpsIC(JSONObject.at("wB").get<double>());

        // fgic->SetVcalibratedKtsIC(JSONObject.at("vc").get<double>());
        fgic->SetVtrueKtsIC(true_airspeed * FPS2KNOT);
        // fgic->SetMachIC(0.0);

        fgic->SetFlightPathAngleDegIC(JSONObject.at("gamma").get<double>());
        // fgic->SetClimbRateFpsIC(JSONObject.at("roc").get<double>());
        fgic->SetVgroundKtsIC(true_airspeed * FPS2KNOT);

        fgic->SetAlphaDegIC(JSONObject.at("alpha").get<double>());
        fgic->SetBetaDegIC(JSONObject.at("beta").get<double>());

        /* winds */
        fgic->SetWindMagKtsIC((double)0.0);
        fgic->SetWindDirDegIC((double)0.0);
        fgic->SetHeadWindKtsIC((double)0.0);
        fgic->SetCrossWindKtsIC((double)0.0);

        // enable the engine
        autopilot.reenable_engine(&FDM);

        // fgic->SetTargetNlfIC();

        fgic->SetTrimRequest(autopilot.trimCondition);

        FDM.ResetToInitialConditions(MODE_1);
        assert(FDM.RunIC());
        runAutoPilot(1);
        
        return;
    }

    py::object JSBSim_Interface::readAircraftStateFromPython(void)
    {
        return py::cast<std::string>(readAircraftState());
    }

    std::string JSBSim_Interface::readAircraftState(void)
    {
        autopilot.readAircraftState(aircraftstateJSONObject);
        return aircraftstateJSONObject.dump();
    }

    py::object JSBSim_Interface::getExtendedJSBSimStateFromPython(void)
    {
        return py::cast<std::string>(getExtendedJSBSimState());
    }

    std::string JSBSim_Interface::getExtendedJSBSimState(void)
    {
        json JSONObject;
        JSONObject["accelerations/Nz"] = FDM.GetPropertyValue("accelerations/Nz");
        JSONObject["accelerations/a-pilot-x-ft_sec2"] = FDM.GetPropertyValue("accelerations/a-pilot-x-ft_sec2");
        JSONObject["accelerations/a-pilot-y-ft_sec2"] = FDM.GetPropertyValue("accelerations/a-pilot-y-ft_sec2");
        JSONObject["accelerations/a-pilot-z-ft_sec2"] = FDM.GetPropertyValue("accelerations/a-pilot-z-ft_sec2");
        JSONObject["accelerations/n-pilot-x-norm"] = FDM.GetPropertyValue("accelerations/n-pilot-x-norm");
        JSONObject["accelerations/n-pilot-y-norm"] = FDM.GetPropertyValue("accelerations/n-pilot-y-norm");
        JSONObject["accelerations/n-pilot-z-norm"] = FDM.GetPropertyValue("accelerations/n-pilot-z-norm");
        JSONObject["accelerations/pdot-rad_sec2"] = FDM.GetPropertyValue("accelerations/pdot-rad_sec2");
        JSONObject["accelerations/qdot-rad_sec2"] = FDM.GetPropertyValue("accelerations/qdot-rad_sec2");
        JSONObject["accelerations/rdot-rad_sec2"] = FDM.GetPropertyValue("accelerations/rdot-rad_sec2");
        JSONObject["accelerations/udot-ft_sec2"] = FDM.GetPropertyValue("accelerations/udot-ft_sec2");
        JSONObject["accelerations/vdot-ft_sec2"] = FDM.GetPropertyValue("accelerations/vdot-ft_sec2");
        JSONObject["accelerations/wdot-ft_sec2"] = FDM.GetPropertyValue("accelerations/wdot-ft_sec2");
        JSONObject["aero/alpha-deg"] = FDM.GetPropertyValue("aero/alpha-deg");
        JSONObject["aero/alpha-max-rad"] = FDM.GetPropertyValue("aero/alpha-max-rad");
        JSONObject["aero/alpha-min-rad"] = FDM.GetPropertyValue("aero/alpha-min-rad");
        JSONObject["aero/alpha-rad"] = FDM.GetPropertyValue("aero/alpha-rad");
        JSONObject["aero/alpha-wing-rad"] = FDM.GetPropertyValue("aero/alpha-wing-rad");
        JSONObject["aero/alphadot-deg_sec"] = FDM.GetPropertyValue("aero/alphadot-deg_sec");
        JSONObject["aero/alphadot-rad_sec"] = FDM.GetPropertyValue("aero/alphadot-rad_sec");
        JSONObject["aero/beta-deg"] = FDM.GetPropertyValue("aero/beta-deg");
        JSONObject["aero/beta-rad"] = FDM.GetPropertyValue("aero/beta-rad");
        JSONObject["aero/betadot-deg_sec"] = FDM.GetPropertyValue("aero/betadot-deg_sec");
        JSONObject["aero/betadot-rad_sec"] = FDM.GetPropertyValue("aero/betadot-rad_sec");
        JSONObject["aero/bi2vel"] = FDM.GetPropertyValue("aero/bi2vel");
        JSONObject["aero/ci2vel"] = FDM.GetPropertyValue("aero/ci2vel");
        JSONObject["aero/cl-squared"] = FDM.GetPropertyValue("aero/cl-squared");
        JSONObject["aero/h_b-cg-ft"] = FDM.GetPropertyValue("aero/h_b-cg-ft");
        JSONObject["aero/h_b-mac-ft"] = FDM.GetPropertyValue("aero/h_b-mac-ft");
        JSONObject["aero/mag-beta-deg"] = FDM.GetPropertyValue("aero/mag-beta-deg");
        JSONObject["aero/mag-beta-rad"] = FDM.GetPropertyValue("aero/mag-beta-rad");
        JSONObject["aero/qbar-area"] = FDM.GetPropertyValue("aero/qbar-area");
        JSONObject["aero/qbar-psf"] = FDM.GetPropertyValue("aero/qbar-psf");
        JSONObject["aero/qbarUV-psf"] = FDM.GetPropertyValue("aero/qbarUV-psf");
        JSONObject["aero/qbarUW-psf"] = FDM.GetPropertyValue("aero/qbarUW-psf");
        JSONObject["aero/stall-hyst-norm"] = FDM.GetPropertyValue("aero/stall-hyst-norm");
        JSONObject["atmosphere/P-psf"] = FDM.GetPropertyValue("atmosphere/P-psf");
        JSONObject["atmosphere/P-sl-psf"] = FDM.GetPropertyValue("atmosphere/P-sl-psf");
        JSONObject["atmosphere/T-R"] = FDM.GetPropertyValue("atmosphere/T-R");
        JSONObject["atmosphere/T-sl-R"] = FDM.GetPropertyValue("atmosphere/T-sl-R");
        JSONObject["atmosphere/T-sl-dev-F"] = FDM.GetPropertyValue("atmosphere/T-sl-dev-F");
        JSONObject["atmosphere/a-fps"] = FDM.GetPropertyValue("atmosphere/a-fps");
        JSONObject["atmosphere/a-ratio"] = FDM.GetPropertyValue("atmosphere/a-ratio");
        JSONObject["atmosphere/a-sl-fps"] = FDM.GetPropertyValue("atmosphere/a-sl-fps");
        JSONObject["atmosphere/crosswind-fps"] = FDM.GetPropertyValue("atmosphere/crosswind-fps");
        JSONObject["atmosphere/delta"] = FDM.GetPropertyValue("atmosphere/delta");
        JSONObject["atmosphere/delta-T"] = FDM.GetPropertyValue("atmosphere/delta-T");
        JSONObject["atmosphere/density-altitude"] = FDM.GetPropertyValue("atmosphere/density-altitude");
        JSONObject["atmosphere/gust-down-fps"] = FDM.GetPropertyValue("atmosphere/gust-down-fps");
        JSONObject["atmosphere/gust-east-fps"] = FDM.GetPropertyValue("atmosphere/gust-east-fps");
        JSONObject["atmosphere/gust-north-fps"] = FDM.GetPropertyValue("atmosphere/gust-north-fps");
        JSONObject["atmosphere/headwind-fps"] = FDM.GetPropertyValue("atmosphere/headwind-fps");
        JSONObject["atmosphere/p-turb-rad_sec"] = FDM.GetPropertyValue("atmosphere/p-turb-rad_sec");
        JSONObject["atmosphere/psiw-rad"] = FDM.GetPropertyValue("atmosphere/psiw-rad");
        JSONObject["atmosphere/q-turb-rad_sec"] = FDM.GetPropertyValue("atmosphere/q-turb-rad_sec");
        JSONObject["atmosphere/r-turb-rad_sec"] = FDM.GetPropertyValue("atmosphere/r-turb-rad_sec");
        JSONObject["atmosphere/rho-sl-slugs_ft3"] = FDM.GetPropertyValue("atmosphere/rho-sl-slugs_ft3");
        JSONObject["atmosphere/rho-slugs_ft3"] = FDM.GetPropertyValue("atmosphere/rho-slugs_ft3");
        JSONObject["atmosphere/sigma"] = FDM.GetPropertyValue("atmosphere/sigma");
        JSONObject["atmosphere/theta"] = FDM.GetPropertyValue("atmosphere/theta");
        JSONObject["atmosphere/turb-gain"] = FDM.GetPropertyValue("atmosphere/turb-gain");
        JSONObject["atmosphere/turb-rate"] = FDM.GetPropertyValue("atmosphere/turb-rate");
        JSONObject["atmosphere/turb-rhythmicity"] = FDM.GetPropertyValue("atmosphere/turb-rhythmicity");
        JSONObject["atmosphere/wind-from-cw"] = FDM.GetPropertyValue("atmosphere/wind-from-cw");
        JSONObject["attitude/heading-true-rad"] = FDM.GetPropertyValue("attitude/heading-true-rad");
        JSONObject["attitude/phi-rad"] = FDM.GetPropertyValue("attitude/phi-rad");
        JSONObject["attitude/pitch-rad"] = FDM.GetPropertyValue("attitude/pitch-rad");
        JSONObject["attitude/psi-rad"] = FDM.GetPropertyValue("attitude/psi-rad");
        JSONObject["attitude/roll-rad"] = FDM.GetPropertyValue("attitude/roll-rad");
        JSONObject["attitude/theta-rad"] = FDM.GetPropertyValue("attitude/theta-rad");
        JSONObject["fcs/aileron-cmd-norm"] = FDM.GetPropertyValue("fcs/aileron-cmd-norm");
        JSONObject["fcs/center-brake-cmd-norm"] = FDM.GetPropertyValue("fcs/center-brake-cmd-norm");
        JSONObject["fcs/elevator-cmd-norm"] = FDM.GetPropertyValue("fcs/elevator-cmd-norm");
        JSONObject["fcs/elevator-pos-deg"] = FDM.GetPropertyValue("fcs/elevator-pos-deg");
        JSONObject["fcs/elevator-pos-norm"] = FDM.GetPropertyValue("fcs/elevator-pos-norm");
        JSONObject["fcs/elevator-pos-rad"] = FDM.GetPropertyValue("fcs/elevator-pos-rad");
        JSONObject["fcs/flap-cmd-norm"] = FDM.GetPropertyValue("fcs/flap-cmd-norm");
        JSONObject["fcs/flap-pos-deg"] = FDM.GetPropertyValue("fcs/flap-pos-deg");
        JSONObject["fcs/flap-pos-norm"] = FDM.GetPropertyValue("fcs/flap-pos-norm");
        JSONObject["fcs/flap-pos-rad"] = FDM.GetPropertyValue("fcs/flap-pos-rad");
        JSONObject["fcs/left-aileron-pos-deg"] = FDM.GetPropertyValue("fcs/left-aileron-pos-deg");
        JSONObject["fcs/left-aileron-pos-norm"] = FDM.GetPropertyValue("fcs/left-aileron-pos-norm");
        JSONObject["fcs/left-aileron-pos-rad"] = FDM.GetPropertyValue("fcs/left-aileron-pos-rad");
        JSONObject["fcs/left-brake-cmd-norm"] = FDM.GetPropertyValue("fcs/left-brake-cmd-norm");
        JSONObject["fcs/mag-elevator-pos-rad"] = FDM.GetPropertyValue("fcs/mag-elevator-pos-rad");
        JSONObject["fcs/mag-left-aileron-pos-rad"] = FDM.GetPropertyValue("fcs/mag-left-aileron-pos-rad");
        JSONObject["fcs/mag-right-aileron-pos-rad"] = FDM.GetPropertyValue("fcs/mag-right-aileron-pos-rad");
        JSONObject["fcs/mag-rudder-pos-rad"] = FDM.GetPropertyValue("fcs/mag-rudder-pos-rad");
        JSONObject["fcs/mag-speedbrake-pos-rad"] = FDM.GetPropertyValue("fcs/mag-speedbrake-pos-rad");
        JSONObject["fcs/mag-spoiler-pos-rad"] = FDM.GetPropertyValue("fcs/mag-spoiler-pos-rad");
        JSONObject["fcs/pitch-trim-cmd-norm"] = FDM.GetPropertyValue("fcs/pitch-trim-cmd-norm");
        JSONObject["fcs/right-aileron-pos-deg"] = FDM.GetPropertyValue("fcs/right-aileron-pos-deg");
        JSONObject["fcs/right-aileron-pos-norm"] = FDM.GetPropertyValue("fcs/right-aileron-pos-norm");
        JSONObject["fcs/right-aileron-pos-rad"] = FDM.GetPropertyValue("fcs/right-aileron-pos-rad");
        JSONObject["fcs/right-brake-cmd-norm"] = FDM.GetPropertyValue("fcs/right-brake-cmd-norm");
        JSONObject["fcs/roll-trim-cmd-norm"] = FDM.GetPropertyValue("fcs/roll-trim-cmd-norm");
        JSONObject["fcs/rudder-cmd-norm"] = FDM.GetPropertyValue("fcs/rudder-cmd-norm");
        JSONObject["fcs/rudder-pos-deg"] = FDM.GetPropertyValue("fcs/rudder-pos-deg");
        JSONObject["fcs/rudder-pos-norm"] = FDM.GetPropertyValue("fcs/rudder-pos-norm");
        JSONObject["fcs/rudder-pos-rad"] = FDM.GetPropertyValue("fcs/rudder-pos-rad");
        JSONObject["fcs/speedbrake-cmd-norm"] = FDM.GetPropertyValue("fcs/speedbrake-cmd-norm");
        JSONObject["fcs/speedbrake-pos-deg"] = FDM.GetPropertyValue("fcs/speedbrake-pos-deg");
        JSONObject["fcs/speedbrake-pos-norm"] = FDM.GetPropertyValue("fcs/speedbrake-pos-norm");
        JSONObject["fcs/speedbrake-pos-rad"] = FDM.GetPropertyValue("fcs/speedbrake-pos-rad");
        JSONObject["fcs/spoiler-cmd-norm"] = FDM.GetPropertyValue("fcs/spoiler-cmd-norm");
        JSONObject["fcs/spoiler-pos-deg"] = FDM.GetPropertyValue("fcs/spoiler-pos-deg");
        JSONObject["fcs/spoiler-pos-norm"] = FDM.GetPropertyValue("fcs/spoiler-pos-norm");
        JSONObject["fcs/spoiler-pos-rad"] = FDM.GetPropertyValue("fcs/spoiler-pos-rad");
        JSONObject["fcs/steer-cmd-norm"] = FDM.GetPropertyValue("fcs/steer-cmd-norm");
        JSONObject["fcs/yaw-trim-cmd-norm"] = FDM.GetPropertyValue("fcs/yaw-trim-cmd-norm");
        JSONObject["flight-path/gamma-rad"] = FDM.GetPropertyValue("flight-path/gamma-rad");
        JSONObject["flight-path/psi-gt-rad"] = FDM.GetPropertyValue("flight-path/psi-gt-rad");
        JSONObject["forces/fbx-aero-lbs"] = FDM.GetPropertyValue("forces/fbx-aero-lbs");
        JSONObject["forces/fbx-gear-lbs"] = FDM.GetPropertyValue("forces/fbx-gear-lbs");
        JSONObject["forces/fbx-prop-lbs"] = FDM.GetPropertyValue("forces/fbx-prop-lbs");
        JSONObject["forces/fbx-total-lbs"] = FDM.GetPropertyValue("forces/fbx-total-lbs");
        JSONObject["forces/fby-aero-lbs"] = FDM.GetPropertyValue("forces/fby-aero-lbs");
        JSONObject["forces/fby-gear-lbs"] = FDM.GetPropertyValue("forces/fby-gear-lbs");
        JSONObject["forces/fby-prop-lbs"] = FDM.GetPropertyValue("forces/fby-prop-lbs");
        JSONObject["forces/fby-total-lbs"] = FDM.GetPropertyValue("forces/fby-total-lbs");
        JSONObject["forces/fbz-aero-lbs"] = FDM.GetPropertyValue("forces/fbz-aero-lbs");
        JSONObject["forces/fbz-gear-lbs"] = FDM.GetPropertyValue("forces/fbz-gear-lbs");
        JSONObject["forces/fbz-prop-lbs"] = FDM.GetPropertyValue("forces/fbz-prop-lbs");
        JSONObject["forces/fbz-total-lbs"] = FDM.GetPropertyValue("forces/fbz-total-lbs");
        JSONObject["forces/fwx-aero-lbs"] = FDM.GetPropertyValue("forces/fwx-aero-lbs");
        JSONObject["forces/fwy-aero-lbs"] = FDM.GetPropertyValue("forces/fwy-aero-lbs");
        JSONObject["forces/fwz-aero-lbs"] = FDM.GetPropertyValue("forces/fwz-aero-lbs");
        JSONObject["forces/hold-down"] = FDM.GetPropertyValue("forces/hold-down");
        JSONObject["forces/lod-norm"] = FDM.GetPropertyValue("forces/lod-norm");
        JSONObject["gear/gear-cmd-norm"] = FDM.GetPropertyValue("gear/gear-cmd-norm");
        JSONObject["gear/gear-pos-norm"] = FDM.GetPropertyValue("gear/gear-pos-norm");
        JSONObject["gear/num-units"] = FDM.GetPropertyValue("gear/num-units");
        JSONObject["ic/alpha-deg"] = FDM.GetPropertyValue("ic/alpha-deg");
        JSONObject["ic/alpha-rad"] = FDM.GetPropertyValue("ic/alpha-rad");
        JSONObject["ic/beta-deg"] = FDM.GetPropertyValue("ic/beta-deg");
        JSONObject["ic/beta-rad"] = FDM.GetPropertyValue("ic/beta-rad");
        JSONObject["ic/gamma-deg"] = FDM.GetPropertyValue("ic/gamma-deg");
        JSONObject["ic/gamma-rad"] = FDM.GetPropertyValue("ic/gamma-rad");
        JSONObject["ic/h-agl-ft"] = FDM.GetPropertyValue("ic/h-agl-ft");
        JSONObject["ic/h-sl-ft"] = FDM.GetPropertyValue("ic/h-sl-ft");
        JSONObject["ic/lat-gc-deg"] = FDM.GetPropertyValue("ic/lat-gc-deg");
        JSONObject["ic/lat-gc-rad"] = FDM.GetPropertyValue("ic/lat-gc-rad");
        JSONObject["ic/long-gc-deg"] = FDM.GetPropertyValue("ic/long-gc-deg");
        JSONObject["ic/long-gc-rad"] = FDM.GetPropertyValue("ic/long-gc-rad");
        JSONObject["ic/mach"] = FDM.GetPropertyValue("ic/mach");
        JSONObject["ic/p-rad_sec"] = FDM.GetPropertyValue("ic/p-rad_sec");
        JSONObject["ic/phi-deg"] = FDM.GetPropertyValue("ic/phi-deg");
        JSONObject["ic/phi-rad"] = FDM.GetPropertyValue("ic/phi-rad");
        JSONObject["ic/psi-true-deg"] = FDM.GetPropertyValue("ic/psi-true-deg");
        JSONObject["ic/psi-true-rad"] = FDM.GetPropertyValue("ic/psi-true-rad");
        JSONObject["ic/q-rad_sec"] = FDM.GetPropertyValue("ic/q-rad_sec");
        JSONObject["ic/r-rad_sec"] = FDM.GetPropertyValue("ic/r-rad_sec");
        JSONObject["ic/roc-fpm"] = FDM.GetPropertyValue("ic/roc-fpm");
        JSONObject["ic/roc-fps"] = FDM.GetPropertyValue("ic/roc-fps");
        JSONObject["ic/sea-level-radius-ft"] = FDM.GetPropertyValue("ic/sea-level-radius-ft");
        JSONObject["ic/terrain-altitude-ft"] = FDM.GetPropertyValue("ic/terrain-altitude-ft");
        JSONObject["ic/theta-deg"] = FDM.GetPropertyValue("ic/theta-deg");
        JSONObject["ic/theta-rad"] = FDM.GetPropertyValue("ic/theta-rad");
        JSONObject["ic/u-fps"] = FDM.GetPropertyValue("ic/u-fps");
        JSONObject["ic/v-fps"] = FDM.GetPropertyValue("ic/v-fps");
        JSONObject["ic/vc-kts"] = FDM.GetPropertyValue("ic/vc-kts");
        JSONObject["ic/vd-fps"] = FDM.GetPropertyValue("ic/vd-fps");
        JSONObject["ic/ve-fps"] = FDM.GetPropertyValue("ic/ve-fps");
        JSONObject["ic/ve-kts"] = FDM.GetPropertyValue("ic/ve-kts");
        JSONObject["ic/vg-fps"] = FDM.GetPropertyValue("ic/vg-fps");
        JSONObject["ic/vg-kts"] = FDM.GetPropertyValue("ic/vg-kts");
        JSONObject["ic/vn-fps"] = FDM.GetPropertyValue("ic/vn-fps");
        JSONObject["ic/vt-fps"] = FDM.GetPropertyValue("ic/vt-fps");
        JSONObject["ic/vt-kts"] = FDM.GetPropertyValue("ic/vt-kts");
        JSONObject["ic/vw-bx-fps"] = FDM.GetPropertyValue("ic/vw-bx-fps");
        JSONObject["ic/vw-by-fps"] = FDM.GetPropertyValue("ic/vw-by-fps");
        JSONObject["ic/vw-bz-fps"] = FDM.GetPropertyValue("ic/vw-bz-fps");
        JSONObject["ic/vw-dir-deg"] = FDM.GetPropertyValue("ic/vw-dir-deg");
        JSONObject["ic/vw-down-fps"] = FDM.GetPropertyValue("ic/vw-down-fps");
        JSONObject["ic/vw-east-fps"] = FDM.GetPropertyValue("ic/vw-east-fps");
        JSONObject["ic/vw-mag-fps"] = FDM.GetPropertyValue("ic/vw-mag-fps");
        JSONObject["ic/vw-north-fps"] = FDM.GetPropertyValue("ic/vw-north-fps");
        JSONObject["ic/w-fps"] = FDM.GetPropertyValue("ic/w-fps");
        JSONObject["inertia/cg-x-in"] = FDM.GetPropertyValue("inertia/cg-x-in");
        JSONObject["inertia/cg-y-in"] = FDM.GetPropertyValue("inertia/cg-y-in");
        JSONObject["inertia/cg-z-in"] = FDM.GetPropertyValue("inertia/cg-z-in");
        JSONObject["inertia/empty-weight-lbs"] = FDM.GetPropertyValue("inertia/empty-weight-lbs");
        JSONObject["inertia/mass-slugs"] = FDM.GetPropertyValue("inertia/mass-slugs");
        JSONObject["inertia/weight-lbs"] = FDM.GetPropertyValue("inertia/weight-lbs");
        JSONObject["metrics/Sh-sqft"] = FDM.GetPropertyValue("metrics/Sh-sqft");
        JSONObject["metrics/Sv-sqft"] = FDM.GetPropertyValue("metrics/Sv-sqft");
        JSONObject["metrics/Sw-sqft"] = FDM.GetPropertyValue("metrics/Sw-sqft");
        JSONObject["metrics/aero-rp-x-in"] = FDM.GetPropertyValue("metrics/aero-rp-x-in");
        JSONObject["metrics/aero-rp-y-in"] = FDM.GetPropertyValue("metrics/aero-rp-y-in");
        JSONObject["metrics/aero-rp-z-in"] = FDM.GetPropertyValue("metrics/aero-rp-z-in");
        JSONObject["metrics/bw-ft"] = FDM.GetPropertyValue("metrics/bw-ft");
        JSONObject["metrics/cbarw-ft"] = FDM.GetPropertyValue("metrics/cbarw-ft");
        JSONObject["metrics/eyepoint-x-in"] = FDM.GetPropertyValue("metrics/eyepoint-x-in");
        JSONObject["metrics/eyepoint-y-in"] = FDM.GetPropertyValue("metrics/eyepoint-y-in");
        JSONObject["metrics/eyepoint-z-in"] = FDM.GetPropertyValue("metrics/eyepoint-z-in");
        JSONObject["metrics/iw-deg"] = FDM.GetPropertyValue("metrics/iw-deg");
        JSONObject["metrics/iw-rad"] = FDM.GetPropertyValue("metrics/iw-rad");
        JSONObject["metrics/lh-ft"] = FDM.GetPropertyValue("metrics/lh-ft");
        JSONObject["metrics/lh-norm"] = FDM.GetPropertyValue("metrics/lh-norm");
        JSONObject["metrics/lv-ft"] = FDM.GetPropertyValue("metrics/lv-ft");
        JSONObject["metrics/lv-norm"] = FDM.GetPropertyValue("metrics/lv-norm");
        JSONObject["metrics/runway-radius"] = FDM.GetPropertyValue("metrics/runway-radius");
        JSONObject["metrics/vbarh-norm"] = FDM.GetPropertyValue("metrics/vbarh-norm");
        JSONObject["metrics/vbarv-norm"] = FDM.GetPropertyValue("metrics/vbarv-norm");
        JSONObject["metrics/visualrefpoint-x-in"] = FDM.GetPropertyValue("metrics/visualrefpoint-x-in");
        JSONObject["metrics/visualrefpoint-y-in"] = FDM.GetPropertyValue("metrics/visualrefpoint-y-in");
        JSONObject["metrics/visualrefpoint-z-in"] = FDM.GetPropertyValue("metrics/visualrefpoint-z-in");
        JSONObject["moments/l-aero-lbsft"] = FDM.GetPropertyValue("moments/l-aero-lbsft");
        JSONObject["moments/l-gear-lbsft"] = FDM.GetPropertyValue("moments/l-gear-lbsft");
        JSONObject["moments/l-prop-lbsft"] = FDM.GetPropertyValue("moments/l-prop-lbsft");
        JSONObject["moments/l-total-lbsft"] = FDM.GetPropertyValue("moments/l-total-lbsft");
        JSONObject["moments/m-aero-lbsft"] = FDM.GetPropertyValue("moments/m-aero-lbsft");
        JSONObject["moments/m-gear-lbsft"] = FDM.GetPropertyValue("moments/m-gear-lbsft");
        JSONObject["moments/m-prop-lbsft"] = FDM.GetPropertyValue("moments/m-prop-lbsft");
        JSONObject["moments/m-total-lbsft"] = FDM.GetPropertyValue("moments/m-total-lbsft");
        JSONObject["moments/n-aero-lbsft"] = FDM.GetPropertyValue("moments/n-aero-lbsft");
        JSONObject["moments/n-gear-lbsft"] = FDM.GetPropertyValue("moments/n-gear-lbsft");
        JSONObject["moments/n-prop-lbsft"] = FDM.GetPropertyValue("moments/n-prop-lbsft");
        JSONObject["moments/n-total-lbsft"] = FDM.GetPropertyValue("moments/n-total-lbsft");
        JSONObject["output-norm"] = FDM.GetPropertyValue("output-norm");
        JSONObject["position/distance-from-start-lat-mt"] = FDM.GetPropertyValue("position/distance-from-start-lat-mt");
        JSONObject["position/distance-from-start-lon-mt"] = FDM.GetPropertyValue("position/distance-from-start-lon-mt");
        JSONObject["position/distance-from-start-mag-mt"] = FDM.GetPropertyValue("position/distance-from-start-mag-mt");
        JSONObject["position/epa-rad"] = FDM.GetPropertyValue("position/epa-rad");
        JSONObject["position/geod-alt-ft"] = FDM.GetPropertyValue("position/geod-alt-ft");
        JSONObject["position/h-agl-ft"] = FDM.GetPropertyValue("position/h-agl-ft");
        JSONObject["position/h-sl-ft"] = FDM.GetPropertyValue("position/h-sl-ft");
        JSONObject["position/h-sl-meters"] = FDM.GetPropertyValue("position/h-sl-meters");
        JSONObject["position/lat-gc-deg"] = FDM.GetPropertyValue("position/lat-gc-deg");
        JSONObject["position/lat-gc-rad"] = FDM.GetPropertyValue("position/lat-gc-rad");
        JSONObject["position/lat-geod-deg"] = FDM.GetPropertyValue("position/lat-geod-deg");
        JSONObject["position/lat-geod-rad"] = FDM.GetPropertyValue("position/lat-geod-rad");
        JSONObject["position/long-gc-deg"] = FDM.GetPropertyValue("position/long-gc-deg");
        JSONObject["position/long-gc-rad"] = FDM.GetPropertyValue("position/long-gc-rad");
        JSONObject["position/radius-to-vehicle-ft"] = FDM.GetPropertyValue("position/radius-to-vehicle-ft");
        JSONObject["position/terrain-elevation-asl-ft"] = FDM.GetPropertyValue("position/terrain-elevation-asl-ft");
        JSONObject["propulsion/active_engine"] = FDM.GetPropertyValue("propulsion/active_engine");
        JSONObject["propulsion/cutoff_cmd"] = FDM.GetPropertyValue("propulsion/cutoff_cmd");
        JSONObject["propulsion/fuel_dump"] = FDM.GetPropertyValue("propulsion/fuel_dump");
        JSONObject["propulsion/magneto_cmd"] = FDM.GetPropertyValue("propulsion/magneto_cmd");
        JSONObject["propulsion/pt-lbs_sqft"] = FDM.GetPropertyValue("propulsion/pt-lbs_sqft");
        JSONObject["propulsion/refuel"] = FDM.GetPropertyValue("propulsion/refuel");
        JSONObject["propulsion/set-running"] = FDM.GetPropertyValue("propulsion/set-running");
        JSONObject["propulsion/starter_cmd"] = FDM.GetPropertyValue("propulsion/starter_cmd");
        JSONObject["propulsion/tat-c"] = FDM.GetPropertyValue("propulsion/tat-c");
        JSONObject["propulsion/tat-r"] = FDM.GetPropertyValue("propulsion/tat-r");
        JSONObject["propulsion/total-fuel-lbs"] = FDM.GetPropertyValue("propulsion/total-fuel-lbs");
        JSONObject["sim-time-sec"] = FDM.GetPropertyValue("sim-time-sec");
        JSONObject["simulation/cycle_duration"] = FDM.GetPropertyValue("simulation/cycle_duration");
        JSONObject["simulation/do_simple_trim"] = FDM.GetPropertyValue("simulation/do_simple_trim");
        JSONObject["simulation/do_trim_analysis"] = FDM.GetPropertyValue("simulation/do_trim_analysis");
        JSONObject["simulation/frame_start_time"] = FDM.GetPropertyValue("simulation/frame_start_time");
        JSONObject["simulation/integrator/position/rotational"] = FDM.GetPropertyValue("simulation/integrator/position/rotational");
        JSONObject["simulation/integrator/position/translational"] = FDM.GetPropertyValue("simulation/integrator/position/translational");
        JSONObject["simulation/integrator/rate/rotational"] = FDM.GetPropertyValue("simulation/integrator/rate/rotational");
        JSONObject["simulation/integrator/rate/translational"] = FDM.GetPropertyValue("simulation/integrator/rate/translational");
        JSONObject["simulation/terminate"] = FDM.GetPropertyValue("simulation/terminate");
        JSONObject["simulation/write-state-file"] = FDM.GetPropertyValue("simulation/write-state-file");
        JSONObject["systems/stall-warn-norm"] = FDM.GetPropertyValue("systems/stall-warn-norm");
        JSONObject["velocities/eci-velocity-mag-fps"] = FDM.GetPropertyValue("velocities/eci-velocity-mag-fps");
        JSONObject["velocities/h-dot-fps"] = FDM.GetPropertyValue("velocities/h-dot-fps");
        JSONObject["velocities/mach"] = FDM.GetPropertyValue("velocities/mach");
        JSONObject["velocities/machU"] = FDM.GetPropertyValue("velocities/machU");
        JSONObject["velocities/p-aero-rad_sec"] = FDM.GetPropertyValue("velocities/p-aero-rad_sec");
        JSONObject["velocities/p-rad_sec"] = FDM.GetPropertyValue("velocities/p-rad_sec");
        JSONObject["velocities/phidot-rad_sec"] = FDM.GetPropertyValue("velocities/phidot-rad_sec");
        JSONObject["velocities/psidot-rad_sec"] = FDM.GetPropertyValue("velocities/psidot-rad_sec");
        JSONObject["velocities/q-aero-rad_sec"] = FDM.GetPropertyValue("velocities/q-aero-rad_sec");
        JSONObject["velocities/q-rad_sec"] = FDM.GetPropertyValue("velocities/q-rad_sec");
        JSONObject["velocities/r-aero-rad_sec"] = FDM.GetPropertyValue("velocities/r-aero-rad_sec");
        JSONObject["velocities/r-rad_sec"] = FDM.GetPropertyValue("velocities/r-rad_sec");
        JSONObject["velocities/thetadot-rad_sec"] = FDM.GetPropertyValue("velocities/thetadot-rad_sec");
        JSONObject["velocities/u-aero-fps"] = FDM.GetPropertyValue("velocities/u-aero-fps");
        JSONObject["velocities/u-fps"] = FDM.GetPropertyValue("velocities/u-fps");
        JSONObject["velocities/v-aero-fps"] = FDM.GetPropertyValue("velocities/v-aero-fps");
        JSONObject["velocities/v-down-fps"] = FDM.GetPropertyValue("velocities/v-down-fps");
        JSONObject["velocities/v-east-fps"] = FDM.GetPropertyValue("velocities/v-east-fps");
        JSONObject["velocities/v-fps"] = FDM.GetPropertyValue("velocities/v-fps");
        JSONObject["velocities/v-north-fps"] = FDM.GetPropertyValue("velocities/v-north-fps");
        JSONObject["velocities/vc-fps"] = FDM.GetPropertyValue("velocities/vc-fps");
        JSONObject["velocities/vc-kts"] = FDM.GetPropertyValue("velocities/vc-kts");
        JSONObject["velocities/ve-fps"] = FDM.GetPropertyValue("velocities/ve-fps");
        JSONObject["velocities/ve-kts"] = FDM.GetPropertyValue("velocities/ve-kts");
        JSONObject["velocities/vg-fps"] = FDM.GetPropertyValue("velocities/vg-fps");
        JSONObject["velocities/vt-fps"] = FDM.GetPropertyValue("velocities/vt-fps");
        JSONObject["velocities/w-aero-fps"] = FDM.GetPropertyValue("velocities/w-aero-fps");
        JSONObject["velocities/w-fps"] = FDM.GetPropertyValue("velocities/w-fps");

        return JSONObject.dump();
    }

}

int main(int argc, char* argv[])
{
    tulsa::JSBSim_Interface jsb;

    std::ifstream aircraftStateFile(tulsa::dirPath.path + tulsa::AIRCRAFT_STATE_PATH);
    std::stringstream aircraftStateBuffer;

    if (!aircraftStateFile)
    {
        std::cout << "ERROR: INVALID JSON FILE PATH FOR AIRCRAFT INPUT STATE: " << tulsa::dirPath.path + tulsa::AIRCRAFT_STATE_PATH << std::endl;
        return EXIT_FAILURE;
    }

    aircraftStateBuffer << aircraftStateFile.rdbuf();
    json jsonObject = json::parse(aircraftStateBuffer.str());

    jsb.initializeJSBSim();
    jsb.writeManeuverConfiguration();
    jsb.resetAircraftState(jsonObject["JSB"].dump());
    jsb.setCommandedManeuver(0);

    for(int i = 0; i < 500; i++)
    {
        jsb.runAutoPilot(5);
        std::cout << jsb.readAircraftState() << std::endl;
    }

    return 0;
}

/* shared library (python) entry */
PYBIND11_MODULE(libJSB_Interface, m)
{
  // autopilot structure
  py::class_<tulsa::JSBSim_Interface>(m, "JSBSim_Interface")

      /* initialization */
      .def(py::init<>()) // constructor
      .def("initializeJSBSimFromPython", &tulsa::JSBSim_Interface::initializeJSBSimFromPython, "Start JSBSim Engine.")
      .def("writeManeuverConfigurationFromPython", &tulsa::JSBSim_Interface::writeManeuverConfigurationFromPython, "Write a custom version of the maneuve configuration. Usually from EVAA. Accepts a JSON payload.")
      .def("setCommandedManeuverFromPython", &tulsa::JSBSim_Interface::setCommandedManeuverFromPython, "Set the commanded maneuver of the autopilot.")
      .def("runAutoPilotFromPython", &tulsa::JSBSim_Interface::runAutoPilotFromPython, "Run N iterations of the AutoPilot and JSBSim Flight Dynamics.")
      .def("resetAircraftStateFromPython", &tulsa::JSBSim_Interface::resetAircraftStateFromPython, "Completely reset the aircraft state to new initial conditions. Accepts a JSON payload atune to tu_evaa_gcas/cfg/aircraftState.json.")
      .def("readAircraftStateFromPython", &tulsa::JSBSim_Interface::readAircraftStateFromPython, "Updates the aircraft state's read-only parameters from JSBSim and returns it as a JSON payload.")
      .def("getExtendedJSBSimStateFromPython", &tulsa::JSBSim_Interface::getExtendedJSBSimStateFromPython, "Get the entire working state of JSBSim as a JSON Payload. This includes ALL property values.");
}
