#ifndef TU_AIRCRAFTSTATE_HPP
#define TU_AIRCRAFTSTATE_HPP

#include <vector>
#include <iostream>
#include <pybind11/pybind11.h>
#include "json/json.hpp"

/* namespace handling */
namespace py = pybind11;
using json = nlohmann::json;

namespace tulsa
{
    // Class used to keep track of aircraft state. Member value reset is handled by calling the "update_readonly" function in main after resetting the simulation/
    class AircraftState
    {

    public:
        /* state values */
        double initLat, initLon, initAlt;
        double latPairs[2], lonPairs[2], altPairs[2]; // Pairs are needed to calculate XYZ euclidean distance from origin. First item in each pair is origin lat/lon/alt.
        double lat, lon, alt;
        double phiB, thetaB, psiB, thetaI, psiI, chi, gamma; //"B" suffixes denote body frame, "I" suffixes denote inertial frame. i.e. psiI is the direction the aircraft is in relative to the origin.
        double uB, vB, wB;
        double vc, vt_kts; // Body frame velocities
        double p, q, r;
        double alpha, initAlpha, beta, nzG;
        double weight;
        double vA, bestRate; // max maneuvering speed, function of weight and best rate of climb speed
        double simDt;
        double simT;
        int apMode;
        bool isSteady = false;

        nlohmann::json jsonObject;

        // populates a json object with information about the aircraft state.
        void populateJSONObject(json& JSONObject)
        {
            JSONObject["initLat"] = initLat;
            JSONObject["initLon"] = initLon;
            JSONObject["initAlt"] = initAlt;
            JSONObject["lat"] = lat;
            JSONObject["lon"] = lon;
            JSONObject["alt"] = alt;
            JSONObject["vc"] = vc;
            JSONObject["vt"] = vt_kts;
            JSONObject["phiB"] = phiB;
            JSONObject["thetaB"] = thetaB;
            JSONObject["psiB"] = psiB;
            JSONObject["chi"] = chi;
            JSONObject["uB"] = uB;
            JSONObject["vB"] = vB;
            JSONObject["wB"] = wB;
            JSONObject["p"] = p;
            JSONObject["q"] = q;
            JSONObject["r"] = r;
            JSONObject["alpha"] = alpha;
            JSONObject["beta"] = beta;
            JSONObject["gamma"] = gamma;
            JSONObject["weight"] = weight;
            JSONObject["nzG"] = nzG;
            // JSONObject["throttleOn"] = throttleOn;
            // JSONObject["isSteady"] = isSteady;
            // JSONObject["rangeFt"] = rangeFt;
            // JSONObject["time"] = time;
            // JSONObject["alphaCmd"] = alphaCmd;
            // JSONObject["betaCmd"] = betaCmd;
            // JSONObject["crsCmd"] = crsCmd;
            // JSONObject["gamCmd"] = gamCmd;
            // JSONObject["phiCmd"] = phiCmd;
            // JSONObject["starter_cmd"] = starter_cmd;
            // JSONObject["total-fuel-lbs"] = total-fuel-lbs;
            // JSONObject["set-running"] = set-running;
            // JSONObject["deltECmd"] = deltECmd;
            // JSONObject["deltE"] = deltE;
            return;
        }
    };

} // namespace tulsa

#endif // TU_AIRCRAFTSTATE_HPP