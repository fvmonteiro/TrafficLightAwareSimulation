/*==========================================================================*/
/*  ControlManager.cpp    											        */
/*  Helps manage controllers used by the autonomous vehicle 				*/
/*                                                                          */
/*  Version of 2022-06	                              Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <unordered_map>
#include <vector>

#include "LongitudinalControllerWithTrafficLights.h"
#include "Vehicle.h"

class EgoVehicle;
class NearbyVehicle;
class TrafficLightACCVehicle;

class ControlManager {
public:

	enum class LongControlType 
	{
		vissim,
		traffic_light_acc
	};

	ControlManager() = default;
	ControlManager(const EgoVehicle& ego_vehicle, bool verbose);
	ControlManager(const EgoVehicle& ego_vehicle);

	LongControlType get_active_longitudinal_controller() const {
		return active_longitudinal_controller;
	}

	color_t get_longitudinal_controller_color() const;
	
	double get_traffic_light_acc_acceleration(
		const TrafficLightACCVehicle& ego_vehicle,
		const std::unordered_map<int, TrafficLight>& traffic_lights);

	double use_vissim_desired_acceleration(const EgoVehicle& ego_vehicle);

private:
	LongitudinalControllerWithTrafficLights
		with_traffic_lights_controller;

	/* Indicates which controller is active. Used for debugging and
	visualization. */
	LongControlType active_longitudinal_controller{ LongControlType::vissim }; 
	
	bool verbose{ false };
};