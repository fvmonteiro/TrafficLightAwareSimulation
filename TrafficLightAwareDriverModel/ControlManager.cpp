#include "ControlManager.h"
#include "EgoVehicle.h"
#include "NearbyVehicle.h"
#include "TrafficLightACCVehicle.h"

ControlManager::ControlManager(const EgoVehicle& ego_vehicle,
	bool verbose) :
	verbose{ verbose } 
{
	if (verbose) 
	{
		std::clog << "Creating control manager " << std::endl;
	}

	bool is_long_control_verbose = verbose;

	switch (ego_vehicle.get_type())
	{
	case VehicleType::traffic_light_acc_car:
	case VehicleType::traffic_light_cacc_car: // both get the same controller
		with_traffic_lights_controller =
			LongitudinalControllerWithTrafficLights(ego_vehicle,
				is_long_control_verbose);
		break;
	default:
		break;
	}
}

ControlManager::ControlManager(const EgoVehicle& ego_vehicle)
	: ControlManager(ego_vehicle, false) {}

color_t ControlManager::get_longitudinal_controller_color() const
{
	return with_traffic_lights_controller.get_state_color();
}

double ControlManager::use_vissim_desired_acceleration(
	const EgoVehicle& ego_vehicle) 
{
	active_longitudinal_controller = LongControlType::vissim;
	return ego_vehicle.get_vissim_acceleration();
}

double ControlManager::get_traffic_light_acc_acceleration(
	const TrafficLightACCVehicle& ego_vehicle,
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	if (verbose) std::clog << "Inside get traffic_light_acc_acceleration\n";

	std::unordered_map<LongitudinalControllerWithTrafficLights::State, double>
		possible_accelerations;

	with_traffic_lights_controller.get_nominal_input(possible_accelerations);
	with_traffic_lights_controller.compute_vehicle_following_input(
		ego_vehicle, possible_accelerations);
	with_traffic_lights_controller.compute_velocity_control_input(
		ego_vehicle, possible_accelerations);
	with_traffic_lights_controller.compute_traffic_light_input(
		ego_vehicle, traffic_lights, possible_accelerations);

	active_longitudinal_controller = LongControlType::traffic_light_acc;

	double ret = with_traffic_lights_controller.
		choose_acceleration(ego_vehicle, possible_accelerations);

	return ret;
}

void ControlManager::print_traffic_lights(const EgoVehicle& ego,
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	std::clog << "veh id=" << ego.get_id() << std::endl;
	for (auto& pair : traffic_lights) std::clog << "tf id=" << pair.first <<
		"(" << pair.second.get_id() << "), ";
	std::clog << std::endl;

	if (verbose) std::clog << "Inside placeholder function\n"
		<< "Getting nominal input" << std::endl;

	std::unordered_map<LongitudinalControllerWithTrafficLights::State, double>
		possible_accelerations;

	with_traffic_lights_controller.get_nominal_input(possible_accelerations);
}