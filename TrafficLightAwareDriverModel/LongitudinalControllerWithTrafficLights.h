/*==========================================================================*/
/*  LongitudinalControllerWithTrafficLights									*/
/*  Provably safe longitudinal vehicle controller that respects traffic     */
/*  lights                                                                  */
/*                                                                          */
/*	Version of 2022-06						   Fernando Valladares Monteiro */
/*==========================================================================*/

#pragma once

#include <unordered_map>

#include "Constants.h"
#include "TrafficLight.h"

/* Forward declaration */
class EgoVehicle;
class TrafficLightACCVehicle;

class LongitudinalControllerWithTrafficLights
{
public:
	enum class State {
		vehicle_following,
		velocity_control,
		traffic_light,
		max_accel,
		too_close,
	};

	LongitudinalControllerWithTrafficLights() = default;
	LongitudinalControllerWithTrafficLights(const EgoVehicle& ego_vehicle,
		bool verbose);

	State get_state() const { return active_mode; };
	double get_gap_error() const { return gap_error; };

	color_t get_state_color() const;
	double get_nominal_input(
		std::unordered_map<State, double>& possible_accelerations);
	bool compute_vehicle_following_input(const EgoVehicle& ego_vehicle,
		std::unordered_map<State, double>& possible_accelerations);
	bool compute_velocity_control_input(const EgoVehicle& ego_vehicle,
		std::unordered_map<State, double>& possible_accelerations);
	bool compute_traffic_light_input(
		const TrafficLightACCVehicle& ego_vehicle,
		const std::unordered_map<int, TrafficLight>& traffic_lights,
		std::unordered_map<State, double>& possible_accelerations);

	double choose_acceleration(const EgoVehicle& ego_vehicle,
		std::unordered_map<State, double>& possible_accelerations);

	/* Printing ----------------------------------------------------------- */
	static std::string mode_to_string(
		State active_mode);

private:
	State active_mode{ State::max_accel };
	
	double max_accel{ 0.0 }; // [m/s2]
	double comfortable_braking{ 0.0 }; // [m/s2] absolute value
	double gap_error{ 0.0 };  // [m] "gap error" considering relative velocity
	double h3{ 0.0 }, dht{ 0.0 }, dhx{ 0.0 };
	bool verbose{ false };

	/* Controller parameters ---------------------------------------------- */

	double time_headway{ 1.0 }; // [s]
	double standstill_distance{ 3.0 };  // [m]
	double veh_foll_gain{ 2.0 };
	double vel_control_gain{ 1.0 };
	double beta{ 4.0 };
	/* -------------------------------------------------------------------- */

	void compute_traffic_light_input_parameters(
		const TrafficLightACCVehicle& ego_vehicle,
		const std::unordered_map<int, TrafficLight>& traffic_lights);
	double compute_gap_error_to_next_traffic_light(
		double distance_to_traffic_light, double ego_vel);
	double compute_transient_safe_set(const TrafficLightACCVehicle& ego_vehicle,
		const std::unordered_map<int, TrafficLight>& traffic_lights);
	double choose_minimum_acceleration(
		std::unordered_map<State, double>& possible_accelerations);

	static const std::unordered_map<State, color_t> state_to_color;
};