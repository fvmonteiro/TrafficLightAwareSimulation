#include <iostream>

#include "EgoVehicle.h"
#include "LongitudinalControllerWithTrafficLights.h"
#include "TrafficLightACCVehicle.h"

LongitudinalControllerWithTrafficLights::
LongitudinalControllerWithTrafficLights(const EgoVehicle& ego_vehicle,
	bool verbose): 
	verbose {verbose}, 
	max_accel {ego_vehicle.get_comfortable_acceleration()},
	comfortable_braking {ego_vehicle.get_comfortable_brake()}
{
	if (verbose)
	{
		std::clog << "Creating traffic-light acc controller" << std::endl;
	}
}

bool LongitudinalControllerWithTrafficLights
::compute_vehicle_following_input(const EgoVehicle& ego_vehicle,
	std::unordered_map<State, double>& possible_accelerations)
{
	if (!ego_vehicle.has_leader()) return false;
	
	std::shared_ptr<NearbyVehicle> leader = ego_vehicle.get_leader();
	double gap = ego_vehicle.compute_gap(leader);
	double ego_vel = ego_vehicle.get_velocity();
	double rel_vel = leader->get_relative_velocity();
	double leader_vel = leader->compute_velocity(ego_vel);
	double safe_gap = time_headway * ego_vel + standstill_distance
		+ (std::pow(ego_vel, 2) - std::pow(leader_vel, 2)) / 2 / comfortable_braking;
	gap_error = gap - safe_gap;

	if (ego_vehicle.get_is_connected() && leader->is_connected())
	{
		double leader_accel = leader->get_acceleration();
		double connected_extra_term = leader_accel / comfortable_braking
			* leader_vel;
		possible_accelerations[State::vehicle_following] =
			(-rel_vel + veh_foll_gain * gap_error + connected_extra_term)
			* comfortable_braking / (comfortable_braking + ego_vel);
	}
	else
	{
		possible_accelerations[State::vehicle_following] =
			(-rel_vel + veh_foll_gain * gap_error)
			/ (time_headway + ego_vel / comfortable_braking);
	}
	return true;
}

bool LongitudinalControllerWithTrafficLights
::compute_velocity_control_input(const EgoVehicle& ego_vehicle,
	std::unordered_map<State, double>& possible_accelerations)
{
	double desired_vel = ego_vehicle.get_desired_velocity();

	double ego_vel = ego_vehicle.get_velocity();
	double vel_error = desired_vel - ego_vel;
	possible_accelerations[State::velocity_control] = 
		vel_control_gain * (vel_error);
	return true;
}

bool LongitudinalControllerWithTrafficLights
::compute_traffic_light_input(const TrafficLightACCVehicle& ego_vehicle,
	const std::unordered_map<int, TrafficLight>& traffic_lights,
	std::unordered_map<State, double>& possible_accelerations)
{
	if (!ego_vehicle.has_next_traffic_light()) return false;

	double ego_vel = ego_vehicle.get_velocity();
	compute_traffic_light_input_parameters(ego_vehicle, traffic_lights);

	if (verbose) std::clog << "beta=" << beta
		<< ", dht=" << dht << ", Vf=" << ego_vel << ", h3=" << h3
		<< std::endl;

	possible_accelerations[State::traffic_light] = 
		comfortable_braking / (beta * comfortable_braking + ego_vel)
		* (dht - ego_vel + h3);
	return true;
}

color_t LongitudinalControllerWithTrafficLights::get_state_color() const
{
	if (state_to_color.find(active_mode) == state_to_color.end())
		return WHITE;
	return state_to_color.at(active_mode);
}

double LongitudinalControllerWithTrafficLights::get_nominal_input(
	std::unordered_map<State, double>& possible_accelerations)
{
	possible_accelerations[State::max_accel] = max_accel;
	return true;
}

double LongitudinalControllerWithTrafficLights::choose_minimum_acceleration(
	std::unordered_map<State, double>& possible_accelerations)
{
	if (verbose) std::clog << "Getting min accel:" << "\n\t";

	double desired_acceleration = 1000; // any high value
	for (const auto& it : possible_accelerations)
	{
		if (verbose) std::clog << mode_to_string(it.first)
			<< "=" << it.second << ", ";

		if (it.second < desired_acceleration)
		{
			desired_acceleration = it.second;
			active_mode = it.first;
		}
	}

	if (verbose) std::clog << "\n";

	return desired_acceleration;
}

double LongitudinalControllerWithTrafficLights::choose_acceleration(
	const EgoVehicle& ego_vehicle,
	std::unordered_map<State, double>& possible_accelerations)
{
	double min_from_inputs =
		choose_minimum_acceleration(possible_accelerations);
	if (!ego_vehicle.has_leader())
	{
		return min_from_inputs;
	}

	double gap = ego_vehicle.compute_gap(ego_vehicle.get_leader());
	double ego_vel = ego_vehicle.get_velocity();
	double leader_vel = ego_vehicle.get_leader()->compute_velocity(ego_vel);
	//double gap_error = gap - time_headway * ego_vel - standstill_distance;
		
	double margin = 0.1; // 0 for connected
	if (gap_error >= -margin)
	{
		return min_from_inputs;
	}
	else
	{
		active_mode = State::too_close;
		return std::max(min_from_inputs,
			-ego_vehicle.get_max_brake());
	}
}

std::string LongitudinalControllerWithTrafficLights::mode_to_string(
	State active_mode)
{
	switch (active_mode)
	{
	case LongitudinalControllerWithTrafficLights::State::vehicle_following:
		return "vehicle following";
	case LongitudinalControllerWithTrafficLights::State::velocity_control:
		return "velocity control";
	case LongitudinalControllerWithTrafficLights::State::traffic_light:
		return "traffic light";
	case LongitudinalControllerWithTrafficLights::State::max_accel:
		return "nominal (max accel)";
		break;
	default:
		return "unknown mode";
		break;
	}
}

void LongitudinalControllerWithTrafficLights
::compute_traffic_light_input_parameters(
	const TrafficLightACCVehicle& ego_vehicle,
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	int next_traffic_light_id = ego_vehicle.get_next_traffic_light_id();
	if (next_traffic_light_id == 0) return;

	if (verbose) std::clog << "computing tf acc params" << std::endl;

	/* hx is like the safe gap/ safe distance to the traffic light */
	double hx = compute_gap_error_to_next_traffic_light(
		ego_vehicle.get_distance_to_next_traffic_light(), 
		ego_vehicle.get_velocity());
	
	/* ht is how the safe set varies over time */
	double ht = compute_transient_safe_set(ego_vehicle, traffic_lights);
	h3 = ht + hx;
}

double LongitudinalControllerWithTrafficLights::
compute_transient_safe_set(const TrafficLightACCVehicle& ego_vehicle,
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	int next_traffic_light_id = ego_vehicle.get_next_traffic_light_id();

	TrafficLight next_traffic_light =
		traffic_lights.at(next_traffic_light_id);
	double distance_between_traffic_lights;
	int next_next_traffic_light_id = next_traffic_light_id + 1;
	if (traffic_lights.find(next_next_traffic_light_id) !=
		traffic_lights.end())
	{
		distance_between_traffic_lights =
			traffic_lights.at(next_next_traffic_light_id).get_position()
			- next_traffic_light.get_position();
	}
	else
	{
		//Any large value
		distance_between_traffic_lights = 1000;
	}

	double ht;
	if (next_traffic_light.get_current_state() == TrafficLight::State::red)
	{
		ht = 0;
		dht = 0;
	}
	else
	{
		double lambda0 = beta * comfortable_braking;
		double time = ego_vehicle.get_time();
		double next_red_time = next_traffic_light.get_time_of_next_red();
		ht = -lambda0 * (time - next_red_time);
		dht = -lambda0;
		if (ht > distance_between_traffic_lights)
		{
			ht = distance_between_traffic_lights;
			dht = 0;
		}
	}
	return ht;
}

double LongitudinalControllerWithTrafficLights::
compute_gap_error_to_next_traffic_light(double distance_to_traffic_light,
	double ego_vel)
{
	/* hx is like the safe gap/ safe distance to the traffic light */
	double hx = distance_to_traffic_light - beta * ego_vel
		- standstill_distance
		- std::pow(ego_vel, 2) / 2 / comfortable_braking;
	return hx;
}

const std::unordered_map<
	LongitudinalControllerWithTrafficLights::State, color_t>
LongitudinalControllerWithTrafficLights::state_to_color{
		{State::max_accel, BLUE_GREEN},
		{State::too_close, RED},
		{State::traffic_light, YELLOW},
		{State::vehicle_following, DARK_GREEN},
		{State::velocity_control, DARK_GREEN},
};
