/*==========================================================================*/
/*  TrafficLight.h	    													*/
/*  Representation of traffic lights.										*/
/*                                                                          */
/*  Version of 2022-06	                              Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <iostream>
#include <string>
#include <vector>

class TrafficLight
{
public:

	enum class State {
		no_traffic_light= 0,
		red=1,
		amber=2,
		green=3
	};

	TrafficLight() = default;
	TrafficLight(int id, double position, double red_duration,
		double green_duration, double amber_duration, bool starts_on_red);
	TrafficLight(int id, double position, double red_duration,
		double green_duration, double amber_duration);

	int get_id() const { return id; };
	double get_position() const { return position; };
	double get_amber_duration() const { return amber_duration; };
	State get_current_state() const { return current_state; };

	void set_current_state(long state) { current_state = State(state); };
	void set_current_state_start_time(double time) { 
		current_state_start_time = time; 
	};

	double get_time_of_next_red() const;
	double get_time_of_last_amber() const;
	double get_time_of_last_green() const;
	double get_time_of_next_green() const;

	friend std::ostream& operator<< (std::ostream& out,
		const TrafficLight& traffic_light);

private:
	// Static parameters
	int id{ 0 };
	double position{ 0 }, red_duration{ 0 }, green_duration{ 0 },
		amber_duration{ 0 };
	bool starts_on_red{ true };
	// State
	State current_state{ State::no_traffic_light };
	double current_state_start_time{ 0.0 };
};

