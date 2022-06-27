/*==========================================================================*/
/*  EgoVehicle.h	    													*/
/*  Base class for partially or fully automated vehicles					*/
/*                                                                          */
/*  Version of 2022-06	                              Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <memory>
#include <vector>

#include "ControlManager.h"
#include "NearbyVehicle.h"
#include "TrafficLight.h"
#include "Vehicle.h"


class EgoVehicle : public Vehicle {
public:

	enum class State {
		lane_keeping,
		intention_to_change_lanes,
	};

	/* Constructor and Destructor ----------------------------------------- */
	EgoVehicle() = default;
	virtual ~EgoVehicle();

	/* Getters and setters ------------------------------------------------ */

	double get_sampling_interval() const { return simulation_time_step; };
	long get_color() const { return color; };
	double get_desired_velocity() const { return desired_velocity; };
	double get_comfortable_acceleration() const { 
		return comfortable_acceleration;
	};
	double get_comfortable_brake() const { return comfortable_brake; };
	double get_desired_lane_angle() const { return desired_lane_angle; };
	int get_relative_target_lane() const { 
		return relative_target_lane.to_int();
	};
	long get_turning_indicator() const { return turning_indicator; };
	long get_vissim_use_preferred_lane() const { 
		return vissim_use_preferred_lane; 
	};
	bool get_is_connected() const { return is_connected; };

	void set_desired_velocity(double desired_velocity) {
		this->desired_velocity = desired_velocity;
	};
	void set_desired_lane_angle(double desired_lane_angle) {
		this->desired_lane_angle = desired_lane_angle;
	};
	void set_turning_indicator(long turning_indicator) {
		this->turning_indicator = turning_indicator;
	};
	void set_vissim_use_preferred_lane(long value) {
		this->vissim_use_preferred_lane = value;
	};

	/* Getters of most recent values -------------------------------------- */
	
	double get_time() const;
	long get_lane() const;
	long get_link() const;
	double get_lateral_position() const;
	RelativeLane get_preferred_relative_lane() const;
	double get_velocity() const;
	double get_acceleration() const;
	double get_desired_acceleration() const;
	double get_vissim_acceleration() const;
	RelativeLane get_active_lane_change_direction() const;
	double get_lane_end_distance() const;
	long get_leader_id() const;
	State get_state() const;

	/* Other getters and setters ------------------------------------------ */

	void set_lane(long lane);
	void set_link(long link);
	void set_lateral_position(double lateral_position);
	void set_velocity(double velocity);
	void set_acceleration(double acceleration);
	void set_vissim_acceleration(double vissim_acceleration);
	void set_active_lane_change_direction(long direction);
	/* Mandatory/route related lane changes */
	void set_preferred_relative_lane(long preferred_relative_lane);
	/* Discretionary lane changes */
	void set_relative_target_lane(long target_relative_lane);
	void set_lane_end_distance(double lane_end_distance,
		long lane_number);
	void read_traffic_light(int traffic_light_id, double distance)
	{
		set_traffic_light_information(traffic_light_id, distance);
	}

	/* Dealing with nearby vehicles --------------------------------------- */

	/* Clears the vector of pointers */
	void clear_nearby_vehicles();
	/* Creates an instance of nearby vehicle and populates it with 
	the given data. */
	void emplace_nearby_vehicle(long id, long relative_lane,
		long relative_position);
	/* Returns the most recently added nearby vehicle */
	std::shared_ptr<NearbyVehicle> peek_nearby_vehicles() const;
	/* Sets the nearby vehicle type */
	void set_nearby_vehicle_type(long type);
	/* Looks at nearby vehicles to find the relevant ones, such 
	as the leader. */
	void analyze_nearby_vehicles()
	{
		find_relevant_nearby_vehicles();
	};
	bool has_leader() const;
	/* Returns a nullptr if there is no leader */
	std::shared_ptr<NearbyVehicle> get_leader() const;
	std::shared_ptr<NearbyVehicle> get_nearby_vehicle_by_id(long nv_id) const;
	/* Computes the bumper-to-bumper distance between vehicles.
	Returns MAX_DISTANCE if nearby_vehicle is empty. */
	double compute_gap(const NearbyVehicle& nearby_vehicle) const;
	/* Computes the bumper-to-bumper distance between vehicles.
	Returns MAX_DISTANCE if nearby_vehicle is a nullptr. */
	double compute_gap(
		const std::shared_ptr<NearbyVehicle> nearby_vehicle) const;
	/* Ego velocity minus leader velocity. Returns zero if there
	is no leader */
	double get_relative_velocity_to_leader();

	/* State-machine related methods ----------------------------------------- */

	void update_state();
	bool is_lane_changing() const override;

	/* Returns the color equivalent to the current state as a long */
	long get_color_by_controller_state();

	/* Control related methods ----------------------------------------------- */

	double get_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights)
	{
		return compute_desired_acceleration(traffic_lights);
	};

	long decide_lane_change_direction();

	/* Methods for logging --------------------------------------------------- */
	bool is_verbose() const { return verbose; };

	/* Print function */
	friend std::ostream& operator<< (std::ostream& out, 
		const EgoVehicle& vehicle);

protected:
	EgoVehicle(long id, VehicleType type, double desired_velocity,
		bool is_lane_change_autonomous, bool is_connected,
		double simulation_time_step, double creation_time, bool verbose);

	ControlManager controller;

	/* Nearby vehicles ------------------------------------------------------- */

	void find_leader();
	std::vector<std::shared_ptr<NearbyVehicle>> nearby_vehicles;

	bool verbose = false; /* used in several parts of the code to print out 
						  vehicle information during tests. */

private:
	/* Computes the longitudinal controller input */
	virtual double compute_desired_acceleration(
		const std::unordered_map<int, TrafficLight>& traffic_lights) = 0;
	virtual bool can_start_lane_change() = 0;
	virtual void set_traffic_light_information(int traffic_light_id,
		double distance) {};
	
	/* Finds the current leader */
	virtual void find_relevant_nearby_vehicles();
	void set_desired_lane_change_direction();

	bool check_if_is_leader(const NearbyVehicle& nearby_vehicle) const;

	/* Estimated parameters used for safe gap computations (no direct 
	equivalent in VISSIM's simulation dynamics) --------------------------- */
	
	double comfortable_brake{ COMFORTABLE_BRAKE }; // [m/s^2]

	std::shared_ptr<NearbyVehicle> leader{ nullptr };
	std::vector<long> leader_id;

	/* Data obtained from VISSIM or generated by internal computations ---- */
	double creation_time{ 0.0 };
	double simulation_time_step{ 0.1 };
	long color{ 0 };
	double desired_velocity{ 0 }; /* from VISSIM's desired 
								  velocity distribution */
	std::vector<long> lane;
	std::vector<long> link;
	std::vector<RelativeLane> preferred_relative_lane;
	/* 0 = only preferable (e.g. European highway)
	   1 = necessary (e.g. before a connector)     */
	long vissim_use_preferred_lane{ 0 };
	/* distance of the front end from the middle of the lane [m]
	(positive = left of the middle, negative = right) */
	std::vector<double> lateral_position;
	std::vector<double> velocity;
	std::vector<double> acceleration;
	std::vector<double> desired_acceleration;
	/* VISSIM suggested acceleration */
	std::vector<double> vissim_acceleration;
	/* +1 = to the left, 0 = none, -1 = to the right */
	std::vector<RelativeLane> active_lane_change_direction;
	/* Determines if we use our lane change decision model or VISSIM's */
	bool is_lane_change_autonomous{ true };
	bool is_connected{ false };
	/* Distance to the end of the lane. Used to avoid missing exits in case
	vehicle couldn't lane change earlier. */
	std::vector<double> lane_end_distance;
	std::vector<State> state;
	double desired_lane_angle{ 0.0 };
	RelativeLane relative_target_lane{ RelativeLane::same };
	long turning_indicator{ 0 };
	
	/* For printing and debugging purporses ------------------------------- */
	static const std::unordered_map<State, std::string> state_to_string_map;

	std::string log_path = "autonomous_vehicle_logs";
	enum class Member 
	{
		creation_time,
		id,
		length,
		width,
		category,
		desired_velocity,
		lane,
		link,
		preferred_relative_lane,
		velocity,
		acceleration,
		desired_acceleration,
		vissim_acceleration,
		leader_id,
		state,
		active_lane_change_direction,
		lane_end_distance,
		type,
	};

	void write_simulation_log(std::vector<Member> members);
	std::string write_header(std::vector<Member> members,
		bool write_size = false);
	std::string write_members(std::vector<Member> members);
	int get_member_size(Member member);
	std::string member_enum_to_string(Member member);
};