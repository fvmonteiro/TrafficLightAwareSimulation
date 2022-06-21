

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "ControlManager.h"
#include "EgoVehicle.h"

EgoVehicle::EgoVehicle(long id, VehicleType type, double desired_velocity,
	bool is_lane_change_autonomous, bool is_connected,
	double simulation_time_step, double creation_time, bool verbose) :
	Vehicle(id, type),
	desired_velocity{ desired_velocity },
	is_lane_change_autonomous { is_lane_change_autonomous },
	is_connected { is_connected },
	simulation_time_step{ simulation_time_step },
	creation_time{ creation_time },
	verbose{ verbose }
{
	this->controller = ControlManager(*this, verbose);
	if (verbose)
	{
		std::clog << "Creating vehicle " << get_id()
			<< " at time " << this->creation_time
			<< ", category " << static_cast<int>(category)
			<< ", type " << static_cast<int>(get_type())
			<< ", des. vel. = " << desired_velocity
			<< std::endl;
	}
}

EgoVehicle::~EgoVehicle() 
{
	std::vector<Member> members{
		Member::creation_time,
		Member::preferred_relative_lane,
		Member::state,
		Member::velocity,
		Member::desired_acceleration,
		Member::active_lane_change_direction,
		Member::leader_id,
	};
	if (verbose) 
	{
		std::clog << write_header(members, true);
		std::clog << "Vehicle " << get_id()
			<< " out of the simulation at time "
			<< get_time() << std::endl;
	}
}

/* "Current" getters ------------------------------------------------------ */

double EgoVehicle::get_time() const 
{
	/* At creation time, the vehicle already has a velocity */
	return creation_time + (velocity.size() - 1) * simulation_time_step; 
}
long EgoVehicle::get_lane() const 
{
	return lane.back(); 
}
long EgoVehicle::get_link() const 
{
	return link.back();
}
double EgoVehicle::get_lateral_position() const 
{
	return lateral_position.back();
}
RelativeLane EgoVehicle::get_preferred_relative_lane() const 
{
	return preferred_relative_lane.back();
}
double EgoVehicle::get_velocity() const 
{
	if (velocity.empty()) /* We shouldn't need to check this condition. 
							This is used to avoid crashing during tests.*/
	{ 
		return 0;
	}
	return velocity.back(); 
}
double EgoVehicle::get_acceleration() const 
{
	return acceleration.back(); 
}
double EgoVehicle::get_desired_acceleration() const 
{
	return desired_acceleration.empty()? 0: desired_acceleration.back();
}
double EgoVehicle::get_vissim_acceleration() const 
{
	return vissim_acceleration.back();
}
RelativeLane EgoVehicle::get_active_lane_change_direction() const 
{
	return active_lane_change_direction.back();
}
//long EgoVehicle::get_vissim_active_lane_change() const {
//	return vissim_active_lane_change.back();
//}
double EgoVehicle::get_lane_end_distance() const 
{
	return lane_end_distance.back();
}
long EgoVehicle::get_leader_id() const 
{
	return leader_id.back();
}
EgoVehicle::State EgoVehicle::get_state() const 
{
	return state.empty() ? State::lane_keeping : state.back();
}
/* ------------------------------------------------------------------------ */

/* Other getters and setters ---------------------------------------------- */

void EgoVehicle::set_lane(long lane) 
{
	this->lane.push_back(lane);
}
void EgoVehicle::set_link(long link) 
{
	this->link.push_back(link);
}
void EgoVehicle::set_lateral_position(double lateral_position) 
{
	this->lateral_position.push_back(lateral_position);
}
void EgoVehicle::set_velocity(double velocity) 
{
	this->velocity.push_back(velocity);
}
void EgoVehicle::set_acceleration(double acceleration) 
{
	this->acceleration.push_back(acceleration);
}
void EgoVehicle::set_vissim_acceleration(double vissim_acceleration) 
{
	this->vissim_acceleration.push_back(vissim_acceleration);
}
void EgoVehicle::set_active_lane_change_direction(long direction) 
{
	this->active_lane_change_direction.push_back(
		RelativeLane::from_long(direction));
}

void EgoVehicle::set_preferred_relative_lane(long preferred_relative_lane) 
{
	this->preferred_relative_lane.push_back(
		RelativeLane::from_long(preferred_relative_lane));
	//set_desired_lane_change_direction(preferred_relative_lane);	
}

void EgoVehicle::set_relative_target_lane(long target_relative_lane) 
{
	this->relative_target_lane = 
		RelativeLane::from_long(target_relative_lane);
	//set_desired_lane_change_direction(target_relative_lane);
}

void EgoVehicle::set_lane_end_distance(double lane_end_distance,
	long lane_number) 
{
	if (lane_number == get_lane()) 
	{
		this->lane_end_distance.push_back(lane_end_distance);
	}
}

/* Nearby Vehicles methods ------------------------------------------------ */

void EgoVehicle::clear_nearby_vehicles() 
{
	nearby_vehicles.clear();
}

void EgoVehicle::emplace_nearby_vehicle(long id, long relative_lane,
	long relative_position) 
{
	/*if (verbose && get_time() > 68) std::clog << "Emplacing nv id=" << id
		<< std::endl;*/
	std::shared_ptr<NearbyVehicle> nearby_vehicle = 
		std::shared_ptr<NearbyVehicle>(new NearbyVehicle(id, relative_lane,
		relative_position));
	nearby_vehicles.push_back(std::move(nearby_vehicle));
}

std::shared_ptr<NearbyVehicle> EgoVehicle::peek_nearby_vehicles() const 
{
	if (!nearby_vehicles.empty()) 
	{
		return nearby_vehicles.back();
	}
	std::clog << "Empty nearby_vehicles container in vehicle  " << get_id()
		<< std::endl;
	return nullptr;
}

void EgoVehicle::set_nearby_vehicle_type(long nv_type)
{
	peek_nearby_vehicles()->set_type(VehicleType(nv_type), type);
}

bool EgoVehicle::has_leader() const 
{
	return leader != nullptr;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_leader() const 
{
	return leader;
}

std::shared_ptr<NearbyVehicle> EgoVehicle::get_nearby_vehicle_by_id(
	long nv_id) const 
{
	for (std::shared_ptr<NearbyVehicle> nv : nearby_vehicles) 
	{
		if (nv->get_id() == nv_id) return nv;
	}
	
	/* If we don't find the id in the current nearby vehicle list, 
	the vehicle is way behind us. In this case, we just create a far away 
	virtual vehicle to force the ego vehicle into low vel. control mode */
	/*std::shared_ptr<NearbyVehicle> virtual_vehicle =
		std::shared_ptr<NearbyVehicle>(new
			NearbyVehicle(nv_id, RelativeLane::same, -3));
	virtual_vehicle->set_relative_velocity(
		nv_vel);
	virtual_vehicle->set_distance();*/
	return nullptr;
}

double EgoVehicle::get_relative_velocity_to_leader() 
{
	return has_leader() ? leader->get_relative_velocity() : 0.0;
}

double EgoVehicle::compute_gap(const NearbyVehicle& nearby_vehicle) const 
{
	/* Vissim's given "distance" is the distance between both front bumpers, 
	so we subtract the length from that. We need to check which vehicle is 
	ahead to determine whose length must be subtracted. */
	if (nearby_vehicle.get_id() <= 0) // "empty" vehicle
	{ 
		return MAX_DISTANCE;
	}
	if (nearby_vehicle.is_ahead()) 
	{
		return nearby_vehicle.get_distance()
			- nearby_vehicle.get_length();
	}
	else 
	{
		return -nearby_vehicle.get_distance() - get_length();
	}
}

double EgoVehicle::compute_gap(
	const std::shared_ptr<NearbyVehicle> nearby_vehicle) const 
{
	if (nearby_vehicle != nullptr) 
	{
		return compute_gap(*nearby_vehicle);
	}
	else 
	{
		return MAX_DISTANCE;
	}
}

void EgoVehicle::find_relevant_nearby_vehicles()
{
	find_leader();
}

void EgoVehicle::find_leader()
{
	std::shared_ptr<NearbyVehicle> old_leader = std::move(leader);
	
	for (auto& nearby_vehicle : nearby_vehicles)
	{
		if (check_if_is_leader(*nearby_vehicle)) leader = nearby_vehicle;
	}
	leader_id.push_back(has_leader() ? leader->get_id() : 0);
}

bool EgoVehicle::check_if_is_leader(const NearbyVehicle& nearby_vehicle) const
{
	if ((nearby_vehicle.is_immediatly_ahead()
		&& nearby_vehicle.is_on_same_lane())
		|| nearby_vehicle.is_cutting_in())
	{
		if (!has_leader()
			|| (nearby_vehicle.get_distance() < leader->get_distance()))
		{
			return true;
		}
	}
	return false;
}

/* State-machine related methods ------------------------------------------ */

void EgoVehicle::update_state() 
{
	set_desired_lane_change_direction();

	State old_state = get_state();
	if (desired_lane_change_direction == RelativeLane::same) 
	{
		state.push_back(State::lane_keeping);
	}
	else 
	{
		state.push_back(State::intention_to_change_lanes);
	}

	/* State change: */
	if (old_state != get_state()) 
	{
		switch (get_state())
		{
		case State::intention_to_change_lanes:
			if (verbose)
			{
				std::clog << "Transition from lane keeping to "
					<< "lane changing" << std::endl;
			}
			break;
		case State::lane_keeping:
			if (verbose)
			{
				std::clog << "Transition from lane changing to "
					<< "lane keeping" << std::endl;
			}
			break;
		default:
			break;
		}
	}
}


bool EgoVehicle::is_lane_changing() const 
{
	return get_active_lane_change_direction() != RelativeLane::same;
}

long EgoVehicle::get_color_by_controller_state() 
{
	return controller.get_longitudinal_controller_color();
}

/* Control related methods ------------------------------------------------ */

long EgoVehicle::decide_lane_change_direction()
{
	if (has_lane_change_intention() && can_start_lane_change())
	{
		return get_desired_lane_change_direction();
	}
	return 0;
}

/* Private methods -------------------------------------------------------- */

void EgoVehicle::set_desired_lane_change_direction() 
{
	/* Both preferred_relative_lane and relative_target_lane indicate
	desire to change lanes. The former indicates preference due to 
	routing, so it takes precedence over the latter. */
	RelativeLane current_preferred_lane = get_preferred_relative_lane();
	desired_lane_change_direction = RelativeLane::same;
	if (current_preferred_lane.is_to_the_left()) 
	{
		desired_lane_change_direction = RelativeLane::left;
	}
	else if (current_preferred_lane.is_to_the_right()) 
	{
		desired_lane_change_direction = RelativeLane::right;
	}
	else if (relative_target_lane.is_to_the_left()) 
	{
		desired_lane_change_direction = RelativeLane::left;
	}
	else if (relative_target_lane.is_to_the_right()) 
	{
		desired_lane_change_direction = RelativeLane::right;
	}
	else 
	{
		desired_lane_change_direction = RelativeLane::same;
	}
}

/* Methods for printing and debugging ------------------------------------- */

void EgoVehicle::write_simulation_log(std::vector<Member> members)
{
	bool write_size = true;
	std::ofstream vehicle_log;
	std::string file_name = "vehicle" + std::to_string(get_id()) + ".txt";
	vehicle_log.open(log_path + "\\" + file_name);
	if (vehicle_log.is_open()) 
	{
		vehicle_log << write_header(members, write_size);
		vehicle_log	<< write_members(members);
		vehicle_log.close();
	}
	else 
	{
		std::clog << "Unable to open file to write log of "
			<< "vehicle " << get_id()
			<< std::endl;
	}
}

std::string EgoVehicle::write_header(
	std::vector<EgoVehicle::Member> members, bool write_size) 
{
	
	std::ostringstream oss;
	for (auto m : members) 
	{
		oss << member_enum_to_string(m);
		if (write_size) 
		{
			oss << " (" << get_member_size(m) << ")";
		}
		oss << ", ";
	}
	oss << std::endl;

	return oss.str();
}

std::string EgoVehicle::write_members(
	std::vector<EgoVehicle::Member> members) 
{

	std::ostringstream oss;

	/* Sanity check: some non critical code mistakes could
	create vector members with different sizes. This prevents
	the log from being written and crashes vissim. Let's avoid
	this and just write a warning message instead.*/
	int n_samples = (int)velocity.size(); /* velocity, lane and link members 
	are the least likely to have the wrong size */
	std::vector<int> deleted_indices;
	for (int i = 0; i < members.size(); i++) 
	{
		Member m = members.at(i);
		if ((get_member_size(m) != 1) // not a scalar
			&& (get_member_size(m) != n_samples)) 
		{
			oss << "Error: member " << member_enum_to_string(m)
				<< " has " << get_member_size(m) << " samples "
				<< "instead of the expected " << n_samples
				<< ". It won't be printed."
				<< std::endl;
			deleted_indices.push_back(i);
		}
	}
	for (int idx : deleted_indices) 
	{
		members.erase(std::next(members.begin(), idx));
	}

	// Write variables over time
	for (int i = 0; i < n_samples; i++) 
	{
		for (auto m : members) 
		{
			switch (m)
			{
			case Member::creation_time:
				oss << creation_time + i * simulation_time_step;
				break;
			case Member::id:
				oss << get_id();
				break;
			case Member::length:
				oss << get_length();
				break;
			case Member::width:
				oss << get_width();
				break;
			case Member::category:
				oss << static_cast<int>(category);
				break;
			case Member::desired_velocity:
				oss << desired_velocity;
				break;
			case Member::lane:
				oss << lane[i];
				break;
			case Member::link:
				oss << link[i];
				break;
			case Member::preferred_relative_lane:
				oss << preferred_relative_lane[i].to_string();
				break;
			case Member::velocity:
				oss << velocity[i];
				break;
			case Member::acceleration:
				oss << acceleration[i];
				break;
			case Member::desired_acceleration:
				oss << desired_acceleration[i];
				break;
			case Member::vissim_acceleration:
				oss << vissim_acceleration[i];
				break;
			case Member::leader_id:
				oss << leader_id[i];
				break;
			case Member::state:
				oss << state_to_string_map.at(state[i]);
				break;
			case Member::active_lane_change_direction:
				oss << active_lane_change_direction[i].to_string();
				break;
			case Member::lane_end_distance:
				oss << lane_end_distance[i];
				break;
			case Member::type:
				oss << static_cast<int>(get_type());
				break;
			default:
				oss << "";
				break;
			}
			oss << ", ";
		}
		oss << std::endl;
	}
	return oss.str();
}

int EgoVehicle::get_member_size(Member member) 
{
	switch (member)
	{
	case Member::creation_time:
	case Member::id:
	case Member::length:
	case Member::width:
	case Member::category:
	case Member::desired_velocity:
	case Member::type:
		return 1;
	case Member::lane:
		return (int)lane.size();
	case Member::link:
		return (int)link.size();
	case Member::preferred_relative_lane:
		return (int)preferred_relative_lane.size();
	case Member::velocity:
		return (int)velocity.size();
	case Member::acceleration:
		return (int)acceleration.size();
	case Member::desired_acceleration:
		return (int)desired_acceleration.size();
	case Member::vissim_acceleration:
		return (int)vissim_acceleration.size();
	case Member::leader_id:
		return (int)leader_id.size();
	case Member::state:
		return (int)state.size();
	case Member::active_lane_change_direction:
		return (int)active_lane_change_direction.size();
	case Member::lane_end_distance:
		return (int)lane_end_distance.size();
	default:
		return 0;
	}
}

std::string EgoVehicle::member_enum_to_string(Member member) 
{
	switch (member)
	{
	case Member::creation_time:
		return "creation_time";
	case Member::id:
		return "id";
	case Member::length:
		return "length";
	case Member::width:
		return "width";
	case Member::category:
		return "category";
	case Member::desired_velocity:
		return "des. vel.";
	case Member::lane:
		return "lane";
	case Member::link:
		return "link";
	case Member::preferred_relative_lane:
		return "pref. rel. lane";
	case Member::velocity:
		return "vel.";
	case Member::acceleration:
		return "accel.";
	case Member::desired_acceleration:
		return "des. accel.";
	case Member::vissim_acceleration:
		return "vissim accel.";
	case Member::leader_id:
		return "leader id";
	case Member::state:
		return "state";
	case Member::active_lane_change_direction:
		return "lc direction";
	case Member::lane_end_distance:
		return "lane end dist";
	default:
		return "unknown memeber";
	}
}


const std::unordered_map<EgoVehicle::State, std::string>
EgoVehicle::state_to_string_map = 
{
	{ State::lane_keeping, "lane keeping" },
	{ State::intention_to_change_lanes, "intention to LC" },
};

std::ostream& operator<< (std::ostream& out, const EgoVehicle& vehicle)
{
	out << "t=" << vehicle.get_time()
		<< ", id=" << vehicle.get_id()
		<< ", type=" << static_cast<int>(vehicle.get_type())
		<< ", state=" 
		<< EgoVehicle::state_to_string_map.at(vehicle.get_state())
		<< ", lane=" << vehicle.get_lane()
		<< ", pref. lane="
		<< vehicle.get_preferred_relative_lane().to_string()
		<< ", use preferred lane="
		<< vehicle.get_vissim_use_preferred_lane()
		<< ", target lane="
		<< vehicle.relative_target_lane.to_string()
		<< ", active lc.="
		<< vehicle.get_active_lane_change_direction().to_string()
		<< ", vel=" << vehicle.get_velocity()
		<< ", accel=" << vehicle.get_acceleration();

	return out;
}
