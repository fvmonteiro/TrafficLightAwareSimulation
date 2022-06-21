#include <iomanip>
#include <iostream>
#include <vector>
#include <sstream>

#include "Constants.h"
#include "NearbyVehicle.h"

NearbyVehicle::NearbyVehicle(long id, RelativeLane relative_lane,
	long relative_position) :
	Vehicle(id),
	relative_lane{ relative_lane },
	relative_position{ relative_position } {}

NearbyVehicle::NearbyVehicle(long id, long relative_lane,
	long relative_position) :
	NearbyVehicle(id, RelativeLane::from_long(relative_lane),
		relative_position) {}

void NearbyVehicle::set_type(VehicleType nv_type, VehicleType ego_type)
{
	if (is_a_connected_type(ego_type) && is_a_connected_type(nv_type))
	{
		this->type = nv_type;
	}
	else
	{
		/* We assume autonomous vehicles are identifiable visually by other
		autonomous vehicles without need for communications. 
		Both CAVs and AVs can differentiate HDVs from all the rest. */
		switch (nv_type)
		{
		case VehicleType::traffic_light_cacc_car:
		case VehicleType::traffic_light_acc_car:
			this->type = VehicleType::traffic_light_acc_car;
			break;
		default:
			this->type = VehicleType::human_driven_car;
			break;
		}
	}
}

bool NearbyVehicle::is_connected() const 
{
	/* The nearby vehicle type is only set to connected if the ego vehicle
	is also connected. So this function returns false when called by a non
	connected vehicle. */
	return type == VehicleType::traffic_light_cacc_car;
}

double NearbyVehicle::compute_velocity(double ego_velocity) const {
	return ego_velocity - relative_velocity;
}

bool NearbyVehicle::is_on_same_lane() const {
	return get_relative_lane() == RelativeLane::same;
}

bool NearbyVehicle::is_immediatly_ahead() const {
	return get_relative_position() == 1;
}

bool NearbyVehicle::is_immediatly_behind() const {
	return get_relative_position() == -1;
}

bool NearbyVehicle::is_ahead() const {
	return get_relative_position() > 0;
}

bool NearbyVehicle::is_behind() const {
	return !is_ahead();
}

bool NearbyVehicle::is_lane_changing() const {
	return lane_change_direction != RelativeLane::same;
}

bool NearbyVehicle::is_cutting_in() const {
	if (is_ahead() && (is_lane_changing()))
	{
		/* The nearby vehicle must be changing lanes towards the ego vehicle
		(that's the first part of the condition below)
		The first condition alone could misidentify the case where a vehicle
		two lanes away moves to an adjacent lane as a cut in. Therefore
		we must check whether the lateral position (with respect to the
		lane center) and the lane change direction have the same sign. */
		bool moving_into_my_lane =
			(relative_lane
				== lane_change_direction.get_opposite())
			&& ((get_lateral_position()
				* lane_change_direction.to_int()) > 0);
		if (moving_into_my_lane) return true;
	}
	return false;
}

std::string NearbyVehicle::to_string() const {
	std::ostringstream oss;

	std::vector<Member> printed_members = {
		Member::id, Member::relative_lane, Member::relative_position,
		Member::lateral_position, Member::distance, Member::relative_velocity,
		Member::lane_change_direction
	};

	for (Member m : printed_members) {
		oss << member_to_string.at(m) << "=";
		switch (m) {
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
			oss << static_cast<int>(get_category());
			break;
		case Member::type:
			oss << static_cast<int>(get_type());
			break;
		case Member::relative_lane:
			oss << relative_lane.to_string();
			break;
		case Member::relative_position:
			oss << relative_position;
			break;
		case Member::lateral_position:
			oss << lateral_position;
			break;
		case Member::distance:
			oss << distance;
			break;
		case Member::relative_velocity:
			oss << relative_velocity;
			break;
		case Member::acceleration:
			oss << acceleration;
			break;
		case Member::lane_change_direction:
			oss << lane_change_direction.to_string();
			break;
		default:
			oss << "unknown class member";
			break;
		}
		oss << ", ";
	}
	return oss.str();
}

std::ostream& operator<<(std::ostream& out, const NearbyVehicle& vehicle)
{
	out << vehicle.to_string();
	return out; // return std::ostream so we can chain calls to operator<<
}
