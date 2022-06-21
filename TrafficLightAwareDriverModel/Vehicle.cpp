#include <iomanip>
//#include <iostream>

#include "Vehicle.h"

Vehicle::Vehicle(long id) : id{ id } {}

Vehicle::Vehicle(long id, VehicleType type) :
	id{ id }, type{ type } 
{
	set_category(static_cast<int>(type) / 100);
}

void Vehicle::set_category(long category) 
{
	/* We only need to set the category once, but VISSIM passes the
	category every time step. */
	if (this->category == VehicleCategory::undefined) 
	{
		this->category = VehicleCategory(category);
		switch (this->category) 
		{
		case VehicleCategory::truck:
			this->max_brake = TRUCK_MAX_BRAKE;
			break;
		case VehicleCategory::car:
			this->max_brake = CAR_MAX_BRAKE;
			break;
		default: // shouldn't happen but assume car if any other category
			this->max_brake = CAR_MAX_BRAKE;
			break;
		}
	}
}

bool Vehicle::is_a_connected_type(VehicleType vehicle_type) const 
{
	return vehicle_type == VehicleType::traffic_light_cacc_car;
}

bool Vehicle::has_lane_change_intention() const 
{
	return desired_lane_change_direction != RelativeLane::same;
}
