/*==========================================================================*/
/*  Vehicle.h		    													*/
/*  Base class for all simulation vehicles									*/
/*                                                                          */
/*  Version of 2022-06	                              Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <vector>

#include "Constants.h"
#include "RelativeLane.h"

class Vehicle
{
public:
	Vehicle(long id);
	Vehicle(long id, VehicleType type);

	/* Getters and setters */

	double get_max_brake() const { return max_brake; };
	long get_id() const { return id; };
	double get_length() const { return length; };
	double get_width() const { return width; };
	VehicleCategory get_category() const { return category; };
	VehicleType get_type() const { return type; };
	long get_desired_lane_change_direction() const {
		return desired_lane_change_direction.to_int();
	};

	void set_length(double length) { this->length = length; };
	void set_width(double width) { this->width = width; };
	/* Also sets the estimated maximum braking of the vehicle. */
	void set_category(long category);
	
	bool has_lane_change_intention() const;

protected:
	virtual ~Vehicle() {};

	bool is_a_connected_type(VehicleType vehicle_type) const;

	double max_brake{ 0.0 }; // [m/s^2]
	double comfortable_acceleration{ COMFORTABLE_ACCELERATION }; // [m/s^2]

	VehicleCategory category{ VehicleCategory::undefined };
	VehicleType type{ VehicleType::undefined };

	RelativeLane desired_lane_change_direction{ RelativeLane::same };

private:
	virtual bool is_lane_changing() const = 0;

	long id{ 0 };
	double length{ 0.0 }; // [m]
	double width{ 0.0 }; // [m]
};

