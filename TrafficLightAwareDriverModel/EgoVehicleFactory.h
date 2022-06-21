#pragma once

#include <memory>

#include "TrafficLightACCVehicle.h"

class EgoVehicleFactory
{
public:

	static std::unique_ptr<EgoVehicle> create_ego_vehicle(long id, int type, 
		double desired_velocity, double simulation_time_step, 
		double creation_time, bool verbose)
	{
		switch (VehicleType(type))
		{
		case VehicleType::traffic_light_acc_car:
			return std::make_unique<TrafficLightACCVehicle>(id,
				desired_velocity,
				simulation_time_step, creation_time, verbose);
		case VehicleType::traffic_light_cacc_car:
			return std::make_unique<TrafficLightCACCVehicle>(id,
				desired_velocity,
				simulation_time_step, creation_time, verbose);
		default:
			std::clog << "Trying to create unknown vehicle type\n" 
				<< "\ttime=" << creation_time
				<< "\tid=" << id 
				<< "\ttype" << type
				<< std::endl;
			return nullptr;
		}
	}
};

