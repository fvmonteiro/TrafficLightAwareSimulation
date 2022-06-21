#include "TrafficLightACCVehicle.h"

bool TrafficLightACCVehicle::has_next_traffic_light() const {
	return next_traffic_light_id != 0;
}

void TrafficLightACCVehicle::set_traffic_light_information(
	int traffic_light_id, double distance)
{
	if (has_next_traffic_light()
		&& (traffic_light_id != next_traffic_light_id))
	{
		time_crossed_last_traffic_light = get_time();
	}
	next_traffic_light_id = traffic_light_id;
	distance_to_next_traffic_light = distance;
}

double TrafficLightACCVehicle::compute_desired_acceleration(
	const std::unordered_map<int, TrafficLight>& traffic_lights)
{
	double desired_acceleration =
		controller.get_traffic_light_acc_acceleration(*this, traffic_lights);
	return desired_acceleration;
}