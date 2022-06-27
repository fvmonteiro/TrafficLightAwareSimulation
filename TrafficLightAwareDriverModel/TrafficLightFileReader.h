/*==========================================================================*/
/*  TrafficLightFileReader.h												*/
/*  Does the interface between the data in a CSV file and the code.			*/
/*                                                                          */
/*  Version of 2022-06	                              Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

#include <string>
#include <unordered_map>

#include "TrafficLight.h"

/* This class implements a simple CSV reader, which get data from the
given address and stores it in TrafficLight objects.*/
class TrafficLightFileReader
{
public:
	static void from_file_to_objects(std::string full_address, 
		std::unordered_map<int, TrafficLight>& traffic_lights);
};

