# TrafficLightAwareSimulation
 
Files used to obtain results presented in: M. Waqas, F.V. Monteiro, and P. Ioannou, "Trade-off Between Safety and Traffic Flow for Connected Autonomous Vehicles in the Presence of Traffic Signals", IEEE ITSC 2022.

This code implements a provably safe longitudinal vehicle controller that respects traffic lights for testing in the simulation software VISSIM.

Structure:
- TrafficLightAwareDriverModel (DLL code):
	- Constants: defines some values used throughout the code
	- ControlManager: manages the controllers used by autonomous vehicles
	- DriverModel: does the interface (reading and writing values) between VISSIM and the external driver model. The skeleton of this file is provided together with VISSIM.
	- EgoVehicle: stores data and describes behavior of automated vehicles
	- EgoVehicleFactory: simple factory to create different ego vehicle subclasses
	- LongitudinalControllerWithTrafficLights: provably safe longitudinal vehicle controller that respects traffic lights
	- NearbyVehicle: manages neighboring vehicles
	- RelativeLane: helps dealing with relative lanes in a more intuitive way
	- SimulationLogger: helps in the creation of log files
	- TrafficLight: represents traffic lights
	- TrafficLightACCVehicle: implements the EgoVehicle class using the proposed longitudinal controllers (with and without V2V)
	- TrafficLightFileReader: does the interface between the data in a CSV file and the code.
	- Vehicle: base class for all vehicles (EgoVehicle and NearbyVehicle)

- VISSIM_networks:
	- dll_log.txt: data written by the DLL during the latest simulation. This file is created automatically once a simulation is run.
	- dll_persistent.txt: simple log of all simulations run using the DLL. . This file is created automatically once the first simulation is run.
	- traffic_lights_study.inpx: VISSIM file with the simulated network
	- traffic_lights_study_source_times.csv: file describing the green, amber and red periods as well as the position of all traffic lights in the simulation. 
	This file is used by the DLL so the CAVs can know the traffic lights periods. It must be updated manually if any alterations to the traffic lights are made in VISSIM.
	- traffic_lights_studyX.sig, X = 1, ..., 11: files used by VISSIM which describe the green, amber and red periods of all traffic lights in the simulation.

	
