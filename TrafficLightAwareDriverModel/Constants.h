/*==========================================================================*/
/*  Constants.h																*/
/*						                              Fernando V. Monteiro  */
/*==========================================================================*/

#pragma once

typedef unsigned long color_t;
/* Based on the _WINGDI_ RGB definition and some tests on VISSIM */
//#define ARGB(a,r,g,b)   ((COLORREF)((((BYTE)(b) | ((WORD)((BYTE)(g)) << 8)) | (((DWORD)(BYTE)(r)) << 16))|((BYTE)(a) << 24)))
#define ARGB(a, r, g, b) ((color_t)((long)(b) + (long)(g)*256 + (long)(r)*256*256 + (unsigned long)(a)*256*256*256))

const color_t BLACK = ARGB(255, 0, 0, 0);
const color_t WHITE = ARGB(255, 255, 255, 255);
const color_t RED = ARGB(255, 255, 0, 0);
const color_t GREEN = ARGB(255, 0, 255, 0);
const color_t BLUE = ARGB(255, 0, 0, 255);
const color_t YELLOW = ARGB(255, 255, 255, 0);
const color_t MAGENTA = ARGB(255, 255, 0, 255);
const color_t CYAN = ARGB(255, 0, 255, 255);

const color_t DARK_RED = ARGB(255, 128, 0, 0);
const color_t DARK_GREEN = ARGB(255, 0, 128, 0);
const color_t DARK_BLUE = ARGB(255, 0, 0, 128);
const color_t DARK_YELLOW = ARGB(255, 196, 196, 0);
const color_t DARK_MAGENTA = ARGB(255, 128, 0, 128); // PURPLE
const color_t DARK_CYAN = ARGB(255, 0, 128, 128);

const color_t LIGHT_BLUE = ARGB(255, 0, 196, 255);
const color_t BLUE_GREEN = ARGB(255, 0, 128, 128);

const double MAX_DISTANCE = 300.0; // [m]
const double CAR_MAX_BRAKE{ 6.0 }; // absolute value [m/s^2]
const double TRUCK_MAX_BRAKE{ 5.5 }; // absolute value  [m/s^2]
const double COMFORTABLE_ACCELERATION{ 2.0 }; // [m/s^2]
const double COMFORTABLE_BRAKE{ 4.0 }; // absolute value [m/s^2]

/* Categories set by VISSIM */
enum class VehicleCategory {
	undefined,
	car = 1,
	truck,
	bus,
	tram,
	pedestrian,
	bike,
};

/* User defined vehicle type. To be consistent with existing type,
follow these rules when defining a new type:
- Types should contain 3 digits
- The first digit of the type should match the vehicle category
of the type */
enum class VehicleType {
	undefined,
	human_driven_car = 100,
	traffic_light_acc_car = 130,
	traffic_light_cacc_car = 135,
	platoon_car = 140,
	truck = 200,
	bus = 300
};
