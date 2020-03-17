#ifndef AVIONICS_MATH_H
#define AVIONICS_MATH_H

#include <math.h>
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Description:
//  Calculate altitude from pressure reading
//
// Parameters:
//  Pressure - [Pa]
//
// Returns:
//  float - float value of altitude approximation
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
static inline uint32_t math_approximate_altitude(float pressure, float temperature, float ref_pres, float ref_alt)
{
	float p_term = powf((ref_pres/(pressure/100)),(1/5.257F))-1;
	float t_term = (temperature/100)+273.15F;
	return (uint32_t)(p_term*t_term)/0.0065F+ref_alt;
}




#endif //AVIONICS_MATH_H
