/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file landingslope.h
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#ifndef LANDINGSLOPE_H_
#define LANDINGSLOPE_H_

#include <math.h>
#include <systemlib/err.h>
#include <mathlib/mathlib.h>

class Landingslope
{
private:
	/* see Documentation/fw_landing_modified.pdf for a plot of the landing slope */
	float _landing_slope_angle_min_rad;
	float _landing_slope_angle_max_rad;
	float _landing_slope_angle_diff_rad_m;
	float _flare_relative_alt;				/**< h_flare,rel in the plot */
	float _motor_lim_relative_alt;
	float _H1_virt;							/**< H1 in the plot */
	float _H0;								/**< h_flare,rel + H1 in the plot */
	float _d1;								/**< d1 in the plot */
	float _flare_constant;
	float _flare_length;					/**< d1 + delta d in the plot */
	float _horizontal_slope_displacement;  /**< delta d in the plot */

	void calculateSlopeValues();

public:
	Landingslope();
	~Landingslope() {}


	/**
	 *
	 * @return relative altitude of point on landing slope at distance to landing waypoint=wp_landing_distance
	 */
	float getLandingSlopeRelativeAltitude(float wp_landing_distance);

	/**
	 *
	 * @return relative altitude of point on landing slope at distance to landing waypoint=wp_landing_distance
	 * Performs check if aircraft is in front of waypoint to avoid climbout
	 */
	float getLandingSlopeRelativeAltitudeSave(float wp_landing_distance, float bearing_lastwp_currwp,
			float bearing_airplane_currwp);


	/**
	 *
	 * @return Absolute altitude of point on landing slope at distance to landing waypoint=wp_landing_distance
	 */
	float getLandingSlopeAbsoluteAltitude(float wp_landing_distance, float wp_altitude);

	/**
	 *
	 * @return Absolute altitude of point on landing slope at distance to landing waypoint=wp_landing_distance
	 * Performs check if aircraft is in front of waypoint to avoid climbout
	 */
	float getLandingSlopeAbsoluteAltitudeSave(float wp_landing_distance, float bearing_lastwp_currwp,
			float bearing_airplane_currwp, float wp_landing_altitude);

	/**
	 *
	 * @return Relative altitude of point on landing slope at distance to landing waypoint=wp_landing_distance
	 */
	__EXPORT static float getLandingSlopeRelativeAltitude(float wp_landing_distance, float horizontal_slope_displacement,
			float flare_length, float landing_slope_angle_min_rad, float landing_slope_angle_max_rad, float landing_slope_angle_diff_rad_m)
	{
		return (wp_landing_distance - horizontal_slope_displacement) * tanf(math::min(landing_slope_angle_max_rad,
			landing_slope_angle_min_rad + landing_slope_angle_diff_rad_m * (wp_landing_distance - flare_length))); //flare_relative_alt is negative
	}

	/**
	 *
	 * @return Absolute altitude of point on landing slope at distance to landing waypoint=wp_landing_distance
	 */
	__EXPORT static float getLandingSlopeAbsoluteAltitude(float wp_landing_distance, float wp_landing_altitude,
			float horizontal_slope_displacement, float flare_length, float landing_slope_angle_min_rad,
			float landing_slope_angle_max_rad, float landing_slope_angle_diff_rad_m)
	{
		return getLandingSlopeRelativeAltitude(wp_landing_distance, horizontal_slope_displacement,
						       flare_length, landing_slope_angle_min_rad, landing_slope_angle_max_rad, landing_slope_angle_diff_rad_m) + wp_landing_altitude;
	}

	/**
	 *
	 * @return distance to landing waypoint of point on landing slope at altitude=slope_altitude
	 */
        __EXPORT static float getLandingSlopeWPDistance(float slope_altitude, float wp_landing_altitude,
			float horizontal_slope_displacement, float landing_slope_angle_min_rad,
			float landing_slope_angle_max_rad, float landing_slope_angle_diff_rad_m)
        {
                float wp_landing_distance = 0;
                float _altitude = 0;

                do {
                        wp_landing_distance += 1/tanf(math::min(landing_slope_angle_max_rad,
						landing_slope_angle_min_rad +
						landing_slope_angle_diff_rad_m * wp_landing_distance));
			_altitude++;
                } while (_altitude < slope_altitude - wp_landing_altitude);
	        wp_landing_distance += (slope_altitude - wp_landing_altitude - _altitude + 1) /
			tanf(math::min(landing_slope_angle_max_rad, landing_slope_angle_min_rad + landing_slope_angle_diff_rad_m * wp_landing_distance)) + horizontal_slope_displacement;
                return wp_landing_distance;
        }

	float getFlareCurveRelativeAltitudeSave(float wp_distance, float bearing_lastwp_currwp, float bearing_airplane_currwp);

	float getFlareCurveAbsoluteAltitudeSave(float wp_distance, float bearing_lastwp_currwp, float bearing_airplane_currwp,
						float wp_altitude);

	void update(float landing_slope_angle_min_rad_new,
		    float landing_slope_angle_max_rad_new,
		    float landing_slope_angle_diff_rad_m_new,
		    float flare_relative_alt_new,
		    float motor_lim_relative_alt_new,
		    float H1_virt_new);


	inline float landing_slope_angle_min_rad() {return _landing_slope_angle_min_rad;}
	inline float landing_slope_angle_max_rad() {return _landing_slope_angle_max_rad;}
	inline float landing_slope_angle_diff_rad_m() {return _landing_slope_angle_diff_rad_m;}
	inline float flare_relative_alt() {return _flare_relative_alt;}
	inline float motor_lim_relative_alt() {return _motor_lim_relative_alt;}
	inline float H1_virt() {return _H1_virt;}
	inline float H0() {return _H0;}
	inline float d1() {return _d1;}
	inline float flare_constant() {return _flare_constant;}
	inline float flare_length() {return _flare_length;}
	inline float horizontal_slope_displacement() {return _horizontal_slope_displacement;}

};


#endif /* LANDINGSLOPE_H_ */
