/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file AttitudeControl.hpp
 *
 * A quaternion based attitude controller.
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 *
 * Publication documenting the implemented Quaternion Attitude Control:
 * Nonlinear Quadrocopter Attitude Control (2013)
 * by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
 * Institute for Dynamic Systems and Control (IDSC), ETH Zurich
 *
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

class AttitudeControl
{
public:
	AttitudeControl() = default;
	~AttitudeControl() = default;

	/**
	 * Set proportional attitude control gain
	 * @param proportional_gain 3D vector containing gains for roll, pitch, yaw
	 * @param yaw_weight A fraction [0,1] deprioritizing yaw compared to roll and pitch
	 */
	void setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight);

	/**
	 * Set hard limit for output rate setpoints
	 * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	/**
	 * Set current angular velocity 
	 * @param rate_prev [rad/s] 3D vector containing previous roll, pitch, yaw rates
	 */
	void setOmega(const matrix::Vector3f &omega) { _omega = omega; }

	/**
	 * Get current angular velocity 
	 * @param rate_prev [rad/s] 3D vector containing previous roll, pitch, yaw rates
	 */
	matrix::Vector3f getOmega() const { 
		return _omega; 
	}

	/**
	 * Set previous Rd
	 * @param rate_prev [rad/s] 3D vector containing previous roll, pitch, yaw rates
	 */
	void setRdPrev(const matrix::Dcmf &Rd_prev) { _Rd_prev = Rd_prev; }

	/**
	 * Get previous Rd
	 * @param rate_prev [rad/s] 3D vector containing previous roll, pitch, yaw rates
	 */
	matrix::Dcmf getRdPrev() const { 
		return _Rd_prev; 
	}

	// /**
	//  * Set previous eOmega
	//  * @param eOmega [rad/s2] 3D vector containing previous roll, pitch, yaw angular error 
	//  */
	// void seteOmegaIntPrev(const matrix::Vector3f &eOmega_prev) { _eOmega_prev = eOmega_prev; }

	// /**
	//  * Get previous eOmega
	//  * @param eOmega [rad/s2] 3D vector containing previous roll, pitch, yaw angular error
	//  */
	// matrix::Vector3f geteOmegaIntPrev() const { 
	// 	return _eOmega_prev; 
	// }

	/**
	 * Set rate setpoints
	 * @param rate_prev [rad/s] 3D vector containing previous roll, pitch, yaw rates
	 */
	void setRateSp(const matrix::Vector3f &rate_setpoint) { _rate_setpoint = rate_setpoint; }

	/**
	 * Get rate setpoints
	 * @param rate_prev [rad/s] 3D vector containing previous roll, pitch, yaw rates
	 */
	matrix::Vector3f getRateSp() const { 
		return _rate_setpoint; 
	}

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint)
	{
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	void resetIntegral() {_eOmega_int.zero(); }

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}

	/**
	 * Run one control loop cycle calculation
	 * @param q estimation of the current vehicle attitude unit quaternion
	 * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
	 */
	matrix::Vector3f update(const matrix::Quatf &q, const bool landed);

private:
	matrix::Vector3f _proportional_gain;
	matrix::Vector3f _rate_limit;
	matrix::Vector3f _omega;
	matrix::Dcmf _Rd_prev;

	matrix::Vector3f _rate_setpoint;

	matrix::Vector3f ratesdes;

	matrix::Vector3f pqrdes;

	hrt_abstime _last_run = 0.0;

	matrix::Vector3f _eOmega_int;

	float _yaw_w{0.f}; ///< yaw weight [0,1] to deprioritize caompared to roll and pitch

	matrix::Quatf _attitude_setpoint_q; ///< latest known attitude setpoint e.g. from position control
	float _yawspeed_setpoint{0.f}; ///< latest known yawspeed feed-forward setpoint

	math::LowPassFilter2p<matrix::Vector3f> _lp_filter{50.f, 100.f}; // 50, 100 works fine
};
