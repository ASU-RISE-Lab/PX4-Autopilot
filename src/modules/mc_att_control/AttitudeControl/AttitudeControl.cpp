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
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

// #include <iostream>


using namespace matrix;

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

matrix::Vector3f AttitudeControl::update(const Quatf &q, const bool landed) 
{
	Quatf qd = _attitude_setpoint_q;

	// // calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	// const Vector3f e_z = q.dcm_z();
	// const Vector3f e_z_d = qd.dcm_z();
	// Quatf qd_red(e_z, e_z_d);

	// if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
	// 	// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
	// 	// full attitude control anyways generates no yaw input and directly takes the combination of
	// 	// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
	// 	qd_red = qd;

	// } else {
	// 	// transform rotation from current to desired thrust vector into a world frame reduced desired attitude
	// 	qd_red *= q;
	// }

	// // mix full and reduced desired attitude
	// Quatf q_mix = qd_red.inversed() * qd;
	// q_mix.canonicalize();
	// // catch numerical problems with the domain of acosf and asinf
	// q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	// q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	// qd = qd_red * Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))));

	// // quaternion attitude control law, qe is rotation from q to qd
	// const Quatf qe = q.inversed() * qd;

	// // using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// // also taking care of the antipodal unit quaternion ambiguity
	// const Vector3f eq = 2.f * qe.canonical().imag();

	// // calculate angular rates setpoint
	// matrix::Vector3f rate_setpoint = eq.emult(_proportional_gain);

	/********************************************************/
	matrix::Dcmf Rd = qd;

	matrix::Dcmf R = q;

	matrix::Dcmf Rd_prev = getRdPrev();
	
	/********************************************************/
	//Calculate desired angular velocity
	/********************************************************/
	
	//float deflecright;

	// /********************************************************/
	// float pitchangdes = Eulerf(qd).theta();
	// float rollangdes = Eulerf(qd).phi();
	// float yawangdes = Eulerf(qd).psi();

	// matrix::Vector3f _angles_prev = getRatePrev();
	// float _rolldesprev = _angles_prev(0);
	// float _pitchdesprev = _angles_prev(1);
	// float _yawdesprev = _angles_prev(2);

	// ratesdes(0) = ((rollangdes - _rolldesprev))/dt;
	// ratesdes(1) = (pitchangdes - _pitchdesprev);
	// ratesdes(2) = (yawangdes - _yawdesprev);

	// pqrdes(0) = ratesdes(0) - ratesdes(2) * sinf(pitchangdes);
	// pqrdes(1) = ratesdes(1) * cosf(rollangdes) + ratesdes(2) * sinf(rollangdes) * cosf(pitchangdes);
	// pqrdes(2) = - ratesdes(1) * sinf(rollangdes) + ratesdes(2) * cosf(rollangdes) * cosf(pitchangdes);

	// _rates_prev(0) = rollangdes;
	// _rates_prev(1) = pitchangdes;
	// _rates_prev(2) = yawangdes;

	// setRatePrev(_rates_prev);

	// Calculate Omega_d
	const hrt_abstime now = hrt_absolute_time();

	// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
	const float dt_eomega = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
	_last_run = now;

	// Differentiate for Omega
	matrix::Matrix3f Omega_d_matrix = Rd.transpose() * (Rd - Rd_prev)/dt_eomega;
	matrix::Vector3f Omega_d_unfilt(Omega_d_matrix(2,1), -Omega_d_matrix(2,0), Omega_d_matrix(1,0));

	// Filter the Omega_d
	const Vector3f Omega_d = _lp_filter.apply(Omega_d_unfilt);

	// Get Omega
	matrix::Vector3f Omega = getOmega();

	// Calculate eOmega
	matrix::Matrix3f RtRd = R.transpose() * Rd;
	matrix::Vector3f eOmega = Omega - RtRd * Omega_d;

	// // Add Integral Control only if we are not landed
	// _eOmega_int = _eOmega_int + eOmega*dt_eomega;
	// if (!landed) {
	// 	_eOmega_int = _eOmega_int + eOmega*dt_eomega;
	// } 

	/* Calculate eR */
	matrix::Matrix3f skewer = Rd.transpose() * R - R.transpose() * Rd;
	matrix::Vector3f eR(0.5f * skewer(2,1), -0.5f *skewer(2,0), 0.5f * skewer(1,0));

	//Vector3f wdes = _angle_p.emult(eR);
	//Vector3f wdes1 = R.transpose() * Rsp * wdes;
	// matrix::Vector3f rates_err = rate_setpoint - rates;

	/* generate control */
	// matrix::Vector3f att_control = rates_p_scaled.emult(rates_err) + _angle_p.emult(eR);
	// matrix::Vector3f att_control = _proportional_gain.emult(eR);
	// matrix::Vector3f att_control = _proportional_gain.emult(eR) - _proportional_gain.emult(eOmega);

	// matrix::Vector3f gain_eR = {0.75, 0.75, 0.75};

	// matrix::Vector3f gain_eOmega = {0.25, 0.25, 0.25};

	// good gains, i thought 
	// matrix::Vector3f gain_eR = {0.5, 0.5, 0.5};
	// matrix::Vector3f gain_eOmega = {0.25, 0.25, 0.25};

	// matrix::Vector3f gain_eOmega = {0.25, 0.25, 0.25}; 

	// matrix::Vector3f gain_eR = {1.2, 0.85, 0.5};
	// matrix::Vector3f gain_eOmega = {0.25, 0.25, 0.15};

	// good gains after integral control
	// matrix::Vector3f gain_eR = {0.85, 0.85, 0.85};
	// matrix::Vector3f gain_eOmega = {0.25, 0.25, 0.25};
	// matrix::Vector3f gain_eOmega_int = {0.15, 0.15, 0.15};

	matrix::Vector3f gain_eR = {0.75, 1.0, 0.75};
	matrix::Vector3f gain_eOmega = {0.25, 0.25, 0.25};
	// matrix::Vector3f gain_eOmega_int = {0.0005, 0.0005, 0.0001};


	// matrix::Vector3f term1 = gain_eOmega_int.emult(_eOmega_int);
 
	matrix::Vector3f att_control = - gain_eR.emult(eR) - gain_eOmega.emult(eOmega); // - term1; // - _eOmega_int ; //  // + _eOmega_int; // 
	// std::cout << term1(1) << std::endl;

	setRdPrev(Rd);

	// /********************************************************/

	// Feed forward the yaw setpoint rate.
	// yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yawspeed_setpoint).
	// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	// such that it can be added to the rates setpoint.
	// if (is_finite(_yawspeed_setpoint)) {
	// 	rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	// }

	// // limit rates
	// for (int i = 0; i < 3; i++) {
	// 	rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	// }

	// return rate_setpoint;
	return att_control;
}
