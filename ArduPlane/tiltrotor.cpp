#include "Plane.h"

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
 */


/*
  calculate maximum tilt change as a proportion from 0 to 1 of tilt
 */
float QuadPlane::tilt_max_change(bool up)
{
    float rate;
    if (up || tilt.max_rate_down_dps <= 0) {
        rate = tilt.max_rate_up_dps;
    } else {
        rate = tilt.max_rate_down_dps;
    }
    if (tilt.tilt_type != TILT_TYPE_BINARY && !up) {
        bool fast_tilt = false;
        if (plane.control_mode == MANUAL) {
            fast_tilt = true;
        }
        if (hal.util->get_soft_armed() && !in_vtol_mode() && !assisted_flight) {
            fast_tilt = true;
        }
        if (fast_tilt) {
            // allow a minimum of 90 DPS in manual or if we are not
            // stabilising, to give fast control
            rate = MAX(rate, 90);
        }
    }
    return rate * plane.G_Dt / 90.0f;
}

/*
  output a slew limited tiltrotor angle. tilt is from 0 to 1
 */
void QuadPlane::tiltrotor_slew(float newtilt)
{
    float max_change = tilt_max_change(newtilt<tilt.current_tilt);
    tilt.current_tilt = constrain_float(newtilt, tilt.current_tilt-max_change, tilt.current_tilt+max_change);

    // translate to 0..1000 range and output
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * tilt.current_tilt);

    // setup tilt compensation
    motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&QuadPlane::tilt_compensate, void, float *, uint8_t));
}


/*
  output a slew limited tiltrotor angle. Elevator output is added to tilt. tilt is from 0 to 1.
 */
void QuadPlane::tiltrotor_slew_elevator(float newtilt)
{
    float max_change_wing = tilt_max_change(newtilt<tilt.current_tilt);
	float max_change_tail = tilt_max_change(newtilt<tilt.tail_tilt);
	float tilt_output = 0;

	if(tilt.current_tilt >= 1 && tilt.tail_tilt >= tilt.max_angle_deg / 90.0f ){
		tilt.tail_tilt += constrain_float(newtilt-tilt.tail_tilt, -max_change_tail, max_change_tail);
	}
	else if(tilt.current_tilt >= tilt.max_angle_deg / 90.0f){
		tilt.current_tilt += constrain_float(newtilt-tilt.current_tilt, -max_change_wing, max_change_wing);
	}
	else{
		tilt.current_tilt += constrain_float(newtilt-tilt.current_tilt, -max_change_wing, max_change_wing);
		tilt.tail_tilt += constrain_float(newtilt-tilt.tail_tilt, -max_change_tail, max_change_tail);
	}
	
	
	tilt_output = constrain_float(tilt.tail_tilt * tilt.tilt_back_fwd + remap.elevator_P_b * constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)/4500.0, -1, 1), 0, 1);
	
    // translate to 0..1000 range and output
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * tilt_output);

    // setup tilt compensation
    motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&QuadPlane::no_tilt_compensation, void, float *, uint8_t));
}


/*
  update motor tilt for continuous tilt servos
 */
void QuadPlane::tiltrotor_continuous_update(void)
{
    // default to inactive
    tilt.motors_active = false;

    // the maximum rate of throttle change
    float max_change;
    
    if (!in_vtol_mode() && (!hal.util->get_soft_armed() || !assisted_flight)) {
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor
		if (remap.enable) {
			tiltrotor_slew_elevator(1);
		} else {
			tiltrotor_slew(1);
		}

        max_change = tilt_max_change(false);
        
        float new_throttle = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01, 0, 1);
        if (tilt.current_tilt < 1) {
            tilt.current_throttle = constrain_float(new_throttle,
                                                    tilt.current_throttle-max_change,
                                                    tilt.current_throttle+max_change);
        } else {
            tilt.current_throttle = new_throttle;
        }
        if (!hal.util->get_soft_armed()) {
            tilt.current_throttle = 0;
        } else {
            // the motors are all the way forward, start using them for fwd thrust
			
			if (remap.enable && !is_zero(tilt.current_throttle)) {
				
				float rudder_out = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_rudder)/4500.0, -1, 1);
				float elevator_out = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)/4500.0, -1, 1);
				if(elevator_out < 0)
					elevator_out = -elevator_out;
				float aileron_out = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)/4500.0, -1, 1);
				
				uint8_t mask = 0x01;
				float output_throttle = 0;
				for(int i = 0; i < 8; i++){
					if ((mask & (uint8_t)tilt.tilt_mask.get()) == 0){
						mask = mask << 1;
						continue;
					}
					output_throttle = 	constrain_float(
										tilt.current_throttle + 
										((mask & (uint8_t)remap.yaw_mask_a)?1.0:0.0) * remap.yaw_P_a * rudder_out + 
										((mask & (uint8_t)remap.yaw_mask_b)?1.0:0.0) * remap.yaw_P_b * rudder_out + 
										((mask & (uint8_t)remap.pitch_mask_a)?1.0:0.0) * remap.pitch_P_a * elevator_out + 
										((mask & (uint8_t)remap.pitch_mask_b)?1.0:0.0) * remap.pitch_P_b * elevator_out +
										((mask & (uint8_t)remap.roll_mask_a)?1.0:0.0) * remap.roll_P_a * aileron_out + 
										((mask & (uint8_t)remap.roll_mask_b)?1.0:0.0) * remap.roll_P_b * aileron_out
										,0
										,1);
					motors->output_motor_mask_not_intrusive(output_throttle, mask);
					
					mask = mask << 1;
					
				}
			} else {
				uint8_t mask = is_zero(tilt.current_throttle)?0:(uint8_t)tilt.tilt_mask.get();
				motors->output_motor_mask(tilt.current_throttle, mask);
				// prevent motor shutdown
				tilt.motors_active = true;
			}
        }
        return;
    }

    // remember the throttle level we're using for VTOL flight
    float motors_throttle = motors->get_throttle();
    max_change = tilt_max_change(motors_throttle<tilt.current_throttle);
    tilt.current_throttle = constrain_float(motors_throttle,
                                            tilt.current_throttle-max_change,
                                            tilt.current_throttle+max_change);
    
    /*
      we are in a VTOL mode. We need to work out how much tilt is
      needed. There are 3 strategies we will use:

      1) in QSTABILIZE or QHOVER the angle will be set to zero. This
         enables these modes to be used as a safe recovery mode.

      2) in fixed wing assisted flight or velocity controlled modes we
         will set the angle based on the demanded forward throttle,
         with a maximum tilt given by Q_TILT_MAX. This relies on
         Q_VFWD_GAIN being set

      3) if we are in TRANSITION_TIMER mode then we are transitioning
         to forward flight and should put the rotors all the way forward
    */
    if (plane.control_mode == QSTABILIZE ||
        plane.control_mode == QHOVER) {
		if(remap.enable) {
			tiltrotor_slew_elevator(0);
		}
		else{
			tiltrotor_slew(0);
		}
        return;
    }

    if (assisted_flight &&
        transition_state >= TRANSITION_TIMER) {
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
        if(remap.enable) {
			tiltrotor_slew_elevator(1);
		}
		else{
			tiltrotor_slew(1);
		}
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
        float settilt = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 50.0f, 0, 1);
		if(remap.enable) {
			tiltrotor_slew_elevator(settilt * tilt.max_angle_deg / 90.0f);
		}
		else{
			tiltrotor_slew(settilt * tilt.max_angle_deg / 90.0f);
		}
        
    }
}


/*
  output a slew limited tiltrotor angle. tilt is 0 or 1
 */
void QuadPlane::tiltrotor_binary_slew(bool forward)
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

    float max_change = tilt_max_change(!forward);
    if (forward) {
        tilt.current_tilt = constrain_float(tilt.current_tilt+max_change, 0, 1);
    } else {
        tilt.current_tilt = constrain_float(tilt.current_tilt-max_change, 0, 1);
    }

    // setup tilt compensation
    motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&QuadPlane::tilt_compensate, void, float *, uint8_t));
}

/*
  update motor tilt for binary tilt servos
 */
void QuadPlane::tiltrotor_binary_update(void)
{
    // motors always active
    tilt.motors_active = true;

    if (!in_vtol_mode()) {
        // we are in pure fixed wing mode. Move the tiltable motors
        // all the way forward and run them as a forward motor
        tiltrotor_binary_slew(true);

        float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
        if (tilt.current_tilt >= 1) {
            uint8_t mask = is_zero(new_throttle)?0:(uint8_t)tilt.tilt_mask.get();
            // the motors are all the way forward, start using them for fwd thrust
            motors->output_motor_mask(new_throttle, mask);
        }
    } else {
        tiltrotor_binary_slew(false);
    }
}


/*
  update motor tilt
 */
void QuadPlane::tiltrotor_update(void)
{
    if (tilt.tilt_mask <= 0) {
        // no motors to tilt
        return;
    }

    if (tilt.tilt_type == TILT_TYPE_BINARY) {
        tiltrotor_binary_update();
    } else {
        tiltrotor_continuous_update();
    }

    if (tilt.tilt_type == TILT_TYPE_VECTORED_YAW) {
        tiltrotor_vectored_yaw();
    }
}


/*
  compensate for tilt in a set of motor outputs

  Compensation is of two forms. The first is to apply _tilt_factor,
  which is a compensation for the reduces vertical thrust when
  tilted. This is supplied by set_motor_tilt_factor().

  The second compensation is to use equal thrust on all tilted motors
  when _tilt_equal_thrust is true. This is used when the motors are
  tilted by a large angle to prevent the roll and yaw controllers from
  causing instability. Typically this would be used when the motors
  are tilted beyond 45 degrees. At this angle it is assumed that roll
  control can be achieved using fixed wing control surfaces and yaw
  control with the remaining multicopter motors (eg. tricopter tail).

  By applying _tilt_equal_thrust the tilted motors effectively become
  a single pitch control motor.

  Note that we use a different strategy for when we are transitioning
  into VTOL as compared to from VTOL flight. The reason for that is
  we want to lean towards higher tilted motor throttle when
  transitioning to fixed wing flight, in order to gain airspeed,
  whereas when transitioning to VTOL flight we want to lean to towards
  lower fwd throttle. So we raise the throttle on the tilted motors
  when transitioning to fixed wing, and lower throttle on tilted
  motors when transitioning to VTOL
 */
void QuadPlane::tilt_compensate_down(float *thrust, uint8_t num_motors)
{
    float inv_tilt_factor;
    if (tilt.current_tilt > 0.98f) {
        inv_tilt_factor = 1.0 / cosf(radians(0.98f*90));
    } else {
        inv_tilt_factor = 1.0 / cosf(radians(tilt.current_tilt*90));
    }

    // when we got past Q_TILT_MAX we gang the tilted motors together
    // to generate equal thrust. This makes them act as a single pitch
    // control motor while preventing them trying to do roll and yaw
    // control while angled over. This greatly improves the stability
    // of the last phase of transitions
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool equal_thrust = (tilt.current_tilt > tilt_threshold);

    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply inv_tilt_factor first
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            thrust[i] *= inv_tilt_factor;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;

    // now constrain and apply _tilt_equal_thrust if enabled
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            if (equal_thrust) {
                thrust[i] = tilt_total / tilt_count;
            }
            largest_tilted = MAX(largest_tilted, thrust[i]);
        }
    }

    // if we are saturating one of the tilted motors then reduce all
    // motors to keep them in proportion to the original thrust. This
    // helps maintain stability when tilted at a large angle
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}


/*
  tilt compensation when transitioning to VTOL flight
 */
void QuadPlane::tilt_compensate_up(float *thrust, uint8_t num_motors)
{
    float tilt_factor = cosf(radians(tilt.current_tilt*90));

    // when we got past Q_TILT_MAX we gang the tilted motors together
    // to generate equal thrust. This makes them act as a single pitch
    // control motor while preventing them trying to do roll and yaw
    // control while angled over. This greatly improves the stability
    // of the last phase of transitions
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool equal_thrust = (tilt.current_tilt > tilt_threshold);

    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply tilt_factor first
    for (uint8_t i=0; i<num_motors; i++) {
        if (!is_motor_tilting(i)) {
            thrust[i] *= tilt_factor;
        } else {
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    // now constrain and apply _tilt_equal_thrust if enabled
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            if (equal_thrust) {
                thrust[i] = tilt_total / tilt_count;
            }
        }
    }
}

/*
  choose up or down tilt compensation based on flight mode When going
  to a fixed wing mode we use tilt_compensate_down, when going to a
  VTOL mode we use tilt_compensate_up
 */
void QuadPlane::tilt_compensate(float *thrust, uint8_t num_motors)
{
    if (tilt.current_tilt <= 0) {
        // the motors are not tilted, no compensation needed
        return;
    }
    if (in_vtol_mode()) {
        // we are transitioning to VTOL flight
        tilt_compensate_up(thrust, num_motors);
    } else {
        tilt_compensate_down(thrust, num_motors);
    }
}

/*
  choose up or down tilt compensation based on flight mode When going
  to a fixed wing mode we use tilt_compensate_down, when going to a
  VTOL mode we use tilt_compensate_up
 */
void QuadPlane::no_tilt_compensation(float *thrust, uint8_t num_motors)
{
        return;
}



/*
  return true if the rotors are fully tilted forward
 */
bool QuadPlane::tiltrotor_fully_fwd(void)
{
    if (tilt.tilt_mask <= 0) {
        return false;
    }
    return (tilt.current_tilt >= 1);
}

/*
  control vectored yaw with tilt multicopters
 */
void QuadPlane::tiltrotor_vectored_yaw(void)
{
    // total angle the tilt can go through
    float total_angle = 90 + tilt.tilt_yaw_angle;
    // output value (0 to 1) to get motors pointed straight up
    float zero_out = tilt.tilt_yaw_angle / total_angle;

    // calculate the basic tilt amount from current_tilt
    float base_output = zero_out + tilt.current_tilt * tilt.tilt_wings_fwd * (1 - zero_out);
    
    float tilt_threshold = (tilt.max_angle_deg/90.0f);
    bool no_yaw = (tilt.current_tilt > tilt_threshold);
	float output_right = 0, output_left = 0;
	
    if (no_yaw) {
		
		if (remap.enable) {
			float elevator_out = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)/4500.0, -1, 1);
			float flap_out = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_flap_auto)/4500.0, -1, 1);
			float aileron_out = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)/4500.0, -1, 1);
			
			output_right = constrain_float(
				base_output 
				+ remap.elevator_P_a * elevator_out 
				+ remap.flap_P * flap_out 
				+ remap.aileron_P * aileron_out
				, 0
				, 1);
				
			output_left = constrain_float(
				base_output 
				+ remap.elevator_P_a * elevator_out 
				+ remap.flap_P * flap_out 
				- remap.aileron_P * aileron_out
				, 0
				, 1);		
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * output_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * output_right);
		
		} else {
			SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * base_output);
			SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * base_output);
		}
    } else {
        float yaw_out = motors->get_yaw();
        float yaw_range = zero_out;

        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * (base_output + yaw_out * yaw_range));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * (base_output - yaw_out * yaw_range));
    }
}
