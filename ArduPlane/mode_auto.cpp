#include "mode.h"
#include "Plane.h"
//#include "navigation.h"

bool ModeAuto::_enter() {
#if HAL_QUADPLANE_ENABLED
    // check if we should refuse auto mode due to a missing takeoff in
    // guided_wait_takeoff state
    if (plane.previous_mode == &plane.mode_guided &&
        quadplane.guided_wait_takeoff_on_mode_enter) {
        if (!plane.mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Takeoff waypoint required");
            quadplane.guided_wait_takeoff = true;
            return false;
        }
    }
    
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
#else
    plane.auto_state.vtol_mode = false;
#endif

    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

#if HAL_SOARING_ENABLED
    plane.g2.soaring_controller.init_cruising();
#endif

    // Check if this is the last waypoint before LAND
    if (plane.is_last_waypoint_before_land()) {
        // Build a straight trajectory to the LAND point
        plane.build_straight_trajectory_to_land();

        // Disable landing modes (flare, preflare)
      //  plane.landing.disable_landing_modes = true; // Устанавливаем флаг
    }

    return true;
}

void ModeAuto::_exit() {
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();

        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id)) {
            restart = false;
        }
#endif
        if (restart) {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModeAuto::update() {
	if (plane.mission.state() == AP_Mission::MISSION_COMPLETE) {
	    if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
	        gcs().send_text(MAV_SEVERITY_INFO, "Landing sequence active, not switching to RTL");
	        return; // Остаёмся в AUTO для выполнения LAND
	    }

	    plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
	    gcs().send_text(MAV_SEVERITY_INFO, "Mission complete, switching to RTL");
	    return;
	}

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        plane.quadplane.control_auto();
        return;
    }
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        return;
    }
#endif

    // Check if this is the last waypoint before LAND
    if (plane.is_last_waypoint_before_land()) {
            plane.build_straight_trajectory_to_land();
          //  plane.landing.disable_landing_modes = true; // Устанавливаем флаг

            // Управляем самолетом для прямолинейного полета
            plane.calc_nav_roll();
            plane.calc_nav_pitch();
            plane.calc_throttle();
            return;
        }

    // Handle TAKEOFF or aborted landing
    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.takeoff_calc_throttle();
    }
    // Handle LAND command
    else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // Restrict roll during landing
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit * 100UL);

        if (plane.landing.is_throttle_suppressed()) {
            // Suppress throttle if landing is complete
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else {
            plane.calc_throttle();
        }
#if AP_SCRIPTING_ENABLED
    } else if (nav_cmd_id == MAV_CMD_NAV_SCRIPT_TIME) {
        // NAV_SCRIPTING: Set target roll and pitch
        plane.nav_roll_cd = ahrs.roll_sensor;
        plane.nav_pitch_cd = ahrs.pitch_sensor;
#endif
    }
    // Regular AUTO flight logic
    else {
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
    if (plane.mission.state() == AP_Mission::MISSION_COMPLETE) {
        if (plane.is_last_waypoint_before_land()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Landing sequence active, not switching to RTL");
            return; // Остаёмся в AUTO
        }

        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Mission complete, switching to RTL");
        return;
    }
}

void ModeAuto::navigate() {
    if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
}

bool ModeAuto::does_auto_navigation() const {
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

bool ModeAuto::does_auto_throttle() const {
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

// returns true if the vehicle can be armed in this mode
bool ModeAuto::_pre_arm_checks(size_t buflen, char *buffer) const {
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.enabled()) {
        if (plane.quadplane.option_is_set(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO) &&
                !plane.quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
            hal.util->snprintf(buffer, buflen, "not in VTOL takeoff");
            return false;
        }
        if (!plane.mission.starts_with_takeoff_cmd()) {
            hal.util->snprintf(buffer, buflen, "missing takeoff waypoint");
            return false;
        }
    }
#endif
    // Note that this bypasses the base class checks
    return true;
}

bool ModeAuto::is_landing() const {
    return (plane.flight_stage == AP_FixedWing::FlightStage::LAND);
}

void ModeAuto::run() {
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        pullup.stabilize_pullup();
        return;
    }
#endif
    
    if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_ALTITUDE_WAIT) {
        wiggle_servos();

        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0);

        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttle);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleLeft);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleRight);

        // Relax attitude control
        reset_controllers();
    } else {
        // Normal flight, run base class
        Mode::run();
    }
}
