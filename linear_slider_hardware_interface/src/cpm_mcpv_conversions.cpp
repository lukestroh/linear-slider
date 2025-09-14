#include <linear_slider_hardware_interface/cpm_mcpv_conversions.h>

double step_to_pos(int step) {
    /* Convert step state to position in meters. Assumes step 0 == position 0 at limit switch closest to motor. 
       Teknic CPM-MCPV-2341S-RLN has step resolution of 1/800, 2 revolutions == 1cm
       1 step * 1rev/800step * 1cm/2rev * 1m/100cm = m
    */
    return static_cast<double>(step / 800 / 2 / 100.0);
}

int pos_to_step(double pos) {
    /* Convert position in meters to step state. Assumes step 0 == position 0 at limit switch closest to motor. 
       Teknic CPM-MCPV-2341S-RLN has step resolution of 1/800, 2 revolutions == 1cm
       1m * 100cm/1m * 2rev/1cm * 800step/1rev = step
    */
    return static_cast<int>(pos * 100 * 2 * 800);
}

double rpm_to_vel(int rpm) {
    /* Convert revolutions per minute of the motor to linear velocity of the slider 
       two turns for each cm of distance
       1 rev/min * 1 min/60s * 1cm/2rev * 1m/100cm = m/s
    */
    return static_cast<double>(rpm / 60.0 / 2.0 / 100.0);
}

int vel_to_rpm(double& vel) {
    /* Convert linear velocity of the slider to revolultions per minute of the motor 
       1cm of distance for two turns
       1 m/s * 60s/min * 100cm/m * 2rev/cm = rev/min
    */
   return static_cast<int>(vel * 60 * 100 * 2);
}