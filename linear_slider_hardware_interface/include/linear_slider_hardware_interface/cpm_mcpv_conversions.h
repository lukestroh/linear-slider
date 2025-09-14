#ifndef __CPM_MCPV_CONVERSIONS_H__
#define __CPM_MCPV_CONVERSIONS_H__

double step_to_pos(int step);

int pos_to_step(double pos);

double rpm_to_vel(int rpm);

int vel_to_rpm(double& vel);


#endif // __CPM_MCPV_CONVERSIONS_H__