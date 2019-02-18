#include "Copter.h"


/******************************************************************************************************************************************
*函数原型：void Copter::update_ground_effect_detector(void)
*函数功能：
*修改日期：2019-2-18
*修改作者：cihang_uav
*备注信息：地效影响函数
********************************************************************************************************************************************/
void Copter::update_ground_effect_detector(void)
{
    if(!g2.gndeffect_comp_enabled || !motors->armed()) //如果地效补偿没有使能，或者电机没有解锁
    {
        // disarmed - disable ground effect and return
        gndeffect_state.takeoff_expected = false;  //期望起飞状态
        gndeffect_state.touchdown_expected = false;//期望着地状态
        ahrs.setTakeoffExpected(gndeffect_state.takeoff_expected); //都设置成0
        ahrs.setTouchdownExpected(gndeffect_state.touchdown_expected); //都设置成0
        return;
    }

    //变量初始化----variable initialization
    uint32_t tnow_ms = millis(); //获取时间
    float xy_des_speed_cms = 0.0f; //水平方向上的期望速度
    float xy_speed_cms = 0.0f; //水平速度
    float des_climb_rate_cms = pos_control->get_desired_velocity().z; //获取垂直方向上的爬升速度

    if (pos_control->is_active_xy()) //如果位置控制器是激活了
    {
        Vector3f vel_target = pos_control->get_vel_target(); //获取目标速度
        vel_target.z = 0.0f;
        xy_des_speed_cms = vel_target.length();
    }

    if (position_ok() || optflow_position_ok()) //GPS位置定位是Ok的，或者采用光流
    {
        Vector3f vel = inertial_nav.get_velocity(); //获取当前无人机的速度
        vel.z = 0.0f;
        xy_speed_cms = vel.length();
    }

    //起飞逻辑-----------------------------takeoff logic

    //如果我们已经解锁，但是还没有起飞------if we are armed and haven't yet taken off
    if (motors->armed() && ap.land_complete && !gndeffect_state.takeoff_expected)
    {
        gndeffect_state.takeoff_expected = true; //设置期望起飞等于1
    }

    //如果我们还没有起飞，复位起飞时间，高度值，完成标志位信息----if we aren't taking off yet, reset the takeoff timer, altitude and complete flag
    const bool throttle_up = flightmode->has_manual_throttle() && channel_throttle->get_control_in() > 0;
    if (!throttle_up && ap.land_complete)
    {
        gndeffect_state.takeoff_time_ms = tnow_ms;
        gndeffect_state.takeoff_alt_cm = inertial_nav.get_altitude();
    }

    // if we are in takeoff_expected and we meet the conditions for having taken off
    // end the takeoff_expected state
    //如果我们期望起飞，我们满足起飞条件，结束起飞状态，超时判断
    if (gndeffect_state.takeoff_expected && (tnow_ms-gndeffect_state.takeoff_time_ms > 5000 || inertial_nav.get_altitude()-gndeffect_state.takeoff_alt_cm > 50.0f)) {
        gndeffect_state.takeoff_expected = false;
    }

    //降落逻辑--------------landing logic
    Vector3f angle_target_rad = attitude_control->get_att_target_euler_cd() * radians(0.01f); //获取目标角度
    bool small_angle_request = cosf(angle_target_rad.x)*cosf(angle_target_rad.y) > cosf(radians(7.5f));
    bool xy_speed_low = (position_ok() || optflow_position_ok()) && xy_speed_cms <= 125.0f;
    bool xy_speed_demand_low = pos_control->is_active_xy() && xy_des_speed_cms <= 125.0f;
    bool slow_horizontal = xy_speed_demand_low || (xy_speed_low && !pos_control->is_active_xy()) || (control_mode == ALT_HOLD && small_angle_request);

    bool descent_demanded = pos_control->is_active_z() && des_climb_rate_cms < 0.0f;
    bool slow_descent_demanded = descent_demanded && des_climb_rate_cms >= -100.0f;
    bool z_speed_low = fabsf(inertial_nav.get_velocity_z()) <= 60.0f;
    bool slow_descent = (slow_descent_demanded || (z_speed_low && descent_demanded));

    gndeffect_state.touchdown_expected = slow_horizontal && slow_descent;

    //如果起飞或着陆，准备EKF地面效应。---Prepare the EKF for ground effect if either takeoff or touchdown is expected.
    ahrs.setTakeoffExpected(gndeffect_state.takeoff_expected);
    ahrs.setTouchdownExpected(gndeffect_state.touchdown_expected);
}


/******************************************************************************************************************************
*                                   File-end
*******************************************************************************************************************************/
