// Copyright 2021 Smile Robotics, inc
#ifndef C_WRAPPER_NAV2__C_WRAPPER_NAV2_H_
#define C_WRAPPER_NAV2__C_WRAPPER_NAV2_H_

extern "C"
{
    int nav2_init();
    // @return 0 means success to send goal
    int nav2_send_goal(const double x, const double y, const double theta);
    // @return 0 means success
    int nav2_wait_until_reach(const double timeout_sec);
    int nav2_cancel_all_goals();
}

#endif  // C_WRAPPER_NAV2__C_WRAPPER_NAV2_H_
