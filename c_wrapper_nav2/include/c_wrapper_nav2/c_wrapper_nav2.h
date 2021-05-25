#pragma once

extern "C"
{
    int nav2_init();
    // @return 0 means success to send goal
    int nav2_send_goal(const double x, const double y, const double theta);
    // @return 0 means success
    int nav2_wait_until_reach(const double timeout_sec);
    int nav2_cancel_all_goals();
}