#include <stdio.h>

#include "pendulum.h"
#include "rigid_body_motion.h"
#include "van_der_pol_example.h"
#include "wmq_debug.h"
#include "wmq_error.h"

int main(void) {
    debug_set_level(LL_DETAIL, WMQ_LOG_OPTION_USE_ODS | WMQ_LOG_OPTION_USE_STDERR | WMQ_LOG_OPTION_SHOW_TIME | WMQ_LOG_OPTION_SHOW_PID | WMQ_LOG_OPTION_SHOW_TID);
    WMQ_LOG(LL_INFO, "starting, sizeof(long unsigned int):%zu sizeof(int):%zu", sizeof(long unsigned int), sizeof(int));

    int rc;

    //rc = van_der_pol_example_driver();
    //WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "van_der_pol_example_driver");

    //rc = van_der_pol_example_low_level();
    //WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "van_der_pol_example_low_level");

    //rc = rigid_body_motion_driver();
    //rc = rigid_body_motion_low_level();
    //WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "rigid_body_motion_low_level");

    rc = pendulum_low_level();
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "pendulum_low_level");
    return rc;
}