#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#define _USE_MATH_DEFINES
#include "math.h"
#include "wmq_error.h"

static int pendulum_func(double t, const double y[], double f[],
                         void *params) {
    (void)(t); /* avoid unused parameter warning */
    double g_l = *(double *)params;
    f[0] = y[2];              //da/dt = wa
    f[1] = y[3];              //db/dt = wb
    f[2] = -g_l * sin(y[0]);  //dwa/dt = -(g/l)*sin(a)
    f[3] = -g_l * sin(y[1]);  //dwa/dt = -(g/l)*sin(b)

    return GSL_SUCCESS;
}

int pendulum_low_level() {
    //const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rk8pd;
    const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rkf45;

    gsl_odeiv2_step *s = gsl_odeiv2_step_alloc(T, 4);
    gsl_odeiv2_control *c = gsl_odeiv2_control_y_new(1e-6, 0.0);
    gsl_odeiv2_evolve *e = gsl_odeiv2_evolve_alloc(4);

    double g_l = 9.81 / 1.0;  //1 meter
    gsl_odeiv2_system sys = {pendulum_func, NULL, 4, &g_l};

    double h = 1e-3;

    int rc = 0;

    double t = 0.0, t1 = 20.0;
    //double y[4] = {M_PI_2, M_PI_4, 0, 0};
    double y[4] = {0, 1.0, 1.0, 0};

    while (t < t1) {
        printf("%.5e %.5e %.5e %.5e %.5e\n", t, y[0], y[1], y[2], y[3]);

        int status = gsl_odeiv2_evolve_apply(e, c, s, &sys, &t, t1, &h, y);
        if (h > 0.1) {
            h = 0.1;
        }

        if (status != GSL_SUCCESS) {
            WMQ_LOG_ERROR("gsl_odeiv2_evolve_apply error %d", status);
            rc = WMQE_EXTERNAL_ERROR;
            break;
        }
        //fprintf(stderr, "%.5f step:%.5f err: %.5f %.5f %.5f %.5f\n", t, h, e->yerr[0], e->yerr[1], e->yerr[2], e->yerr[3]);
    }

    printf("%.5e %.5e %.5e %.5e %.5e\n", t, y[0], y[1], y[2], y[3]);

    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free(s);
    return rc;
}