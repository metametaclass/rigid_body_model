#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "wmq_error.h"

#define G_ACCEL 9.81

static int rbm_func(double t, const double y[], double f[],
                    void *params) {
    (void)(t); /* avoid unused parameter warning */
    //double weight = *(double *)params;
    f[0] = y[2];      //dx/dt = vx
    f[1] = y[3];      //dy/dt = vy
    f[2] = 0;         //dvx/dt = 0
    f[3] = -G_ACCEL;  //dvy/dt = -g
    return GSL_SUCCESS;
}

int rigid_body_motion_driver() {
    double weight = 1;
    gsl_odeiv2_system sys = {rbm_func, NULL, 4, &weight};
    //x,y
    //vx,vy

    gsl_odeiv2_driver *d = gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rkf45, 1e-6, 1e-6, 0.0);
    int i;
    int rc = 0;
    int N = 100;
    double t = 0.0, t1 = 10.0;
    double y[4] = {0.0, 0.0, 10.0 * 0.707, 10.0 * 0.707};
    //double y[4] = {0.0, 0.0, 0, 10.0};

    for (i = 1; i <= N; i++) {
        printf("%.5e %.5e %.5e %.5e %.5e\n", t, y[0], y[1], y[2], y[3]);

        double ti = i * t1 / N;
        int status = gsl_odeiv2_driver_apply(d, &t, ti, y);

        if (status != GSL_SUCCESS) {
            WMQ_LOG_ERROR("gsl_odeiv2_evolve_apply error %d", status);
            rc = WMQE_EXTERNAL_ERROR;
            break;
        }
        if (y[1] < 0) {
            //below initial level
            break;
        }
    }
    printf("%.5e %.5e %.5e %.5e %.5e\n", t, y[0], y[1], y[2], y[3]);

    gsl_odeiv2_driver_free(d);
    return rc;
}

int rigid_body_motion_low_level() {
    //const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rk8pd;
    const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rkf45;

    gsl_odeiv2_step *s = gsl_odeiv2_step_alloc(T, 4);
    gsl_odeiv2_control *c = gsl_odeiv2_control_y_new(1e-6, 0.0);
    gsl_odeiv2_evolve *e = gsl_odeiv2_evolve_alloc(4);

    double weight = 1;
    gsl_odeiv2_system sys = {rbm_func, NULL, 4, &weight};
    //x,y
    //vx,vy

    double h = 1e-6;

    int rc = 0;

    double t = 0.0, t1 = 10.0;
    double y[4] = {0.0, 0.0, 10.0 * 0.707, 10.0 * 0.707};
    //double y[4] = {0.0, 0.0, 0.0, 100.0};

    while (t < t1 && y[1] >= -0.001) {
        printf("%.5e %.5e %.5e %.5e %.5e\n", t, y[0], y[1], y[2], y[3]);

        int status = gsl_odeiv2_evolve_apply(e, c, s,
                                             &sys,
                                             &t, t1,
                                             &h, y);
        //printf("%.5f\n", h);
        if (status != GSL_SUCCESS) {
            WMQ_LOG_ERROR("gsl_odeiv2_evolve_apply error %d", status);
            rc = WMQE_EXTERNAL_ERROR;
            break;
        }
    }

    printf("%.5e %.5e %.5e %.5e %.5e\n", t, y[0], y[1], y[2], y[3]);

    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free(s);
    return rc;
}
