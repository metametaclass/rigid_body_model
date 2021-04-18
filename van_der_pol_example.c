#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "wmq_error.h"

static int vdpe_func(double t, const double y[], double f[],
                     void *params) {
    (void)(t); /* avoid unused parameter warning */
    double mu = *(double *)params;
    f[0] = y[1];
    f[1] = -y[0] - mu * y[1] * (y[0] * y[0] - 1);
    return GSL_SUCCESS;
}

static int vdpe_jac(double t, const double y[], double *dfdy,
                    double dfdt[], void *params) {
    (void)(t); /* avoid unused parameter warning */
    double mu = *(double *)params;
    gsl_matrix_view dfdy_mat = gsl_matrix_view_array(dfdy, 2, 2);
    gsl_matrix *m = &dfdy_mat.matrix;
    gsl_matrix_set(m, 0, 0, 0.0);
    gsl_matrix_set(m, 0, 1, 1.0);
    gsl_matrix_set(m, 1, 0, -2.0 * mu * y[0] * y[1] - 1.0);
    gsl_matrix_set(m, 1, 1, -mu * (y[0] * y[0] - 1.0));
    dfdt[0] = 0.0;
    dfdt[1] = 0.0;
    return GSL_SUCCESS;
}

int van_der_pol_example_driver() {
    double mu = 10;
    gsl_odeiv2_system sys = {vdpe_func, vdpe_jac, 2, &mu};

    gsl_odeiv2_driver *d =
        gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rk8pd,
                                      1e-6, 1e-6, 0.0);
    int i;
    int rc;
    double t = 0.0, t1 = 100.0;
    double y[2] = {1.0, 0.0};

    for (i = 1; i <= 100; i++) {
        double ti = i * t1 / 100.0;
        int status = gsl_odeiv2_driver_apply(d, &t, ti, y);

        if (status != GSL_SUCCESS) {
            WMQ_LOG_ERROR("gsl_odeiv2_evolve_apply error %d", status);
            rc = WMQE_EXTERNAL_ERROR;
            break;
        }

        printf("%.5e %.5e %.5e\n", t, y[0], y[1]);
    }

    gsl_odeiv2_driver_free(d);
    return rc;
}

int van_der_pol_example_low_level() {
    const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rk8pd;
    //const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rkf45;

    gsl_odeiv2_step *s = gsl_odeiv2_step_alloc(T, 2);
    gsl_odeiv2_control *c = gsl_odeiv2_control_y_new(1e-6, 0.0);
    gsl_odeiv2_evolve *e = gsl_odeiv2_evolve_alloc(2);

    double mu = 10;
    gsl_odeiv2_system sys = {vdpe_func, vdpe_jac, 2, &mu};

    double t = 0.0, t1 = 100.0;
    double h = 1e-6;
    double y[2] = {1.0, 0.0};

    int rc = 0;

    while (t < t1) {
        int status = gsl_odeiv2_evolve_apply(e, c, s,
                                             &sys,
                                             &t, t1,
                                             &h, y);

        if (status != GSL_SUCCESS) {
            WMQ_LOG_ERROR("gsl_odeiv2_evolve_apply error %d", status);
            rc = WMQE_EXTERNAL_ERROR;
            break;
        }

        printf("%.5e %.5e %.5e\n", t, y[0], y[1]);
    }

    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free(s);
    return rc;
}