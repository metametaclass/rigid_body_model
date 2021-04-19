#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "wmq_error.h"
#include "phys_constants.h"

static int free_fall_func(double t, const double y[], double f[],
                          void *params)
{
    (void)(t); /* avoid unused parameter warning */
    //double weight = *(double *)params;
    f[0] = y[2];     //dx/dt = vx
    f[1] = y[3];     //dy/dt = vy
    f[2] = 0;        //dvx/dt = 0
    f[3] = -G_ACCEL; //dvy/dt = -g
    //f[3] = -(y[1] * 100);  //dvy/dt = -w*y
    return GSL_SUCCESS;
}

static int free_fall_jac(double t, const double y[], double *dfdy,
                         double dfdt[], void *params)
{
    (void)(t); /* avoid unused parameter warning */
    (void)(params);
    gsl_matrix_view dfdy_mat = gsl_matrix_view_array(dfdy, 4, 4);
    gsl_matrix *m = &dfdy_mat.matrix;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            gsl_matrix_set(m, i, j, 0.0);
        }
    }

    gsl_matrix_set(m, 0, 2, 1.0); //dvx/dvx
    gsl_matrix_set(m, 1, 3, 1.0); //dvy/dvy

    dfdt[0] = 0.0;
    dfdt[1] = 0.0;
    dfdt[2] = 0.0;
    dfdt[3] = 0.0;
    return GSL_SUCCESS;
}

int free_fall_driver()
{
    double weight = 1;
    gsl_odeiv2_system sys = {free_fall_func, free_fall_jac, 4, &weight};
    //x,y
    //vx,vy

    gsl_odeiv2_driver *d = gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rkf45, 1e-6, 1e-6, 0.0);
    int i;
    int rc = 0;
    int N = 100;
    double t = 0.0, t1 = 10.0;
    double y[4] = {0.0, 0.0, 10.0 * 0.707, 10.0 * 0.707};
    //double y[4] = {0.0, 0.0, 0, 10.0};

    for (i = 1; i <= N; i++)
    {
        printf("%.5e %.5e %.5e %.5e %.5e\n", t, y[0], y[1], y[2], y[3]);

        double ti = i * t1 / N;
        int status = gsl_odeiv2_driver_apply(d, &t, ti, y);

        if (status != GSL_SUCCESS)
        {
            WMQ_LOG_ERROR("gsl_odeiv2_evolve_apply error %d", status);
            rc = WMQE_EXTERNAL_ERROR;
            break;
        }
        if (y[1] < 0)
        {
            //below initial level
            break;
        }
    }
    printf("%.5e %.5e %.5e %.5e %.5e\n", t, y[0], y[1], y[2], y[3]);

    gsl_odeiv2_driver_free(d);
    return rc;
}

int free_fall_low_level()
{
    //const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rk8pd;
    const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rkf45;

    gsl_odeiv2_step *s = gsl_odeiv2_step_alloc(T, 4);
    gsl_odeiv2_control *c = gsl_odeiv2_control_y_new(1e-6, 0.0);
    gsl_odeiv2_evolve *e = gsl_odeiv2_evolve_alloc(4);

    double weight = 1;
    gsl_odeiv2_system sys = {free_fall_func, free_fall_jac, 4, &weight};
    //x,y
    //vx,vy

    double h = 1e-3;

    int rc = 0;

    double t = 0.0, t1 = 10.0;
    double y[4] = {0.0, 0.0, 10.0 * 0.707, 10.0 * 0.707};

    while (t < t1 && y[1] >= -0.001)
    {
        printf("%.5e %.5e %.5e %.5e %.5e\n", t, y[0], y[1], y[2], y[3]);

        int status = gsl_odeiv2_evolve_apply(e, c, s, &sys, &t, t1, &h, y);

        //ode solver has exact solution for differential equations with constant second derivative with any step, so step size has unbound growth
        if (h > 0.1)
        {
            h = 0.1;
        }

        if (status != GSL_SUCCESS)
        {
            WMQ_LOG_ERROR("gsl_odeiv2_evolve_apply error %d", status);
            rc = WMQE_EXTERNAL_ERROR;
            break;
        }
        fprintf(stderr, "%.5f step:%.5f err: %.5f %.5f %.5f %.5f\n", t, h, e->yerr[0], e->yerr[1], e->yerr[2], e->yerr[3]);
    }

    printf("%.5e %.5e %.5e %.5e %.5e\n", t, y[0], y[1], y[2], y[3]);

    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free(s);
    return rc;
}
