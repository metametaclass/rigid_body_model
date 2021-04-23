#include <gsl/gsl_blas.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "gsl_utils.h"
#include "inverse_matrix.h"
#include "phys_constants.h"
#include "quaternion.h"
#include "wmq_error.h"
// In the "xyz (pitch-roll-yaw) convention," theta is pitch, psi is roll, and phi is yaw.

//https://www.cs.cmu.edu/~baraff/pbm/rigid1.pdf
//x,y,z (NED coordinates)
//unit quaternion (s, ux, uy, uz) -> rotation matrix / psi, theta, phi (roll(body x axis rotation),pitch (body y axis rotation),yaw (body z axis rotation))

//d(velocify) <- forces
//d(angular moment) <- torques

#define RBM_DIMENSION 13

int rbm_func(double t, const double y[], double f[],
             void *params) {
    (void)(t); /* avoid unused parameter warning */

    //inverted inertia tensor in body coordinates
    gsl_matrix_view Ibody_inv_view = gsl_matrix_view_array((double *)params, 3, 3);

    f[0] = y[7];  //dx/dt = vx
    f[1] = y[8];  //dy/dt = vy
    f[2] = y[9];  //dz/dt = vz

    //TODO: check structure alignment?
    //q = y[3..6]
    quaternion *q = (quaternion *)&y[3];

    //R = quaternion_to_matrix(normalize(q))
    quaternion q_norm = quaternion_normalized(*q);
    double R[9];  //rotation matrix in row-major order
    quaternion_to_rotation_matrix(q_norm, R);
    gsl_matrix_view R_view = gsl_matrix_view_array(R, 3, 3);

    //Iinv = R * Ibodyinv * Transpose(R);
    double Iinv_1[9] = {0.0};  //0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    gsl_matrix_view Iinv_view_1 = gsl_matrix_view_array(Iinv_1, 3, 3);
    int rc = gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, &R_view.matrix, &Ibody_inv_view.matrix, 0.0, &Iinv_view_1.matrix);
    if (rc != 0) {
        WMQ_LOG_ERROR("gsl_blas_dgemm: step 1 %d", rc);
        return rc;
    }
    double Iinv_2[9] = {0.0};  //0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    gsl_matrix_view Iinv_view_2 = gsl_matrix_view_array(Iinv_2, 3, 3);
    rc = gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &Iinv_view_1.matrix, &R_view.matrix, 0.0, &Iinv_view_2.matrix);
    if (rc != 0) {
        WMQ_LOG_ERROR("gsl_blas_dgemm: step 2 %d", rc);
        return rc;
    }

    //L = y[10..12]
    _gsl_vector_const_view angular_moment = gsl_vector_const_view_array(&y[10], 3);
    double angular_velocity[4] = {0.0};  //0.0, 0.0, 0.0, 0.0
    gsl_vector_view angular_velocity_view = gsl_vector_view_array(&angular_velocity[1], 3);
    //omega = Iinv * L (y[10..12])
    rc = gsl_blas_dgemv(CblasNoTrans, 1.0, &Iinv_view_2.matrix, &angular_moment.vector, 0.0, &angular_velocity_view.vector);
    if (rc != 0) {
        WMQ_LOG_ERROR("gsl_blas_dgemv: step 2 %d", rc);
        return rc;
    }

    //dq/dt = 1/2* omega * q
    quaternion *angular_velocity_quaterion = (quaternion *)angular_velocity;
    quaternion dq = quaternion_multiply(*angular_velocity_quaterion, *q);
    quaternion_inplace_multiply_scalar(&dq, 0.5);

    f[3] = dq.w;
    f[4] = dq.x;
    f[5] = dq.y;
    f[6] = dq.z;

    f[7] = 0;
    f[8] = 0;
    //f[9] = G_ACCEL; // z - down axis
    f[9] = 0;  // z - down axis

    //no torques, no changes in angular moment
    f[10] = 0;
    f[11] = 0;
    f[12] = 0;

    //double r = (rand() / (0.5 * RAND_MAX)) - 1.0;
    double r = rand() * 1.0 / RAND_MAX;

    if (r < 0.01) {
        f[12] = 10000;
        //f[11] = 50000;
    }

    return GSL_SUCCESS;
}

void dump_state(double t, double y_state[]) {
    printf("%.5e", t);
    for (int i = 0; i < RBM_DIMENSION; i++) {
        printf(" %.5e", y_state[i]);
    }
    printf("\n");
}

int rigid_body_motion_low_level() {
    const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rk8pd;
    //const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rkf45;

    gsl_odeiv2_step *s = gsl_odeiv2_step_alloc(T, RBM_DIMENSION);
    gsl_odeiv2_control *c = gsl_odeiv2_control_y_new(1e-6, 0.0);
    gsl_odeiv2_evolve *e = gsl_odeiv2_evolve_alloc(RBM_DIMENSION);

    //3-axis symmetrical body
    double inertia_tensor_body1[9] = {
        1.0,
        0.0,
        0.0,

        0.0,
        0.5,
        0.0,

        0.0,
        0.0,
        2.0,
    };

    //openscad\handle.off

    double inertia_tensor_body[9] = {5.55781127e+04, -3.78956126e-15, -3.61256812e-01,
                                     -3.78956126e-15, 6.03715315e+04, 1.69772344e-12,
                                     -3.61256812e-01, 1.69772344e-12, 3.03519112e+04};

    double inertia_tensor_body_inverted[9] = {0};
    for (int i = 0; i < 9; i++) {
        inertia_tensor_body_inverted[i] = inertia_tensor_body[i];
    }

    int rc = inverse_matrix(inertia_tensor_body_inverted, 3);
    WMQ_CHECK_ERROR_AND_RETURN_RESULT(rc, "inverse_matrix");

    gsl_matrix_view I_inv_view = gsl_matrix_view_array(inertia_tensor_body_inverted, 3, 3);

    //gsl_matrix_print(&I_inv_view.matrix);

    /*
    double inertia_tensor_body_inverted[9] = {
        1.0,
        0.0,
        0.0,

        0.0,
        2.0,
        0.0,

        0.0,
        0.0,
        0.5,
    };*/

    gsl_odeiv2_system sys = {rbm_func, NULL, RBM_DIMENSION, &inertia_tensor_body_inverted};

    double h = 1e-3;

    rc = 0;

    double t = 0.0, t1 = 100.0;

    //x,y,z=0
    //w,x,y,z = 1,0,0,0 //unit quaternion, no rotation cos(a/2), v*sin(a/2)
    //velocity = 1,0,-10
    //TODO: find initial angular moment from inertia tensor ang angular speed
    // L = I*w (w=0,1,0; rotation about y axis)
    double y[RBM_DIMENSION] = {/*x,y,z*/ 0.0, 0.0, 0.0, /*q.w,q.x,q.y,q.z*/ 1.0, 0.0, 0.0, 0.0, /*vx,vy,vz*/ 0.0, 0.0, 0.0, /*Lx,Ly,Lz*/ 500000.0, 0.0, 0.0};

    //double prev_t = t;

    while (t < t1) {
        dump_state(t, y);

        int status = gsl_odeiv2_evolve_apply(e, c, s, &sys, &t, t1, &h, y);

        if (h > 0.1) {
            h = 0.1;
        }

        if (status != GSL_SUCCESS) {
            WMQ_LOG_ERROR("gsl_odeiv2_evolve_apply error %d", status);
            rc = WMQE_EXTERNAL_ERROR;
            break;
        }

        //small perturbation for https://en.wikipedia.org/wiki/Tennis_racket_theorem
        /*if (t > 10.0 && prev_t <= 10.0)
        {
            y[11] = y[11] + 0.001;
        }
        else
        {
            y[11] = 0.0;
        }*/

        //prev_t = t;
        //fprintf(stderr, "%.5f step:%.5f err: %.5f %.5f %.5f %.5f\n", t, h, e->yerr[0], e->yerr[1], e->yerr[2], e->yerr[3]);
    }

    dump_state(t, y);

    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free(s);
    return rc;
}
