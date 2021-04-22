//#include <gsl/gsl_blas.h>
#include <gsl/gsl_errno.h>
#include <lapacke.h>

#include "wmq_error.h"

int inverse_matrix(double m[], int size) {
    //https://stackoverflow.com/questions/3519959/computing-the-inverse-of-a-matrix-using-lapack-in-c

    int pivot[size];
    int err;
    dgetrf_(&size, &size, m, &size, pivot, &err);
    if (err < 0) {
        WMQ_LOG_ERROR("dgetrf_: invalid argument %d", -err);
        return WMQE_EXTERNAL_ERROR;
    }
    if (err > 0) {
        WMQ_LOG_ERROR("dgetrf_: singular decomposition");
        return WMQE_EXTERNAL_ERROR;
    }

    int lwork = size;
    double work[lwork];

    dgetri_(&size, m, &size, pivot, work, &lwork, &err);
    if (err < 0) {
        WMQ_LOG_ERROR("dgetri_: invalid argument %d", -err);
        return WMQE_EXTERNAL_ERROR;
    }
    if (err > 0) {
        WMQ_LOG_ERROR("dgetri_: singular decomposition");
        return WMQE_EXTERNAL_ERROR;
    }

    return 0;
}