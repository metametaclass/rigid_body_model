#include <gsl/gsl_matrix.h>
#include <stdio.h>

#include "wmq_error.h"

int gsl_matrix_fprint(FILE *f, const gsl_matrix *m) {
    int status, n = 0;

    for (size_t i = 0; i < m->size1; i++) {
        for (size_t j = 0; j < m->size2; j++) {
            if ((status = fprintf(f, "%g ", gsl_matrix_get(m, i, j))) < 0)
                return WMQE_EXTERNAL_ERROR;
            n += status;
        }

        if ((status = fprintf(f, "\n")) < 0)
            return -1;
        n += status;
    }

    return 0;
}

int gsl_matrix_print(const gsl_matrix *m) {
    return gsl_matrix_fprint(stderr, m);
}