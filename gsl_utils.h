#pragma once
#include <gsl/gsl_matrix.h>

int gsl_matrix_fprint(FILE *f, const gsl_matrix *m);

int gsl_matrix_print(const gsl_matrix *m);