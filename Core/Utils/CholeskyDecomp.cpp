/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "CholeskyDecomp.h"

CholeskyDecomp::CholeskyDecomp()
 : L(0)
{
    cholmod_start(&Common);
}

CholeskyDecomp::~CholeskyDecomp()
{
    cholmod_finish(&Common);
}

void CholeskyDecomp::freeFactor()
{
    assert(L);
    cholmod_free_factor(&L, &Common);
    L = 0;
}

Eigen::VectorXd CholeskyDecomp::solve(const Jacobian & jacobian, const Eigen::VectorXd & residual, const bool firstRun)
{
    cholmod_sparse * At = cholmod_allocate_sparse(jacobian.cols(), jacobian.rows.size(), jacobian.nonZero(), true, true, 0, CHOLMOD_REAL, &Common);

    int* p = (int*) At->p;
    int* i = (int*) At->i;
    double* x = (double*) At->x;
    int n = 0;
    *p = n;

    for(size_t r = 0; r < jacobian.rows.size(); r++)
    {
        memcpy(i, jacobian.rows.at(r)->indices, jacobian.rows.at(r)->nonZeros() * sizeof(int));
        memcpy(x, jacobian.rows.at(r)->vals, jacobian.rows.at(r)->nonZeros()* sizeof(double));

        i += jacobian.rows.at(r)->nonZeros();
        x += jacobian.rows.at(r)->nonZeros();
        n += jacobian.rows.at(r)->nonZeros();
        p++;
        *p = n;
    }

    if(firstRun)
    {
        assert(!L);

        L = cholmod_analyze(At, &Common);
    }

    cholmod_factor * L_factor = cholmod_copy_factor(L, &Common);

    cholmod_factorize(At, L_factor, &Common);

    cholmod_change_factor(CHOLMOD_REAL, true, false, true, true, L_factor, &Common);

    cholmod_dense* Arhs = cholmod_zeros(At->ncol, 1, CHOLMOD_REAL, &Common);

    memcpy(Arhs->x, residual.data(), At->ncol * sizeof(double));

    cholmod_dense* Atb = cholmod_zeros(At->nrow, 1, CHOLMOD_REAL, &Common);

    double alpha[2] = { 1., 0. };
    double beta[2] = { 0., 0. };

    cholmod_sdmult(At, 0, alpha, beta, Arhs, Atb, &Common);

    cholmod_dense* Atb_perm = cholmod_solve(CHOLMOD_P, L_factor, Atb, &Common);

    cholmod_dense * rhs = cholmod_solve(CHOLMOD_L, L_factor, Atb_perm, &Common);

    cholmod_dense* delta_cm = cholmod_solve(CHOLMOD_Lt, L_factor, rhs, &Common);

    Eigen::VectorXd delta(rhs->nrow);

    for(size_t i = 0; i < At->nrow; i++)
    {
        delta(((int *)L_factor->Perm)[i]) = ((double*)delta_cm->x)[i];
    }

    cholmod_free_dense(&delta_cm, &Common);
    cholmod_free_dense(&Atb_perm, &Common);
    cholmod_free_dense(&Atb, &Common);
    cholmod_free_dense(&Arhs, &Common);
    cholmod_free_sparse(&At, &Common);
    cholmod_free_dense(&rhs, &Common);
    cholmod_free_factor(&L_factor, &Common);

    return delta;
}
