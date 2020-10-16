/*
ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622 : A fast customized optimization solver.

Copyright (C) 2013-2020 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif

#ifndef ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_H
#define ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_H

#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif


/* DATA TYPE ------------------------------------------------------------*/
typedef double ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float;


typedef double ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622interface_float;

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622
#define MISRA_C_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622 (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622
#define RESTRICT_CODE_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622 (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622
#define SET_PRINTLEVEL_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622    (0)
#endif

/* timing */
#ifndef SET_TIMING_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622
#define SET_TIMING_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622         (200)	

/* scaling factor of line search (affine direction) */
#define SET_LS_SCALE_AFF_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622  (ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float)(0.9)      

/* scaling factor of line search (combined direction) */
#define SET_LS_SCALE_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622      (ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float)(0.95)  

/* minimum required step size in each iteration */
#define SET_LS_MINSTEP_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622    (ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float)(1E-08)

/* maximum step size (combined direction) */
#define SET_LS_MAXSTEP_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622    (ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float)(0.995)

/* desired relative duality gap */
#define SET_ACC_RDGAP_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622     (ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622     (ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622   (ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622  (ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float)(1E-06)

/* desired maximum violation of stationarity (only checked if value is > 0) */
#define SET_ACC_KKTSTAT_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622  (ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float)(-1)

/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622 (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622   (2)

/* no progress in line search possible */
#define NOPROGRESS_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622   (-7)

/* fatal internal error - nans occurring */
#define NAN_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622  (-10)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622   (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622   (-12)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622  (-100)


/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_params
{
    /* matrix of size [3 x 4] (column major format) */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float eq_C[12];

    /* matrix of size [3 x 4] (column major format) */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float eq_D[12];

    /* vector of size 3 */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float eq_c[3];

    /* matrix of size [4 x 4] (column major format) */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float cost_H_fin[16];

    /* matrix of size [4 x 4] (column major format) */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float cost_H[16];

    /* Scalar */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float solver_timeout;

    /* Scalar */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float timeout_estimate_coeff;

} ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_output
{
    /* vector of size 1 */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float u0[1];

} ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_info
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float res_ineq;

    /* primal objective */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float pobj;	
	
    /* dual objective */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float dobj;	

    /* duality gap := pobj - dobj */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float rdgap;		

	/* infinity norm of gradient of Lagrangian*/
	ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float gradient_lag_norm;

    /* duality measure */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float mu;

	/* duality measure (after affine step) */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float mu_aff;
	
    /* centering parameter */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float step_aff;
    
    /* step size (combined direction) */
    ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float step_cc;    

	/* solvertime */
	ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_float solvetime;   



} ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_info;









/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* User License expires on: (UTC) Friday, March 12, 2021 9:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: Indefinite */
/* Solver Generation Request Id: 9edb5708-15a9-40a1-b43f-948cbf741c11 */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif
extern solver_int32_default ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_solve(ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_params *params, ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_output *output, ARIS_Euler_MPC_embotech_single_integrator_hor5_20201016001622_info *info, FILE *fs);

#ifdef __cplusplus
}
#endif

#endif
