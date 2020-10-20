import numpy
import ctypes

name = "MPC_embotech_single_integrator_Full_scale_launch_20201020185541_tunkapgen"
requires_callback = False
lib = "lib/libMPC_embotech_single_integrator_Full_scale_launch_20201020185541_tunkapgen.so"
lib_static = "lib/libMPC_embotech_single_integrator_Full_scale_launch_20201020185541_tunkapgen.a"
c_header = "include/MPC_embotech_single_integrator_Full_scale_launch_20201020185541_tunkapgen.h"

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("eq_C"                , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   4),   12),
 ("eq_D"                , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   4),   12),
 ("eq_c"                , "dense" , ""               , ctypes.c_double, numpy.float64, (  3,   1),    3),
 ("cost_H_fin"          , "dense" , ""               , ctypes.c_double, numpy.float64, (  4,   4),   16),
 ("cost_H"              , "dense" , ""               , ctypes.c_double, numpy.float64, (  4,   4),   16)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("u0"                  , ""      , ""               , ctypes.c_double, numpy.float64,     (  1,),    1)]

# Info Struct Fields
info = \
[('it', ctypes.c_int32),
('it2opt', ctypes.c_int32),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('pobj', ctypes.c_double),
('dobj', ctypes.c_double),
('dgap', ctypes.c_double),
('rdgap', ctypes.c_double),
('gradient_lag_norm', ctypes.c_double),
('mu', ctypes.c_double),
('mu_aff', ctypes.c_double),
('sigma', ctypes.c_double),
('lsit_aff', ctypes.c_int32),
('lsit_cc', ctypes.c_int32),
('step_aff', ctypes.c_double),
('step_cc', ctypes.c_double),
('solvetime', ctypes.c_double)
]