"""race_planner. CasADi's bundled OpenBLAS (under IPOPT's MUMPS) runs one thread per core by
default; on the planner's problems its threads mostly spin (an L0 lap of Monza: 35 threads, ~7
cores busy, 3.3x slower than on one), so unless the caller has set them, one. Set here, before
anything imports casadi; the plans are the same (regress-check: same on all six runs)."""
import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
