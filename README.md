# pose-graph-optimizaion

Sample codes to conduct 2D pose graph optimization with Ceres Solver.

# Dependences

- Eigen 3.3 or later
- Ceres Solver 1.12.0 or later
- Gflags 2.2.0 or later
- Python with matplotlib

# Build

```bash
$ git clone https://github.com/shinsumicco/pose-graph-optimization.git
$ cd pose-graph-optimization
$ mkdir build
$ cd build
$ cmake ..
$ make -j4
```

# Optimize

```bash
$ cd pose-graph-optimization/build
$ bin/se2_optimize --filename ../sample/manhattan.g2o
Number of poses: 3500
Number of constraints: 5598

Solver Summary (v 1.13.0-eigen-(3.3.4)-lapack-suitesparse-(4.5.5)-cxsparse-(3.1.9)-no_openmp)

                                     Original                  Reduced
Parameter blocks                        10500                    10497
Parameters                              10500                    10497
Residual blocks                          5598                     5598
Residual                                16794                    16794

Minimizer                        TRUST_REGION

Sparse linear algebra library    SUITE_SPARSE
Trust region strategy     LEVENBERG_MARQUARDT

                                        Given                     Used
Linear solver          SPARSE_NORMAL_CHOLESKY   SPARSE_NORMAL_CHOLESKY
Threads                                     1                        1
Linear solver threads                       1                        1
Linear solver ordering              AUTOMATIC                    10497

Cost:
Initial                          3.457147e+04
Final                            7.303833e+01
Change                           3.449843e+04

Minimizer iterations                       17
Successful steps                           17
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                           0.0215

  Residual evaluation                  0.0096
  Jacobian evaluation                  0.0282
  Linear solver                        0.1898
Minimizer                              0.2525

Postprocessor                          0.0005
Total                                  0.2745

Termination:                      CONVERGENCE (Function tolerance reached. |cost_change|/cost: 2.743214e-07 <= 1.000000e-06)

```

# Visualize

```bash
$ cd pose-graph-optimization
$ python script/plot_results.py build/poses_original.txt build/poses_optimized.txt
```
