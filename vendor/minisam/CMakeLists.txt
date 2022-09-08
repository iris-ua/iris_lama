set(minisam_SRC
    minisam/core/Factor.cpp
    minisam/core/FactorGraph.cpp
    minisam/core/Key.cpp
    minisam/core/LossFunction.cpp
    minisam/core/SchurComplement.cpp
    minisam/core/VariableOrdering.cpp
    minisam/core/Variables.cpp
    minisam/utils/testAssertions.cpp
    minisam/utils/Timer.cpp
    minisam/linear/ConjugateGradient.cpp
    minisam/linear/Covariance.cpp
    minisam/linear/DenseCholesky.cpp
    minisam/linear/SchurComplementDenseSolver.cpp
    minisam/linear/SparseCholesky.cpp
    minisam/linear/SquareRootCholesky.cpp
    minisam/nonlinear/DoglegOptimizer.cpp
    minisam/nonlinear/GaussNewtonOptimizer.cpp
    minisam/nonlinear/LevenbergMarquardtOptimizer.cpp
    minisam/nonlinear/linearization.cpp
    minisam/nonlinear/MarginalCovariance.cpp
    minisam/nonlinear/NonlinearOptimizer.cpp
    minisam/nonlinear/NumericalFactor.cpp
    minisam/nonlinear/SparsityPattern.cpp
)

add_library(minisam OBJECT ${minisam_SRC})
target_include_directories(minisam PRIVATE ${EIGEN3_INCLUDE_DIR})
