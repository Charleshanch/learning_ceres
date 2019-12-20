/**
 * @file   learn_ceres.cpp
 * @author Chenhao Han <hanch0820@gmail.com>
 * @date   12/20/2019
 * @brief  The exe for testing and learning usage of ceres library
 *
 * @copyright
 * Copyright (C) 2019.
 * All rights reserved.
 */

///@ Ceres
#include <ceres/ceres.h>

///@ Google Logging
#include <glog/logging.h>

///@ STL
#include <iostream>

///@ functor to evaluate
struct CostFunctor
{
  template <typename T>
  bool operator()(const T* const x, T* residual) const
  {
    residual[0] = T(10.0) - x[0];
    return true;
  }
};

struct NumericDiffCostFunctor
{
  bool operator()(const double* const x, double* residual) const
  {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

class QuadraticCostFunction : public ceres::SizedCostFunction<1,1>
{
public:
  virtual ~QuadraticCostFunction() {}
  virtual bool Evaluate(double const* const* parameters,
                        double* residual,
                        double** jacobians)const
  {
    const double x = parameters[0][0];
    residual[0] = 10 - x;

    // compute jacobian if required
    if (jacobians != NULL && jacobians[0] != NULL)
    {
      jacobians[0][0] = -1;
    }

    return true;
  }
};

///@ functors for Powell's function
struct F1
{
  template <typename T>
  bool operator()(const T* const x1, const T* const x2, T* residual) const
  {
    residual[0] = x1[0] + T(10.0)*x2[0];
    return true;
  }
};

struct F2
{
  template<typename T>
  bool operator()(const T* const x3, const T* const x4, T* residual) const
  {
    residual[0] = T(sqrt(5.0)) * (x3[0] - x4[0]);
    return true;
  }
};

struct F3
{
  template<typename T>
  bool operator()(const T* const x2, const T* const x3, T* residual) const
  {
    residual[0] = (x2[0] - T(2.0)*x3[0]) * (x2[0] - T(2.0)*x3[0]);
    return true;
  }
};

struct F4
{
  template<typename T>
  bool operator()(const T* const x1, const T* const x4, T* residual) const
  {
    residual[0] = T(sqrt(10.0)) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
    return true;
  }
};

DEFINE_string(minimizer, "trust_region",
              "Minimizer type to use, choices are: line_search & trust_region");

int main(int argc, char** argv)
{
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ///@ variable to solve for with initial value
  double init_x = 5.0;
  double x = init_x;

  ///@ build problem
  ceres::Problem problem;

  ///@ hello world problem
  {
    ///@ setup cost function
    // ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor,1,1>(new CostFunctor);
    // problem.AddResidualBlock(cost_function, NULL, &x);
  }

  ///@ numeric derivatives
  {
    ///@ setup cost function
    // ceres::CostFunction* cost_function = new ceres::NumericDiffCostFunction<NumericDiffCostFunctor,ceres::CENTRAL,1,1>(new NumericDiffCostFunctor);
    // problem.AddResidualBlock(cost_function, NULL, &x);
  }

  ///@ analytic derivatives
  {
    // ceres::CostFunction* cost_function = new QuadraticCostFunction;
    // problem.AddResidualBlock(cost_function,NULL,&x);
  }

  ///@ powell's function
  double x1 = 3.0; double x2 = -1.0; double x3 = 0.0; double x4 = 1.0;
  {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F1,1,1,1>(new F1),NULL,&x1,&x2);
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F2,1,1,1>(new F2),NULL,&x3,&x4);
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F3,1,1,1>(new F3),NULL,&x2,&x3);
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<F4,1,1,1>(new F4),NULL,&x1,&x4);
  }

  ///@ run the solver
  ceres::Solver::Options options;
  LOG_IF(FATAL,!ceres::StringToMinimizerType(FLAGS_minimizer,
                                             &options.minimizer_type))
        << "Invalid minimizer: " << FLAGS_minimizer
        << ", valid options are: trust_region and line_search.";
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  std::cout << "Initial x1 = " << x1
          << ", x2 = " << x2
          << ", x3 = " << x3
          << ", x4 = " << x4
          << "\n";

  ceres::Solver::Summary summary;
  ceres::Solve(options,&problem,&summary);

  std::cout << summary.FullReport() << "\n";
  // std::cout << "x : " << init_x
  //           << "-> " << x << std::endl;
  std::cout << "Final x1 = " << x1
            << ", x2 = " << x2
            << ", x3 = " << x3
            << ", x4 = " << x4
            << "\n";

  return 0;
}
