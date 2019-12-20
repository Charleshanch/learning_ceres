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

int main(int argc, char** argv)
{
  // google::InitGoogleLogging(argv[0]);

  ///@ variable to solve for with initial value
  double init_x = 5.0;
  double x = init_x;

  ///@ build problem
  ceres::Problem problem;

  ///@ setup cost function
  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor,1,1>(new CostFunctor);

  problem.AddResidualBlock(cost_function, NULL, &x);

  ///@ run the solver
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;

  ceres::Solve(options,&problem,&summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << init_x
            << "-> " << x << std::endl;


  return 0;
}
