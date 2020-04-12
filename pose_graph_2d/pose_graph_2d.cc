// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2016 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: vitus@google.com (Michael Vitus)
//
// An example of solving a graph-based formulation of Simultaneous Localization
// and Mapping (SLAM). It reads a 2D pose graph problem definition file in the
// g2o format, formulates and solves the Ceres optimization problem, and outputs
// the original and optimized poses to file for plotting.

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "angle_local_parameterization.h"
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "pose_graph_2d_error_term.h"
#include "types_extended.h"

DEFINE_string(input, "", "The pose graph definition filename in g2o format.");

namespace ceres {
namespace examples {

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.

void BuildOptimizationProblem(const std::vector<ConstraintBase*>& constraints,
                              std::map<int, PoseBase*>& poses,
                              ceres::Problem* problem) {
  CHECK(!poses.empty());
  CHECK(problem != NULL);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  ceres::LossFunction* loss_function = NULL;
  ceres::LocalParameterization* angle_local_parameterization =
      AngleLocalParameterization::Create();

  for (std::vector<ConstraintBase*>::const_iterator constraints_iter =
           constraints.begin();
       constraints_iter != constraints.end(); ++constraints_iter) {

    if ((*constraints_iter)->type == ceres::examples::EDGE_SE2){
      //this edge is from pose to pose
      const Constraint2d* constraint = static_cast<Constraint2d*>(*constraints_iter);

      std::map<int, PoseBase*>::iterator pose_begin_iter =
          poses.find(constraint->id_begin);
      CHECK(pose_begin_iter != poses.end())
          << "Pose with ID: " << constraint->id_begin << " not found.";
      Pose2d* pose_begin =  static_cast<Pose2d*>(poses[pose_begin_iter->first]);


      std::map<int, PoseBase*>::iterator pose_end_iter =
          poses.find(constraint->id_end);
      CHECK(pose_end_iter != poses.end())
          << "Pose with ID: " << constraint->id_end << " not found.";
      Pose2d* pose_end =  static_cast<Pose2d*>(poses[pose_end_iter->first]);


      const Eigen::Matrix3d sqrt_information =
          constraint->information.llt().matrixL();
      // Ceres will take ownership of the pointer.
      ceres::CostFunction* cost_function = PoseGraph2dErrorTerm::Create(
          constraint->x, constraint->y, constraint->yaw_radians, sqrt_information);
      problem->AddResidualBlock(
          cost_function, loss_function, &pose_begin->x,
          &pose_begin->y, &pose_begin->yaw_radians,
          &pose_end->x, &pose_end->y,
          &pose_end->yaw_radians);

      problem->SetParameterization(&pose_begin->yaw_radians,
                                  angle_local_parameterization);
      problem->SetParameterization(&pose_end->yaw_radians,
                                  angle_local_parameterization);

    }else if((*constraints_iter)->type == ceres::examples::EDGE_SE2_XY){

        //this edge is from pose to landmark

        const Constraint2dXY* constraint = static_cast<Constraint2dXY*>(*constraints_iter);

        std::map<int, PoseBase*>::iterator pose_begin_iter =
            poses.find(constraint->id_begin);
        CHECK(pose_begin_iter != poses.end())
            << "Pose with ID: " << constraint->id_begin << " not found.";
        Pose2d* pose_begin =  static_cast<Pose2d*>(poses[pose_begin_iter->first]);


        std::map<int, PoseBase*>::iterator pose_end_iter =
            poses.find(constraint->id_end);
        CHECK(pose_end_iter != poses.end())
            << "Pose with ID: " << constraint->id_end << " not found.";
        Pose2dXY* pose_end =  static_cast<Pose2dXY*>(poses[pose_end_iter->first]);


        const Eigen::Matrix2d sqrt_information =
            constraint->information.llt().matrixL();
        // Ceres will take ownership of the pointer.
        ceres::CostFunction* cost_function = PoseXYGraph2dErrorTerm::Create(
            constraint->x, constraint->y, sqrt_information);
        problem->AddResidualBlock(
            cost_function, loss_function, &pose_begin->x,
            &pose_begin->y, &pose_begin->yaw_radians,
            &pose_end->x, &pose_end->y);

        problem->SetParameterization(&pose_begin->yaw_radians,
                                    angle_local_parameterization);
    }
  }

  // The pose graph optimization problem has three DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigate this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  std::map<int, PoseBase*>::iterator pose_start_iter = poses.begin();
  CHECK(pose_start_iter != poses.end()) << "There are no poses.";
  Pose2d* pose_start =  static_cast<Pose2d*>(poses[pose_start_iter->first]);
  problem->SetParameterBlockConstant(&pose_start->x);
  problem->SetParameterBlockConstant(&pose_start->y);
  problem->SetParameterBlockConstant(&pose_start->yaw_radians);
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem* problem) {
  CHECK(problem != NULL);

  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

// Output the poses to the file with format: ID x y yaw_radians.
bool OutputPoses(const std::string& filename,
                 std::map<int, PoseBase*> &poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
   std::cerr << "Error opening the file: " << filename << '\n';
   return false;
  }
  for (std::map<int, PoseBase*>::const_iterator poses_iter = poses.begin();
       poses_iter != poses.end(); ++poses_iter) {
    //const std::map<int, Pose2d>::value_type& pair = poses_iter->first;
    int id = poses_iter->first;

    Pose2d* pose =  static_cast<Pose2d*>(poses[id]);

    outfile <<  id << " " << pose->x << " " << pose->y
            << ' ' << pose->yaw_radians << '\n';
  }
  return true;
}


void ReadConstraint(std::ifstream& infile,
                    std::vector<ConstraintBase*> &constraints) {
  Constraint2d* constraint = new Constraint2d();
  constraint->deposit(infile);
  constraint -> type = ceres::examples::EDGE_SE2;
  constraints.push_back(constraint);
}


void ReadConstraintXY(std::ifstream& infile,
                    std::vector<ConstraintBase*> &constraints) {
  Constraint2dXY* constraint = new Constraint2dXY();
  constraint->deposit(infile);
  constraint -> type = ceres::examples::EDGE_SE2_XY;
  constraints.push_back(constraint);
}

bool ReadVertex(std::ifstream& infile,
                std::map<int, PoseBase*> &poses) {

  int id;
  double x, y, yaw;
  infile >> id >> x >> y >> yaw;

  // Ensure we don't have duplicate poses.
  if (poses.find(id) != poses.end()) {
    LOG(ERROR) << "Duplicate vertex with ID: " << id;
    return false;
  }


  Pose2d* pose = new Pose2d(x,y,yaw);
  poses[id] = pose;
  poses[id] -> type = ceres::examples::VERTEX_SE2;

  return true;
}

bool ReadVertexXY(std::ifstream& infile,
                std::map<int, PoseBase*> &poses) {
  int id;
  double x, y;
  infile >> id >> x >> y;

  // Ensure we don't have duplicate poses.
  if (poses.find(id) != poses.end()) {
    LOG(ERROR) << "Duplicate vertex with ID: " << id;
    return false;
  }

  Pose2dXY* pose = new Pose2dXY(x,y);
  poses[id] = pose;
  poses[id] -> type = ceres::examples::VERTEX_XY;

  return true;
}

bool ReadG2oFile(const std::string& filename,
                 std::map<int, PoseBase*> &poses,
                 std::vector<ConstraintBase*> &constraints) {
  CHECK(poses.empty());
  CHECK(constraints.empty());

  poses.clear();
  constraints.clear();

  std::ifstream infile(filename.c_str());
  if (!infile) {
    return false;
  }

  std::string data_type;
  while (infile.good()) {
    // Read whether the type is a node or a constraint.
    infile >> data_type;
    if (data_type == Pose2d::name()) {
      if (!ReadVertex(infile, poses)) {
        return false;
      }
    }
    else if (data_type == Pose2dXY::name()){
      if (!ReadVertexXY(infile, poses)) {
        return false;
      }
    }
    else if (data_type == Constraint2d::name()) {
      ReadConstraint(infile, constraints);
    }
    else if (data_type == Constraint2dXY::name()) {
      ReadConstraintXY(infile, constraints);
    }else {
      LOG(ERROR) << "Unknown data type: " << data_type;
      return false;
    }

    // Clear any trailing whitespace from the line.
    infile >> std::ws;
  }

  return true;
}

}  // namespace examples
}  // namespace ceres

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_input != "") << "Need to specify the filename to read.";

  std::map<int, ceres::examples::PoseBase*> poses;
  std::vector<ceres::examples::ConstraintBase*> constraints;

  CHECK(ceres::examples::ReadG2oFile(FLAGS_input, poses, constraints))
      << "Error reading the file: " << FLAGS_input;

  std::cout << "Number of poses: " << poses.size() << '\n';
  std::cout << "Number of constraints: " << constraints.size() << '\n';

  CHECK(ceres::examples::OutputPoses("poses_original.txt", poses))
      << "Error outputting to poses_original.txt";

  ceres::Problem problem;
  ceres::examples::BuildOptimizationProblem(constraints, poses, &problem);

  CHECK(ceres::examples::SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  CHECK(ceres::examples::OutputPoses("poses_optimized.txt", poses))
      << "Error outputting to poses_original.txt";

  return 0;
}
