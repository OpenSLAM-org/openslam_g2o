// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <iostream>
#include <map>
#include <vector>

#include "rand.h"
#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "edge_se2.h"
#include "edge_se2_pointxy.h"
#include "se2.h"

#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

struct Landmark
{
  int id;
  Vector2d truePose;
  Vector2d simulatedPose;
  vector<int> seenBy;
  Landmark() : id(-1) {}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
typedef vector<Landmark*> LandmarkVector;
typedef map<int, map<int, LandmarkVector> > LandmarkGrid;

/**
 * simulated pose of the robot
 */
struct GridPose
{
  int id;
  SE2 truePose;
  SE2 simulatorPose;
  LandmarkVector landmarks;      ///< the landmarks observed by this node
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct GridEdge
{
  int from;
  int to;
  SE2 trueTransf;
  SE2 simulatorTransf;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

enum MotionType {
  MO_LEFT, MO_RIGHT,
  MO_NUM_ELEMS
};

typedef vector<GridPose, Eigen::aligned_allocator<GridPose> >  PosesVector;
typedef vector<const GridPose*> PosesPtrVector;
typedef map<int, map<int, PosesPtrVector> > PosesGrid;

SE2 sampleTransformation(const SE2& trueMotion_, const Vector2d& transNoise, double rotNoise)
{
  Vector3d trueMotion = trueMotion_.toVector();
  SE2 noiseMotion(
      trueMotion[0] + Rand::gauss_rand(0.0, transNoise[0]),
      trueMotion[1] + Rand::gauss_rand(0.0, transNoise[1]),
      trueMotion[2] + Rand::gauss_rand(0.0, rotNoise));
  return noiseMotion;
}

GridPose generateNewPose(const GridPose& prev, const SE2& trueMotion, const Vector2d& transNoise, double rotNoise)
{
  GridPose nextPose;
  nextPose.id = prev.id + 1;
  nextPose.truePose = prev.truePose * trueMotion;
  SE2 noiseMotion = sampleTransformation(trueMotion, transNoise, rotNoise);
  nextPose.simulatorPose = prev.simulatorPose * noiseMotion;
  return nextPose;
}

SE2 getMotion(int motionDirection, double stepLen)
{
  switch (motionDirection) {
    case MO_LEFT:
      return SE2(stepLen, 0, 0.5*M_PI);
    case MO_RIGHT:
      return SE2(stepLen, 0, -0.5*M_PI);
    default:
      cerr << "Unknown motion direction" << endl;
      return SE2(stepLen, 0, -0.5*M_PI);
  }
}

int main()
{
  // simulate a robot observing landmarks while travelling on a grid
  int steps = 5;
  double stepLen = 1.0;
  int numNodes = 300;
  int boundArea = 50;
  int seed = time(0);

  double maxSensorRangeLandmarks = 2.5 * stepLen;

  int landMarksPerSquareMeter = 1;
  double observationProb = 0.8;

  int landmarksRange=2;

  Vector2d transNoise(0.05, 0.01);
  double rotNoise = DEG2RAD(2.);
  Vector2d landmarkNoise(0.05, 0.05);

  Vector2d bound(boundArea, boundArea);
  Rand::seed_rand(seed);

  VectorXd probLimits(MO_NUM_ELEMS);
  for (int i = 0; i < probLimits.size(); ++i)
    probLimits[i] = (i + 1) / (double) MO_NUM_ELEMS;

  
  Matrix3d covariance;
  covariance.fill(0.);
  covariance(0, 0) = transNoise[0]*transNoise[0];
  covariance(1, 1) = transNoise[1]*transNoise[1];
  covariance(2, 2) = rotNoise*rotNoise;
  Matrix3d information = covariance.inverse();

  SE2 maxStepTransf(stepLen * steps, 0, 0);
  PosesVector poses;
  LandmarkVector landmarks;
  GridPose firstPose;
  firstPose.id = 0;
  firstPose.truePose = SE2(0,0,0);
  firstPose.simulatorPose = SE2(0,0,0);
  poses.push_back(firstPose);
  cerr << "sampling nodes ...";
  while ((int)poses.size() < numNodes) {
    // add straight motions
    for (int i = 1; i < steps && (int)poses.size() < numNodes; ++i) {
      GridPose nextGridPose = generateNewPose(poses.back(), SE2(stepLen,0,0), transNoise, rotNoise);
      poses.push_back(nextGridPose);
    }
    if ((int)poses.size() == numNodes)
      break;

    // sample a new motion direction
    double sampleMove = Rand::uniform_rand(0., 1.);
    int motionDirection = 0;
    while (probLimits[motionDirection] < sampleMove && motionDirection+1 < MO_NUM_ELEMS) {
      motionDirection++;
    }

    SE2 nextMotionStep = getMotion(motionDirection, stepLen);
    GridPose nextGridPose = generateNewPose(poses.back(), nextMotionStep, transNoise, rotNoise);

    // check whether we will walk outside the boundaries in the next iteration
    SE2 nextStepFinalPose = nextGridPose.truePose * maxStepTransf;
    if (fabs(nextStepFinalPose.translation().x()) >= bound[0] || fabs(nextStepFinalPose.translation().y()) >= bound[1]) {
      //cerr << "b";
      // will be outside boundaries using this
      for (int i = 0; i < MO_NUM_ELEMS; ++i) {
        nextMotionStep = getMotion(i, stepLen);
        nextGridPose = generateNewPose(poses.back(), nextMotionStep, transNoise, rotNoise);
        nextStepFinalPose = nextGridPose.truePose * maxStepTransf;
        if (fabs(nextStepFinalPose.translation().x()) < bound[0] && fabs(nextStepFinalPose.translation().y()) < bound[1])
          break;
      }
    }

    poses.push_back(nextGridPose);
  }
  cerr << "done." << endl;

  // creating landmarks along the trajectory
  cerr << "Creating landmarks ... ";
  LandmarkGrid grid;
  for (PosesVector::const_iterator it = poses.begin(); it != poses.end(); ++it) {
    int ccx = lrint(it->truePose.translation().x());
    int ccy = lrint(it->truePose.translation().y());
    for (int a=-landmarksRange; a<=landmarksRange; a++)
      for (int b=-landmarksRange; b<=landmarksRange; b++){
        int cx=ccx+a;
        int cy=ccy+b;
        LandmarkVector& landmarksForCell = grid[cx][cy];
        if (landmarksForCell.size() == 0) {
          for (int i = 0; i < landMarksPerSquareMeter; ++i) {
            Landmark* l = new Landmark();
            double offx, offy;
            do {
              offx = Rand::uniform_rand(-0.5*stepLen, 0.5*stepLen);
              offy = Rand::uniform_rand(-0.5*stepLen, 0.5*stepLen);
            } while (hypot_sqr(offx, offy) < 0.25*0.25);
            l->truePose[0] = cx + offx;
            l->truePose[1] = cy + offy;
            landmarksForCell.push_back(l);
            landmarks.push_back(l);
          }
        }
      }
  }
  cerr << "done." << endl;

  cerr << "Creating landmark observations for the simulated poses ... ";
  double maxSensorSqr = maxSensorRangeLandmarks * maxSensorRangeLandmarks;
  int globalId = 0;
  for (PosesVector::iterator it = poses.begin(); it != poses.end(); ++it) {
    GridPose& pv = *it;
    int cx = lrint(it->truePose.translation().x());
    int cy = lrint(it->truePose.translation().y());
    int numGridCells = (int)(maxSensorRangeLandmarks) + 1;

    pv.id = globalId++;
    SE2 trueInv = pv.truePose.inverse();

    for (int xx = cx - numGridCells; xx <= cx + numGridCells; ++xx)
      for (int yy = cy - numGridCells; yy <= cy + numGridCells; ++yy) {
        LandmarkVector& landmarksForCell = grid[xx][yy];
        if (landmarksForCell.size() == 0)
          continue;
        for (size_t i = 0; i < landmarksForCell.size(); ++i) {
          Landmark* l = landmarksForCell[i];
          double dSqr = hypot_sqr(pv.truePose.translation().x() - l->truePose.x(), pv.truePose.translation().y() - l->truePose.y());
          if (dSqr > maxSensorSqr)
            continue;
          double obs = Rand::uniform_rand(0.0, 1.0);
          if (obs > observationProb) // we do not see this one...
            continue;
          if (l->id < 0)
            l->id = globalId++;
          if (l->seenBy.size() == 0) {
            Vector2d trueObservation = trueInv * l->truePose;
            Vector2d observation = trueObservation;
            observation[0] += Rand::gauss_rand(0., landmarkNoise[0]);
            observation[1] += Rand::gauss_rand(0., landmarkNoise[1]);
            l->simulatedPose = pv.simulatorPose * observation;
          }
          l->seenBy.push_back(pv.id);
          pv.landmarks.push_back(l);
        }
      }

  }
  cerr << "done." << endl;


  /*********************************************************************************
   * creating the optimization problem
   ********************************************************************************/

  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  // allocating the optimizer
  SparseOptimizer optimizer;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* solver = new SlamBlockSolver(&optimizer, linearSolver);
  optimizer.setSolver(solver);

  // adding the odometry to the optimizer
  // first adding all the vertices
  cerr << "Adding robot poses ... ";
  for (size_t i = 0; i < poses.size(); ++i) {
    const GridPose& p = poses[i];
    const SE2& t = p.simulatorPose; 
    VertexSE2* robot =  new VertexSE2;
    robot->setId(p.id);
    robot->setEstimate(t);
    optimizer.addVertex(robot);
  }
  cerr << "done." << endl;

  // second add the odometry constraints
  cerr << "Adding odometry measurements ... ";
  for (size_t i = 1; i < poses.size(); ++i) {
    const GridPose& prev = poses[i-1];
    const GridPose& p = poses[i];

    SE2 transf = prev.simulatorPose.inverse() * p.simulatorPose;

    EdgeSE2* odometry = new EdgeSE2;
    odometry->vertices()[0] = optimizer.vertex(prev.id);
    odometry->vertices()[1] = optimizer.vertex(p.id);
    odometry->setMeasurement(transf);
    odometry->setInverseMeasurement(transf.inverse());
    odometry->setInformation(information);
    optimizer.addEdge(odometry);
  }
  cerr << "done." << endl;

  // add the landmark observations
  {
    Matrix2d covariance; covariance.fill(0.);
    covariance(0, 0) = landmarkNoise[0]*landmarkNoise[0];
    covariance(1, 1) = landmarkNoise[1]*landmarkNoise[1];
    Matrix2d information = covariance.inverse();

    cerr << "add landmark vertices ... ";
    for (size_t i = 0; i < poses.size(); ++i) {
      const GridPose& p = poses[i];
      for (size_t j = 0; j < p.landmarks.size(); ++j) {
        Landmark* l = p.landmarks[j];
        if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) {
          VertexPointXY* landmark = new VertexPointXY;
          landmark->setId(l->id);
          landmark->setEstimate(l->simulatedPose);
          optimizer.addVertex(landmark);
        }
      }
    }
    cerr << "done." << endl;

    cerr << "add landmark observations ... ";
    for (size_t i = 0; i < poses.size(); ++i) {
      const GridPose& p = poses[i];
      SE2 trueInv = p.truePose.inverse();
      for (size_t j = 0; j < p.landmarks.size(); ++j) {
        Landmark* l = p.landmarks[j];
        Vector2d observation;
        if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) { // write the initial position of the landmark
          observation = p.simulatorPose.inverse() * l->simulatedPose;
        } else {
          // create observation for the LANDMARK using the true positions
          Vector2d trueObservation = trueInv * l->truePose;
          observation = trueObservation;
          observation[0] += Rand::gauss_rand(0., landmarkNoise[0]);
          observation[1] += Rand::gauss_rand(0., landmarkNoise[1]);
        }

        EdgeSE2PointXY* landmarkObservation =  new EdgeSE2PointXY;
        landmarkObservation->vertices()[0] = optimizer.vertex(p.id);
        landmarkObservation->vertices()[1] = optimizer.vertex(l->id);
        landmarkObservation->setMeasurement(observation);
        landmarkObservation->setInverseMeasurement(-1.*observation);
        landmarkObservation->setInformation(information);
        optimizer.addEdge(landmarkObservation);
      }
    }
    cerr << "done." << endl;
  }



  /*********************************************************************************
   * optimization
   ********************************************************************************/

  // prepare and run the optimization
  VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(optimizer.vertex(0));
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  cerr << "Optimizing" << endl;
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  cerr << "done." << endl;

  // TODO does nothing so far
  return 0;
}
