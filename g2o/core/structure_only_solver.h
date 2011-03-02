// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef STRUCTURE_ONLY_SOLVER_H
#define STRUCTURE_ONLY_SOLVER_H

#include "hyper_graph.h"
#include "base_vertex.h"
#include "base_binary_edge.h"


namespace g2o
{

/**
 * \brief This is a solver for "structure-only" optimization"
 *
 * Given the problem of landmark-based SLAM or bundle adjustment, this class performs
 * optimization on the landmarks while the poses are kept fixed. This can be done very
 * efficiently, since the position on th landmarks are indepdented given the poses are
 * known.
 *
 * This class slightly misuses the API of g2o. It is designed in a way, it can work
 * on the very same graph which reflects the problem of landmark-based SLAM, or bundle
 * adjustment and which is meant to be solved using the Schur complement. Thus, it can
 * be called just before or after joint-optimization without the need of additional
 * setup.
 *
 * Therefore, it is assumed that the graph has the following structure:
 *  - ...
 */
template <int PointDoF>
class StructureOnlySolver
{
public:
  StructureOnlySolver()
  {
    _verbose = true;
  }

  void calc(g2o::HyperGraph::VertexIDMap & vertices,
            int num_iters,
            list<int> & nan_list,
            double step_factor = 0.75,
            double min_step_length = 0.1)
  {
    double chi2_sum=0;
    double old_chi2_sum=0;


    for (g2o::HyperGraph::VertexIDMap::iterator it_v=vertices.begin();
         it_v!=vertices.end(); ++it_v)
    {
      bool stop = false;


      g2o::OptimizableGraph::Vertex  * v
          = dynamic_cast<g2o::OptimizableGraph::Vertex  *>(it_v->second);

      if(v->marginalized()==0)
        continue;

      assert(v->dimension() == PointDoF);

      g2o::HyperGraph::EdgeSet & track = v->edges();

      assert(track.size()>=2);

      double chi2 = 0;


      for (g2o::HyperGraph::EdgeSet::iterator it_t=track.begin();
           it_t!=track.end(); ++it_t)
      {
        g2o::OptimizableGraph::Edge * e
            = dynamic_cast<g2o::OptimizableGraph::Edge *>(*it_t);

        e->computeError();
        if(e->robustKernel())
        {
          e->robustifyError();
        }
        chi2 += e->chi2();
      }


      old_chi2_sum += chi2;
      if (v->fixed()==false)
      {
        for (int i_g=0; i_g<num_iters; ++i_g)
        {
          double rho = 0; //Assign value, so the compiler is not complaining...

          Matrix<double, PointDoF,PointDoF> H_pp;
          H_pp.setZero();


          v->mapHessianMemory(&(H_pp(0,0)));
          v->clearQuadraticForm();

          g2o::HyperGraph::EdgeSet & track = v->edges();


          assert(track.size()>=2);

          double max_err = 0;

          for (g2o::HyperGraph::EdgeSet::iterator it_t=track.begin();
               it_t!=track.end(); ++it_t)
          {

            g2o::OptimizableGraph::Edge * e
                = dynamic_cast<g2o::OptimizableGraph::Edge *>(*it_t);

            assert(v==e->vertices()[0]);

            g2o::OptimizableGraph::Vertex * frame_v
                = dynamic_cast<g2o::OptimizableGraph::Vertex *>
                (e->vertices()[1]);

            bool remember_fix_status = frame_v->fixed();
            frame_v->setFixed(true);
            // Fix frame to prevent g2o to compute its Jacobian

            e->computeError();
            if(e->robustKernel())
            {
              e->robustifyError();
            }
            e->linearizeOplus();



            e->constructQuadraticForm();

            // Restore frame's initial fixed() values
            frame_v->setFixed(remember_fix_status);
          }

          Matrix<double,PointDoF,1> b(v->bData());

          if (b.norm()<0.0000000000000000001)
          {
            stop = true;
            break;
          }
          Matrix<double,PointDoF,1> delta_p;

          delta_p = H_pp.ldlt().solve(b);

          double new_chi2 = 0;

          double new_max_err=0;

          double step_length = 1.;
          do
          {
            v->push();

            Matrix<double,PointDoF,1> delta_p_times_length
                = delta_p*step_length;
            v->oplus(&(delta_p_times_length[0]));
            for (g2o::HyperGraph::EdgeSet::iterator it_t=track.begin();
                 it_t!=track.end(); ++it_t)
            {
              g2o::OptimizableGraph::Edge * e
                  = dynamic_cast<g2o::OptimizableGraph::Edge *>(*it_t);
              e->computeError();
              if(e->robustKernel())
              {
                e->robustifyError();
              }
              new_chi2 += e->chi2();
            }
            if (isnan(new_chi2))
            {
              v->discardTop();
              nan_list.push_back(v->id());

              rho = -1;
              break;

            }
            rho = (chi2-new_chi2);
            if (rho>0)
            {
              v->discardTop();
              max_err = new_max_err;
              chi2 = new_chi2;
              break;
            }
            else
            {
              v->pop();
              step_length *= step_factor;
            }
          }while(step_length>=min_step_length);

          if (rho<=0)
            break;
        }
      }
      chi2_sum += chi2;
    }
    if (_verbose>0)
      std::cout << " chi vs. new_chi2 "
                << old_chi2_sum
                << " vs. "
                << chi2_sum
                << std::endl;
  }

  bool verbose() const
  {
    return _verbose;
  }

  void setVerbose(bool verbose)
  {
    _verbose = verbose;
  }

protected:
  bool _verbose;

};

}

#endif
