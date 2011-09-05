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

#include "graph_optimizer_sparse.h"

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <iterator>
#include <cassert>
#include <algorithm>

#include "estimate_propagator.h"
#include "solver.h"
#include "batch_stats.h"
#include "hyper_graph_action.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/stuff/macros.h"
#include "g2o/config.h"

namespace g2o{
  using namespace std;


  SparseOptimizer::SparseOptimizer(){
    _method=GaussNewton;
    _currentLambda = -1.;
    _solver = 0;
    _forceStopFlag = 0;
    _tau = 1e-5;
    _goodStepUpperScale = 2./3.;
    _goodStepLowerScale = 1./3.;
    _userLambdaInit = 0;
    _statistics = 0;
    _maxTrialsAfterFailure = 10;
    _graphActions.resize(AT_NUM_ELEMENTS);
  }

  SparseOptimizer::~SparseOptimizer(){
    delete _solver;
    delete[] _statistics;
  }

  void SparseOptimizer::computeActiveErrors()
  {
    // call the callbacks in case there is something registered
    HyperGraphActionSet& actions = _graphActions[AT_COMPUTEACTIVERROR];
    if (actions.size() > 0) {
      for (HyperGraphActionSet::iterator it = actions.begin(); it != actions.end(); ++it)
        (*(*it))(this);
    }

    for (EdgeContainer::const_iterator it = _activeEdges.begin(); it != _activeEdges.end(); it++) {
      OptimizableGraph::Edge* e = *it;
      e->computeError();
      if (e->robustKernel()) { 
        e->robustifyError();
      }
    }
  }

  double SparseOptimizer::activeChi2( ) const
  {
    double chi = 0.0;
    for (EdgeContainer::const_iterator it = _activeEdges.begin(); it != _activeEdges.end(); it++) {
      const OptimizableGraph::Edge* e = *it;
      chi += e->chi2();
    }
    return chi;
  }

  OptimizableGraph::Vertex* SparseOptimizer::findGauge(){
    if (vertices().empty())
      return 0;

    int maxDim=0;
    for (HyperGraph::VertexIDMap::iterator it=vertices().begin(); it!=vertices().end(); it++){
      OptimizableGraph::Vertex* v=static_cast<OptimizableGraph::Vertex*>(it->second); 
      maxDim=std::max(maxDim,v->dimension());
    }
    
    OptimizableGraph::Vertex* rut=0;
    for (HyperGraph::VertexIDMap::iterator it=vertices().begin(); it!=vertices().end(); it++){
      OptimizableGraph::Vertex* v=static_cast<OptimizableGraph::Vertex*>(it->second);
      if (v->dimension()==maxDim){
        rut=v;
        break;
      }
    }
    return rut;
  }

  bool SparseOptimizer::gaugeFreedom()
  {
    if (vertices().empty())
      return false;

    int maxDim=0;
    for (HyperGraph::VertexIDMap::iterator it=vertices().begin(); it!=vertices().end(); it++){
      OptimizableGraph::Vertex* v=static_cast<OptimizableGraph::Vertex*>(it->second); 
      maxDim = std::max(maxDim,v->dimension());
    }

    for (HyperGraph::VertexIDMap::iterator it=vertices().begin(); it!=vertices().end(); it++){
      OptimizableGraph::Vertex* v=static_cast<OptimizableGraph::Vertex*>(it->second);
      if (v->dimension() == maxDim) {
        // test for full dimension prior
        for (HyperGraph::EdgeSet::const_iterator eit = v->edges().begin(); eit != v->edges().end(); ++eit) {
          OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*eit);
          if (e->vertices().size() == 1 && e->dimension() == maxDim)
            return false;
        }
      }
    }
    return true;
  }

  bool SparseOptimizer::buildIndexMapping(SparseOptimizer::VertexContainer& vlist){
    if (! vlist.size()){
      _ivMap.clear();
      return false;
    }

    _ivMap.resize(vlist.size());
    size_t i = 0;
    for (int k=0; k<2; k++)
      for (VertexContainer::iterator it=vlist.begin(); it!=vlist.end(); it++){
      OptimizableGraph::Vertex* v = *it;
      if (! v->fixed()){
        if (static_cast<int>(v->marginalized()) == k){
          v->setTempIndex(i);
          _ivMap[i]=v;
          i++;
        }
      }
      else {
        v->setTempIndex(-1);
      }
    }
    _ivMap.resize(i);
    return true;
  }

  void SparseOptimizer::clearIndexMapping(){
    for (size_t i=0; i<_ivMap.size(); i++){
      _ivMap[i]->setTempIndex(-1);
      _ivMap[i]=0;
    }
  }


  double SparseOptimizer::computeLambdaInit() {
    if (_userLambdaInit>0)
      return _userLambdaInit;
    double maxDiagonal=0.;
    for (size_t k = 0; k < _ivMap.size(); k++) {
      OptimizableGraph::Vertex* v=_ivMap[k];
      assert(v);
      int dim = v->dimension();
      for (int j = 0; j < dim; ++j){
        maxDiagonal = std::max(fabs(v->hessian(j,j)),maxDiagonal); 
      }
    }
    return _tau*maxDiagonal;
  }

  double SparseOptimizer::computeScale(double currentLMLambda, Solver* solver) {
    double scale=0;
    //double xSum=0;
    //double bSum=0;
    for (size_t j=0; j < solver->vectorSize(); j++){
      //xSum += solver->x()[j];
      //bSum += solver->b()[j];
      scale += solver->x()[j] * (currentLMLambda * solver->x()[j] + solver->b()[j]);
    }
    //cerr << PVAR(xSum) << " " << PVAR(bSum) << endl;
    return scale;
  }


  bool SparseOptimizer::initializeOptimization(int level){
    HyperGraph::VertexSet vset;
    for (VertexIDMap::iterator it=vertices().begin(); it!=vertices().end(); it++)
      vset.insert(it->second);
    return initializeOptimization(vset,level);
  }

  bool SparseOptimizer::initializeOptimization(HyperGraph::VertexSet& vset, int level){
    clearIndexMapping();
    _activeVertices.clear();
    _activeVertices.reserve(vset.size());
    _activeEdges.clear();
    set<Edge*> auxEdgeSet; // temporary structure to avoid duplicates
    for (HyperGraph::VertexSet::iterator it=vset.begin(); it!=vset.end(); it++){
      OptimizableGraph::Vertex* v= (OptimizableGraph::Vertex*) *it;
      const OptimizableGraph::EdgeSet& vEdges=v->edges();
      // count if there are edges in that level. If not remove from the pool
      int levelEdges=0;
      for (OptimizableGraph::EdgeSet::const_iterator it=vEdges.begin(); it!=vEdges.end(); it++){
        OptimizableGraph::Edge* e=reinterpret_cast<OptimizableGraph::Edge*>(*it);
        if (level < 0 || e->level() == level) {

          bool allVerticesOK = true;
          for (vector<HyperGraph::Vertex*>::const_iterator vit = e->vertices().begin(); vit != e->vertices().end(); ++vit) {
            if (vset.find(*vit) == vset.end()) {
              allVerticesOK = false;
              break;
            }
          }
          if (allVerticesOK) {
            auxEdgeSet.insert(reinterpret_cast<OptimizableGraph::Edge*>(*it));
            levelEdges++;
          }

        }
      }
      if (levelEdges){
        _activeVertices.push_back(v);
      }
    }

    _activeEdges.reserve(auxEdgeSet.size());
    for (set<Edge*>::iterator it = auxEdgeSet.begin(); it != auxEdgeSet.end(); ++it)
      _activeEdges.push_back(*it);

    sortVectorContainers();
    return buildIndexMapping(_activeVertices);
  }

  bool SparseOptimizer::initializeOptimization(HyperGraph::EdgeSet& eset){
    clearIndexMapping();
    _activeVertices.clear();
    _activeEdges.clear();
    _activeEdges.reserve(eset.size());
    set<Vertex*> auxVertexSet; // temporary structure to avoid duplicates
    for (HyperGraph::EdgeSet::iterator it=eset.begin(); it!=eset.end(); it++){
      OptimizableGraph::Edge* e=(OptimizableGraph::Edge*)(*it);
      for (vector<HyperGraph::Vertex*>::const_iterator vit = e->vertices().begin(); vit != e->vertices().end(); ++vit) {
        auxVertexSet.insert(static_cast<OptimizableGraph::Vertex*>(*vit));
      }
      _activeEdges.push_back(reinterpret_cast<OptimizableGraph::Edge*>(*it));
    }

    _activeVertices.reserve(auxVertexSet.size());
    for (set<Vertex*>::iterator it = auxVertexSet.begin(); it != auxVertexSet.end(); ++it)
      _activeVertices.push_back(*it);

    sortVectorContainers();
    return buildIndexMapping(_activeVertices);
  }

  void SparseOptimizer::computeInitialGuess()
  {
    OptimizableGraph::VertexSet emptySet;
    std::set<Vertex*> backupVertices;
    HyperGraph::VertexSet fixedVertices; // these are the root nodes where to start the initialization
    for (EdgeContainer::iterator it = _activeEdges.begin(); it != _activeEdges.end(); ++it) {
      OptimizableGraph::Edge* e = *it;
      for (size_t i = 0; i < e->vertices().size(); ++i) {
        OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(e->vertices()[i]);
        if (v->fixed())
          fixedVertices.insert(v);
        else { // check for having a prior which is able to fully initialize a vertex
          for (EdgeSet::const_iterator vedgeIt = v->edges().begin(); vedgeIt != v->edges().end(); ++vedgeIt) {
            OptimizableGraph::Edge* vedge = static_cast<OptimizableGraph::Edge*>(*vedgeIt);
            if (vedge->vertices().size() == 1 && vedge->initialEstimatePossible(emptySet, v) > 0.) {
              //cerr << "Initialize with prior for " << v->id() << endl;
              vedge->initialEstimate(emptySet, v);
              fixedVertices.insert(v);
            }
          }
        }
        if (v->tempIndex() == -1) {
          std::set<Vertex*>::const_iterator foundIt = backupVertices.find(v);
          if (foundIt == backupVertices.end()) {
            v->push();
            backupVertices.insert(v);
          }
        }
      }
    }

    EstimatePropagator estimatePropagator(this);
    EstimatePropagator::PropagateCost costFunction(this);
    estimatePropagator.propagate(fixedVertices, costFunction);

    // restoring the vertices that should not be initialized
    for (std::set<Vertex*>::iterator it = backupVertices.begin(); it != backupVertices.end(); ++it) {
      Vertex* v = *it;
      v->pop();
    }
    if (verbose()) {
      computeActiveErrors();
      cerr << "iteration= -1\t chi2= " << activeChi2()
          << "\t time= 0.0"
          << "\t cumTime= 0.0"
          << "\t (using initial guess from spanning tree)" << endl;
    }
  }

  int SparseOptimizer::optimize(int iterations, bool online)
  {
    bool useSchur=false;
    for (VertexContainer::iterator it=_activeVertices.begin(); it!=_activeVertices.end(); it++){
      OptimizableGraph::Vertex* v= *it;
      if (v->marginalized()){
        useSchur=true;
        break;
      }
    }

    double t;
    Solver* solver = _solver;
    if (useSchur){
      if  (_solver->supportsSchur())
        _solver->setSchur(true);
    } else {
      if  (_solver->supportsSchur())
        _solver->setSchur(false);
    }
    solver->init(online);
    solver->setLevenberg(_method == LevenbergMarquardt);

    int cjIterations=0;
    double cumTime=0;
    double currentChi=0;
    double tempChi=0;
    if (_method==LevenbergMarquardt){
      computeActiveErrors();
      currentChi=activeChi2();
      tempChi=currentChi;
    }
    bool ok=true;

    double ni=2.;
    //double currentLMLambda=1.;

    for (int i=0; i<iterations && ! terminate() && ok; i++){
      preIteration(i);

      G2OBatchStatistics* cstat = _statistics ? &(_statistics[i]) : 0;
      globalStats = cstat;
      if (cstat) {
        cstat->iteration = i;
        cstat->numEdges =  _activeEdges.size();
        cstat->numVertices = _activeVertices.size();
      }

      double ts =  get_time();
      if (i == 0 && !online) { // built up the CCS structure, here due to easy time measure
        ok = solver->buildStructure();
        if (! ok) {
          cerr << __PRETTY_FUNCTION__ << ": Failure while building CCS structure" << endl;
          return 0;
        }
      }

      t=get_time();
      computeActiveErrors();
      if (cstat)
        cstat->timeResiduals = get_time()-t;

      t=get_time();
      linearizeSystem();
      if (cstat)
        cstat->timeLinearize = get_time()-t;

      t=get_time();
      solver->buildSystem();
      if (cstat)
        cstat->timeQuadraticForm = get_time()-t;

      if (_method==GaussNewton){
        t=get_time();
        ok = solver->solve();
        if (cstat)
          cstat->timeLinearSolution = get_time()-t;

        t=get_time();
        update(solver->x());
        if (cstat) {
          cstat->timeUpdate = get_time()-t;
        }
      } else if (_method==LevenbergMarquardt){

        if (i==0) {
	  _currentLambda = computeLambdaInit();
	  //cerr << PVAR(_currentLambda) << endl;
        }

        double rho=0;
        int qmax=0;
        do {
          push(_activeVertices);
          if (cstat)
            cstat->levenbergIterations++;
          // update the diagonal of the system matrix
          t=get_time();
          solver->setLambda(_currentLambda);
          bool ok2 = solver->solve();
          if (cstat)
            cstat->timeLinearSolution+=get_time()-t;

          t=get_time();
          update(solver->x());
          if (cstat) {
            cstat->timeUpdate = get_time()-t;
          }
          //cerr << "Update (Levenberg)   " << get_time()-t <<  "sec" << endl;

          // restore the diagonal
          solver->setLambda(- _currentLambda);

          t=get_time();
          computeActiveErrors();
          tempChi=activeChi2();
          //cerr << "Error (Levenberg)   " << get_time()-t <<  "sec" << endl;

          if (! ok2)
            tempChi=std::numeric_limits<double>::max();

          rho = (currentChi-tempChi);
          double scale = computeScale(_currentLambda, solver);
          scale += 1e-3;
          //cerr << PVAR(scale) << endl;
          //cerr << PVAR(rho) << endl;
          // cerr << PVAR(tempChi) << endl;
          // cerr << PVAR(currentChi) << endl;
          //cerr << PVAR(_currentLambda) << endl;
          rho /=  scale ;
          // cerr << PVAR(rho) << endl;

          if (rho>0){ // last step was good
            // cerr << "step was good! " << endl; 
            // cerr << " delta_chi: " << currentChi-tempChi; 
            // cerr << " scale: " << scale << endl;
            // cerr << " lambda_old: "<< _currentLambda;
            // cerr << " rho: " << rho; 

            double alpha = 1.-pow((2*rho-1),3);
            // cerr << " alpha: " << alpha;

            // crop lambda between minimum and maximum factors
            alpha = std::min(alpha, _goodStepUpperScale);
            double scaleFactor = std::max(_goodStepLowerScale, alpha);
            // cerr << " scaleFactor: " << scaleFactor;

            _currentLambda *= scaleFactor;
            // cerr << " lambda_new: "<< _currentLambda << endl;

            ni = 2;
            currentChi=tempChi;
            t=get_time();
            discardTop(_activeVertices);
            //cerr << "discardTop (Levenberg)   " << get_time()-t <<  "sec" << endl;
          } else {
	    //cerr << "step was bad, lambda should increase! "; 
	    //cerr << PVAR(tempChi - currentChi) << " ";
	    //cerr << "lambda_old: "<< _currentLambda;
            _currentLambda*=ni;
            //cerr << "lambda_new: "<< _currentLambda << endl;
            ni*=2;
            t=get_time();
            pop(_activeVertices); // restore the last state before trying to optimize
            //cerr << "Pop (Levenberg)   " << get_time()-t <<  "sec" << endl;
          }
          qmax++;
        } while (rho<0 && qmax < _maxTrialsAfterFailure && ! terminate());

        if (qmax == _maxTrialsAfterFailure || rho ==0)
          i=iterations;
      } // end LevenbergMarquardt

      bool errorComputed = false;
      if (cstat) {
        computeActiveErrors();
        errorComputed = true;
        cstat->chi2 = activeChi2();
        cstat->timeIteration = get_time()-ts;
      }

      double dts=get_time()-ts;
      cumTime+=dts;
      if (verbose()){
        if (! errorComputed)
          computeActiveErrors();
        cerr << "iteration= " << i
          << "\t chi2= " << FIXED(activeChi2())
          << "\t time= " << dts
          << "\t cumTime= " << cumTime
          << "\t lambda= " << FIXED(_currentLambda)
          << "\t edges= " << _activeEdges.size()
          << "\t schur= " << useSchur  << endl;
      }
      ++cjIterations; 
      postIteration(i);
    }
    if (! ok)
      return 0;
    return cjIterations;
  }


  void SparseOptimizer::linearizeSystem()
  {
#   ifdef G2O_OPENMP
#   pragma omp parallel for default (shared) if (_activeEdges.size() > 50)
#   endif
    for (size_t k = 0; k < _activeEdges.size(); ++k) {
      OptimizableGraph::Edge* e = _activeEdges[k];
      e->linearizeOplus(); // jacobian of the nodes' oplus (manifold)
    }
  }

  void SparseOptimizer::update(double* update)
  {
    // update the graph by calling oplus on the vertices
    for (size_t i=0; i < _ivMap.size(); ++i) {
      OptimizableGraph::Vertex* v= _ivMap[i];
      v->oplus(update);
      update += v->dimension();
    }
  }

  bool SparseOptimizer::updateInitialization(HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset)
  {
    std::vector<HyperGraph::Vertex*> newVertices;
    newVertices.reserve(vset.size());
    _activeVertices.reserve(_activeVertices.size() + vset.size());
    //for (HyperGraph::VertexSet::iterator it = vset.begin(); it != vset.end(); ++it)
      //_activeVertices.push_back(static_cast<OptimizableGraph::Vertex*>(*it));
    _activeEdges.reserve(_activeEdges.size() + eset.size());
    for (HyperGraph::EdgeSet::iterator it = eset.begin(); it != eset.end(); ++it)
      _activeEdges.push_back(static_cast<OptimizableGraph::Edge*>(*it));
    
    // update the index mapping
    size_t next = _ivMap.size();
    for (HyperGraph::VertexSet::iterator it = vset.begin(); it != vset.end(); ++it) {
      OptimizableGraph::Vertex* v=static_cast<OptimizableGraph::Vertex*>(*it);
      if (! v->fixed()){
        if (! v->marginalized()){
          v->setTempIndex(next);
          _ivMap.push_back(v);
          newVertices.push_back(v);
          _activeVertices.push_back(v);
          next++;
        } 
        else // not supported right now
          abort();
      }
      else {
        v->setTempIndex(-1);
      }
    }

    //if (newVertices.size() != vset.size())
    //cerr << __PRETTY_FUNCTION__ << ": something went wrong " << PVAR(vset.size()) << " " << PVAR(newVertices.size()) << endl;
    return _solver->updateStructure(newVertices, eset);
  }

  void SparseOptimizer::sortVectorContainers()
  {
    // sort vector structures to get deterministic ordering based on IDs
    sort(_activeVertices.begin(), _activeVertices.end(), VertexIDCompare());
    sort(_activeEdges.begin(), _activeEdges.end(), EdgeIDCompare());
  }

  void SparseOptimizer::clear() {
    _ivMap.clear();
    _activeVertices.clear();
    _activeEdges.clear();
    HyperGraph::clear();
  }

  SparseOptimizer::VertexContainer::const_iterator SparseOptimizer::findActiveVertex(OptimizableGraph::Vertex* v) const
  {
    VertexContainer::const_iterator lower = lower_bound(_activeVertices.begin(), _activeVertices.end(), v, VertexIDCompare());
    if (lower == _activeVertices.end())
      return _activeVertices.end();
    if ((*lower) == v)
      return lower;
    return _activeVertices.end();
  }

  SparseOptimizer::EdgeContainer::const_iterator SparseOptimizer::findActiveEdge(OptimizableGraph::Edge* e) const
  {
    EdgeContainer::const_iterator lower = lower_bound(_activeEdges.begin(), _activeEdges.end(), e, EdgeIDCompare());
    if (lower == _activeEdges.end())
      return _activeEdges.end();
    if ((*lower) == e)
      return lower;
    return _activeEdges.end();
  }

  void SparseOptimizer::push(SparseOptimizer::VertexContainer& vlist)
  {
    for (VertexContainer::iterator it = vlist.begin(); it != vlist.end(); ++it)
      (*it)->push();
  }

  void SparseOptimizer::pop(SparseOptimizer::VertexContainer& vlist)
  {
    for (VertexContainer::iterator it = vlist.begin(); it != vlist.end(); ++it)
      (*it)->pop();
  }

  void SparseOptimizer::push(HyperGraph::VertexSet& vlist)
  {
    for (HyperGraph::VertexSet::iterator it = vlist.begin(); it != vlist.end(); ++it) {
      OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(*it);
      if (v)
	v->push();
      else 
	cerr << "FATAL PUSH SET" << endl;
    }
  }

  void SparseOptimizer::pop(HyperGraph::VertexSet& vlist)
  {
    for (HyperGraph::VertexSet::iterator it = vlist.begin(); it != vlist.end(); ++it){
      OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*> (*it);
      if (v)
	v->pop();
      else 
	cerr << "FATAL POP SET" << endl;
    }
  }

  void SparseOptimizer::discardTop(SparseOptimizer::VertexContainer& vlist)
  {
    for (VertexContainer::iterator it = vlist.begin(); it != vlist.end(); ++it)
      (*it)->discardTop();
  }

  void SparseOptimizer::setVerbose(bool verbose)
  {
    _verbose = verbose;
  }

  void SparseOptimizer::setMethod(SparseOptimizer::Method method)
  {
    _method = method;
  }

  void SparseOptimizer::setSolver(Solver* solver)
  {
    _solver = solver;
  }

  void SparseOptimizer::setUserLambdaInit(double lambda)
  {
    _userLambdaInit = lambda;
  }

  bool SparseOptimizer::computeMarginals()
  {
    return _solver->computeMarginals();
  }

  bool SparseOptimizer::computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices){
    return _solver->computeMarginals(spinv, blockIndices);
  }


  void SparseOptimizer::setMaxTrialsAfterFailure(int max_trials)
  {
    _maxTrialsAfterFailure = max_trials;
  }

  void SparseOptimizer::setForceStopFlag(bool* flag)
  {
    _forceStopFlag=flag;
  }

  bool SparseOptimizer::removeVertex(Vertex* v)
  {
    if (v->tempIndex() >= 0) {
      clearIndexMapping();
      _ivMap.clear();
    }
    return HyperGraph::removeVertex(v);
  }

  bool SparseOptimizer::addComputeErrorAction(HyperGraphAction* action)
  {
    std::pair<HyperGraphActionSet::iterator, bool> insertResult = _graphActions[AT_COMPUTEACTIVERROR].insert(action);
    return insertResult.second;
  }

  bool SparseOptimizer::removeComputeErrorAction(HyperGraphAction* action)
  {
    return _graphActions[AT_COMPUTEACTIVERROR].erase(action) > 0;
  }

} // end namespace
