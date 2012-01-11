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

#ifndef G2O__GRAPH_OPTIMIZER_CHOL_H_
#define G2O__GRAPH_OPTIMIZER_CHOL_H_

#include "optimizable_graph.h"

#include "solver.h"

#include <map>

// forward declaration
namespace g2o {
class ActivePathCostFunction;
}

namespace g2o {

  struct G2OBatchStatistics;

  struct SparseOptimizer : public OptimizableGraph {

    enum Method{GaussNewton, LevenbergMarquardt};

    enum {
      AT_COMPUTEACTIVERROR = OptimizableGraph::AT_NUM_ELEMENTS,
      AT_NUM_ELEMENTS, // keep as last element
    };

    friend class ActivePathCostFunction;

    // Attention: _solver & _statistics is own by SparseOptimizer and will be
    // deleted in its destructor.
    SparseOptimizer();
    virtual ~SparseOptimizer();

    // new interface for the optimizer
    // the old functions will be dropped
    /**
       Initializes the structures for optimizing a portion of the graph specified by a subset of edges.
       Before calling it be sure to invoke marginalized() and fixed() to the vertices you want to include in the 
       schurr complement or to set as fixed during the optimization.
       @param eset: the subgraph to be optimized.
       @returns false if somethings goes wrong
     */
    virtual bool initializeOptimization(HyperGraph::EdgeSet& eset);

    /**
       Initializes the structures for optimizing a portion of the graph specified by a subset of vertices.
       Before calling it be sure to invoke marginalized() and fixed() to the vertices you want to include in the 
       schurr complement or to set as fixed during the optimization.
       @param vset: the subgraph to be optimized.
       @param level: is the level (in multilevel optimization)
       @returns false if somethings goes wrong
     */
    virtual bool initializeOptimization(HyperGraph::VertexSet& vset, int level=0);

    /**
       Initializes the structures for optimizing the whole graph.
       Before calling it be sure to invoke marginalized() and fixed() to the vertices you want to include in the 
       schurr complement or to set as fixed during the optimization.
       @param level: is the level (in multilevel optimization)
       @returns false if somethings goes wrong
     */
    virtual bool initializeOptimization(int level=0);

    /**
     * HACK updating the internal structures for online processing
     */
    virtual bool updateInitialization(HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset);
  
    /**
       Propagates an initial guess from the vertex specified as origin.
       It should be called after initializeOptimization(...), as it relies on the _activeVertices/_edges structures.
       It constructs a set of trees starting from the nodes in the graph which are fixed and eligible for preforming optimization.
       The trees are constructed by utlizing a cost-function specified.
       @param costFunction: the cost function used
       @patam maxDistance: the distance where to stop the search
     */
    virtual void computeInitialGuess();


    /**
       starts one optimization run given the current configuration of the graph, 
       and the current settings stored in the class instance.
       It can be called only after initializeOptimization
     */
    int optimize(int iterations, bool online = false);

    /**
     * computes the block diagonal elements of the inverted hessian
     * and stores them in the nodes of the graph.
     */
    bool computeMarginals();

    /**
     * computes the blocks of the inverse of the specified pattern.
     * the pattern is given via pairs <row, col> of the blocks in the hessian
     * @param blockIndices: the pattern
     * @param spinv: the sparse block matrix with the result
     * @returns false if the operation is not supported by the solver
     */
    bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices);

    //! finds a gauge in the graph to remove the undefined dof.
    // The gauge should be fixed() and then the optimization can work (if no additional dof are in
    // the system. The default implementation returns a node with maximum dimension.
    virtual Vertex* findGauge();

    bool gaugeFreedom();

    /**returns the cached chi2 of the active portion of the graph*/
    double activeChi2() const;

    //! verbose information during optimization
    bool verbose()  const {return _verbose;}
    void setVerbose(bool verbose);

    //! accessor for the optimization method (gauss-newton or levenberg marquardt)
    Method method() const { return _method;}
    void setMethod(Method method);

    /**
     * sets a variable checked at every iteration to force a user stop. The iteration exits when the variable is true;
     */
    void setForceStopFlag(bool* flag);

    //! if external stop flag is given, return its state. False otherwise
    bool terminate() {return _forceStopFlag ? (*_forceStopFlag) : false; }

    //! the index mapping of the vertices
    const VertexContainer& indexMapping() const {return _ivMap;}
    //! the vertices active in the current optimization
    const VertexContainer& activeVertices() const { return _activeVertices;}
    //! the edges active in the current optimization
    const EdgeContainer& activeEdges() const { return _activeEdges;}

    virtual bool removeVertex(Vertex* v);

    /**
     * search for an edge in _activeEdges and return the iterator pointing to it
     * getActiveVertices().end() if not found
     */
    VertexContainer::const_iterator findActiveVertex(OptimizableGraph::Vertex* v) const;
    /**
     * search for an edge in _activeEdges and return the iterator pointing to it
     * getActiveEdges().end() if not found
     */
    EdgeContainer::const_iterator findActiveEdge(OptimizableGraph::Edge* e) const;

    //! the solver used by the optimizer
    const Solver* solver() const { return _solver;}
    Solver* solver() { return _solver;}
    void setSolver(Solver* solver);

    //! push the estimate of a subset of the variables onto a stack
    void push(SparseOptimizer::VertexContainer& vlist);
    //! push the estimate of a subset of the variables onto a stack
    void push(HyperGraph::VertexSet& vlist);
    //! pop (restore) the estimate a subset of the variables from the stack
    void pop(SparseOptimizer::VertexContainer& vlist);
    //! pop (restore) the estimate a subset of the variables from the stack
    void pop(HyperGraph::VertexSet& vlist);

    //! ignore the latest stored element on the stack, remove it from the stack but do not restore the estimate
    void discardTop(SparseOptimizer::VertexContainer& vlist);

    //! return the lambda set by the user, if < 0 the SparseOptimizer will compute the initial lambda
    double userLambdaInit() {return _userLambdaInit;}
    //! specify the initial lambda used for the first iteraion, if not given the SparseOptimizer tries to compute a suitable value
    void setUserLambdaInit(double lambda);

    /**
       clears the graph, and polishes some intermediate structures
     */
    virtual void clear();

    /**
     * computes the error vectors of all edges in the activeSet, and caches them
     */
    void computeActiveErrors();

    /**
     * Linearizes the system by computing the Jacobians for the nodes
     * and edges in the graph
     */
    void linearizeSystem();

    /**
     * update the estimate of the active vertices 
     * @param update: the double vector containing the stacked
     * elements of the increments on the vertices.
     */
    void update(double* update);

    //! return the currently used damping factor
    double currentLambda() const { return _currentLambda;}

    //! the number of internal iteration if an update step increases chi^2 within Levenberg-Marquardt
    void setMaxTrialsAfterFailure(int max_trials);

    //! get the number of inner iterations for Levenberg-Marquardt
    int maxTrialsAfterFailure() const { return _maxTrialsAfterFailure;}

    /**** callbacks ****/
    //! add an action to be executed before the error vectors are computed
    bool addComputeErrorAction(HyperGraphAction* action);
    //! remove an action that should no longer be execured before computing the error vectors
    bool removeComputeErrorAction(HyperGraphAction* action);

    protected:
    bool* _forceStopFlag;
    bool _verbose;

    Method _method;
    double _currentLambda;
    int _maxTrialsAfterFailure;

    VertexContainer _ivMap;
    VertexContainer _activeVertices;   ///< sorted according to VertexIDCompare
    EdgeContainer _activeEdges;        ///< sorted according to EdgeIDCompare

    void sortVectorContainers();
 
    Solver* _solver;

    // LM parameters
    double _tau;
    double _userLambdaInit;
    double _goodStepLowerScale; ///< lower bound for lambda decrease if a good LM step
    double _goodStepUpperScale; ///< upper bound for lambda decrease if a good LM step

    /**
     * helper for Levenberg, this function computes the initial damping factor, if the user did not
     * specify an own value, see setUserLambdaInit()
     */
    double computeLambdaInit();
    double computeScale(double currentLMLambda, Solver* solver);

    /**
     * builds the mapping of the active vertices to the (block) row / column in the Hessian
     */
    bool buildIndexMapping(SparseOptimizer::VertexContainer& vlist);
    void clearIndexMapping();

  public:
    G2OBatchStatistics* _statistics;   ///< global statistics of the optimizer, e.g., timing, num-non-zeros

  };

} // end namespace

#endif
