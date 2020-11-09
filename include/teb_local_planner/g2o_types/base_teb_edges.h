/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef _BASE_TEB_EDGES_H_
#define _BASE_TEB_EDGES_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/timed_elastic_band.h>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>

#include <cmath>
#include <memory>
#include <type_traits>

namespace teb_local_planner
{
    
    
/**
 * @class BaseTebUnaryEdge
 * @brief Base edge connecting a single vertex in the TEB optimization problem
 * 
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with additional information for the dedicated TEB problem (e.g. config).
 * The destructor erases the edge in all attached vertices in order to allow keeping the vertices valid in subsequent g2o optimization calls.
 * Memory of edges should be freed by calling the clearEdge method of the g2o optimzier class.
 * @see BaseTebMultiEdge, BaseTebBinaryEdge, g2o::BaseBinaryEdge, g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */   
template <int D, typename E, typename VertexXi>
class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi>
{
public:
            
  using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
  using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;
    
  /**
  * @brief Compute and return error / cost value.
  * 
  * This method is called by TebOptimalPlanner::computeCurrentCost to obtain the current cost.
  * @return 2D Cost / error vector [nh cost, backward drive dir cost]^T
  */     
  ErrorVector& getError()
  {
    computeError();
    return _error;
  }
  
  /**
   * @brief Read values from input stream
   */  	
  virtual bool read(std::istream& is)
  {
    // TODO generic read
    return true;
  }

  /**
   * @brief Write values to an output stream
   */    
  virtual bool write(std::ostream& os) const
  {
    // TODO generic write
    return os.good();
  }

  /**
   * @brief Assign the TebConfig class for parameters.
   * @param cfg TebConfig class
   */ 
  void setTebConfig(const TebConfig& cfg)
  {
    cfg_ = &cfg;
  }
    
protected:
    
  using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
  using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;
  
  const TebConfig* cfg_; //!< Store TebConfig class for parameters
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
};

/**
 * @class BaseTebBinaryEdge
 * @brief Base edge connecting two vertices in the TEB optimization problem
 * 
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with additional information for the dedicated TEB problem (e.g. config).
 * The destructor erases the edge in all attached vertices in order to allow keeping the vertices valid in subsequent g2o optimization calls.
 * Memory of edges should be freed by calling the clearEdge method of the g2o optimzier class.
 * @see BaseTebMultiEdge, BaseTebUnaryEdge, g2o::BaseBinaryEdge, g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */    
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
{
public:
    
  using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
  using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

  /**
  * @brief Compute and return error / cost value.
  * 
  * This method is called by TebOptimalPlanner::computeCurrentCost to obtain the current cost.
  * @return 2D Cost / error vector [nh cost, backward drive dir cost]^T
  */     
  ErrorVector& getError()
  {
    computeError();
    return _error;
  }
  
  /**
   * @brief Read values from input stream
   */  	
  virtual bool read(std::istream& is)
  {
    // TODO generic read
    return true;
  }

  /**
   * @brief Write values to an output stream
   */    
  virtual bool write(std::ostream& os) const
  {
    // TODO generic write
    return os.good();
  }

  /**
   * @brief Assign the TebConfig class for parameters.
   * @param cfg TebConfig class
   */ 
  void setTebConfig(const TebConfig& cfg)
  {
    cfg_ = &cfg;
  }
  
protected:
  
  using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
  using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;
    
  const TebConfig* cfg_; //!< Store TebConfig class for parameters
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
};


/**
 * @class BaseTebMultiEdge
 * @brief Base edge connecting two vertices in the TEB optimization problem
 * 
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with additional information for the dedicated TEB problem (e.g. config).
 * The destructor erases the edge in all attached vertices in order to allow keeping the vertices valid in subsequent g2o optimization calls.
 * Memory of edges should be freed by calling the clearEdge method of the g2o optimzier class.
 * @see BaseTebBinaryEdge, BaseTebUnaryEdge, g2o::BaseBinaryEdge, g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */    
template <int D, typename E>
class BaseTebMultiEdge : public g2o::BaseMultiEdge<D, E>
{
public:
  
  using typename g2o::BaseMultiEdge<D, E>::ErrorVector;
  using g2o::BaseMultiEdge<D, E>::computeError;
    
  // Overwrites resize() from the parent class
  virtual void resize(size_t size)
  {
      g2o::BaseMultiEdge<D, E>::resize(size);
      
      for(std::size_t i=0; i<_vertices.size(); ++i)
        _vertices[i] = NULL;
  }

  /**
  * @brief Compute and return error / cost value.
  * 
  * This method is called by TebOptimalPlanner::computeCurrentCost to obtain the current cost.
  * @return 2D Cost / error vector [nh cost, backward drive dir cost]^T
  */     
  ErrorVector& getError()
  {
    computeError();
    return _error;
  }
  
  /**
   * @brief Read values from input stream
   */  	
  virtual bool read(std::istream& is)
  {
    // TODO generic read
    return true;
  }

  /**
   * @brief Write values to an output stream
   */    
  virtual bool write(std::ostream& os) const
  {
    // TODO generic write
    return os.good();
  }

  /**
   * @brief Assign the TebConfig class for parameters.
   * @param cfg TebConfig class
   */ 
  void setTebConfig(const TebConfig& cfg)
  {
    cfg_ = &cfg;
  }
  
protected:
    
  using g2o::BaseMultiEdge<D, E>::_error;
  using g2o::BaseMultiEdge<D, E>::_vertices;
  
  const TebConfig* cfg_; //!< Store TebConfig class for parameters
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
};

namespace internal {

inline const VertexPose*
toPose(const g2o::HyperGraph::Vertex* _v) {
  return dynamic_cast<const VertexPose*>(_v);
}

inline const VertexTimeDiff*
toDt(const g2o::HyperGraph::Vertex* _v) {
  return dynamic_cast<const VertexTimeDiff*>(_v);
}

}  // namespace internal

// below edges with a 'physical' meaning

/// @brief Edge with a time-diff vertex
template <int D, typename E>
struct BaseEdgeDt : public BaseTebUnaryEdge<D, E, VertexTimeDiff> {
protected:
  inline const VertexTimeDiff*
  getDt0() const {
    return internal::toDt(this->_vertices[0]);
  };
};

/// @brief Edge with a pose vertex
template <int D, typename E>
struct BaseEdgePose : public BaseTebUnaryEdge<D, E, VertexPose> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  inline const VertexPose*
  getPose0() const {
    return internal::toPose(this->_vertices[0]);
  };
};

/// @brief Edge connecting two poses
template <int D, typename E>
struct BaseEdgeTwoPoses : public BaseTebBinaryEdge<D, E, VertexPose, VertexPose> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  inline const VertexPose*
  getPose0() const {
    return internal::toPose(this->_vertices[0]);
  }

  inline const VertexPose*
  getPose1() const {
    return internal::toPose(this->_vertices[1]);
  }
};

/// @brief Edge representing velocity
template <int D, typename E>
struct BaseEdgeVelocity : public BaseTebMultiEdge<D, E> {
  BaseEdgeVelocity() { BaseTebMultiEdge<D, E>::resize(3); }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  inline const VertexPose*
  getPose0() const {
    return internal::toPose(this->_vertices[0]);
  }

  inline const VertexPose*
  getPose1() const {
    return internal::toPose(this->_vertices[1]);
  }

  inline const VertexTimeDiff*
  getDt0() const {
    return internal::toDt(this->_vertices[2]);
  };
};

/// @brief Edge representing acceleration
template <int D, typename E>
struct BaseEdgeAcceleration : public BaseTebMultiEdge<D, E> {
  BaseEdgeAcceleration() { BaseTebMultiEdge<D, E>::resize(5); }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  inline const VertexPose*
  getPose0() const {
    return internal::toPose(this->_vertices[0]);
  }

  inline const VertexPose*
  getPose1() const {
    return internal::toPose(this->_vertices[1]);
  }

  inline const VertexPose*
  getPose2() const {
    return internal::toPose(this->_vertices[2]);
  }

  inline const VertexTimeDiff*
  getDt0() const {
    return internal::toDt(this->_vertices[3]);
  };

  inline const VertexTimeDiff*
  getDt1() const {
    return internal::toDt(this->_vertices[4]);
  };
};

namespace internal {

// below some trickery to construct edges with different types of vertices
// we use three overloads of the setVertex function:
// 1. A base overload which is basically no-op
// 2. An overload where we add exactly one vertex
// 3. A variadic overload which will do the looping

using OgEdge = g2o::OptimizableGraph::Edge;
using OgVertex = g2o::OptimizableGraph::Vertex;

// note: suppress -wunused-parameter warning
// the base overload which is a noop
inline void setVertex(__attribute__((unused)) OgEdge &_edge,
                      __attribute__((unused)) size_t _counter) noexcept {}

// one edge overload
template <typename Iter>
inline void setVertex(OgEdge &_edge, size_t _counter, Iter &_iter) {
  _edge.setVertex(_counter, &*_iter++);
}

// variadic overload
// note: we don't inline here but go for full recursion.
template <typename Iter, typename... OtherIters>
void setVertex(OgEdge &_edge, size_t _counter, Iter &_iter,
               OtherIters &... _rem) {
  setVertex(_edge, _counter, _iter);
  setVertex(_edge, ++_counter, _rem...);
}

using RefVectorXd = Eigen::Ref<const Eigen::VectorXd>;
using g2o::HyperGraph;

// this is a 'generic' way to construct edges for g2o
// from here we will call the variadic setVertex functions above

template <typename Edge, typename Iter, typename... OtherIters>
void addEdges(HyperGraph &_graph, const TebConfig &_cfg,
              const RefVectorXd &_diag, size_t _n, Iter _iter,
              OtherIters... _rem) {
  // diagonal of the info matrix
  // defining any element as zero will force to ignore the edge
  if ((_diag.array() == 0).all())
    return;

  // setup the info matrix
  const typename Edge::InformationType info = _diag.asDiagonal();

  // generate _n edges...
  for (size_t nn = 0; nn != _n; ++nn) {
    std::unique_ptr<Edge> edge(new Edge());
    // do the template expansion
    setVertex(*edge, 0, _iter, _rem...);
    edge->setInformation(info);
    edge->setConfig(_cfg);

    // add the edge to the graph
    // _graph owns the edge now
    _graph.addEdge(edge.release());
  }
}

// below some trickery to check if a class derives from a templated base class
// adjusted from
// https://stackoverflow.com/questions/34672441/stdis-base-of-for-template-classes
template <template <int, typename...> class Base, typename Derived>
struct is_base_of_impl {
  template <int D, typename... Ts>
  static constexpr std::true_type test(const Base<D, Ts...> *) {
    return {};
  }

  static constexpr std::false_type test(...) { return {}; }

  using type = decltype(test(std::declval<Derived *>()));
};

template <template <int, typename...> class Base, typename Derived>
using is_base_of = typename is_base_of_impl<Base, Derived>::type;

// below the specialization for edge-families.
// see the 'physical' edges in above.
// we will use SFINAE to find the right addEdges function based on edge type.
// https://en.cppreference.com/w/cpp/language/sfinae for details

using Teb = TimedElasticBand;
/**
 * @brief helper function to create two-pose based edge families
 */
template <typename Edge,
          typename std::enable_if<is_base_of<BaseEdgeTwoPoses, Edge>::value,
                                  int>::type = 0>
void addEdges(HyperGraph &_graph, Teb &_teb, const TebConfig &_cfg,
              const RefVectorXd &_info) {
  const auto &poses = _teb.poses();
  if (poses.size() < 2)
    return;

  const auto N = poses.size() - 1;
  addEdges<Edge>(_graph, _cfg, _info, N, poses.begin(),
                 std::next(poses.begin()));
}

/**
 * @brief helper function to create velocity-based edge families
 */
template <typename Edge,
          typename std::enable_if<is_base_of<BaseEdgeVelocity, Edge>::value,
                                  int>::type = 0>
void addEdges(HyperGraph &_graph, Teb &_teb, const TebConfig &_cfg,
              const RefVectorXd &_info) {
  const auto &poses = _teb.poses();
  const auto &times = _teb.timediffs();
  if (poses.size() < 2)
    return;

  const auto N = poses.size() - 1;
  addEdges<Edge>(_graph, _cfg, _info, N, poses.begin(),
                 std::next(poses.begin()), times.begin());
}

/**
 * @brief helper function to create acceleration-based edge families
 */
template <typename Edge,
          typename std::enable_if<is_base_of<BaseEdgeAcceleration, Edge>::value,
                                  int>::type = 0>
void addEdges(HyperGraph &_graph, Teb &_teb, const TebConfig &_cfg,
              const RefVectorXd &_info) {
  const auto &poses = _teb.poses();
  const auto &times = _teb.timediffs();
  if (poses.size() < 3)
    return;

  const auto N = poses.size() - 2;
  // setup the poses
  auto p1 = poses.begin();
  auto p2 = std::next(p1);
  auto p3 = std::next(p2);
  // setup the times
  auto t1 = times.begin();
  auto t2 = std::next(t1);

  // pass to the addEdges
  addEdges<Edge>(_graph, _cfg, _info, N, p1, p2, p3, t1, t2);
}

} // namespace internal

} // end namespace

#endif
