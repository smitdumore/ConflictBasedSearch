/***************************************************************/
// https://www.cs.cmu.edu/~motionplanning/lecture/Asearch_v8.pdf
/***************************************************************/

#pragma once

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>
#include "neighbor.hpp"
#include "planresult.hpp"

// #define REBUILT_FOCAL_LIST
// #define CHECK_FOCAL_LIST

namespace MultiRobotPlanning {

/*!
  \example a_star_epsilon.cpp Simple example using a 2D grid world and
  up/down/left/right
  actions
*/

/*! \brief A*_epsilon Algorithm to find the shortest path with a given
suboptimality bound (also known as focal search)

This class implements the A*_epsilon algorithm, an informed search
algorithm
that finds the shortest path for a given map up to a suboptimality factor.
It uses an admissible heuristic (to keep track of the optimum) and an
inadmissible heuristic (
to guide the search within a suboptimal bound w.)

Details of the algorithm can be found in the following paper:\n
Judea Pearl, Jin H. Kim:\n
"Studies in Semi-Admissible Heuristics."" IEEE Trans. Pattern Anal. Mach.
Intell. 4(4): 392-399 (1982)\n
https://doi.org/10.1109/TPAMI.1982.4767270

This class can either use a fibonacci heap, or a d-ary heap. The latter is the
default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap instead.

\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Environment This class needs to provide the custom logic. In
    particular, it needs to support the following functions:
  - `Cost admissibleHeuristic(const State& s)`\n
    This function can return 0 if no suitable heuristic is available.

  - `Cost focalStateHeuristic(const State& s, Cost gScore)`\n
    This function computes a (potentially inadmissible) heuristic for the given
state.

  - `Cost focalTransitionHeuristic(const State& s1, const State& s2, Cost
gScoreS1, Cost gScoreS2)`\n
    This function computes a (potentially inadmissible) heuristic for the given
state transition.

  - `bool isSolution(const State& s)`\n
    Return true if the given state is a goal state.

  - `void getNeighbors(const State& s, std::vector<Neighbor<State, Action,
   int> >& neighbors)`\n
    Fill the list of neighboring state for the given state s.

  - `void onExpandNode(const State& s, int fScore, int gScore)`\n
    This function is called on every expansion and can be used for statistical
purposes.

  - `void onDiscover(const State& s, int fScore, int gScore)`\n
    This function is called on every node discovery and can be used for
   statistical purposes.

    \tparam StateHasher A class to convert a state to a hash value. Default:
   std::hash<State>
*/
template <typename State, typename Action, typename Cost, typename Environment, typename StateHasher = std::hash<State>>
class AStarWeighted {
 public:
    AStarWeighted(Environment& environment, float w): m_env(environment), m_w(w) {}


    bool search(const State& startState, PlanResult<State, Action, Cost>& solution) {
        solution.states.clear();
        solution.states.push_back(std::make_pair<>(startState, 0));
        solution.actions.clear();
        solution.cost = 0;

        openSet_t openSet;
        focalSet_t focalSet;  // subset of open nodes that are within suboptimality bound
        
        // Maps states to position in the priority queue
        // State -> state_key (state hasher provides hash function for the map)
        std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToHeap;

        std::unordered_set<State, StateHasher> closedSet;
        
        // for backtracking
        std::unordered_map<State, std::tuple<State, Action, Cost, Cost>, StateHasher> cameFrom;

        auto handle = openSet.push(Node(startState, m_env.admissibleHeuristic(startState), 0, 0));
        stateToHeap.insert(std::make_pair<>(startState, handle));
        (*handle).handle = handle;

        // push the same node into the focal set
        focalSet.push(handle);

        std::vector<Neighbor<State, Action, Cost> > neighbors;
        neighbors.reserve(10);

        // current best score the cost of the node that was pushed (start)
        Cost bestFScore = (*handle).fScore; 

        while (!openSet.empty()) {
            
            
            
            // first save old best score
            Cost oldBestFScore = bestFScore;

            // get new best score
            bestFScore = openSet.top().fScore;
            
            // The best exploration currently in the open set has become more expensive
            // update focal list
            // This triggers an incremental update to include newly eligible nodes in the focal set.

            // When it decreases, no new nodes need to be added, so there’s nothing to do 
            // (unless you want to optimize the size of the focal set by removing nodes).
            if (bestFScore > oldBestFScore) {

                // add the newest eligible nodes 
                auto iter = openSet.ordered_begin();
                auto iterEnd = openSet.ordered_end();
                for (; iter != iterEnd; ++iter) {
                    Cost val = iter->fScore;
                    if (val > oldBestFScore * m_w && val <= bestFScore * m_w) {
                        const Node& n = *iter;
                        focalSet.push(n.handle);
                    }
                    if (val > bestFScore * m_w) {
                        break;
                    }
                }
            }

            auto currentHandle = focalSet.top();
            Node current = *currentHandle;
            m_env.onExpandNode(current.state, current.fScore, current.gScore);

            // found goal
            if (m_env.isSolution(current.state)) {
                solution.states.clear();
                solution.actions.clear();
                auto iter = cameFrom.find(current.state);

                // backtrack
                while (iter != cameFrom.end()) {
                    solution.states.push_back(
                        std::make_pair<>(iter->first, std::get<3>(iter->second)));
                    solution.actions.push_back(std::make_pair<>(
                        std::get<1>(iter->second), std::get<2>(iter->second)));
                    iter = cameFrom.find(std::get<0>(iter->second));
                }

                solution.states.push_back(std::make_pair<>(startState, 0));
                std::reverse(solution.states.begin(), solution.states.end());
                std::reverse(solution.actions.begin(), solution.actions.end());
                solution.cost = current.gScore;
                solution.fmin = openSet.top().fScore;

                return true;
            }
        
            focalSet.pop();
            openSet.erase(currentHandle);
            stateToHeap.erase(current.state);
            closedSet.insert(current.state);

            // traverse neighbors
            neighbors.clear();
            m_env.getNeighbors(current.state, neighbors);
            for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {
                
                // if neigbor node not visited
                if (closedSet.find(neighbor.state) == closedSet.end()) {
                        
                    Cost tentative_gScore = current.gScore + neighbor.cost;
                    auto iter = stateToHeap.find(neighbor.state);
                    
                    if (iter == stateToHeap.end()) {  // Discover a new node
                        // std::cout << "  this is a new node" << std::endl;
                        Cost fScore = tentative_gScore + m_env.admissibleHeuristic(neighbor.state);
                        
                        // ?
                        Cost focalHeuristic =
                            current.focalHeuristic +
                            m_env.focalStateHeuristic(neighbor.state, tentative_gScore) +
                            m_env.focalTransitionHeuristic(current.state, neighbor.state,
                                                        current.gScore,
                                                        tentative_gScore);
                                                        
                        auto handle = openSet.push(
                            Node(neighbor.state, fScore, tentative_gScore, focalHeuristic));
                        (*handle).handle = handle;

                        if (fScore <= bestFScore * m_w) {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focalSet.push(handle);
                        }

                        stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
                        m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
                        // std::cout << "  this is a new node " << fScore << "," <<
                        // tentative_gScore << std::endl;
                    } else {
                        auto handle = iter->second;
                        // We found this node before with a better path
                        if (tentative_gScore >= (*handle).gScore) {
                            continue;
                        }

                        Cost last_gScore = (*handle).gScore;
                        Cost last_fScore = (*handle).fScore;
                        // std::cout << "  this is an old node: " << tentative_gScore << ","
                        // << last_gScore << " " << *handle << std::endl;
                        // update f and gScore
                        Cost delta = last_gScore - tentative_gScore;
                        (*handle).gScore = tentative_gScore;
                        (*handle).fScore -= delta;
                        openSet.increase(handle);
                        m_env.onDiscover(neighbor.state, (*handle).fScore,
                                        (*handle).gScore);
                        
                        if ((*handle).fScore <= bestFScore * m_w &&
                            last_fScore > bestFScore * m_w) {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focalSet.push(handle);
                        }
                    }

                    // Best path for this node so far
                    // TODO: this is not the best way to update "cameFrom", but otherwise
                    // default c'tors of State and Action are required
                    cameFrom.erase(neighbor.state);
                    cameFrom.insert(std::make_pair<>(
                        neighbor.state,
                        std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                            tentative_gScore)));

                } // if neighbor not visited ends
            
            } // searching neighbors ends 

        } // priority queue ends

        return false;
    } // search ends

 private:
  struct Node;


  typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;
// typedef typename boost::heap::fibonacci_heap<fibHeapHandle_t,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;

  struct Node {
    Node(const State& state, Cost fScore, Cost gScore, Cost focalHeuristic)
        : state(state),
          fScore(fScore),
          gScore(gScore),
          focalHeuristic(focalHeuristic) {}

    bool operator<(const Node& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if (fScore != other.fScore) {
        return fScore > other.fScore;
      } else {
        return gScore < other.gScore;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state << " fScore: " << node.fScore
         << " gScore: " << node.gScore << " focal: " << node.focalHeuristic;
      return os;
    }

    State state;

    Cost fScore;
    Cost gScore;
    Cost focalHeuristic;

    fibHeapHandle_t handle;

  };

  struct compareFocalHeuristic {
    bool operator()(const fibHeapHandle_t& h1,
                    const fibHeapHandle_t& h2) const {
      // Sort order (see "Improved Solvers for Bounded-Suboptimal Multi-Agent
      // Path Finding" by Cohen et. al.)
      // 1. lowest focalHeuristic
      // 2. lowest fScore
      // 3. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
        return (*h1).focalHeuristic > (*h2).focalHeuristic;
        // } else if ((*h1).fScore != (*h2).fScore) {
        //   return (*h1).fScore > (*h2).fScore;
      } else if ((*h1).fScore != (*h2).fScore) {
        return (*h1).fScore > (*h2).fScore;
      } else {
        return (*h1).gScore < (*h2).gScore;
      }
    }
  };

  // typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
  // typedef typename openSet_t::handle_type fibHeapHandle_t;
  typedef typename boost::heap::fibonacci_heap<
      fibHeapHandle_t, boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;

 private:
  Environment& m_env;
  float m_w;
};

}  // namespace MultiRobotPlanning
