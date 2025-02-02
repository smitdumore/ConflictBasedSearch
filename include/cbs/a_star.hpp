#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "neighbor.hpp"
#include "planresult.hpp"

namespace MultiRobotPlanning {

/*!
  \example a_star.cpp Simple example using a 2D grid world and
  up/down/left/right
  actions
*/

/*! \brief A* Algorithm to find the shortest path


\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Environment This class needs to provide the custom A* logic. In
    particular, it needs to support the following functions:
  - `Cost admissibleHeuristic(const State& s)`\n
    This function can return 0 if no suitable heuristic is available.

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
class AStar {

    public:

        // constructor
        AStar(Environment& environment) : m_env(environment) {}

        // Function that runs A star search
        bool search(const State& startState, PlanResult<State, Action, Cost>& solution, Cost initialCost = 0) {
            
            // solution is an object of class planresult

            solution.states.clear();
            solution.states.push_back(std::make_pair<>(startState, 0));
            solution.actions.clear();
            solution.cost = 0;

            // priority queue
            boost::heap::fibonacci_heap<Node> openSet;

            // Maps states to position in the priority queue
            // State -> state_key (state hasher provides hash function for the map)
            std::unordered_map<State, stateKeyInHeap_t, StateHasher> stateToHeap;
            
            // Visited
            std::unordered_set<State, StateHasher> closedSet;
            
            // keeps track of parent (helpful for backtracking the path)
            std::unordered_map<State, std::tuple<State, Action, Cost, Cost>, StateHasher> cameFrom;

            // Node is a struct that goes into the priority queue
            // Adding Start Node to the priority queue (Node instance is created in place)
            auto handle = openSet.push( Node(startState, m_env.admissibleHeuristic(startState), initialCost) );
            
            // store handle of current node in the heap, so that we can directly edit it using the handle
            stateToHeap.insert(std::make_pair<>(startState, handle));
            (*handle).handle = handle;
            
            // This will store neighbors in the future
            std::vector<Neighbor<State, Action, Cost> > neighbors;
            neighbors.reserve(10);


            while (!openSet.empty()) {
                
                Node current = openSet.top();
                m_env.onExpandNode(current.state, current.fScore, current.gScore); // ?

                // check if search reached the goal ?
                if (m_env.isSolution(current.state)) {
                    solution.states.clear();
                    solution.actions.clear();
                    auto iter = cameFrom.find(current.state);

                    // backtracking
                    while (iter != cameFrom.end()) {
                        solution.states.push_back(
                            std::make_pair<>(iter->first, std::get<3>(iter->second)));
                        solution.actions.push_back(std::make_pair<>(
                            std::get<1>(iter->second), std::get<2>(iter->second)));
                        iter = cameFrom.find(std::get<0>(iter->second));
                    }

                    solution.states.push_back(std::make_pair<>(startState, initialCost));
                    std::reverse(solution.states.begin(), solution.states.end());
                    std::reverse(solution.actions.begin(), solution.actions.end());
                    solution.cost = current.gScore;
                    solution.fmin = current.fScore;

                    return true;
                }

                openSet.pop();
                closedSet.insert(current.state); // visited
                stateToHeap.erase(current.state); // Its closed so we don't need to keep track of it in heap
               

                // traverse neighbors
                neighbors.clear();
                m_env.getNeighbors(current.state, neighbors);

                for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {

                    // if neigbor node not visited
                    if (closedSet.find(neighbor.state) == closedSet.end()) {
                        
                        Cost tentative_gScore = current.gScore + neighbor.cost;
                        auto iter = stateToHeap.find(neighbor.state); // find neighbor in heap 
                        
                        if (iter == stateToHeap.end()) {  // Discover a new node
                            Cost fScore =
                                tentative_gScore + m_env.admissibleHeuristic(neighbor.state);
                            auto handle =
                                openSet.push(Node(neighbor.state, fScore, tentative_gScore));
                            (*handle).handle = handle;
                            stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
                            m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
                            // std::cout << "  this is a new node " << fScore << "," <<
                            // tentative_gScore << std::endl;
                        } else { // this node is already in the heap
                            auto handle = iter->second;
                            // std::cout << "  this is an old node: " << tentative_gScore << ","
                            // << (*handle).gScore << std::endl;

                            // We found this node before with a better path
                            if (tentative_gScore >= (*handle).gScore) {
                            continue; // this is the skipping logic, read README
                            }

                            // update f and gScore
                            Cost delta = (*handle).gScore - tentative_gScore;
                            (*handle).gScore = tentative_gScore;
                            (*handle).fScore -= delta;
                            openSet.increase(handle); // rearrage the heap
                            m_env.onDiscover(neighbor.state, (*handle).fScore,
                                            (*handle).gScore);
                        }

                        // Best path for this node so far
                        // TODO: this is not the best way to update "cameFrom", but otherwise
                        // default c'tors of State and Action are required
                        cameFrom.erase(neighbor.state);
                        cameFrom.insert(std::make_pair<>(
                            neighbor.state,
                            std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                                tentative_gScore)));
                    }
                
                } // neighbors loop ends
            } // pq ends

            return false;
        } //  search function ends

    private:

    struct Node {
        Node(const State& state, Cost fScore, Cost gScore)
            : state(state), fScore(fScore), gScore(gScore) {}

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
            << " gScore: " << node.gScore;
        return os;
        }

        State state;

        Cost fScore;
        Cost gScore;

    #ifdef USE_FIBONACCI_HEAP
        typename boost::heap::fibonacci_heap<Node>::handle_type handle;
    #else
        typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                        boost::heap::mutable_<true> >::handle_type
            handle;
    #endif
    };
    
    typedef typename  boost::heap::fibonacci_heap<Node>::handle_type stateKeyInHeap_t;

    // #ifdef USE_FIBONACCI_HEAP
    // //typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
    // typedef typename openSet_t::handle_type stateKeyInHeap_t;
    // #else
    // typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
    //                                         boost::heap::mutable_<true> >
    //     openSet_t;
    // typedef typename openSet_t::handle_type stateKeyInHeap_t;
    // #endif

    private:
    Environment& m_env;
    };

}  // namespace libMultiRobotPlanning
