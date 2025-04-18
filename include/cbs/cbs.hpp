#pragma once

#include <map>
#include "common.hpp"
#include "a_star.hpp"


namespace MultiRobotPlanning {

template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class CBS {
  
  private:
    struct HighLevelNode {
      std::vector<PlanResult<State, Action, Cost> > solution;
      std::vector<Constraints> constraints;

      Cost cost;

      int id;

      typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                      boost::heap::mutable_<true> >::handle_type
          handle;

      bool operator<(const HighLevelNode& n) const {
        // if (cost != n.cost)
        return cost > n.cost;
        // return id > n.id;
      }

      friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
        os << "id: " << c.id << " cost: " << c.cost << std::endl;
        for (size_t i = 0; i < c.solution.size(); ++i) {
          os << "Agent: " << i << std::endl;
          os << " States:" << std::endl;
          for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
            os << "  " << c.solution[i].states[t].first << std::endl;
          }
          os << " Constraints:" << std::endl;
          os << c.constraints[i];
          os << " cost: " << c.solution[i].cost << std::endl;
        }
        return os;
      }
    };

    struct LowLevelEnvironment {
      LowLevelEnvironment(Environment& env, size_t agentIdx,
                          const Constraints& constraints)
          : m_env(env)
      // , m_agentIdx(agentIdx)
      // , m_constraints(constraints)
      {
        m_env.setLowLevelContext(agentIdx, &constraints);
      }

      Cost admissibleHeuristic(const State& s) {
        return m_env.admissibleHeuristic(s);
      }

      bool isSolution(const State& s) { return m_env.isSolution(s); }

      void getNeighbors(const State& s,
                        std::vector<Neighbor<State, Action, Cost> >& neighbors) {
        m_env.getNeighbors(s, neighbors);
      }

      void onExpandNode(const State& s, Cost fScore, Cost gScore) {
        // std::cout << "LL expand: " << s << std::endl;
        m_env.onExpandLowLevelNode(s, fScore, gScore);
      }

      void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
        // std::cout << "LL discover: " << s << std::endl;
        // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
      }

        private:
          Environment& m_env;
          // size_t m_agentIdx;
          // const Constraints& m_constraints;
    };

  private:
    Environment& m_env;
    typedef AStar<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
    
    // Maximum number of high-level nodes to explore before terminating search
    const size_t m_maxNodes;

  public:
  // Constructor with default max nodes value
  CBS(Environment& environment, size_t maxNodes = 10000) 
      : m_env(environment), m_maxNodes(maxNodes) {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost> >& solution) {

    /******** Root of the conflict tree *********/ 
    // Every conflict tree node is a snapshot of time
    // It has the paths for all the robots and all the constraints at that time
    // Conflict is agentd colliding and a Constraint is a restriction, know the difference
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;
    
    /*** LowLevelSearch for every Agent ****/
    for (size_t i = 0; i < initialStates.size(); ++i) {

      // Initialize low level environment
      LowLevelEnvironment llenv(m_env, i, start.constraints[i]);

      // This is the a_star instance
      // We pass the low level env to the search because the constraints are include in the low level env
      LowLevelSearch_t lowLevel_search(llenv); 
      bool success = lowLevel_search.search(initialStates[i], start.solution[i]); // Initial low level search
      if (!success) {
        return false;          // Return false if the ith agent cant find a path due to map obstacles 
      }

      start.cost += start.solution[i].cost;
    }
    /*** Done with first steps ****/
    /******** End of Root initialization *********/ 


    // BFS traversal of the tree
    // std::priority_queue<HighLevelNode> open;
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >
        open;

    auto handle = open.push(start);
    (*handle).handle = handle;

    solution.clear();
    int id = 1; // id for the first child, root id was 0
    
    // Node count tracking
    size_t nodesExpanded = 0;

    while (!open.empty()) {
      // Check if maximum node count reached
      if (nodesExpanded >= m_maxNodes) {
        std::cout << "CBS search terminated: reached maximum node count limit (" 
                  << m_maxNodes << " nodes)" << std::endl;
        return false;
      }
      
      // Current Conflict Tree Node
      HighLevelNode currCTNode = open.top();
      open.pop();
      
      // Increment node counter
      nodesExpanded++;

      Conflict conflict; 
      if (!m_env.getFirstConflict(currCTNode.solution, conflict)) {       // Environment owns finding conflicts
        // if didnt get any conflict then return the solution of that node
        std::cout << "CBS done; cost: " << currCTNode.cost << std::endl;
        solution = currCTNode.solution;
        return true;
      }

      // Create additional nodes to resolve conflict
      // Also convert conflicts into constraints for next CT node search

      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints); // Constraints data struct stores mapping of agent id to its resp. constraints

      // Visit neighbors step in BFS
      // Creating a new node for every constraint
      for (const auto& c : constraints) {
        
        size_t i = c.first; // agent id
        
        HighLevelNode newNode = currCTNode;
        newNode.id = id;
        
        assert(!newNode.constraints[i].overlap(c.second)); // checks for duplicated constraints

        // Add the current constraint to the current CT node
        newNode.constraints[i].add(c.second); // i is the agent id obtained from the current contraint
        newNode.cost -= newNode.solution[i].cost;

        // Run low level stuff again        
        LowLevelEnvironment llenv(m_env, i, newNode.constraints[i]);
        LowLevelSearch_t lowLevel(llenv);
        bool success = lowLevel.search(initialStates[i], newNode.solution[i]);

        newNode.cost += newNode.solution[i].cost;

        if (success) {
          // std::cout << "  success. cost: " << newNode.cost << std::endl;
          auto handle = open.push(newNode);
          (*handle).handle = handle;
        }

        // new id for new CT nodes
        ++id;
      }
    }

    // TODO: Future Improvements for Search Termination
    // 1. Implement conflict cycling detection to identify when the algorithm is cycling through
    //    the same conflicts repeatedly, indicating a potential infinite loop
    // 
    // 2. Add plateau detection to terminate search when solution quality doesn't improve
    //    after exploring a significant number of nodes
    //
    // 3. Consider adding a time-based limit to ensure the algorithm terminates
    //    within a reasonable time frame regardless of node count

    return false;
  }
};

}  // namespace libMultiRobotPlanning
