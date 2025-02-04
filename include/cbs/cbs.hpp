#pragma once

#include <map>

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

  public:
  CBS(Environment& environment) : m_env(environment) {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost> >& solution) {

    /******** Root of the conflict tree *********/ 
    // Every conflict tree node is a snapshot of time
    // It has the paths for all the robots and all the constraints at that time
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;
    
    /*** LowLevelSearch for avery Agent ****/
    for (size_t i = 0; i < initialStates.size(); ++i) {
      // if (   i < solution.size()
      //     && solution[i].states.size() > 1) {
      //   start.solution[i] = solution[i];
      //   std::cout << "use existing solution for agent: " << i << std::endl;
      // } else {
      LowLevelEnvironment llenv(m_env, i, start.constraints[i]);
      LowLevelSearch_t lowLevel(llenv); // this is the a_star instance
      bool success = lowLevel.search(initialStates[i], start.solution[i]);   // first search in the root
      if (!success) {
        return false;          // constraints are empty, so if search returned false then the ith agent cant find a path due to map obstacles 
      }
      // }
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

    while (!open.empty()) {
      HighLevelNode P = open.top();
      m_env.onExpandHighLevelNode(P.cost);
      // std::cout << "expand: " << P << std::endl;

      open.pop();

      Conflict conflict;
      if (!m_env.getFirstConflict(P.solution, conflict)) {
        // if didnt get any conflict then return the solution of that node
        std::cout << "done; cost: " << P.cost << std::endl;
        solution = P.solution;
        return true;
      }

      // create additional nodes to resolve conflict
      // std::cout << "Found conflict: " << conflict << std::endl;
      // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
      // conflict.type << std::endl;

      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);
      for (const auto& c : constraints) {
        // std::cout << "Add HL node for " << c.first << std::endl;
        size_t i = c.first;
        // std::cout << "create child with id " << id << std::endl;
        HighLevelNode newNode = P;
        newNode.id = id;
        // (optional) check that this constraint was not included already
        // std::cout << newNode.constraints[i] << std::endl;
        // std::cout << c.second << std::endl;
        assert(!newNode.constraints[i].overlap(c.second));

        newNode.constraints[i].add(c.second);

        newNode.cost -= newNode.solution[i].cost;

        LowLevelEnvironment llenv(m_env, i, newNode.constraints[i]);
        LowLevelSearch_t lowLevel(llenv);
        bool success = lowLevel.search(initialStates[i], newNode.solution[i]);

        newNode.cost += newNode.solution[i].cost;

        if (success) {
          // std::cout << "  success. cost: " << newNode.cost << std::endl;
          auto handle = open.push(newNode);
          (*handle).handle = handle;
        }

        ++id;
      }
    }

    return false;
  }
};

}  // namespace libMultiRobotPlanning
