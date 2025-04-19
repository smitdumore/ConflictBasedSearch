#pragma once

#include <map>
#include "common.hpp"
#include "a_star.hpp"

using namespace MultiRobotPlanning;

/*******************************************ENVIRONMENT****************************************************/
class Environment {
 public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
              std::vector<Location> goals, bool disappearAtGoal = false, int spaceSlack = 2)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
        m_disappearAtGoal(disappearAtGoal),
        m_spaceSlack(spaceSlack)
  {
  }

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  int admissibleHeuristic(const State& s) {
    // std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
    // s.y] << std::endl;
    // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
    return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);
  }

  bool isSolution(const State& s) {
    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
    neighbors.clear();
    {
      State n(s.time + 1, s.x, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, 1)); // WAIT action
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Left, 1)); // LEFT action
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Right, 1)); // RIGHT action
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1)); // UP action
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Down, 1)); // DOWN action
      }
    }
  }

  bool getFirstConflict( const std::vector<PlanResult<State, Action, int> >& solution,
                          Conflict& result) {

    /**
     * @param : solution is a CBS solution, its size is the number of agents
     *          It is essentially a vector of paths of all robots (x,y,time) 
    */

    // Find maximum timestep
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    const size_t no_of_agents = solution.size();

    // Using a unified approach for all conflict types with direction-sensitive proximity detection
    
    // Check for vertex conflicts and proximity conflicts
    for (int t = 0; t <= max_t; ++t) {
      for (size_t i = 0; i < no_of_agents; ++i) {
        State state1 = getState(i, solution, t);
        
        for (size_t j = i + 1; j < no_of_agents; ++j) {
          State state2 = getState(j, solution, t);
          
          // First check for vertex conflicts (exact same position)
          if (state1.equalExceptTime(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            return true;
          }
          
          // Then check for proximity conflicts with direction-of-travel focus
          int dx = std::abs(state1.x - state2.x);
          int dy = std::abs(state1.y - state2.y);
          
          // If space slack is 1, we don't check for proximity conflicts (default CBS behavior)
          if (m_spaceSlack > 1) {
            // Agents in the same lane need to maintain the full space slack
            if (dy == 0 && dx < m_spaceSlack) {
              result.time = t;
              result.agent1 = i;
              result.agent2 = j;
              result.type = Conflict::Proximity;
              result.x1 = state1.x;
              result.y1 = state1.y;
              result.x2 = state2.x;
              result.y2 = state2.y;
              return true;
            }
            // Agents in adjacent lanes (dy=1) need reduced spacing in the direction of travel
            else if (dy == 1 && dx < m_spaceSlack - 1) {
              result.time = t;
              result.agent1 = i;
              result.agent2 = j;
              result.type = Conflict::Proximity;
              result.x1 = state1.x;
              result.y1 = state1.y;
              result.x2 = state2.x;
              result.y2 = state2.y;
              return true;
            }
            // Agents further apart vertically have no proximity constraints
          }
        }
      }
    }

    // Check for edge conflicts (agents swapping positions)
    for (int t = 0; t < max_t; ++t) {
      for (size_t i = 0; i < no_of_agents; ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        
        for (size_t j = i + 1; j < no_of_agents; ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          
          // Check if agents are swapping positions
          if (state1a.equalExceptTime(state2b) && state1b.equalExceptTime(state2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }

  void createConstraintsFromConflict( const Conflict& conflict, std::map<size_t, Constraints>& constraints) {

    /**
     * @param : constriants : It maps Agent to constraints data struct 
    */

    if (conflict.type == Conflict::Vertex) {
      Constraints c1; // This data structure holds 2 sets of vertex and edge constraints

      // Vertex Constraint : Dont occupy x1,y1 at t
      c1.vertexConstraints.emplace(VertexConstraint(conflict.time, conflict.x1, conflict.y1));

      // Mapping agents involved to their resp. constraints
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } 
    else if (conflict.type == Conflict::Edge) {
      Constraints c1; // This data structure holds 2 sets of vertex and edge constraints
      c1.edgeConstraints.emplace(EdgeConstraint(conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1; // Mapping agent to its resp. constraint
      
      Constraints c2; // This data structure holds 2 sets of vertex and edge constraints
      c2.edgeConstraints.emplace(EdgeConstraint(conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2; // Mapping agent to its resp. constraint
    }
    else if (conflict.type == Conflict::Proximity) {
      // For proximity conflicts, we create vertex constraints
      // We constrain one agent from being at its position at that time
      // And the other agent from being at its position at that time
      // This forces one of them to find an alternative path

      // SLACK IS ONLY IN THE DIRECTION OF TRAVEL, WE INFLATE THE NEXT X NODES IN THE PATH , 
      // NOT ALL THE 4/8 CONNECTED NEIGHBORS OF THE NODES ON THE PATH 
      
      Constraints c1;
      c1.vertexConstraints.emplace(VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      
      Constraints c2;
      c2.vertexConstraints.emplace(VertexConstraint(conflict.time, conflict.x2, conflict.y2));
      constraints[conflict.agent2] = c2;
    }
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int> >& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    if (m_disappearAtGoal) {
      // This is a trick to avoid changing the rest of the code significantly
      // After an agent disappeared, put it at a unique but invalid position
      // This will cause all calls to equalExceptTime(.) to return false.
      return State(-1, -1 * (agentIdx + 1), -1);
    }
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool transitionValid(const State& s1, const State& s2) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }
#if 0
  // We use another A* search for simplicity
  // we compute the shortest path to each goal by using the fact that our getNeighbor function is
  // symmetric and by not terminating the AStar search until the queue is empty
  void computeHeuristic()
  {
    class HeuristicEnvironment
    {
    public:
      HeuristicEnvironment(
        size_t dimx,
        size_t dimy,
        const std::unordered_set<Location>& obstacles,
        std::vector<int>* heuristic)
        : m_dimx(dimx)
        , m_dimy(dimy)
        , m_obstacles(obstacles)
        , m_heuristic(heuristic)
      {
      }

      int admissibleHeuristic(
        const Location& s)
      {
        return 0;
      }

      bool isSolution(
        const Location& s)
      {
        return false;
      }

      void getNeighbors(
        const Location& s,
        std::vector<Neighbor<Location, Action, int> >& neighbors)
      {
        neighbors.clear();

        {
          Location n(s.x-1, s.y);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Left, 1));
          }
        }
        {
          Location n(s.x+1, s.y);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Right, 1));
          }
        }
        {
          Location n(s.x, s.y+1);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Up, 1));
          }
        }
        {
          Location n(s.x, s.y-1);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Down, 1));
          }
        }
      }

      void onExpandNode(
        const Location& s,
        int fScore,
        int gScore)
      {
      }

      void onDiscover(
        const Location& s,
        int fScore,
        int gScore)
      {
        (*m_heuristic)[s.x + m_dimx * s.y] = gScore;
      }

    private:
      bool stateValid(
        const Location& s)
      {
        return    s.x >= 0
               && s.x < m_dimx
               && s.y >= 0
               && s.y < m_dimy
               && m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end();
      }

    private:
      int m_dimx;
      int m_dimy;
      const std::unordered_set<Location>& m_obstacles;
      std::vector<int>* m_heuristic;

    };

    m_heuristic.resize(m_goals.size());

    std::vector< Neighbor<State, Action, int> > neighbors;

    for (size_t i = 0; i < m_goals.size(); ++i) {
      m_heuristic[i].assign(m_dimx * m_dimy, std::numeric_limits<int>::max());
      HeuristicEnvironment henv(m_dimx, m_dimy, m_obstacles, &m_heuristic[i]);
      AStar<Location, Action, int, HeuristicEnvironment> astar(henv);
      PlanResult<Location, Action, int> dummy;
      astar.search(m_goals[i], dummy);
      m_heuristic[i][m_goals[i].x + m_dimx * m_goals[i].y] = 0;
    }
  }
#endif
 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<Location> m_obstacles;
  std::vector<Location> m_goals;
  // std::vector< std::vector<int> > m_heuristic;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  bool m_disappearAtGoal;
  int m_spaceSlack;
};
/*******************************************ENVIRONMENT****************************************************/

