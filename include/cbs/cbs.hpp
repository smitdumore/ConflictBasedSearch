#pragma once

#include <map>

#include "a_star.hpp"
#include "location.hpp"
#include <opencv2/opencv.hpp>


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
    std::map<int, Conflict> nodeConflicts;

  public:
  CBS(Environment& environment) : m_env(environment) {}

  std::map<int, int> parentMap;  // child_id → parent_id

  void visualizeNode(int nodeId,
                   const std::vector<PlanResult<State, Action, int>>& solution,
                   const Conflict* conflict,
                   const std::unordered_set<Location>& obstacles,
                   int dimx, int dimy)
  {
    const int scale = 40;
    const int agentRadius = 10;
    const int maxThickness = 8;

    cv::Mat img(dimy * scale, dimx * scale, CV_8UC3, cv::Scalar(255, 255, 255));

    // Draw obstacles
    for (const auto& obs : obstacles) {
        cv::rectangle(img,
                      cv::Point(obs.x * scale, obs.y * scale),
                      cv::Point((obs.x + 1) * scale, (obs.y + 1) * scale),
                      cv::Scalar(0, 0, 0), cv::FILLED);
    }

    // Assign consistent random color to each agent
    auto getColorForAgent = [](int agentId) -> cv::Scalar {
        cv::RNG rng(agentId * 1000);
        return cv::Scalar(rng.uniform(64, 255), rng.uniform(64, 255), rng.uniform(64, 255));
    };

    // ✅ Correct: thickest drawn first
    for (size_t agent = 0; agent < solution.size(); ++agent) {
        const auto& path = solution[agent].states;
        cv::Scalar color = getColorForAgent(agent);

        // Thickest first, decreasing
        int thickness = maxThickness - static_cast<int>((maxThickness - 1) * (float(agent) / (solution.size() - 1)));

        for (size_t i = 1; i < path.size(); ++i) {
          int t_prev = path[i - 1].second;
          int t_curr = path[i].second;

          // If conflict is present, only draw up to conflict time
          if (conflict && (t_prev > conflict->time || t_curr > conflict->time)) {
              break;
          }

          const auto& prev = path[i - 1].first;
          const auto& curr = path[i].first;

          cv::line(img,
                  cv::Point(prev.x * scale + scale / 2, prev.y * scale + scale / 2),
                  cv::Point(curr.x * scale + scale / 2, curr.y * scale + scale / 2),
                  color, thickness);
        }
    }

    // Draw agent positions on top
    for (size_t agent = 0; agent < solution.size(); ++agent) {
        const auto& path = solution[agent].states;
        if (!path.empty()) {
            const auto& p = path[0].first;
            cv::Scalar color = getColorForAgent(agent);
            cv::circle(img,
                       cv::Point(p.x * scale + scale / 2, p.y * scale + scale / 2),
                       agentRadius, color, cv::FILLED);
        }
    }

    // Draw conflict location on top
    std::string winTitle = "Node " + std::to_string(nodeId);

    // Reduced circle radius for subtler visualization
    int conflictMarkerRadius = agentRadius;

    if (conflict != nullptr) {
      std::cout << "[Node " << nodeId << "] conflict time: " << conflict->time
              << " type: " << (conflict->type == Conflict::Vertex ? "Vertex" : "Edge")
              << std::endl;
    }


    // After everything else is drawn, draw conflict marker
    // Define a red color
    if (conflict != nullptr) {
      // Define a red color
      cv::Scalar red(0, 0, 255);
      int crossSize = scale / 2;
      int thickness = 2;

      if (conflict->type == Conflict::Vertex) {
          // Draw red X at (x1, y1)
          cv::Point center(conflict->x1 * scale + scale / 2, conflict->y1 * scale + scale / 2);
          cv::line(img, center + cv::Point(-crossSize / 2, -crossSize / 2),
                        center + cv::Point(crossSize / 2,  crossSize / 2),
                        red, thickness);
          cv::line(img, center + cv::Point(-crossSize / 2,  crossSize / 2),
                        center + cv::Point(crossSize / 2, -crossSize / 2),
                        red, thickness);
      } else if (conflict->type == Conflict::Edge) {
          // Midpoint of (x1,y1) and (x2,y2)
          cv::Point p1(conflict->x1 * scale + scale / 2, conflict->y1 * scale + scale / 2);
          cv::Point p2(conflict->x2 * scale + scale / 2, conflict->y2 * scale + scale / 2);
          cv::Point mid = (p1 + p2) / 2;

          // Draw red X at midpoint
          cv::line(img, mid + cv::Point(-crossSize / 2, -crossSize / 2),
                        mid + cv::Point(crossSize / 2,  crossSize / 2),
                        red, thickness);
          cv::line(img, mid + cv::Point(-crossSize / 2,  crossSize / 2),
                        mid + cv::Point(crossSize / 2, -crossSize / 2),
                        red, thickness);
      }
    }

    

    std::string filename = "node_" + std::to_string(nodeId) + ".png";
    cv::waitKey(1);   // <-- force OpenCV to finalize all drawing operations
    cv::imwrite(filename, img);

  }

  std::map<int, int>& getParentMap() { return parentMap; }

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

      while (!open.empty()) {

        // Current Conflict Tree Node
        HighLevelNode currCTNode = open.top();
        open.pop();

        // Visualize the node with the conflict that created it
        auto it = nodeConflicts.find(currCTNode.id);
        const Conflict* conflictPtr = (it != nodeConflicts.end()) ? &(it->second) : nullptr;
        visualizeNode(currCTNode.id, currCTNode.solution, conflictPtr,
                      m_env.getObstacles(), m_env.getWidth(), m_env.getHeight());

        Conflict conflict; 
        bool hasConflict = m_env.getFirstConflict(currCTNode.solution, conflict);

        if (!hasConflict) {
          // if didnt get any conflict then return the solution of that node
          std::cout << "CBS done; cost: " << currCTNode.cost << std::endl;

           // Visualize solution node (no conflict)
          visualizeNode(currCTNode.id, currCTNode.solution, nullptr,
                  m_env.getObstacles(), m_env.getWidth(), m_env.getHeight());

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
          parentMap[newNode.id] = currCTNode.id;
          nodeConflicts[newNode.id] = conflict;
          
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

      return false;
    }
  };

}  // namespace libMultiRobotPlanning
