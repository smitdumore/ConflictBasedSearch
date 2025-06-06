#pragma once

namespace MultiRobotPlanning {

/*!
    This class represents a transition from the current state applying an action
   with the cost to take that action.

    \tparam State Custom state for the search. Needs to be copy'able
    \tparam Action Custom action for the search. Needs to be copy'able
    \tparam Cost Custom Cost type (integer or floating point types)
*/
template <typename State, typename Action, typename Cost>
struct Neighbor {
  Neighbor(const State& state, const Action& action, Cost cost)
      : state(state), action(action), cost(cost) {}

  //! neighboring state
  State state;
  //! action to get to the neighboring state
  Action action;
  //! cost to get to the neighboring state
  Cost cost;
};

}  // namespace MultiRobotPlanning
