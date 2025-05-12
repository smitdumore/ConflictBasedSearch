# Table of Contents
- **[How to run](#how-to-run)**
- **[References](#references)**
- **[Mutable vs Non-Mutable Priority Queue for Astar](#mutable-vs-non-mutable-priority-queue-for-astar)**
- **[Wait Action in A star](#wait-action-in-a-star)**
- **[ECBS and Weighted Astar](#ecbs-and-weighted-astar)**

# How to run : 
```
cd build
./run_cbs -i ../maps/map_experiment.yaml -o output.yaml #deprecated
./run_app
```
./interactive_cbs.py maps/map_7by7_obst_agents6.yaml
```

cd ..
python3 visualize.py maps/map_experiment.yaml /home/smit/catkin_ws/src/cbs/build/output.yaml #deprecated
```



# References :
1. https://github.com/whoenig/libMultiRobotPlanning./run_cbs -i ../map_5by5_obst_agents2.yaml -o output.yaml
2. https://github.com/yangda75/naiveMAPF/
3. https://www.cs.cmu.edu/~motionplanning/lecture/Asearch_v8.pdf

# Mutable vs Non-Mutable Priority Queue for Astar

1. We keep pushing new (better) entries, so multiple copies of the same node can end up in the queue—some with better f values, some with worse.
2. When a worse entry “rises” to the top, we detect it’s outdated using this condition
    ```
    if (fCurrent > dist[xCurrent][yCurrent] + heuristic(...)) {
        // skip this old copy
    }
    ```
3. In other words, we have a cheaper path to that node now, so there’s no point in spending time exploring the old, more expensive path

## Why we re-open closed nodes ?

## Grid Setup
Consider a 3×3 grid with cells labeled (row, col):
    ```
    (0,0)   (0,1)   (0,2)
    (1,0)   (1,1)   (1,2)
    (2,0)   (2,1)   (2,2)
    ```
    
Let’s define the cost to enter each cell (i.e., grid[r][c]):

    ```
        Column 0	Column 1	Column 2
    Row 0	1	       5	      1
    Row 1	1	       1	      1
    Row 2	1	       1	      1
    ```

- grid[0][0] = 1 means it “costs 1” to move onto (0,0) from an adjacent cell.
- grid[0][1] = 5 means it “costs 5” to move onto (0,1).
… and so on. A 0 would mean blocked, but here all are non-zero for simplicity.
### Start and Goal
- Start at (0,0).
- Goal at (2,2).

## Step-by-Step Walkthrough
### Initialization

- dist[][] is set to a large value everywhere. Then we set:
- ```dist[0][0] = 0 ```, because the cost to “start” is zero from itself.
- ```closed[][]``` is ```false``` initially for all cells.

### Push (0,0) into the PQ

- ```f(0,0) = g(0,0) + h(0,0)``` (whatever the heuristic is—just assume we push it with some f-value).
- Priority queue (PQ): [ (0,0) ] (lowest f on top).

### Pop (0,0)

- We pop ```(0,0)``` from the PQ.
- Mark ```closed[0][0] = true``` because we’re finalizing its cost.

### Explore neighbors of (0,0)
- Let’s assume 4-direction movement (up/down/left/right) for simplicity:

- (1,0) is valid
- ```tentative_g = dist[0][0] + grid[1][0]```
- ```dist[0][0] = 0, grid[1][0] = 1```, so ```tentative_g = 1```.
- ```tentative_g < dist[1][0]```? (which was ∞), so yes, update:
- 
    ```
    dist[1][0] = 1;
    cameFrom[1][0] = (0,0);
    closed[1][0] is false, so we just push it into PQ.
    ```
- (0,1) is valid
- ```tentative_g = dist[0][0] + grid[0][1] = 0 + 5 = 5```.
- ```tentative_g < dist[0][1]```? Yes (was ∞), so:
-
    ```
    dist[0][1] = 5;
    cameFrom[0][1] = (0,0);
    closed[0][1] is false, push into PQ.
    ```
Now PQ has (1,0) and (0,1).

### Pop the next node from PQ: Assume (1,0) (it likely has the smaller f-value if the heuristic is consistent).

- Mark closed[1][0] = true.

### Neighbors of (1,0)

- (2,0)
- ```tentative_g = dist[1][0] + grid[2][0] = 1 + 1 = 2.```
- ```tentative_g < dist[2][0]? Yes, so dist[2][0] = 2. Push (2,0).```
- (1,1)
- ```tentative_g = dist[1][0] + grid[1][1] = 1 + 1 = 2.```
- ```tentative_g < dist[1][1]? Yes, so dist[1][1] = 2.``` Push ```(1,1).```
- (0,0) is already closed, but that’s our start—usually we don’t go back. Or if we do check, we see it’s not better.
- Now PQ might have (0,1), (2,0), (1,1).

### Pop the next node—say it’s ```(1,1)``` because it might have a smaller f-value than ```(0,1)``` (which has g=5).

- Mark ```closed[1][1] = true```.

### Neighbors of (1,1)

- Let’s look at ```(0,1)```:
- ```tentative_g = dist[1][1] + grid[0][1] = 2 + 5 = 7```.
- Compare with ```dist[0][1]``` which is 5.
- ```7 < 5?``` No, it’s bigger. Not better. We do nothing.
- Look at ```(1,0)``` (already closed, we came from there).
- Look at ```(1,2)``` (if within bounds) or ```(2,1)```, etc. Let’s pick ```(2,1)```:
- ```tentative_g = dist[1][1] + grid[2][1] = 2 + 1 = 3.```
- ```dist[2][1]``` was ∞, so set ```dist[2][1] = 3```, push ```(2,1)``` in PQ.

### At some point, let’s say ```(0,1)``` is expanded

- If we pop ```(0,1)``` with ```g=5```, we set ```closed[0][1] = true.```
- Then we check its neighbors, like ```(0,2)```, ```(1,1)```, ```(0,0)```. Possibly ```(1,1)``` is already closed with g=2, so we check:
- ```tentative_g = 5 + grid[1][1] = 6.```
- ```dist[1][1]``` is 2. ```6 < 2```? No. Not better, skip.

### Imagine We Discover a Cheaper Path
Here’s the moment that triggers “re-open”:

Let’s say we eventually pop (2,0) with g=2. We check neighbor (2,1):

- We might get a new cost for (2,1) that is cheaper than 3 (imagine if grid[2][1] was 0.5 or something smaller—just a hypothetical to show improvement).
- If that improvement is discovered after (2,1) was already “closed”, then ```tentative_g < dist[2][1]```. We do:
- 
    ```
    dist[2][1] = newCheaperValue;
    if (closed[2][1]) {
        closed[2][1] = false;  // re-open
    }
    pq.push({ newF, 2, 1 });
    ```

- That means we thought (2,1) was done with cost=3, but we found a new path cost=2.5, so we “un-close” (2,1) and push it again. This is re-opening.

# "Wait" Action in A star

- Yes, you can absolutely include a “wait” action in an A* search.

## Cost Assignment:
- If a “wait” action has zero cost (or a very small cost), and your goal is just to minimize travel distance, the search could exploit that to remain in place indefinitely if that somehow avoids a large cost move. This is often undesirable.
- Usually, you’d give the “wait” action a time or step cost just like a movement. If each step—no matter whether you move or wait—costs 1, then waiting isn’t “free” and won’t be abused.
- In implementation the ```getNeighbors``` will return 5 neighbors ```left, right, up, down, wait```
- 
    ```
    std::vector<int> vecx = {0,  0, -1, 1,  0};
    std::vector<int> vecy = {-1, 1,  0, 0,  0};
    ```

# ECBS and Weighted Astar

## Clarifying the Focal Set in Weighted A*
- bestFScore is the current minimum f-value in the openSet.
- The focal set is a subset of nodes that are reasonably close to this minimum defined by the condition:
    ```
    f(n)≤w×bestFScore
    ```

### Example:

- Assume bestFScore = 10 and w=1.2.
- The suboptimality bound is 1.2×10=12.
- The focal set will include all nodes with f(n)≤12.