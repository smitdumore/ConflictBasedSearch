#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

static const int INF = std::numeric_limits<int>::max();

// We can move in 8 directions (including diagonals)
int dir_x[8] = { 1, -1,  0,  0,  1,  1, -1, -1 };
int dir_y[8] = { 0,  0,  1, -1,  1, -1,  1, -1 };
constexpr int movement_size = 8;

// Simple heuristic function (e.g., Manhattan, Chebyshev, or Octile)
int heuristic(int x, int y, int goalX, int goalY) {
    // For demonstration, we’ll just do Manhattan
    // but keep in mind for diagonals you may want
    // Chebyshev or Octile to maintain consistency.
    return std::abs(goalX - x) + std::abs(goalY - y);
}

// To store info in our priority queue
// We'll store (fCost, x, y). The queue is min-heap by fCost
struct Node {
    int f;
    int x;
    int y;
};

// Comparator for our min-heap
// We want the smallest 'f' on top
struct CompareNode {
    bool operator()(const Node& a, const Node& b) {
        return a.f > b.f; // bigger f has lower priority
    }
};

// Reconstruct path from a 'cameFrom' map
std::vector<std::pair<int,int>> backtrack(
    const std::vector<std::vector<std::pair<int,int>>>& cameFrom,
    std::pair<int,int> start,
    std::pair<int,int> goal)
{
    std::vector<std::pair<int,int>> path;
    auto current = goal;
    while (current != start) {
        path.push_back(current);
        current = cameFrom[current.first][current.second];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

/**
 * A* with Re-Expansions
 *
 * \param grid 2D matrix of costs. 0 means blocked, >0 means cost to enter.
 * \param n dimension of the grid
 * \param startX, startY starting cell
 * \param goalX, goalY goal cell
 *
 * \return path as a list of (x,y) from start -> goal, empty if no path.
 */
std::vector<std::pair<int,int>> aStarReexpansion(
    const std::vector<std::vector<int>>& grid,
    int n,
    int startX, int startY,
    int goalX, int goalY)
{
    // If start or goal is blocked, return empty
    if (grid[startX][startY] == 0 || grid[goalX][goalY] == 0) {
        return {};
    }

    // distance array (g-cost)
    std::vector<std::vector<int>> dist(n, std::vector<int>(n, INF));

    // closed[x][y] = have we permanently closed this cell?
    // If we discover a cheaper path after it's closed, we will reopen it
    std::vector<std::vector<bool>> closed(n, std::vector<bool>(n, false));

    // "cameFrom[x][y]" to help reconstruct path
    // We'll store a parent coordinate
    std::vector<std::vector<std::pair<int,int>>> cameFrom(n, 
        std::vector<std::pair<int,int>>(n, {-1, -1}));

    // Min-heap priority queue: we store (f, x, y)
    std::priority_queue<Node, std::vector<Node>, CompareNode> pq;

    // Initialize start
    dist[startX][startY] = 0;
    int hStart = heuristic(startX, startY, goalX, goalY);
    pq.push({ dist[startX][startY] + hStart, startX, startY });
    cameFrom[startX][startY] = {startX, startY}; // parent of start = itself

    while (!pq.empty()) {
        Node top = pq.top();
        pq.pop();

        int fCurrent = top.f;
        int xCurrent = top.x;
        int yCurrent = top.y;

        // If we've *already* improved this cell after pushing 'top' to the queue,
        // or if we have re-closed it, we skip.
        // explanatino in readme
        if (fCurrent > dist[xCurrent][yCurrent] + heuristic(xCurrent, yCurrent, goalX, goalY)) {
            continue; // it's stale
        }

        // If this cell was closed with a *better* cost after 'top' was pushed, skip
        if (closed[xCurrent][yCurrent]) {
            continue;
        }

        // If we've reached the goal, reconstruct path!
        if (xCurrent == goalX && yCurrent == goalY) {
            // build the path
            return backtrack(cameFrom, {startX, startY}, {goalX, goalY});
        }

        // Otherwise, we "close" it now
        closed[xCurrent][yCurrent] = true;

        // Explore neighbors
        for (int i = 0; i < movement_size; i++) {
            int nx = xCurrent + dir_x[i];
            int ny = yCurrent + dir_y[i];
            if (nx < 0 || nx >= n || ny < 0 || ny >= n) {
                continue; // out of bounds
            }
            if (grid[nx][ny] == 0) {
                continue; // blocked cell
            }
            // cost to move onto neighbor
            int stepCost = grid[nx][ny];

            int tentative_g = dist[xCurrent][yCurrent] + stepCost;

            // If we found a better path to neighbor
            if (tentative_g < dist[nx][ny]) {
                dist[nx][ny] = tentative_g;
                cameFrom[nx][ny] = {xCurrent, yCurrent};

                int newF = tentative_g + heuristic(nx, ny, goalX, goalY);

                // If it was closed before, we "re-open" it
                // we are here because "tentative_g < dist[nx][ny]" this condition passed
                // 
                if (closed[nx][ny]) {
                    closed[nx][ny] = false;
                }

                // push updated cost into the queue
                pq.push({ newF, nx, ny });
            }
        }
    }

    // If we exit the while loop, there's no path
    return {};
}

/*
int main() {
    // Example usage
    // Grid: 1 = passable with cost=1, 0 = blocked
    std::vector<std::vector<int>> grid = {
        {1, 1, 1},
        {1, 0, 1},
        {1, 1, 1},
    };
    int n = 3;

    // Start
    int startX = 0, startY = 0;
    // Goal
    int goalX = 2, goalY = 2;

    auto path = aStarReexpansion(grid, n, startX, startY, goalX, goalY);

    if (!path.empty()) {
        std::cout << "Path found:\n";
        for (auto& p : path) {
            std::cout << "(" << p.first << ", " << p.second << ") -> ";
        }
        std::cout << "GOAL\n";
    } else {
        std::cout << "No path found!\n";
    }

    return 0;
}
*/
