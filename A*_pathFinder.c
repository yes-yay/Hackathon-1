// Online C compiler to run C program online
//sri rama jayam 
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

// Define the dimensions of the grid
#define ROWS 4
#define COLS 4

// Define structure for a cell
typedef struct {
    int x, y;
} Cell;

// Define structure for a node
typedef struct {
    Cell position;
    int g; // cost from start to this node
    int h; // heuristic cost (estimated cost to goal)
    int f; // total cost (f = g + h)
} Node;

// Function to calculate the Manhattan distance heuristic
int calculateHeuristic(Cell current, Cell goal) {
    return abs(current.x - goal.x) + abs(current.y - goal.y);
}

// Function to check if a cell is inside the grid
bool isValid(Cell cell) {
    return cell.x >= 0 && cell.x < ROWS && cell.y >= 0 && cell.y < COLS;
}

// Function to check if a cell is blocked (contains a non-zero value)
bool isBlocked(int grid[ROWS][COLS], Cell cell) {
    return grid[cell.x][cell.y] != 0;
}

// Function to perform A* search and store the path
bool aStar(int grid[ROWS][COLS], Cell start, Cell end, Cell path[ROWS * COLS], int* pathLength) {
    // Define 8 possible movement directions (including diagonals)
    int dx[] = {1, 1, 1, 0, 0, -1, -1, -1};
    int dy[] = {1, 0, -1, 1, -1, 1, 0, -1};

    // Create an open list (to be explored) and a closed list (already explored)
    Node openList[ROWS * COLS];
    Node closedList[ROWS * COLS];
    int openSize = 0, closedSize = 0;

    // Initialize the start node
    Node startNode = {start, 0, calculateHeuristic(start, end), 0};
    openList[openSize++] = startNode;

    while (openSize > 0) {
        // Find the node with the lowest total cost (f) in the open list
        int currentIndex = 0;
        for (int i = 1; i < openSize; i++) {
            if (openList[i].f < openList[currentIndex].f) {
                currentIndex = i;
            }
        }

        // Get the current node from the open list
        Node currentNode = openList[currentIndex];

        // Remove the current node from the open list
        for (int i = currentIndex; i < openSize - 1; i++) {
            openList[i] = openList[i + 1];
        }
        openSize--;

        // Add the current node to the closed list
        closedList[closedSize++] = currentNode;

        // If the current node is the goal, construct the path
        if (currentNode.position.x == end.x && currentNode.position.y == end.y) {
            int index = closedSize - 1;
            while (index >= 0) {
                path[*pathLength] = closedList[index].position;
                (*pathLength)++;
                index--;
            }
            return true;
        }

        // Explore neighbors
        for (int i = 0; i < 8; i++) {
            Cell neighborCell = {currentNode.position.x + dx[i], currentNode.position.y + dy[i]};

            // Check if the neighbor is valid and not blocked
            if (isValid(neighborCell) && !isBlocked(grid, neighborCell)) {
                // Calculate tentative g score
                int tentativeG = currentNode.g + 1;

                // Check if the neighbor is already in the closed list with a lower g score
                bool inClosedList = false;
                for (int j = 0; j < closedSize; j++) {
                    if (closedList[j].position.x == neighborCell.x && closedList[j].position.y == neighborCell.y) {
                        inClosedList = true;
                        break;
                    }
                }

                // If the neighbor is in the closed list with a lower g score, skip it
                if (inClosedList && tentativeG >= currentNode.g) {
                    continue;
                }

                // Check if the neighbor is in the open list or has a lower g score
                bool inOpenList = false;
                for (int j = 0; j < openSize; j++) {
                    if (openList[j].position.x == neighborCell.x && openList[j].position.y == neighborCell.y) {
                        inOpenList = true;
                        break;
                    }
                }

                // If the neighbor is not in the open list or has a lower g score, add it to the open list
                if (!inOpenList || tentativeG < openList[currentIndex].g) {
                    Node neighborNode = {neighborCell, tentativeG, calculateHeuristic(neighborCell, end), 0};
                    neighborNode.f = neighborNode.g + neighborNode.h;
                    openList[openSize++] = neighborNode;
                }
            }
        }
    }

    // If the open list is empty and the goal is not reached, there is no path
    return false;
}

int main() {
    int grid[ROWS][COLS] = {
        {1, 0, 1, 1},
        {0, 0, 0, 1},
        {1, 0, 1, 0},
        {1, 0, 0, 0}
    };

    Cell start = {0, 0};
    Cell end = {3, 3};

    Cell path[ROWS * COLS];
    int pathLength = 0;

    if (aStar(grid, start, end, path, &pathLength)) {
        printf("Path exists using A* algorithm:\n");
        for (int i = pathLength - 1; i >= 0; i--) {
            printf("(%d, %d) ", path[i].x, path[i].y);
        }
        printf("\n");
    } else {
        printf("No path exists using A* algorithm.\n");
    }

    return 0;
}
