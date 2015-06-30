#ifndef __astar__astar__
#define __astar__astar__

#include <vector>

/*!
 * @brief Position in a path or map with coordinates.
 */
struct Point {
    Point(int x = 0, int y = 0);
    int x, y;
};

/*!
 * @typedef Path
 * @brief List of points
 */
typedef std::vector<Point> Path;

/*!
 * @brief Run the A* path-finding algorithm
 * @param size Maximum X & Y size of the map
 * @param start Position to begin searching from
 * @param target Position to end the path at
 * @param obstacles List of positions to skip
 * @return list of points in the computed path from start to target.
 *   If no paths are possible, returns a list containing only target.
 */
Path astar(int size, Point start, Point target, Path obstacles);

#endif /* defined(__astar__astar__) */
