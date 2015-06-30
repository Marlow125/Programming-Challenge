#include "astar.h"
#include <math.h>
#include <map>
#include <set>
#include <sstream>
#include <memory>
#include <stdexcept>
#include <iostream>

const float robot_width = 2;

template<class P>
static double distance(P one, P two)
{
    return sqrt(pow(one.x - two.x, 2) + pow(one.y - two.y, 2));
}

template<class P, class Container>
static inline bool position_open(P pos, Container& obstacles, int size)
{
    if (pos.x < 0 || pos.y < 0 || pos.x > size || pos.y > size)
        return false; // out of bounds
    for (typename Container::iterator it = obstacles.begin(); it < obstacles.end(); ++it)
        if (*it == pos)
            return false; // blocked
    return true;
}

class Node {
public:
    void compute_costs()
    {
        if (parent != NULL) {
            g = distance(*parent, *this);
            if (goal == NULL)
                goal = parent->goal;
        } else {
            g = 1;
        }
        if (goal != NULL) {
            h = distance(*this, *goal);
        } else {
            h = 0;
        }
    }
    Node()
    {
    }
    Node(Point pt, Node *parent = NULL, Node *goal = NULL) : x(pt.x), y(pt.y), parent(parent), goal(goal), dist(0)
    {
    }
    void copy(Node &other)
    {
        x = other.x;
        y = other.y;
        parent = other.parent;
        goal = other.goal;
        compute_costs();
    }
    int x, y, g, h;
    float dist;
    Node *parent;
    Node *goal;
    friend bool operator==(const Node& one, const Node& two)
    {
        return one.x == two.x && one.y == two.y;
    }
    friend bool operator!=(const Node& one, const Node& two)
    {
        return one.x != two.x || one.y != two.y;
    }
    friend bool operator<(const Node& one, const Node& two)
    {
        return one.x < two.x && one.y < two.y;
    }
    int total_cost(void)
    {
        return g + h;
    }
    template<class Container>
    std::vector<Node> successors(Container &obstacles, int size)
    {
        std::vector<Node> successors;
        for (int xd = -1; xd <= 1; xd++) {
            for (int yd = -1; yd <= 1; yd++) {
                Node n({x + xd, y + yd}, this);
                if (!position_open(n, obstacles, size))
                    continue;
                if (n == *this || (parent != NULL && n == *parent))
                    continue;
                successors.push_back(n);
            }
        }
        return successors;
    }
};

Point::Point(int x, int y) : x(x), y(y)
{
}

template<class Container>
static typename Container::iterator get(Container& list, Node pt)
{
    for (typename Container::iterator it = list.begin(); it < list.end(); ++it)
        if (**it == pt)
            return it;
    return list.end();
}

template<class Container>
static Node* min(Container& stp)
{
    if (stp.empty())
        throw std::runtime_error("stp empty");
    Node *current = *stp.begin();
    int H = current->total_cost();
    for (typename Container::iterator it = stp.begin(); it < stp.end(); ++it) {
        if ((*it)->total_cost() < H) {
            H = (*it)->total_cost();
            current = *it;
        }
    }
    return current;

	
// Where the magic happens
Path astar(int size, Point start, Point target, Path obstacles)
{
    size_t nodeMemMax = size * size;
    Node *nodeMem = new Node[sizeof(Node) * nodeMemMax];
    size_t nodeMemSP = 0;
    Node targetNode(target);
    // open array contain points that still need to be processed
    // closed array contains that have
    std::vector<Node*> open, closed;
    {
        Node startNode(start, NULL, &targetNode);
        Node *startNodeS = ++nodeMemSP + nodeMem;
        startNodeS->copy(startNode);
        open.push_back(startNodeS);
    }
    while (!open.empty()) {
        // get the point from the open array with the smallest total cost
        // this will be the next point the path will follow
        Node *current = min(open);
        for (auto it = open.begin(); it < open.end(); ++it) {
            if (*it == current) {
                open.erase(it); // find the point in the open array and delete it (because it is moving to closed)
                break;
            }
        }
        if (*current == targetNode) {
            // reached the target, done
            targetNode.parent = current->parent;
            break;
        }
        // get a list of all points that the path can potentially take from the current point
        // it checks to make sure it does not chart a course through an obstacle
        for (Node successor : current->successors(obstacles, size)) {
            auto oIt = get(open, successor), cIt = get(closed, successor);
            if ((oIt != open.end() && (*oIt)->total_cost() - current->total_cost() <= 0)
                 || (cIt != closed.end()))
                continue;
            // if this point is already in a list and has more cost,
            // or if it is too close to an obstacle, ignore it
            // remove the point from an array if its already there
            // the new one might have a different parent or cost
            if (oIt != open.end()) {
                open.erase(oIt);
                // may leak memory here, but using delete causes lots of errors
            }
            if (cIt != closed.end()) {
                closed.erase(cIt);
            }
            if (nodeMemSP >= nodeMemMax) {
                throw new std::runtime_error("Node storage stack ran out of memory");
            }
            Node *valid = ++nodeMemSP + nodeMem; // get a new node from the stack
            valid->copy(successor);
            // submit this new point to process next time to extend the path
            open.push_back(valid);
        }
        // mark the current point as traversed
        closed.push_back(current);
    }
    Path results;
    // Traverse backwards from the target by jumping to each successive parent
    Node *p = &targetNode;
    while (p != NULL) {
        results.push_back({p->x, p->y});
        p = p->parent;
    }
    // Free some memory from the pointers
    delete [] nodeMem;
    return results;
}
