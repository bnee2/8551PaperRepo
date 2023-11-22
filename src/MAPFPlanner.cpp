#include <MAPFPlanner.h>
#include <random>

int WINDOW_SIZE = 16;
int GLOBAL_TIMESTEP = 0;
MAPFPlanner::CTNode* MEMORY_CBS_NODE = nullptr;

/*
Represents a node in the high-level constraint tree
*/
struct MAPFPlanner::CTNode
{
    vector<Constraint> constraints;  // The constraints that apply to this node
    vector<list<pair<int, int>>> solution;  // A path from current position to goal for each agent
    int cost;  // The total path cost of the solution
    int depth;  // The depth of the CT node
    int remainingActions;  // How many more actions can this node support

    CTNode(vector<Constraint> _constraints, int _depth):
        constraints(_constraints), depth(_depth) {}
};

struct AstarNode
{
    int location;   // Map index of position
    int direction;  // Direction of position
    int f;          // Estimated total cost to goal; sum of g and h
    int g;          // Total cost along from start to this position
    int h;          // Estimated additional cost to goal from this position
    AstarNode* parent;  // The node that leads to this node
    int timeStep;   // Stores the time step that this position will be reached at
    bool closed = false;    // Whether this node has been fully explored or not

    AstarNode(int _location, int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}

    AstarNode(int _location,int _direction, int _g, int _h, int _timeStep, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),timeStep(_timeStep),parent(_parent) {}
};

// Comparison operator for A* nodes
struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) {
            return a->g <= b->g;
        }
        else {
            return a->f > b->f;
        }
    }
};

// Map preprocessing section. Not used currently
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    cout << "planner initialize done" << endl;
}

// Planning algorithm called at each time step to compute the action for the agent
void MAPFPlanner::plan(int time_limit, vector<Action> & actions) 
{
    // WHCAPlan(time_limit, actions);
    ICTSPlan(time_limit, actions);
    // CBSPlan(time_limit, actions);
    GLOBAL_TIMESTEP++;
}

/*
*****************************************************************************************
*******************************************************************************************
*******************************************************************************************


                                Windowed Hierarchical Cooperative A*


*******************************************************************************************
*******************************************************************************************
*/

/*
Plans agent paths using WHCA* (windowed hierarchical cooperative A*)
Greedy search that optimizes each agent's path cost in order of priority
Based on David Silver's 2005 paper "Cooperative Pathfinding"
*/
void MAPFPlanner::WHCAPlan(int time_limit, vector<Action> & actions)
{

    if (MEMORY_CBS_NODE == nullptr || MEMORY_CBS_NODE->remainingActions == 0)
    {
        // delete MEMORY_CBS_NODE;
        actions = std::vector<Action>(env->curr_states.size(), Action::W);
        vector<list<pair<int,int>>> solution;  // Caching for found solutions
        solution.resize(env->num_of_agents);
        int minPathLength = INT_MAX;

        // Randomize agent planning order
        vector<int> agentIndexes = createRandomAgentIndexes(env->num_of_agents);

        // Initialize reservation table with current agent states so that nobody steps on someone else
        std::unordered_map<string, bool> reservationTable;

        for (auto & i : agentIndexes)
        {
            list<pair<int,int>> path;

            // No path planned if agent has no goal
            if (env->goal_locations[i].empty()) 
            {
                path.push_back({env->curr_states[i].location, env->curr_states[i].orientation});
            } 
            // Plan agent path using HCA*
            else 
            {
                path = WHCAPath(i,
                                env->curr_states[i].location,
                                env->curr_states[i].orientation,
                                env->goal_locations[i].front().first,
                                reservationTable,
                                WINDOW_SIZE);
            }

            // Try to break cycles by moving in suboptimal direction if it's open
            if (path.empty())
            {
                AttemptForwardMoveIfAllowed(i, reservationTable, path);
            }

            // We can't move unless we found a path
            // Once path is computed, action is move forward if next location on path is different from current location
            // Otherwise, the correct action is to compute a direction to turn in
            if ((path.front().first != env->curr_states[i].location) && !path.empty())
            {
                actions[i] = Action::FW;  // Move forward
            } 
            else if (path.front().second != env->curr_states[i].orientation)
            {
                int orientationDiff = path.front().second - env->curr_states[i].orientation;
                if (orientationDiff == 1 || orientationDiff == -3)
                {
                    actions[i] = Action::CR; // Clockwise rotation
                } 
                else if (orientationDiff == -1 || orientationDiff == 3)
                {
                    actions[i] = Action::CCR; // Counter clockwise rotation
                } 
                // Path cannot result in orientation diff of 2 since it would represent that as two 90-degree rotations
            }

            if (path.empty())
            {
                minPathLength = -1;
            }
            else
            {
                path.pop_front();
                if (minPathLength > path.size())
                {
                    minPathLength = path.size();
                }
                solution[i] = path;
            }
        }
        if (minPathLength > 0)
        {
            MEMORY_CBS_NODE = new CTNode(vector<Constraint>(),0);
            MEMORY_CBS_NODE->solution = solution;
            MEMORY_CBS_NODE->remainingActions = minPathLength;
        }
        else
        {
            DebugPrint("At least one agent had no solution");
            MEMORY_CBS_NODE = nullptr;
        }
    }
    else
    {
        actions = ExtractActions(MEMORY_CBS_NODE);
        RemoveFirstPathNode(MEMORY_CBS_NODE);
    }

    return;
}

/*
Plans agent paths using WHCA* (windowed hierarchical cooperative A*)

start is the map index to start from
startDirection is the direction that the agent is starting in

The difference between this function and the single_agent_plan function is that this one respects
a reservation table.
*/
list<pair<int,int>> MAPFPlanner::WHCAPath(int agentIndex, int start, int startDirection, int end, std::unordered_map<string,bool> & reservationTable, int searchWindow = 10000)
{
    list<pair<int,int>> path;
    priority_queue<AstarNode*, vector<AstarNode*>, cmp> openList;
    unordered_set<int> closedList;
    unordered_map<int, AstarNode*> allNodes;  // Used to delete A* nodes after search is complete

    // Initialize data structures to start search
    AstarNode* startNode = new AstarNode(start, startDirection,
                                        0, single_agent_plan(start, startDirection, end).size(), //getManhattanDistance(start, end),  //
                                        0, nullptr);
    openList.push(startNode);
    allNodes[start*4 + startDirection] = startNode;

    while (!openList.empty())
    {
        // Select "best" node from top of list
        AstarNode* currentNode = openList.top();
        openList.pop();
        closedList.emplace(currentNode->location*4 + currentNode->direction);  // Closed list contains ints for every closed state

        // If node is goal, construct path and break
        if (currentNode->location == end
            || currentNode->timeStep == searchWindow)
        {
            while(currentNode->parent!=NULL) 
            {
                path.emplace_front(make_pair(currentNode->location, currentNode->direction));
                currentNode = currentNode->parent;
            }

            int timeStep = 1;  // First node in path is not the current node that the robot is on
            for (auto index = path.begin(); index != path.end(); ++index)
            {
                reservationTable[GetTableKey(timeStep, index->first)] = true;
                timeStep++;
            }

            break;
        }

        // Otherwise, get all neighbors of the expanded node
        list<pair<int,int>> neighbors = WHCAGetNeighbors(agentIndex, currentNode->location, currentNode->direction, currentNode->timeStep, reservationTable);
        for (const pair<int,int>& neighbor: neighbors)
        {
            // If the current neighbor is in the closed list, skip it
            if (closedList.find(neighbor.first*4 + neighbor.second) != closedList.end())
            {
                continue;
            }

            // If it already exists in the list of existing nodes, we might need to update it's shortest path value
            if (allNodes.find(neighbor.first*4 + neighbor.second) != allNodes.end())
            {
                AstarNode* oldNode = allNodes[neighbor.first*4 + neighbor.second];
                if (currentNode->g + 1 < oldNode->g)  // +1 because g to this neighbor will be one higher than g to current node
                {
                    oldNode->g = currentNode->g+1;
                    oldNode->f = currentNode->h+oldNode->g;
                    oldNode->parent = currentNode;
                }
            }
            // If it doesn't exist, we'll create a new node for it
            else
            {
                AstarNode* newNode = new AstarNode(neighbor.first, neighbor.second,
                    currentNode->g+1, single_agent_plan(neighbor.first, neighbor.second, end).size(),  //getManhattanDistance(neighbor.first,end),  //single_agent_plan(neighbor.first, neighbor.second, end).size(),
                    currentNode->timeStep+1, currentNode);
                openList.push(newNode);
                allNodes[neighbor.first*4 + neighbor.second] = newNode;
            }
        }
    }

    if (openList.empty())
    {
        DebugPrint("No path found for agent " + std::to_string(agentIndex));
    }

    // Clean up data structures and return path
    for (auto node: allNodes)
    {
        delete node.second;
    }
    allNodes.clear();
    return path;
}

/*
Determines what the valid neighbors are during a WHCA search.

location is the CURRENT location
direction is the CURRENT direction
timeStep is the CURRENT time step

The difference between this and the original A* getNeighbors function is that this function
must check the reservation table to avoid conflicts with higher-priority agents.
*/
list<pair<int,int>> MAPFPlanner::WHCAGetNeighbors(int agentIndex, int location, int direction, int timeStep, std::unordered_map<string,bool> & reservationTable)
{
    list<pair<int,int>> neighbors;
    int candidates[4] = { location + 1, location + env->cols, location - 1, location - env->cols};

    // forward
    int forwardLocation = candidates[direction];
    int newDirection = direction;
    if (WHCAValidateMove(agentIndex, timeStep + 1, location, forwardLocation, reservationTable)
        && WHCAValidateMove(agentIndex, timeStep, location, forwardLocation, reservationTable))
    {
        neighbors.emplace_back(make_pair(forwardLocation, newDirection));
    }

    // turn left
    newDirection = direction-1;
    if (newDirection == -1)
    {
        newDirection = 3;
    }
    neighbors.emplace_back(make_pair(location, newDirection));

    // turn right
    newDirection = direction+1;
    if (newDirection == 4)
    {
        newDirection = 0;
    }
    neighbors.emplace_back(make_pair(location, newDirection));

    // wait
    neighbors.emplace_back(make_pair(location, direction));

    return neighbors;
}

/*
Validates a move for WHCA* search by checking the map boundaries and the reservation table.

timeStep is the timeStep to validate
destination is the map index to validate

Returns true if the move is valid and false otherwise.
*/
bool MAPFPlanner::WHCAValidateMove(int agentIndex, int timeStep, int start, int destination, std::unordered_map<string,bool> & reservationTable)
{
    int startX = start%env->cols;
    int startY = start/env->cols;
    int destX = destination%env->cols;
    int destY = destination/env->cols;
    int occupyingAgent = 0;
    if (abs(destX-startX) + abs(destY-startY) > 1
        || destX >= env->cols
        || destY >= env->rows
        || destX < 0
        || destY < 0
        || env->map[destination] == 1
        || WHCAIsTileOpen(timeStep, destination, reservationTable) == false
        || WHCAIsTileOpen(timeStep+1, destination, reservationTable) == false
        || WHCAIsTileOpen(timeStep+2, destination, reservationTable) == false
        || (timeStep == 1 && LocationIsCurrentlyOccupied(agentIndex, destination, occupyingAgent))
        )
    {
        return false;
    }
    return true;
}

/*
Checks the reservation table to see if the agent can occupy
position <destination> at time <timeStep>

Returns true if the position is open and false if the position is reserved
*/
bool MAPFPlanner::WHCAIsTileOpen(int timeStep, int destination, std::unordered_map<string,bool> & reservationTable)
{
    return reservationTable.find(GetTableKey(timeStep, destination)) == reservationTable.end();
}

/*
Uses A* to compute a path for an agent to follow to their goal.
Does not consider collisions with other agents.

Returns a path of <int,int> pairs where the first int is the map index and the second
int is the orientation.
*/
list<pair<int,int>> MAPFPlanner::single_agent_plan(int start, int start_direct, int end)
{
    list<pair<int,int>> path;
    priority_queue<AstarNode*, vector<AstarNode*>, cmp> open_list;
    unordered_set<int> close_list;
    unordered_map<int, AstarNode*> all_nodes;  // Used to delete A* nodes after search is complete

    // Initialize data structures to start search
    AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    open_list.push(s);
    all_nodes[start*4 + start_direct] = s;

    while (!open_list.empty())
    {
        // Select "best" node from top of list
        AstarNode* curr = open_list.top();
        open_list.pop();
        close_list.emplace(curr->location*4 + curr->direction);

        // If node is goal, construct path and break
        if (curr->location == end)
        {
            while(curr->parent!=NULL) 
            {
                path.emplace_front(make_pair(curr->location, curr->direction));
                curr = curr->parent;
            }
            break;
        }

        // Otherwise, get all neighbors of the expanded node
        list<pair<int,int>> neighbors = getNeighbors(curr->location, curr->direction);
        for (const pair<int,int>& neighbor: neighbors)
        {
            // If the current neighbor is in the closed list, skip it
            if (close_list.find(neighbor.first*4 + neighbor.second) != close_list.end())
            {
                continue;
            }

            // If it already exists in the list of existing nodes, we might need to update it's shortest path value
            if (all_nodes.find(neighbor.first*4 + neighbor.second) != all_nodes.end())
            {
                AstarNode* old = all_nodes[neighbor.first*4 + neighbor.second];
                if (curr->g + 1 < old->g)
                {
                    old->g = curr->g+1;
                    old->f = old->h+old->g;
                    old->parent = curr;
                }
            }
            // If it doesn't exist, we'll create a new node for it
            else
            {
                AstarNode* newNode = new AstarNode(neighbor.first, neighbor.second,
                    curr->g+1,getManhattanDistance(neighbor.first,end), curr);
                open_list.push(newNode);
                all_nodes[neighbor.first*4 + neighbor.second] = newNode;
            }
        }
    }

    // Clean up data structures and return path
    for (auto n: all_nodes)
    {
        delete n.second;
    }
    all_nodes.clear();
    return path;
}


int MAPFPlanner::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = loc1%env->cols;
    int loc1_y = loc1/env->cols;
    int loc2_x = loc2%env->cols;
    int loc2_y = loc2/env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

/*
Determines whether a move is valid. A valid move must be within the
map boundaries and must be to an accessible square on the map. It also
cannot move the agent more than one total square.

Returns true if the move is valid and false otherwise.
*/
bool MAPFPlanner::validateMove(int dest, int start)
{
    int loc_x = dest%env->cols;
    int loc_y = dest/env->cols;
    if (loc_x >= env->rows || loc_y >= env->cols || env->map[dest] == 1) {

        return false;
    }

    int loc2_x = start%env->cols;
    int loc2_y = start/env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1) {
        return false;
    }

    return true;
}

// Returns neighbor nodes for a given location and orientation on the map
list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction)
{
    list<pair<int,int>> neighbors;

    // forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));

    // turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(make_pair(location,new_direction));

    // turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(make_pair(location,new_direction));

    // wait
    neighbors.emplace_back(make_pair(location,direction));

    return neighbors;
}

// Creates randomized agent indexes for hierarchical planning
std::vector<int> MAPFPlanner::createRandomAgentIndexes(int numAgents)
{
    std::vector<int> agentIndexes;
    for (int i = 0; i < numAgents; i++)
    {
        agentIndexes.push_back(i);
    }
    std::random_shuffle(agentIndexes.begin(), agentIndexes.end());
    return agentIndexes;
}

/*
Returns a hash key to index into the reservation table
Uses the format <time step>^<destination map index>
*/
std::string MAPFPlanner::GetTableKey(int timeStep, int destination)
{
    int xPos = destination%env->cols;
    int yPos = destination/env->cols;
    std::string tableKey = std::to_string(timeStep) + "^" + std::to_string(xPos) + "^" + std::to_string(yPos);
    return tableKey;
}

/*
Prints an error message to the console
*/
void MAPFPlanner::DebugPrint(std::string message)
{
    std::cout << message + "\n";
}

/*
Check if a location is currently occupied to prevent edge conflicts.
Also prevents following conflicts.

Returns true if the location is occupied at by a different agent at timestep 0
and false otherwise.
*/
bool MAPFPlanner::LocationIsCurrentlyOccupied(int agentIndex, int location, int & occupyingAgent)
{
    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (i == agentIndex)
        {
            continue;
        }
        if (env->curr_states[i].location == location)
        {
            occupyingAgent = i;
            return true;
        }
    }
    return false;
}

/*
Attempts to break cycles by having robot move in a suboptimal direction if no path was found to goal
*/
void MAPFPlanner::AttemptForwardMoveIfAllowed(int agentIndex, std::unordered_map<string,bool> & reservationTable, list<pair<int,int>> & path)
{
    int location = env->curr_states[agentIndex].location;
    int direction = env->curr_states[agentIndex].orientation;
    int candidates[4] = { location + 1, location + env->cols, location - 1, location - env->cols};
    int forwardLocation = candidates[direction];
    if (WHCAValidateMove(agentIndex, 1, location, forwardLocation, reservationTable))
    {
        path.emplace_front(make_pair(forwardLocation, direction));
    }
}

/*
Checks if two actions are both rotations. Can be used to prevent rotation-based cycles.
*/
bool MAPFPlanner::ConsecutiveRotations(Action action1, Action action2)
{
    if ((action1 == Action::CR || action1 == Action::CCR) &&
        (action2 == Action::CR || action2 == Action::CCR))
    {
        return true;
    }
    return false;
}

/*
*****************************************************************************************
*******************************************************************************************
*******************************************************************************************


                                Increasing Cost Tree Search 


*******************************************************************************************
*******************************************************************************************
*/

/*
Represents a single node in the increasing cost tree
*/
struct MAPFPlanner::ICTNode
{
    int totalCost = 0;  // Total cost of paths in node
    vector<int> agentCosts;  // Maps agent index to individual agent path cost

    ICTNode(int _totalCost, vector<int> _agentCosts):
        totalCost(_totalCost), agentCosts(_agentCosts) {}
};

/*
Represents a single node in an multivalue decision diagram
*/
struct MAPFPlanner::MDDNode
{
    int location;  // The map index location of the node
    int orientation;  // The orientation of the node
    int depth;  // The depth of the node
    vector<MDDNode*> parents;  // Parents of the node (closer to top level)
    vector<MDDNode*> children;  // Children of the node (closer to bottom level)
    int goalLocation;   // This agent's goal location
    int completionTime; // The time this agent will reach this location

    MDDNode(int _location, int _orientation, int _depth, int _goal, int _compTime):
        location(_location), orientation(_orientation), depth(_depth), goalLocation(_goal), completionTime(_compTime) {}
};

// Comparison operator for MDD nodes
struct MAPFPlanner::CompareMDDNode
{
    bool operator()(MDDNode* a, MDDNode* b)
    {
        // Returns true if a should be removed AFTER b
        // We need to explore shallower depths first, so this should return true when a is deeper than b
        return a->depth >= b->depth;
    }
};

/*
Plans agent paths for the current time step using ICTS
Optimizes total path cost over all agents
Based on Sharon et. al's 2012 paper "The increasing cost tree search for optimal multi-agent pathﬁnding"

Uses simple triple pruning. Does not yet have support for enhanced triple pruning.
*/
void MAPFPlanner::ICTSPlan(int timeLimit, vector<Action> & actions)
{
    if (MEMORY_CBS_NODE == nullptr || MEMORY_CBS_NODE->remainingActions == 0)
    {

        // if (GLOBAL_TIMESTEP == 1)
        // {
        //     for (int taskNum = 0; taskNum < 10000; taskNum++)
        //     {
        //         int taskMapIndex = 9999999;
        //         while (taskMapIndex > env->map.size() || env->map[taskMapIndex] != 0)
        //         {
        //             taskMapIndex = rand() % env->map.size();
        //         }
        //         DebugPrint(std::to_string(taskMapIndex));
        //     }
        // }

        delete MEMORY_CBS_NODE;
        actions = std::vector<Action>(env->curr_states.size(), Action::W);  // Returns actions to main simulator
        vector<vector<ICTNode*>> icTree = GenerateEmptyICTree();  // Represents the increasing cost tree with one level per inner vector
        unordered_map<std::string, ICTNode*> allICTNodes;  // Used to delete ICT nodes after search is complete
        int treeLevel = 0;  // Used to maintain the level of the tree to search
        bool solved = false;  // Used to exit search

        ICTNode* rootNode = GenerateICTRoot();
        icTree[0].push_back(rootNode);
        allICTNodes[GetICTNodeIdentifier(rootNode->agentCosts)] = rootNode;

        // Searches tree until we break; we break when we find a solution
        for (treeLevel; treeLevel < INT_MAX && !solved; treeLevel++)
        {
            // Search all nodes in the given level until we find a solution
            for (ICTNode* currentNode : icTree[treeLevel])
            {
                solved = LowLevelSearchICTNode(currentNode, actions);
                if (solved) {
                    break;
                }
            }

            // If we didn't find a solution, generate the next level
            if (!solved)
            {
                GenerateICTNodeChildren(icTree, allICTNodes, treeLevel);
            }
        }

        // Free memory from ICTNodes
        for (auto ictNode: allICTNodes)
        {
            delete ictNode.second;
        }
        allICTNodes.clear();
    }
    else
    {
        actions = ExtractActions(MEMORY_CBS_NODE);
        RemoveFirstPathNode(MEMORY_CBS_NODE);
    }
}

/*
Generates a default IC tree, which is really just a vector of vectors of ICT node pointers
*/
vector<vector<MAPFPlanner::ICTNode*>> MAPFPlanner::GenerateEmptyICTree()
{
    vector<vector<ICTNode*>> icTree;
    vector<ICTNode*> icTreeLevel;
    icTree.push_back(icTreeLevel);
    return icTree;
}

/*
Generates the root node for the ICT by using A* for each agent to determine optimal individual path costs
*/
MAPFPlanner::ICTNode* MAPFPlanner::GenerateICTRoot()
{
    int totalCost = 0;
    vector<int> agentCosts;
    list<pair<int,int>> path;

    for (int i = 0; i < env->num_of_agents; i++)
    {
        path = single_agent_plan(env->curr_states[i].location,
                                    env->curr_states[i].orientation,
                                    env->goal_locations[i].front().first);
        totalCost += path.size();
        agentCosts.push_back(path.size());
    }

    return new ICTNode(totalCost, agentCosts);
}

/*
Generates a string identifier for an ICT node
Identifier has the format c1^c2^...^cn^ where ci is the cost for agent i
*/
std::string MAPFPlanner::GetICTNodeIdentifier(vector<int> agentCosts)
{
    std::string costString = "";
    for (int cost : agentCosts)
    {
        costString += std::to_string(cost) + "^";
    }
    return costString;
}

/*
Performs the low-level goal-checking search on an ICT node
*/
bool MAPFPlanner::LowLevelSearchICTNode(MAPFPlanner::ICTNode* currentNode, vector<Action>& actions)
{
    vector<MDDNode*> agentMDDs;  // Single-agent MDDi root nodes
    vector<MDDNode*> parentMDDs;  // Parent MDD vector, empty for entry to recursive DFS
    vector<unordered_map<std::string, MDDNode*>> nodeDeleteList;  // Used to clean up MDDi nodes
    int fillDepth = *std::max_element(currentNode->agentCosts.begin(), currentNode->agentCosts.end());
    int minDepth = *std::min_element(currentNode->agentCosts.begin(), currentNode->agentCosts.end());

    // Generate MDDi for each of k agents
    for (int i = 0; i < currentNode->agentCosts.size(); i++)
    {
        agentMDDs.push_back(
            BuildAgentMDD(
                currentNode->agentCosts[i],
                fillDepth,
                env->curr_states[i].location,
                env->curr_states[i].orientation,
                env->goal_locations[i].front().first,
                nodeDeleteList));

        // if (GLOBAL_TIMESTEP > 120)
        // {
        //     DebugPrint("MDD for agent " + std::to_string(i) + " with goal " + std::to_string(env->goal_locations[i].front().first) + " at time step " + std::to_string(GLOBAL_TIMESTEP));
        //     PrintMDD(agentMDDs[i]);
        //     DebugPrint("");
        // }

    }

    // Prune for each each triple of agents
    if (CanPruneICTNode(agentMDDs, fillDepth))
    {
        FreeMDDTree(nodeDeleteList);
        return false;
    }

    // Search MDDk for solution, then free the MDDi trees and return the result
    vector<vector<MDDNode*>> pathStack;
    bool result = DepthFirstSearchMDDk(0, fillDepth, agentMDDs, parentMDDs, pathStack, actions, minDepth);
    FreeMDDTree(nodeDeleteList);
    return result;
}

/*
Attempts to prune the current ICT node by checking for triple conflicts (conflicts) in
the 3-agent search space.

Returns true if the node can be pruned (because it contains a conflict) and false when
the node cannot be pruned (because no triple conflicts were found).
*/
bool MAPFPlanner::CanPruneICTNode(vector<MAPFPlanner::MDDNode*>& agentMDDs, int targetDepth)
{
    vector<MDDNode*> parentMDDs;
    vector<vector<MDDNode*>> pathStack;
    vector<Action> actions (agentMDDs.size(), Action::W);

    for (int i = 0; i < agentMDDs.size(); i++)
    {
        for (int j = i+1; j < agentMDDs.size(); j++)
        {
            // If agent A finishes before agent B and agent B attempts to cross agent A's goal, there is no solution
            vector<MDDNode*> pruningSearchMDDs = { agentMDDs[i], agentMDDs[j] };
            if (CanPruneBasedOnCompletionTimes(pruningSearchMDDs))
            {
                return true;
            }
        }
    }

    for (int i = 0; i < agentMDDs.size(); i++)
    {
        for (int j = i+1; j < agentMDDs.size(); j++)
        {
            for (int k = j+1; k < agentMDDs.size(); k++)
            {
                vector<MDDNode*> pruningSearchMDDs = { agentMDDs[i], agentMDDs[j], agentMDDs[k] };

                // If this search fails, we know we can prune this ICT node
                if (!DepthFirstSearchMDDk(0, targetDepth, pruningSearchMDDs, parentMDDs, pathStack, actions, -1))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

/*
Attempts to prune an ICT node based on completion times of each agent

Returns true if the node can be pruned and false otherwise
*/
bool MAPFPlanner::CanPruneBasedOnCompletionTimes(vector<MAPFPlanner::MDDNode*>& agentMDDs)
{
    unordered_map<int, int> goalAndCompletionTimes;  // Maps location to completion time
    vector<vector<int>> nodeLocationsAtTimestep;

    for (int i = 0; i < 150; i++)
    {
        nodeLocationsAtTimestep.push_back(vector<int>());
    }
    
    for (MDDNode* mdd: agentMDDs)
    {
        // Create goal and completion times of OTHER agents
        goalAndCompletionTimes.clear();
        for (MDDNode* mdd2: agentMDDs)
        {
            if (mdd->goalLocation != mdd2->goalLocation)
            {
                goalAndCompletionTimes[mdd2->goalLocation] = mdd2->completionTime;
            }
        }

        // Create empty vectors for each pre-goal timestep
        for (int i = 0; i < 150; i++)
        {
            nodeLocationsAtTimestep[i].clear();
        }

        // Store each pre-goal location in vectors
        priority_queue<MDDNode*, vector<MDDNode*>, CompareMDDNode> openList;  // Stores nodes that we're searching on
        openList.push(mdd);
        MDDNode* currentNode;
        while (!openList.empty())
        {
            currentNode=openList.top();
            openList.pop();

            if (currentNode->depth > currentNode->completionTime)
            {
                break;
            }

            if (std::find(nodeLocationsAtTimestep[currentNode->depth].begin(), 
                    nodeLocationsAtTimestep[currentNode->depth].end(), 
                    currentNode->location) == nodeLocationsAtTimestep[currentNode->depth].end())
                {
                    nodeLocationsAtTimestep[currentNode->depth].push_back(currentNode->location);
                }
            for (MDDNode* child: currentNode->children)
            {
                openList.push(child);
            }
        }

        // If another agent reached goal at this location at same or earlier timestep, we can't find solution because it will sit on it
        for (int timeStep = 0; timeStep < nodeLocationsAtTimestep.size(); timeStep++)
        {
            if (nodeLocationsAtTimestep[timeStep].size() == 1)
            {
                int singleLocation = nodeLocationsAtTimestep[timeStep].front();
                if (goalAndCompletionTimes.find(singleLocation) != goalAndCompletionTimes.end())
                {
                    if (goalAndCompletionTimes[singleLocation] <= timeStep)
                    {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

/*
Performs recursive depth-first search on the MDDk search space
Returns true when a solution is found and false otherwise
*/
bool MAPFPlanner::DepthFirstSearchMDDk(int depth, int targetDepth, vector<MAPFPlanner::MDDNode*>& agentMDDs, vector<MAPFPlanner::MDDNode*>& parentMDDs, vector<vector<MAPFPlanner::MDDNode*>> pathStack, vector<Action>& actions, int minDepth)
{
    // Return false if we couldn't unify these nodes
    if (!UnifyNodes(depth, agentMDDs, parentMDDs, false))
    {
        return false;
    }

    // We're not going to find anything there if there are zero neighbors for some reason
    bool containsZero = false;
    vector<int> neighborCount = GetNeighborCount(agentMDDs, containsZero);
    if (containsZero && depth != targetDepth)
    {
        DebugPrint("Zero neighbors found for a search at depth " + std::to_string(depth));
        return false;
    }

    pathStack.push_back(agentMDDs);
    // Return true and build path if we unified and they were goal nodes
    if (depth == targetDepth)
    {
        ConstructMDDPaths(pathStack, actions);
        if (minDepth > 0)
        {
            MEMORY_CBS_NODE = new CTNode(vector<Constraint>(),0);
            MEMORY_CBS_NODE->solution = ConvertPathStackToPaths(pathStack, minDepth);
            MEMORY_CBS_NODE->remainingActions = MEMORY_CBS_NODE->solution[0].size();
        }
        return true;
    }
    // Try to search deeper if we unified but they were not goal nodes
    else
    {
        vector<MDDNode*> newAgentMDDs(agentMDDs.size(), nullptr);
        vector<int> visitedNeighbors(agentMDDs.size(), 0);
        visitedNeighbors[0] = -1;  // Initialize first element to -1 so first call to FindUnvisitedNeighbors finds first neighbor

        while (FindUnvisitedNeighbor(agentMDDs, newAgentMDDs, visitedNeighbors, neighborCount))
        {
            if(DepthFirstSearchMDDk(depth+1, targetDepth, newAgentMDDs, agentMDDs, pathStack, actions, minDepth))
            {
                return true;
            }
        }
        return false;
    }
}

/*
Prints an agent MDD
*/
void MAPFPlanner::PrintMDD(MAPFPlanner::MDDNode* mdd)
{
    int currentDepth = -1;
    MDDNode* currentNode;
    unordered_map<std::string, MDDNode*> allNodes;  // Used to find existing nodes
    priority_queue<MDDNode*, vector<MDDNode*>, CompareMDDNode> openList;  // Stores nodes that we're searching on
    openList.push(mdd);
    allNodes[GetMDDiNodeIdentifier(mdd->location, mdd->orientation, 0)] = mdd;

    while (!openList.empty())
    {
        currentNode=openList.top();
        openList.pop();

        if (currentNode->depth > currentDepth)
        {
            currentDepth++;
            std::cout << "\n (Depth " + std::to_string(currentDepth) + ") ";
        }

        if (allNodes.find(GetMDDiNodeIdentifier(currentNode->location, currentNode->orientation, currentNode->depth)) == allNodes.end())
        {
            std::cout << currentNode->location << " ";
            allNodes[GetMDDiNodeIdentifier(currentNode->location, currentNode->orientation, currentNode->depth)] = currentNode;
        }

        for (MDDNode* child: currentNode->children)
        {
            openList.push(child);
        }
    }
}

/*
Converts the MDD pathstack to a standard path that fits in a CBS node
*/
vector<list<pair<int, int>>> MAPFPlanner::ConvertPathStackToPaths(vector<vector<MAPFPlanner::MDDNode*>> pathStack, int realDepth)
{
    vector<list<pair<int, int>>> path;

    for (int agent = 0; agent < pathStack[0].size(); agent++)
    {
        path.push_back(list<pair<int,int>>());
    }

    for (int depth = 1; depth < realDepth; depth++)
    {
        for (int agent = 0; agent < pathStack[0].size(); agent++)
        {
            pair<int,int> pathNode = make_pair(pathStack[depth][agent]->location, pathStack[depth][agent]->orientation);
            path[agent].push_back(pathNode);
        }
    }
    return path;
}

/*
Counts the number of neighbors for each MDD node.

Returns a vector of numbers representing the size of the neighbors vector for each MDDNode in-order.
*/
vector<int> MAPFPlanner::GetNeighborCount(vector<MAPFPlanner::MDDNode*>& agentMDDs, bool& containsZero)
{
    vector<int> neighborCounts;
    containsZero = false;
    for (MDDNode* agentMDD: agentMDDs)
    {
        neighborCounts.push_back(agentMDD->children.size());
        if (agentMDD->children.empty())
        {
            containsZero = true;
        }
    }
    return neighborCounts;
}

/*
Finds the next unvisited neighbor that is one level down from the current set of MDDs.
This basically works like carryover addition; add at lowest level, carry when necessary to reset based on number of neighbors for each node.
*/
bool MAPFPlanner::FindUnvisitedNeighbor(vector<MAPFPlanner::MDDNode*>& agentMDDs, vector<MAPFPlanner::MDDNode*>& newAgentMDDs, vector<int>& visitedNeighbors, vector<int>& neighborCount)
{
    bool incremented = false;
    for (int i = 0; i < neighborCount.size(); i++)
    {
        if (visitedNeighbors[i]+1 < neighborCount[i])
        {
            visitedNeighbors[i] += 1;
            incremented = true;
            break;
        }
        else
        {
            visitedNeighbors[i] = 0;
        }
    }

    if (incremented)
    {
        for (int i = 0; i < neighborCount.size(); i++)
        {
            newAgentMDDs[i] = agentMDDs[i]->children[visitedNeighbors[i]];
        }
        return true;
    }
    return false;
}

/*
Attempt to unify nodes from the given MDDis. Add to visited nodes after attempted unification.

Returns false if the nodes could not be unified / are conflicted.
Returns true if the nodes were successfully unified.
*/
bool MAPFPlanner::UnifyNodes(int depth, vector<MAPFPlanner::MDDNode*>& agentMDDs, vector<MAPFPlanner::MDDNode*>& parentMDDs, bool pruningMode)
{
    unordered_map<int, bool> occupiedPositions;  // Set to true when a position is occupied
    vector<int> unifiedPositions;  // Contains the positions in the unified root node
    bool conflicted = false;

    for (int i = 0; i < agentMDDs.size(); i++)
    {
        int location = agentMDDs[i]->location;
        // Check for vertex conflict, where two agents occupy same position at same time
        if (occupiedPositions.find(location) != occupiedPositions.end())
        {

            conflicted = true;

            // Results in vertex conflicts for some reason
            // // If collision occurred, permit it if it occurred after either agent has reached goal
            // for (int otherAgentIndex = 0; otherAgentIndex < agentMDDs.size(); otherAgentIndex++)
            // {
            //     if (otherAgentIndex == i)
            //     {
            //         continue;
            //     }
            //     MDDNode* agent1 = agentMDDs[i];
            //     MDDNode* agent2 = agentMDDs[otherAgentIndex];
            //     if (agent1->depth < agent1->completionTime &&
            //         agent2->depth < agent2->completionTime &&
            //         agent1->location == agent2->location)
            //     {
            //         conflicted = true;
            //     }
            // }

        }
        occupiedPositions[location] = true;
        unifiedPositions.push_back(location);
        for (int j = i+1; j < parentMDDs.size(); j++)
        {
            // Check for edge conflicts, where two agents swap places
            if (agentMDDs[i]->location == parentMDDs[j]->location && agentMDDs[j]->location == parentMDDs[i]->location)
            {
                conflicted = true;
            }
        }
    }
    return !conflicted;
}

/*
Generates ICT nodes at a given depth
Must have generated nodes at previous depth to use this function
*/
void MAPFPlanner::GenerateICTNodeChildren(vector<vector<ICTNode*>>& icTree, unordered_map<std::string,ICTNode*>& allICTNodes, int depth)
{
    vector<ICTNode*> newLevel;
    icTree.push_back(newLevel);

    // For each ICT node at the prior level
    for (ICTNode* node : icTree[depth])
    {
        // For each cost to adjust in this ICT node
        for (int i = 0; i < node->agentCosts.size(); i++)
        {
            vector<int> newCosts = node->agentCosts;
            newCosts[i] = node->agentCosts[i]+1;

            // If this node doesn't already exist, add it
            if (allICTNodes.find(GetICTNodeIdentifier(newCosts)) == allICTNodes.end())
            {
                ICTNode* newNode = new ICTNode(node->totalCost+1, newCosts);
                icTree[depth+1].push_back(newNode);
                allICTNodes[GetICTNodeIdentifier(newCosts)] = newNode;
            }
        }
    }
    DebugPrint("Created level " + std::to_string(depth+1) + " with node count " + std::to_string(icTree[depth+1].size()));
}

/*
Builds a single-agent MDD. Returns a pointer to the root node of the MDD, which is a DAG.
The MDD will be built to the "fill height", which should be passed in as the height of the tallest MDD for this ICT node.
In contrast, the "height" parameter should be set to the height to build this agent's MDD to.
*/
MAPFPlanner::MDDNode* MAPFPlanner::BuildAgentMDD(int height, int fillHeight, int start, int startOrientation, int goal, vector<unordered_map<std::string,MAPFPlanner::MDDNode*>>& nodeDeleteList)
{
    unordered_map<std::string, MDDNode*> allNodes;  // Used to find existing nodes
    priority_queue<MDDNode*, vector<MDDNode*>, CompareMDDNode> openList;  // Stores nodes that we're searching on

    MDDNode* rootNode = new MDDNode(start, startOrientation, 0, goal, height);
    openList.push(rootNode);
    allNodes[GetMDDiNodeIdentifier(start, startOrientation, 0)] = rootNode;

    // Run BFS until reaching search depth
    while (openList.top()->depth < height)
    {
        MDDNode* currentNode = openList.top();
        openList.pop();
        int neighborDepth = currentNode->depth+1;

        list<pair<int,int>> neighbors = getNeighbors(currentNode->location, currentNode->orientation);
        for (const pair<int,int>& neighbor: neighbors)
        {
            // At max depth we only build the sink node
            if (neighborDepth < height || neighbor.first == goal)
            {
                // If it already exists in the list of existing nodes, we need to add a link
                if (allNodes.find(GetMDDiNodeIdentifier(neighbor.first, neighbor.second, neighborDepth)) != allNodes.end())
                {
                    MDDNode* foundNode = allNodes[GetMDDiNodeIdentifier(neighbor.first, neighbor.second, neighborDepth)];
                    foundNode->parents.push_back(currentNode);
                    currentNode->children.push_back(foundNode);
                    // openList.push(foundNode);  I don't think I need to push it onto the open list multiple times at the same search level
                }
                // If it doesn't exist, we'll create a new node for it
                else
                {
                    MDDNode* newNode = new MDDNode(neighbor.first, neighbor.second, neighborDepth, goal, height);
                    newNode->parents.push_back(currentNode);
                    currentNode->children.push_back(newNode);
                    openList.push(newNode);
                    allNodes[GetMDDiNodeIdentifier(neighbor.first, neighbor.second, neighborDepth)] = newNode;
                }
            }
        }
    }

    // Add goal nodes until height equals fill height
    MDDNode* goalNode = openList.top();
    PruneNodesThatDontLeadToGoal(goalNode, allNodes);
    while (goalNode->depth < fillHeight)
    {
        MDDNode* newNode = new MDDNode(goalNode->location, goalNode->orientation, goalNode->depth+1, goal, height);
        newNode->parents.push_back(goalNode);
        goalNode->children.push_back(newNode);
        allNodes[GetMDDiNodeIdentifier(goalNode->location, goalNode->orientation, goalNode->depth+1)] = newNode;
        goalNode = newNode;
    }

    nodeDeleteList.push_back(allNodes);
    // DebugPrint("Created new agent MDD with " + std::to_string(allNodes.size()) + " nodes");
    return rootNode;
}

/*
Deletes all data associated with nodes that are not used in the path. This includes:
- NOT DONE Freeing the heap data associated with them
- NOT DONE Removing them from the allNodes node tracker
- Removing them from the parent and child vectors of nodes that are used in path
*/
void MAPFPlanner::PruneNodesThatDontLeadToGoal(MAPFPlanner::MDDNode* goalNode, unordered_map<std::string, MAPFPlanner::MDDNode*>& allNodes)
{
    unordered_map<std::string, MDDNode*> nodesUsedInPath;
    list<MDDNode*> goalPathNodes;
    MDDNode* currentNode;

    // Mark each node along path as used
    goalPathNodes.push_back(goalNode);
    while (!goalPathNodes.empty())
    {
        currentNode = goalPathNodes.front();
        goalPathNodes.pop_front();
        string nodeIdentifier = GetMDDiNodeIdentifier(currentNode->location, currentNode->orientation, currentNode->depth);
        nodesUsedInPath[nodeIdentifier] = currentNode;
        for (MDDNode* parentNode: currentNode->parents)
        {
            goalPathNodes.push_back(parentNode);
        }
    }

    // Iterate over every node generated. If unused, delete it. If used, clean up it's parent and child vectors.
    for (pair<std::string, MDDNode*> nodePair: allNodes)
    {
        currentNode = nodePair.second;
        string nodeIdentifier = GetMDDiNodeIdentifier(currentNode->location, currentNode->orientation, currentNode->depth);

        // // If not used, delete it
        // if (nodesUsedInPath.find(nodeIdentifier) == nodesUsedInPath.end())
        // {
        //     delete allNodes[nodeIdentifier];
        //     allNodes.erase(nodeIdentifier);
        // }
        // // If used, clean up parent and child vectors
        // else
        // {

        // Clean up parent and child pointers in used nodes
        if (nodesUsedInPath.find(nodeIdentifier) != nodesUsedInPath.end())
        {
            for (int i = 0; i < currentNode->parents.size(); i++)
            {
                MDDNode* parentNode = currentNode->parents[i];
                string parentIdentifier = GetMDDiNodeIdentifier(parentNode->location, parentNode->orientation, parentNode->depth);
                if (nodesUsedInPath.find(parentIdentifier) == nodesUsedInPath.end())
                {
                    currentNode->parents.erase(currentNode->parents.begin()+i);
                }
            }
            for (int i = 0; i < currentNode->children.size(); i++)
            {
                MDDNode* childNode = currentNode->children[i];
                string childIdentifier = GetMDDiNodeIdentifier(childNode->location, childNode->orientation, childNode->depth);
                if (nodesUsedInPath.find(childIdentifier) == nodesUsedInPath.end())
                {
                    currentNode->children.erase(currentNode->children.begin()+i);
                    i--;
                }
            }
        }
    }
}

/*
Gets a delimited string identifier for a single-agent MDD node
Has the format location^orientation^depth
*/
std::string MAPFPlanner::GetMDDiNodeIdentifier(int location, int orientation, int depth)
{
    return std::to_string(location) + "^" + std::to_string(orientation) + "^" + std::to_string(depth);
}

/*
Gets a delimited string identifier for a multi-agent MDD node
Has the format depth^location1^location2^...^locationk
*/
std::string MAPFPlanner::GetMDDkNodeIdentifier(int depth, vector<int>& positions)
{
    std::string identifier = "";
    identifier += std::to_string(depth);
    for (int position: positions)
    {
        identifier += std::to_string(position);
    }
    return identifier;
}

/*
Cleans up MDDi trees by freeing them from memory
*/
void MAPFPlanner::FreeMDDTree(vector<unordered_map<std::string, MAPFPlanner::MDDNode*>>& nodeDeleteList)
{
    for (unordered_map list: nodeDeleteList)
    {
        for (auto node: list)
        {
            delete node.second;
        }
        list.clear();
    }
    nodeDeleteList.clear();
}

/*
Construct paths using the path stack to return actions for the agents
*/
void MAPFPlanner::ConstructMDDPaths(vector<vector<MAPFPlanner::MDDNode*>> pathStack, vector<Action>& actions)
{
    for (int i = 0; i < pathStack[0].size(); i++)
    {
        if (pathStack.size() > 1)
        {
            if (pathStack[0][i]->location != pathStack[1][i]->location)
            {
                actions[i] = Action::FW;  // Move forward
            } 
            else if (pathStack[0][i]->orientation != pathStack[1][i]->orientation)
            {
                int orientationDiff = pathStack[1][i]->orientation - pathStack[0][i]->orientation;
                if (orientationDiff == 1 || orientationDiff == -3)
                {
                    actions[i] = Action::CR; // Clockwise rotation
                } 
                else if (orientationDiff == -1 || orientationDiff == 3)
                {
                    actions[i] = Action::CCR; // Counter clockwise rotation
                } 
                // Path cannot result in orientation diff of 2 since it would represent that as two 90-degree rotations
            }
        }
    }
}

/*
*****************************************************************************************
*******************************************************************************************
*******************************************************************************************


                                Constraint-Based Search


*******************************************************************************************
*******************************************************************************************
*/


// Comparison operator for CT nodes
struct MAPFPlanner::CompareCTNode
{
    bool operator()(CTNode* a, CTNode* b)
    {
        // Returns true if a should be removed AFTER b
        // We need to explore lower costs first, so this should return true when a is less expensive than b
        return a->cost >= b->cost;
    }
};


/*
Represents a single constraint in the high-level constraint tree
*/
struct MAPFPlanner::Constraint
{
    int agentIndex;  // The agent restricted by this constraint
    int position;  // The position that the agent cannot occupy
    int time;  // The time at which the agent cannot occupy the position

    Constraint() {}

    Constraint(int _agentIndex, int _position, int _time):
        agentIndex(_agentIndex), position(_position), time(_time) {}
};

/*
Plans agent paths for the current time step using conflict-based search
Optimizes total path cost over all agents
Based on Sharon et. al's 2015 paper "Conflict-based search for optimal multi-agent path finding"
*/
void MAPFPlanner::CBSPlan(int time_limit, vector<Action>& actions)
{
    if (MEMORY_CBS_NODE == nullptr || MEMORY_CBS_NODE->remainingActions == 0)
    {
        delete MEMORY_CBS_NODE;
        priority_queue<CTNode*, vector<CTNode*>, CompareCTNode> openList;  // PQ used for best-first search in constraint tree
        unordered_map<std::string, CTNode*> allNodes;  // Hash map used to free memory after search
        vector<Constraint> constraints;  // Empty constraints list for root node

        CTNode* rootNode = new CTNode(constraints, 0);
        LowLevelSolveCTNode(rootNode);
        allNodes[GetCTNodeIdentifier(rootNode)] = rootNode;
        openList.push(rootNode);

        bool solved = false;
        CTNode* currentNode;
        while(!solved)
        {
            currentNode = openList.top();
            openList.pop();

            Constraint c1, c2;
            solved = IsValidSolution(currentNode, c1, c2);

            if (!solved)
            {
                CTNode* firstChild = CreateCTNodeChild(currentNode, c1);
                openList.push(firstChild);
                allNodes[GetCTNodeIdentifier(firstChild)] = firstChild;

                CTNode* secondChild = CreateCTNodeChild(currentNode, c2);
                openList.push(secondChild);
                allNodes[GetCTNodeIdentifier(secondChild)] = secondChild;
            }
        }

        // Log actions for each agent using current node's path
        actions = ExtractActions(currentNode);
        MEMORY_CBS_NODE = new CTNode(currentNode->constraints, currentNode->depth);
        MEMORY_CBS_NODE->solution = currentNode->solution;
        MEMORY_CBS_NODE->cost = currentNode->cost;
        RemoveFirstPathNode(MEMORY_CBS_NODE);

        // Clean up data structures and return path
        for (pair<std::string, CTNode*> nodePair: allNodes)
        {
            delete nodePair.second;
        }
        allNodes.clear();
    }
    else
    {
        actions = ExtractActions(MEMORY_CBS_NODE);
        RemoveFirstPathNode(MEMORY_CBS_NODE);
    }
}

/*
Removes the first node in the path for each agent and decrements the remaining path counter
*/
void MAPFPlanner::RemoveFirstPathNode(MAPFPlanner::CTNode* memoryNode)
{
    int minActions = INT_MAX;
    for (int i = 0; i < memoryNode->solution.size(); i++)
    {
        if (!memoryNode->solution[i].empty())
        {
            memoryNode->solution[i].pop_front();
        }
        if (memoryNode->solution[i].size() < minActions)
        {
            minActions = memoryNode->solution[i].size();
        }
    }
    memoryNode->remainingActions = minActions;
}

/*
Finds a solution to a CT node that is consisten with that node's constraints.
The solution is not necessarily valid / non-conflicting.
*/
void MAPFPlanner::LowLevelSolveCTNode(MAPFPlanner::CTNode* node)
{
    std::unordered_map<string, bool> reservationTable;
    int costSum = 0;
    for (int i = 0; i < env->num_of_agents; i++)
    {
        ResetReservationTable(i, reservationTable, node->constraints);
        list<pair<int,int>> path = WHCAPath(i,
            env->curr_states[i].location,
            env->curr_states[i].orientation,
            env->goal_locations[i].front().first,
            reservationTable);
        if (path.size() == 0)
        {
            path.emplace_front(make_pair(env->curr_states[i].location, env->curr_states[i].orientation));
        }
        node->solution.push_back(path);
        costSum += path.size();
    }
    node->cost = costSum;
}

/*
Resets the reservation table for each agent in a CBS planning step
*/
void MAPFPlanner::ResetReservationTable(int agentIndex, std::unordered_map<string,bool>& reservationTable, vector<Constraint>& agentConstraints)
{
    reservationTable.clear();
    for (Constraint c: agentConstraints)
    {
        if (c.agentIndex == agentIndex)
        {
            reservationTable[GetTableKey(c.time, c.position)] = true;
            reservationTable[GetTableKey(c.time+1, c.position)] = true;
            reservationTable[GetTableKey(c.time+2, c.position)] = true;
        }
    }
}

/*
Gets a string identifier for a CT node.

Returns a string with the format "depth^agentIndex1^position1^time^agentIndex2..."
*/
std::string MAPFPlanner::GetCTNodeIdentifier(MAPFPlanner::CTNode* node)
{
    std::string id = std::to_string(node->depth);
    for (Constraint c: node->constraints)
    {
        id += std::to_string(c.agentIndex) + "^" + std::to_string(c.position) + "^" +std::to_string(c.time);
    }
    return id;
}

/*
Checks a set of paths for an edge conflict

Simplifies the notion of edge conflict by also preventing following conflicts so that
I don't need to check both agents involved in the conflict.

Returns true if this path contains an edge conflict and false otherwise.
*/
bool MAPFPlanner::ContainsEdgeConflict(vector<list<pair<int,int>>> solution, int& agent1, int& agent2, int& bannedPosition1, int& bannedPosition2, int& conflictTime)
{
    unordered_map<int,int> previousPositions;  // Maps position to agent index
    vector<vector<pair<int,int>>> solutionVec;
    int maxSize = 0;

    // Vectorize solutions for easier searching
    for (int i = 0; i < solution.size(); i++)
    {
        vector<pair<int,int>> newVec (solution[i].begin(), solution[i].end());
        solutionVec.push_back(newVec);
        if (maxSize < newVec.size())
        {
            maxSize = newVec.size();
        }
        previousPositions[newVec[0].first] = i;
    }

    // Check 
    for (int timeStep = 1; timeStep < maxSize; timeStep++)
    {
        for (int agentAIndex = 0; agentAIndex < env->num_of_agents; agentAIndex++)
        {
            if (timeStep < solutionVec[agentAIndex].size())
            {
                // Check whether agent A occupies agent B's last position
                int agentAPosition = solutionVec[agentAIndex][timeStep].first;
                if (previousPositions.find(agentAPosition) != previousPositions.end() && previousPositions[agentAPosition] != agentAIndex)
                {
                    // Check whether agent B occupies agent A's last position
                    int agentBIndex = previousPositions[agentAPosition]; 
                    int agentBPosition = solutionVec[agentBIndex][timeStep].first;
                    if (previousPositions.find(agentBPosition) != previousPositions.end() && previousPositions[agentBPosition] != agentBIndex)
                    {
                        agent1 = agentAIndex;
                        agent2 = agentBIndex;
                        bannedPosition1 = agentAPosition;
                        bannedPosition2 = agentBPosition;
                        conflictTime = timeStep;
                        return true;
                    }
                }
            }
        }

        // Update previous positions map
        previousPositions.clear();
        for (int agentIndex = 0; agentIndex < env->num_of_agents; agentIndex++)
        {
            if (timeStep < solutionVec[agentIndex].size())
            {
                previousPositions[solutionVec[agentIndex][timeStep].first] = agentIndex;
            }
        }
    }

    return false;
}

/*
Checks whether a solution is valid (i.e. has no edge or vertex conflicts between agents)

Returns true when the solution is valid and false otherwise. If this function returns false, c1 and c2 are set
with constraint data to prevent the collision.
*/
bool MAPFPlanner::IsValidSolution(MAPFPlanner::CTNode* node, MAPFPlanner::Constraint& c1, MAPFPlanner::Constraint& c2)
{
    std::unordered_map<string, bool> reservationTable;
    std::unordered_map<string, int> reservingAgentIndexes;

    int agent1, agent2, bannedPosition1, bannedPosition2, conflictTime;
    if (ContainsEdgeConflict(node->solution, agent1, agent2, bannedPosition1, bannedPosition2, conflictTime))
    {
        // Build new constraint for this agent
        c1.agentIndex = agent1;
        // c1.time = conflictTime+1;
        c1.time = conflictTime;
        c1.position = bannedPosition1;

        // Build new constraint for other conflicted agent
        c2.agentIndex = agent2;
        // c2.time = conflictTime+1;
        c2.time = conflictTime;
        c2.position = bannedPosition2;

        // This is not a valid solution, return false
        return false;
    }

    for (int i = 0; i < node->solution.size(); i++)
    {
        int timeStep = 0;
        // int timeStep = 1;  // First node in path is not the current node that the robot is on
        for (list<pair<int,int>>::iterator j = node->solution[i].begin(); j != node->solution[i].end(); j++)
        {
            // WHCAIsTileOpen catches vertex conflicts, IsEdgeConflict catches edge conflicts
            if (!WHCAIsTileOpen(timeStep, j->first, reservationTable))
            {
                // Build new constraint for this agent
                c1.agentIndex = i;
                c1.time = timeStep;
                c1.position = j->first;

                // Build new constraint for other conflicted agent
                c2.agentIndex = reservingAgentIndexes[GetTableKey(timeStep, j->first)];
                c2.time = timeStep;
                c2.position = j->first;

                // This is not a valid solution, return false
                return false;
            }
            else
            {
                reservationTable[GetTableKey(timeStep, j->first)] = true;
                reservingAgentIndexes[GetTableKey(timeStep, j->first)] = i;
                timeStep++;
            }
        }
    }
    return true;
}

/*
Creates, solves, and returns a new CT node. The solution will be consistent with its constraints, but
is not guaranteed to be valid (i.e. non-conflicting).

Returns a pointer to the new node.
*/
MAPFPlanner::CTNode* MAPFPlanner::CreateCTNodeChild(MAPFPlanner::CTNode* node, MAPFPlanner::Constraint& constraint)
{
    CTNode* newNode = new CTNode(node->constraints, node->depth+1);
    newNode->constraints.push_back(constraint);
    LowLevelSolveCTNode(newNode);
    return newNode;
}

/*
Converts the path found by the search into actions for each agent.
*/
vector<Action> MAPFPlanner::ExtractActions(MAPFPlanner::CTNode* node)
{
    vector<Action> actions = vector<Action>(env->curr_states.size(), Action::W);

    // We can't move unless we found a path
    // Once path is computed, action is move forward if next location on path is different from current location
    // Otherwise, the correct action is to compute a direction to turn in
    for (int i = 0; i < node->solution.size(); i++)
    {
        list<pair<int, int>> path = node->solution[i];
        if ((path.front().first != env->curr_states[i].location) && !path.empty())
        {
            actions[i] = Action::FW;  // Move forward
        } 
        else if (path.front().second != env->curr_states[i].orientation)
        {
            int orientationDiff = path.front().second - env->curr_states[i].orientation;
            if (orientationDiff == 1 || orientationDiff == -3)
            {
                actions[i] = Action::CR; // Clockwise rotation
            } 
            else if (orientationDiff == -1 || orientationDiff == 3)
            {
                actions[i] = Action::CCR; // Counter clockwise rotation
            } 
            // Path cannot result in orientation diff of 2 since it would represent that as two 90-degree rotations
        }
    }

    return actions;
}