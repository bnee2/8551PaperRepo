#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"


class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env;};

    // Preprocessing time allotted to the solution is limited to 30 minutes
    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    // Start kit dummy implementation
    std::list<pair<int,int>>single_agent_plan(int start, int start_direct, int end);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc, int loc2);

    // WHCA* functions
    void WHCAPlan(int time_limit, vector<Action> & actions);
    vector<int> createRandomAgentIndexes(int numAgents);
    std::list<pair<int,int>> WHCAPath(int agentIndex, int start, int start_direct, int end, std::unordered_map<string,bool> & reservationTable, int searchWindow);
    std::list<pair<int,int>> WHCAGetNeighbors(int agentIndex, int location, int direction, int timeStep, std::unordered_map<string,bool> & reservationTable);
    bool WHCAValidateMove(int agentIndex, int timeStep, int start, int destination, std::unordered_map<string,bool> & reservationTable);
    bool WHCAIsTileOpen(int timeStep, int destination, std::unordered_map<string,bool> & reservationTable);
    bool LocationIsCurrentlyOccupied(int agentIndex, int location, int & occupyingAgent);
    int TrueDistanceToGoal(int start, int end);
    void AttemptForwardMoveIfAllowed(int agentIndex, std::unordered_map<string,bool> & reservationTable, list<pair<int,int>> & path);
    std::string GetTableKey(int timeStep, int destination);
    bool ConsecutiveRotations(Action action1, Action action2);
    void DebugPrint(std::string message);

    // ICTS Functions
    struct ICTNode;
    struct MDDNode;
    struct CompareMDDNode;
    void ICTSPlan(int timeLimit, vector<Action>& actions);
    vector<vector<ICTNode*>> GenerateEmptyICTree();
    ICTNode* GenerateICTRoot();
    std::string GetICTNodeIdentifier(vector<int> agentCosts);
    bool LowLevelSearchICTNode(ICTNode* currentNode, vector<Action>& actions);
    void GenerateICTNodeChildren(vector<vector<ICTNode*>>& icTree, unordered_map<std::string,ICTNode*>& allICTNodes, int depth);
    MDDNode* BuildAgentMDD(int height, int fillHeight, int start, int startOrientation, int goal, vector<unordered_map<std::string,MDDNode*>>& nodeDeleteList);
    void FreeMDDTree(vector<unordered_map<std::string, MDDNode*>>& nodeDeleteList);
    std::string GetMDDiNodeIdentifier(int location, int orientation, int depth);
    std::string GetMDDkNodeIdentifier(int depth, vector<int>& positions);
    bool SearchMDDk(vector<MDDNode*>& agentMDDs, vector<Action>& actions);
    bool CanPruneICTNode(vector<MAPFPlanner::MDDNode*>& agentMDDs, int targetDepth);
    bool DepthFirstSearchMDDk(int depth, int targetDepth, vector<MDDNode*>& agentMDDs, vector<MDDNode*>& parentMDDs, vector<vector<MDDNode*>> pathStack, vector<Action>& actions, int minDepth);
    bool UnifyNodes(int depth, vector<MDDNode*>& agentMDDs, vector<MDDNode*>& parentMDDs, bool pruningMode);
    vector<int> GetNeighborCount(vector<MDDNode*>& agentMDDs, bool& containsZero);
    bool FindUnvisitedNeighbor(vector<MDDNode*>& agentMDDs, vector<MDDNode*>& newAgentMDDs, vector<int>& visitedNeighbors, vector<int>& neighborCount);
    void ConstructMDDPaths(vector<vector<MDDNode*>> pathStack, vector<Action>& actions);
    void PruneNodesThatDontLeadToGoal(MDDNode* goalNode, unordered_map<std::string, MDDNode*>& allNodes);
    vector<list<pair<int, int>>> ConvertPathStackToPaths(vector<vector<MDDNode*>> pathStack, int realDepth);
    void PrintMDD(MDDNode* mdd);
    bool CanPruneBasedOnCompletionTimes(vector<MDDNode*>& agentMDDs);

    // CBS Functions
    struct CTNode;
    struct CompareCTNode;
    struct Constraint;
    void CBSPlan(int time_limit, vector<Action>& actions);
    void LowLevelSolveCTNode(CTNode* node);
    std::string GetCTNodeIdentifier(CTNode* node);
    bool IsValidSolution(CTNode* node, Constraint& c1, Constraint& c2);
    CTNode* CreateCTNodeChild(CTNode* node, Constraint& constraint);
    vector<Action> ExtractActions(CTNode* node);
    void ResetReservationTable(int agentIndex, std::unordered_map<string,bool>& reservationTable, vector<Constraint>& agentConstraints);
    void RemoveFirstPathNode(CTNode* memoryNode);
    list<pair<int,int>> CBSPath(int agentIndex, int start, int startDirection, int end, std::unordered_map<string,bool> & reservationTable);
    bool ContainsEdgeConflict(vector<list<pair<int,int>>> solution, int& agent1, int& agent2, int& bannedPosition1, int& bannedPosition2, int& conflictTime);
};
