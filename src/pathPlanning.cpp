#include "utility.h"

class PathPlanning : public ParamServer
{
public:

    std::mutex mtx;
    ros::Timer pathUpdateTimer;

    ros::Subscriber subGlobalPath;
    ros::Subscriber subObstacleMap;

    ros::Publisher pubExecutePath;
    ros::Publisher pubSearchedPath;
    ros::Publisher pubPRMGraph;
    ros::Publisher pubSingleSourcePaths;

    ros::Publisher pub_delete_openplanner;

    nav_msgs::OccupancyGrid occupancyMap2D;

    PointType robotPoint;
    PointType fixedPoint;

    PointType lastRobotPoint;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    vector<state_t*> nodeList;
    vector<vector<state_t*> > adjacencyMatrix;

    int adjacency_width_grid;
    int adjacency_length_grid;

    nav_msgs::Path globalPathMessage;
    nav_msgs::Path executePath;
    nav_msgs::Path globalPath;
    nav_msgs::Path centerPath;
    nav_msgs::Path remainingPath;
    nav_msgs::Path searchedPath;
    vector<nav_msgs::Path> alternativePaths;

    RolloutGeneratorNS::RolloutGenerator openPlannerRollOut;

    PathPlanning()
    {
        subGlobalPath = nh.subscribe<nav_msgs::Path>("planning/server/path_blueprint_smooth", 5, &PathPlanning::pathHandler, this);
        subObstacleMap = nh.subscribe<nav_msgs::OccupancyGrid>("planning/obstacle/map_inflated", 5, &PathPlanning::mapHandler, this);

        pubPRMGraph = nh.advertise<visualization_msgs::MarkerArray>("planning/planning/prm_graph", 5);
        pubSingleSourcePaths = nh.advertise<visualization_msgs::MarkerArray>("planning/planning/prm_single_source_paths", 5);

        pub_delete_openplanner = nh.advertise<visualization_msgs::MarkerArray>("planning/planning/open_planner", 5);

        pubExecutePath = nh.advertise<nav_msgs::Path>("planning/planning/execute_path", 1);
        pubSearchedPath = nh.advertise<nav_msgs::Path>("planning/planning/searched_path", 1);

        adjacency_width_grid = -1;
        adjacency_length_grid = -1;

        pathUpdateTimer = nh.createTimer(ros::Duration(1.0/10.0), &PathPlanning::updatePath, this);
    }

    void pathHandler(const nav_msgs::Path::ConstPtr& pathMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (pathMsg->poses.size() <= 1)
        {
            ROS_WARN("Empty global path received.");
            return;
        }

        globalPathMessage = *pathMsg;

        // treate the original centerPath from open_planner as searched path
        openPlannerRollOut.run(transform, globalPathMessage, globalPath, searchedPath, remainingPath, alternativePaths);
        executePath = combinePaths(searchedPath, remainingPath);
        pubSearchedPath.publish(executePath);

        // subGlobalPath.shutdown();
        ROS_INFO("\033[1;32m Global Path recieved. \033[0m");
    }

    void mapHandler(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        occupancyMap2D = *mapMsg;
    }

    nav_msgs::Path combinePaths(nav_msgs::Path pathFront, nav_msgs::Path pathBack)
    {
        nav_msgs::Path pathOut = pathFront;

        if (pathBack.poses.size() > 0)
            pathOut.poses.insert(pathOut.poses.end(), pathBack.poses.begin(), pathBack.poses.end());

        return processPath(pathOut);
    }

    float distance(state_t* state_from, state_t* state_to){
        return sqrt((state_to->x-state_from->x)*(state_to->x-state_from->x) + 
                    (state_to->y-state_from->y)*(state_to->y-state_from->y) + 
                    (state_to->z-state_from->z)*(state_to->z-state_from->z));
    }

    bool isIncollision(float x, float y)
    {
        int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
        int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);
        int index = index_x + index_y * occupancyMap2D.info.width;

        if (index_x < 0 || index_x >= occupancyMap2D.info.width ||
            index_y < 0 || index_y >= occupancyMap2D.info.height)
            return false;

        if (occupancyMap2D.data[index] == 100)
            return true;
        else
            return false;
    }

    float getCloseCollisionCost(float x, float y)
    {
        int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
        int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);
        int index = index_x + index_y * occupancyMap2D.info.width;

        if (index_x < 0 || index_x >= occupancyMap2D.info.width ||
            index_y < 0 || index_y >= occupancyMap2D.info.height)
            return false;

        if (occupancyMap2D.data[index] > 0)
            return float(occupancyMap2D.data[index]);
        else
            return 0;
    }

    bool isCloseCollision(float x, float y)
    {
        int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
        int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);
        int index = index_x + index_y * occupancyMap2D.info.width;

        if (index_x < 0 || index_x >= occupancyMap2D.info.width ||
            index_y < 0 || index_y >= occupancyMap2D.info.height)
            return false;

        if (occupancyMap2D.data[index] > 50)
            return true;
        else
            return false;
    }

    void buildAdjacencyMatrix()
    {
        openPlannerRollOut.run(transform, globalPathMessage, globalPath, centerPath, remainingPath, alternativePaths);

        // clear memory
        for (int i = 0; i < nodeList.size(); ++i)
        {
            state_t *stateCur = nodeList[i];
            delete stateCur;
        }
        nodeList.clear();

        // allocate vector size
        adjacency_width_grid = alternativePaths.size();
        adjacency_length_grid = alternativePaths[0].poses.size();

        adjacencyMatrix.resize(adjacency_width_grid);
        for (int i = 0; i < adjacency_width_grid; ++i)
            adjacencyMatrix[i].resize(adjacency_length_grid);

        // create new states for adjacency matrix
        for (int i = 0; i < alternativePaths.size(); ++i)
        {
            for (int j = 0; j < alternativePaths[i].poses.size(); ++j)
            {
                state_t *newState = new state_t;
                newState->x = alternativePaths[i].poses[j].pose.position.x;
                newState->y = alternativePaths[i].poses[j].pose.position.y;
                newState->z = 0;
                newState->theta = tf::getYaw(alternativePaths[i].poses[j].pose.orientation);

                newState->idx = i;
                newState->idy = j;
                newState->validFlag = true;
                newState->stateId = nodeList.size();

                nodeList.push_back(newState);
                adjacencyMatrix[i][j] = newState;
            }
        }
    }

    void connectAdjacencyMatrix()
    {
        // check node collision first
        for (int i = 0; i < nodeList.size(); ++i)
        {
            if (isIncollision(nodeList[i]->x, nodeList[i]->y))
            {
                nodeList[i]->validFlag = false;
                continue;
            }
        }

        int connection_length = 1;
        // connect adjacency matrix
        for (int i = 0; i < adjacency_width_grid; ++i)
        {
            for (int j = 0; j < adjacency_length_grid; ++j)
            {
                if (adjacencyMatrix[i][j]->validFlag == false)
                    continue;

                state_t* state_from = adjacencyMatrix[i][j];

                for (int m = -connection_length; m <= connection_length; ++m)
                {
                    for (int n = -connection_length; n <= connection_length; ++n)
                    {
                        if (m == 0 && n == 0) // do not add itself
                            continue;

                        int id_x = i + m;
                        int id_y = j + n;
                        if (id_x < 0 || id_x >= adjacency_width_grid || id_y < 0 || id_y >= adjacency_length_grid)
                            continue;

                        if (adjacencyMatrix[id_x][id_y]->validFlag == false)
                            continue;

                        state_t* state_to = adjacencyMatrix[id_x][id_y];

                        float edgeCosts[NUM_COSTS];
                        if(edgePropagation(state_from, state_to, edgeCosts) == true)
                        {
                            neighbor_t thisNeighbor;
                            for (int q = 0; q < NUM_COSTS; ++q)
                                thisNeighbor.edgeCosts[q] = edgeCosts[q];
                            thisNeighbor.neighbor = state_to;
                            state_from->neighborList.push_back(thisNeighbor);
                        }
                    }
                }
            }
        } 
    }

    bool edgePropagation(state_t *state_from, state_t *state_to, float edgeCosts[NUM_COSTS]){
        // 0. initialize edgeCosts
        for (int i = 0; i < NUM_COSTS; ++i)
            edgeCosts[i] = 0;
        // 1. segment the edge for collision checking
        int steps = round(distance(state_from, state_to) / _mapResolution);
        float stepX = (state_to->x - state_from->x) / (float)steps;
        float stepY = (state_to->y - state_from->y) / (float)steps;
        float stepZ = (state_to->z - state_from->z) / (float)steps;
        // 2. allocate memory for a state, this state must be deleted after collision checking
        state_t *stateCur = new state_t;;
        stateCur->x = state_from->x;
        stateCur->y = state_from->y;
        stateCur->z = state_from->z;

        // 3. collision checking loop
        for (int stepCount = 0; stepCount < steps; ++stepCount)
        {
            stateCur->x += stepX;
            stateCur->y += stepY;
            stateCur->z += stepZ;

            if (isIncollision(stateCur->x, stateCur->y))
            {
                delete stateCur;
                return false;
            }

            // close to obstacle cost
            edgeCosts[0] += _mapResolution * getCloseCollisionCost(stateCur->x, stateCur->y);
        }

        // lane change cost
        edgeCosts[1] = (float)abs(state_from->idx - state_to->idx);

        // not center lane cost
        // if (state_from->idx != _rollOutCenter || state_to->idx != _rollOutCenter)
        //     edgeCosts[2] += distance(state_from, state_to);

        // distance cost
        edgeCosts[2] = distance(state_from, state_to); // distance cost

        // treat first column nodes all the same
        if (state_from->idy == 0 && state_to->idy == 0)
            for (int i = 0; i < NUM_COSTS; ++i)
                edgeCosts[i] = 0;

        // treat last column nodes all the same
        if (state_from->idy == adjacency_length_grid - 1 && state_to->idy == adjacency_length_grid - 1)
            for (int i = 0; i < NUM_COSTS; ++i)
                edgeCosts[i] = 0;

        delete stateCur;
        return true;
    }

    bool searchAdjacencyMatrix()
    {
        // 1. reset costs
        for (int i = 0; i < nodeList.size(); ++i)
            for (int j = 0; j < NUM_COSTS; ++j)
                nodeList[i]->costsToRoot[j] = FLT_MAX;

        // 2. define start state
        state_t *startState = adjacencyMatrix[int(adjacency_width_grid/2)][0];
        for (int i = 0; i < NUM_COSTS; ++i)
            startState->costsToRoot[i] = 0;

        // 3. search graph
        vector<state_t*> Queue;
        Queue.push_back(startState);

        while(Queue.size() > 0 && ros::ok())
        {
            // find the state that can offer lowest cost in this depth and remove it from Queue
            state_t *fromState = minCostStateInQueue(Queue);
            Queue.erase(remove(Queue.begin(), Queue.end(), fromState), Queue.end());
            // loop through all neighbors of this state
            for (int i = 0; i < fromState->neighborList.size(); ++i)
            {
                state_t *toState = fromState->neighborList[i].neighbor;

                if (toState->validFlag == false)
                    continue;
                // Loop through cost hierarchy
                for (int costIndex = 0; costIndex < NUM_COSTS; ++costIndex)
                {
                    float thisCost = fromState->costsToRoot[costIndex] + fromState->neighborList[i].edgeCosts[costIndex];
                    // If cost can be improved, update this node with new costs
                    if (thisCost < toState->costsToRoot[costIndex]){
                        updateCosts(fromState, toState, i); // update toState's costToRoot
                        toState->parentState = fromState; // change parent for toState
                        Queue.push_back(toState);
                    }
                    // If cost is same, go to compare secondary cost
                    else if (thisCost == toState->costsToRoot[costIndex]){
                        continue;
                    }
                    // If cost becomes higher, abort this propagation
                    else
                        break;
                }
            }
        }

        // 4. find goal state
        Queue.clear();
        for (int i = 0; i < adjacency_width_grid; ++i)
            Queue.push_back(adjacencyMatrix[i][adjacency_length_grid-1]);

        state_t* goalState = minCostStateInQueue(Queue);

        // 5. extract path
        if (goalState->parentState == NULL)
        {
            failureOccurred();
        } else {
            searchedPath = extractPath(goalState);
            pubSearchedPath.publish(searchedPath);
            executePath = combinePaths(searchedPath, remainingPath);
        }

        return true;
    }

    void failureOccurred()
    {
        if (pointDistance(fixedPoint, robotPoint) > 0.25)
            fixedPoint = robotPoint;
        // search failed, let the robot stay in its position
        nav_msgs::Path pathOut;
        pathOut.header.frame_id = "map";
        pathOut.header.stamp = ros::Time();

        geometry_msgs::PoseStamped poseCur;
        poseCur.header.stamp = ros::Time();
        poseCur.header.frame_id = "map";

        poseCur.pose.position.x = fixedPoint.x;
        poseCur.pose.position.y = fixedPoint.y;
        poseCur.pose.position.z = fixedPoint.z;
        poseCur.pose.orientation = tf::createQuaternionMsgFromYaw(double(fixedPoint.intensity));

        pathOut.poses.push_back(poseCur);

        searchedPath = pathOut;
        executePath = pathOut;
        ROS_WARN("No path found, stay stationary!");
    }

    nav_msgs::Path extractPath(state_t* stateCur)
    {
        nav_msgs::Path pathOut;
        pathOut.header.frame_id = "map";
        pathOut.header.stamp = ros::Time();

        while (ros::ok())
        {
            geometry_msgs::PoseStamped poseCur;
            poseCur.header.stamp = ros::Time();
            poseCur.header.frame_id = "map";

            poseCur.pose.position.x = stateCur->x;
            poseCur.pose.position.y = stateCur->y;
            poseCur.pose.position.z = stateCur->z;

            pathOut.poses.insert(pathOut.poses.begin(), poseCur);

            if (stateCur->parentState == NULL)
                break;
            else
                stateCur = stateCur->parentState;
        }

        pathOut = processPath(pathOut);
        return pathOut;
    }

    void updateCosts(state_t* fromState, state_t* toState, int neighborInd){
        for (int i = 0; i < NUM_COSTS; ++i)
            toState->costsToRoot[i] = fromState->costsToRoot[i] + fromState->neighborList[neighborInd].edgeCosts[i];
    }

    state_t* minCostStateInQueue(vector<state_t*> Queue)
    {
        // Loop through cost hierarchy
        for (int costIndex = 0; costIndex < NUM_COSTS; ++costIndex)
        {
            vector<state_t*> tempQueue;
            float minCost = FLT_MAX;
            // loop through nodes saved in Queue
            for (vector<state_t*>::const_iterator iter2 = Queue.begin(); iter2 != Queue.end(); ++iter2){
                state_t* thisState = *iter2;
                // if cost is lower, we put it in tempQueue in case other nodes can offer same cost
                if (thisState->costsToRoot[costIndex] < minCost){
                    minCost = thisState->costsToRoot[costIndex];
                    tempQueue.clear();
                    tempQueue.push_back(thisState);
                }
                // same cost can be offered by other nodes, we save them to tempQueue for next loop (secondary cost)
                else if (thisState->costsToRoot[costIndex] == minCost)
                    tempQueue.push_back(thisState);
            }
            // Queue is used again for next loop
            Queue.clear();
            Queue = tempQueue;
        }
        // If cost hierarchy is selected correctly, there will be only one element left in Queue (no other ties)
        return Queue[0];
    }

    void visualization()
    {
        if (pubPRMGraph.getNumSubscribers() != 0)
        {
            visualization_msgs::MarkerArray markerArray;
            geometry_msgs::Point p;

            // PRM nodes visualization
            visualization_msgs::Marker markerNode;
            markerNode.header.frame_id = "map";
            markerNode.header.stamp = ros::Time::now();
            markerNode.action = visualization_msgs::Marker::ADD;
            markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
            markerNode.ns = "nodes";
            markerNode.id = 2;
            markerNode.scale.x = 0.03; markerNode.scale.y = 0.03; markerNode.scale.z = 0.03; 
            markerNode.color.r = 0; markerNode.color.g = 1; markerNode.color.b = 1;
            markerNode.color.a = 1;

            for (int i = 0; i < adjacency_width_grid; ++i)
            {
                for (int j = 0; j < adjacency_length_grid; ++j)
                {
                    if (adjacencyMatrix[i][j]->validFlag == false)
                        continue;
                    p.x = adjacencyMatrix[i][j]->x;
                    p.y = adjacencyMatrix[i][j]->y;
                    p.z = adjacencyMatrix[i][j]->z + 0.015;
                    markerNode.points.push_back(p);
                }
            }

            // PRM edge visualization
            visualization_msgs::Marker markerEdge;
            markerEdge.header.frame_id = "map";
            markerEdge.header.stamp = ros::Time::now();
            markerEdge.action = visualization_msgs::Marker::ADD;
            markerEdge.type = visualization_msgs::Marker::LINE_LIST;
            markerEdge.ns = "edges";
            markerEdge.id = 3;
            markerEdge.scale.x = 0.01; markerEdge.scale.y = 0.01; markerEdge.scale.z = 0.01;
            markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
            markerEdge.color.a = 1;

            for (int i = 0; i < adjacency_width_grid; ++i)
            {
                for (int j = 0; j < adjacency_length_grid; ++j)
                {
                    if (adjacencyMatrix[i][j]->validFlag == false)
                        continue;
                    int numNeighbors = adjacencyMatrix[i][j]->neighborList.size();
                    for (int k = 0; k < numNeighbors; ++k)
                    {
                        if (adjacencyMatrix[i][j]->neighborList[k].neighbor->validFlag == false)
                            continue;
                        p.x = adjacencyMatrix[i][j]->x;
                        p.y = adjacencyMatrix[i][j]->y;
                        p.z = adjacencyMatrix[i][j]->z + 0.005;
                        markerEdge.points.push_back(p);
                        p.x = adjacencyMatrix[i][j]->neighborList[k].neighbor->x;
                        p.y = adjacencyMatrix[i][j]->neighborList[k].neighbor->y;
                        p.z = adjacencyMatrix[i][j]->neighborList[k].neighbor->z + 0.005;
                        markerEdge.points.push_back(p);
                    }
                }
            }

            // push to markerarray and publish
            markerArray.markers.push_back(markerNode);
            markerArray.markers.push_back(markerEdge);
            pubPRMGraph.publish(markerArray);
        }

        // 4. Single Source Shortest Paths
        if (pubSingleSourcePaths.getNumSubscribers() != 0){

            visualization_msgs::MarkerArray markerArray;
            geometry_msgs::Point p;

            // single source path visualization
            visualization_msgs::Marker markersPath;
            markersPath.header.frame_id = "map";
            markersPath.header.stamp = ros::Time::now();
            markersPath.action = visualization_msgs::Marker::ADD;
            markersPath.type = visualization_msgs::Marker::LINE_LIST;
            markersPath.ns = "path";
            markersPath.id = 4;
            markersPath.scale.x = 0.02; markersPath.scale.y = 0.02; markersPath.scale.z = 0.02;
            markersPath.color.r = 0.3; markersPath.color.g = 0; markersPath.color.b = 1.0;
            markersPath.color.a = 1.0;

            for (int i = 0; i < nodeList.size(); ++i){
                if (nodeList[i]->parentState == NULL)
                    continue;
                if (nodeList[i]->validFlag == false)
                    continue;
                p.x = nodeList[i]->x;
                p.y = nodeList[i]->y;
                p.z = nodeList[i]->z + 0.1;
                markersPath.points.push_back(p);
                p.x = nodeList[i]->parentState->x;
                p.y = nodeList[i]->parentState->y;
                p.z = nodeList[i]->parentState->z + 0.01;
                markersPath.points.push_back(p);
            }
            // push to markerarray and publish
            markerArray.markers.push_back(markersPath);
            pubSingleSourcePaths.publish(markerArray);
        }
    }

    bool getRobotPosition()
    {
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return false; }
        
        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = 0;//transform.getOrigin().z();

        double roll, pitch, yaw;
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(roll, pitch, yaw);
        robotPoint.intensity = yaw;

        if (globalPathMessage.poses.size() == 0)
            return false;

        if (occupancyMap2D.data.size() == 0)
            return false;

        lastRobotPoint = robotPoint;

        return true;
    }

    void publishPath()
    {
        int size = executePath.poses.size();
        if (size <= 2)
        {
            pubExecutePath.publish(executePath);
            return;
        }

        // truncate path
        int min_id = -1;
        float min_dist = FLT_MAX;

        for (int i = 0; i < size; ++i)
        {
            PointType p;
            p.x = executePath.poses[i].pose.position.x;
            p.y = executePath.poses[i].pose.position.y;
            p.z = robotPoint.z;

            float dist = pointDistance(p, robotPoint);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_id = i;
            }
        }

        if (min_id >= 0 && min_dist < 1.0)
            executePath.poses.erase(executePath.poses.begin(), executePath.poses.begin() + min_id);

        pubExecutePath.publish(executePath);
    }

    bool needNewPath()
    {
        bool needFlag = false;
        // no path available
        if (searchedPath.poses.size() <= 1)
            needFlag = true;
        
        // path in collision
        for (int i = 0; i < executePath.poses.size(); ++i)
        {
            float x = executePath.poses[i].pose.position.x;
            float y = executePath.poses[i].pose.position.y;

            if (isIncollision(x, y))// || isCloseCollision(x, y))
            {
                ROS_WARN("Obstacles on path, re-planning.");
                needFlag = true;
                break;
            }
        }

        if (needFlag == true)
            return true;

        return false;
    }

    void updatePath(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(mtx);

        if (getRobotPosition() == false) return;

        if (needNewPath())
        {
            buildAdjacencyMatrix();

            connectAdjacencyMatrix();

            searchAdjacencyMatrix();

            visualization();
        }

        publishPath();
    }
};


int main(int argc, char** argv){

    ros::init(argc, argv, "lexicographic_planning");
    
    PathPlanning pp;

    ROS_INFO("\033[1;32m----> lexicographic_planning: Path Planning Started.\033[0m");

    ros::spin();

    return 0;
}