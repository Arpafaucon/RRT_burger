// int publishEnvironment(lcm_t *lcm, region &regionOperating, region &regionGoal, list<region *> &obstacles)
// {

//     // Publish the environment
//     lcmtypes_environment_t *environment = (lcmtypes_environment_t *)malloc(sizeof(lcmtypes_environment_t));

//     environment->operating.center[0] = regionOperating.center[0];
//     environment->operating.center[1] = regionOperating.center[1];
//     environment->operating.center[2] = regionOperating.center[2];
//     environment->operating.size[0] = regionOperating.size[0];
//     environment->operating.size[1] = regionOperating.size[1];
//     environment->operating.size[2] = regionOperating.size[2];

//     environment->goal.center[0] = regionGoal.center[0];
//     environment->goal.center[1] = regionGoal.center[1];
//     environment->goal.center[2] = regionGoal.center[2];
//     environment->goal.size[0] = regionGoal.size[0];
//     environment->goal.size[1] = regionGoal.size[1];
//     environment->goal.size[2] = regionGoal.size[2];

//     environment->num_obstacles = obstacles.size();

//     if (environment->num_obstacles > 0)
//         environment->obstacles = (lcmtypes_region_3d_t *)malloc(sizeof(lcmtypes_region_3d_t));

//     int idx_obstacles = 0;
//     for (list<region *>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++)
//     {

//         region *obstacleCurr = *iter;

//         environment->obstacles[idx_obstacles].center[0] = obstacleCurr->center[0];
//         environment->obstacles[idx_obstacles].center[1] = obstacleCurr->center[1];
//         environment->obstacles[idx_obstacles].center[2] = obstacleCurr->center[2];
//         environment->obstacles[idx_obstacles].size[0] = obstacleCurr->size[0];
//         environment->obstacles[idx_obstacles].size[1] = obstacleCurr->size[1];
//         environment->obstacles[idx_obstacles].size[2] = obstacleCurr->size[2];

//         idx_obstacles++;
//     }

//     lcmtypes_environment_t_publish(lcm, "ENVIRONMENT", environment);

//     return 1;
// }

// int publishTraj(lcm_t *lcm, planner_t &planner, System &system)
// {

//     cout << "Publishing trajectory -- start" << endl;

//     vertex_t &vertexBest = planner.getBestVertex();

//     if (&vertexBest == NULL)
//     {
//         cout << "No best vertex" << endl;
//         return 0;
//     }

//     list<double *> stateList;

//     planner.getBestTrajectory(stateList);

//     lcmtypes_trajectory_t *opttraj = (lcmtypes_trajectory_t *)malloc(sizeof(lcmtypes_trajectory_t));

//     opttraj->num_states = stateList.size();
//     opttraj->states = (lcmtypes_state_t *)malloc(opttraj->num_states * sizeof(lcmtypes_state_t));

//     int stateIndex = 0;
//     for (list<double *>::iterator iter = stateList.begin(); iter != stateList.end(); iter++)
//     {

//         double *stateRef = *iter;
//         opttraj->states[stateIndex].x = stateRef[0];
//         opttraj->states[stateIndex].y = stateRef[1];
//         if (system.getNumDimensions() > 2)
//             opttraj->states[stateIndex].z = stateRef[2];
//         else
//             opttraj->states[stateIndex].z = 0.0;

//         delete[] stateRef;

//         stateIndex++;
//     }

//     lcmtypes_trajectory_t_publish(lcm, "TRAJECTORY", opttraj);

//     lcmtypes_trajectory_t_destroy(opttraj);

//     cout << "Publishing trajectory -- end" << endl;

//     return 1;
// }

// int publishTree(lcm_t *lcm, planner_t &planner, System &system)
// {

// cout << "Publishing the tree -- start" << endl;

// bool plot3d = (system.getNumDimensions() > 2);

// lcmtypes_graph_t *graph = (lcmtypes_graph_t *)malloc(sizeof(lcmtypes_graph_t));
// graph->num_vertices = planner.numVertices;

// if (graph->num_vertices > 0)
// {

//     graph->vertices = (lcmtypes_vertex_t *)malloc(graph->num_vertices * sizeof(lcmtypes_vertex_t));

//     int vertexIndex = 0;
//     for (list<vertex_t *>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++)
//     {

//         vertex_t &vertexCurr = **iter;
//         State &stateCurr = vertexCurr.getState();

//         graph->vertices[vertexIndex].state.x = stateCurr[0];
//         graph->vertices[vertexIndex].state.y = stateCurr[1];
//         if (plot3d)
//             graph->vertices[vertexIndex].state.z = stateCurr[2];
//         else
//             graph->vertices[vertexIndex].state.z = 0.0;

//         vertexIndex++;
//     }
// }
// else
// {
//     graph->vertices = NULL;
// }

// if (graph->num_vertices > 1)
// {

//     graph->num_edges = graph->num_vertices - 1;
//     graph->edges = (lcmtypes_edge_t *)malloc(graph->num_edges * sizeof(lcmtypes_edge_t));

//     int edgeIndex = 0;
//     for (list<vertex_t *>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++)
//     {

//         vertex_t &vertexCurr = **iter;

//         vertex_t &vertexParent = vertexCurr.getParent();

//         if (&vertexParent == NULL)
//             continue;

//         State &stateCurr = vertexCurr.getState();
//         State &stateParent = vertexParent.getState();

//         graph->edges[edgeIndex].vertex_src.state.x = stateParent[0];
//         graph->edges[edgeIndex].vertex_src.state.y = stateParent[1];
//         if (plot3d)
//             graph->edges[edgeIndex].vertex_src.state.z = stateParent[2];
//         else
//             graph->edges[edgeIndex].vertex_src.state.z = 0.0;

//         graph->edges[edgeIndex].vertex_dst.state.x = stateCurr[0];
//         graph->edges[edgeIndex].vertex_dst.state.y = stateCurr[1];
//         if (plot3d)
//             graph->edges[edgeIndex].vertex_dst.state.z = stateCurr[2];
//         else
//             graph->edges[edgeIndex].vertex_dst.state.z = 0.0;

//         graph->edges[edgeIndex].trajectory.num_states = 0;
//         graph->edges[edgeIndex].trajectory.states = NULL;

//         edgeIndex++;
//     }
// }
// else
// {
//     graph->num_edges = 0;
//     graph->edges = NULL;
// }

// lcmtypes_graph_t_publish(lcm, "GRAPH", graph);

// lcmtypes_graph_t_destroy(graph);

// cout << "Publishing the tree -- end" << endl;

// return 1;
// }
