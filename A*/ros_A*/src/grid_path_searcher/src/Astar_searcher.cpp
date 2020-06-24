#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

//这里初始化了整个地图，给每一个栅格初始化并new
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);//地图左上角，原点在中间
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);//地图右下角
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;//地图栅格索引
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);//初始化了整个地图，id为0，在回调函数中初始化了障碍物信息
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    //障碍物越界，返回空
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;//降维处理，将障碍物信息用一维data数组表示
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) //根据节点索引求坐标
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) //根据坐标求索引
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}
//障碍物判断，有障碍物置1
inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}
int count1=1;
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   cout<<"count: "<<count1<<endl;
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    */
    Vector3i next_idx;
    Vector3d next_pt;
    GridNodePtr nextPtr;
    double gValue;

    for(int i=-1;i<=1;i++){
        {
            for(int j=-1;j<=1;j++){
                for(int k=-1;k<=1;k++){
                    if(i==0&&j==0&&k==0)//不添加本身
                        continue;
                    //不越界
                    else if(currentPtr->index(0)+i<0||
                            currentPtr->index(0)+i>=GLX_SIZE||
                            currentPtr->index(1)+j<0||
                            currentPtr->index(1)+j>=GLY_SIZE||
                            currentPtr->index(2)+k<0||
                            currentPtr->index(2)+k>=GLZ_SIZE){
                        continue;
                    }
                    else{
                        next_idx(0)=currentPtr->index(0)+i;
                        next_idx(1)=currentPtr->index(1)+j;
                        next_idx(2)=currentPtr->index(2)+k;
                        if(isOccupied(next_idx)){//存在障碍,不算邻居节点
                            continue;
                            }
                        else{
                            nextPtr=GridNodeMap[next_idx(0)][next_idx(1)][next_idx(2)];
                            //当前节点到下一节点的g值
                            gValue=sqrt(pow(next_idx(0)-currentPtr->index(0),2)+
                                        pow(next_idx(1)-currentPtr->index(1),2)+
                                        pow(next_idx(2)-currentPtr->index(2),2));
                            neighborPtrSets.push_back(nextPtr);
                            edgeCostSets.push_back(gValue);
                        }
                        }  
                        }
                    }
                }
            }
            count1++;
}


double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    */
    //采用曼哈顿距离，返回下标索引的差值
    double hVal;
    hVal=abs(node1->index(0)-node2->index(0))+
         abs(node1->index(1)-node2->index(1))+
         abs(node1->index(2)-node2->index(2));

    return hVal;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);
    cout<<"endPtr->coord"<<endPtr->coord;

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; //1为进入openset，-1为进入closeset，0为初始值
    startPtr -> coord = start_pt;

    startPtr->cameFrom=startPtr;//起始节点父节点为本身

    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    /*
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    */
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet.empty() ){
        /*
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        */

        currentPtr=openSet.begin()->second;//将最小f值置为当前节点
        //currentPtr->index=coord2gridIndex(currentPtr->coord);//获得当前节点的下标
        currentPtr->id=-1;//表示加入closeSet
        openSet.erase(openSet.begin());//从openset中移除

        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            goalPtr=currentPtr;
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost is %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        /*
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below       
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            //cout<<neighborPtrSets.size()<<endl;
            /*
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set   
            */
            neighborPtr=neighborPtrSets[i];

            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                */
                neighborPtr->id=1;//加入openset
                neighborPtr->cameFrom=currentPtr;//给新节点设置父节点
                neighborPtr->gScore=edgeCostSets[i]+currentPtr->gScore;
                neighborPtr->fScore=getHeu(neighborPtr,endPtr)+edgeCostSets[i];
                openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
                continue;
            }//邻居节点已经在openset中了则说明，其父节点已经由上一步设置好了
            else if(neighborPtr -> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                */
                //通过当前节点达到已经在openset中节点是否更近，更近则更换父节点
                double temp_gValue=(currentPtr->gScore)+
                                    (sqrt(pow(neighborPtr->index(0)-currentPtr->index(0),2)+
                                          pow(neighborPtr->index(1)-currentPtr->index(1),2)+
                                          pow(neighborPtr->index(2)-currentPtr->index(2),2)));
                if(temp_gValue>neighborPtr->gScore){//如果比原来的大，跳过
                    continue;
                }
                else{
                        neighborPtr->gScore=temp_gValue;
                        neighborPtr->cameFrom=currentPtr;
                        //找到当前键值，删除之后，再添加
                        for(auto it=openSet.begin();it!=openSet.end();it++){
                            if(it->second->index==neighborPtr->index){
                                openSet.erase(it);
                                break;
                                }
                            }
                        openSet.insert(make_pair((temp_gValue+getHeu(neighborPtr,endPtr)),neighborPtr));
                    continue;
                }
                
            }
            else{//this node is in closed set
                /*
                please write your code below 
                */
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    path.clear();
    gridPath.clear();
    /*
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below     
    */
    GridNodePtr tempPtr=goalPtr->cameFrom;
    cout<<"goalPtr->coord"<<goalPtr->coord;
    //gridPath.push_back(tempPtr);
    while(tempPtr->index!=startPtr->index){
        gridPath.push_back(tempPtr);
        tempPtr=tempPtr->cameFrom;
        
    }
    cout<<"gridPath.size()"<<gridPath.size()<<endl;

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}