#include<iostream>
#include<queue>
#include<stack>
using namespace std;
#define N 5

int BFS_maze[5][5] = {
    { 0, 1, 1, 0, 0 },
    { 0, 0, 1, 1, 0 },
    { 0, 1, 1, 1, 0 },
    { 1, 0, 0, 0, 0 },
    { 0, 0, 1, 1, 0 }
};
int DFS_maze[5][5] = {
    { 0, 1, 1, 0, 0 },
    { 0, 0, 1, 0, 1 },
    { 0, 0, 1, 0, 0 },
    { 1, 1, 0, 0, 1 },
    { 0, 0, 1, 0, 0 }
};
int visited_BFS[N]={0};
int visited_DFS[N]={0};
void BFS(int s){
    queue<int> q;
    visited_BFS[s]=1;
    q.push(s);
    while (!q.empty())
    {
        int curr=q.front();
        cout<<curr<<endl;
        q.pop();
        for(int i=0;i<N;i++){
            if(!visited_BFS[i]&&BFS_maze[curr][i]==1){
                visited_BFS[i]=1;
                q.push(i);
            }
        }

    }
}
void DFS(int a){
    stack<int> s;
    if(!visited_DFS[a]){
        s.push(a);
    }
    while(!s.empty()){
        int cur=s.top();
        for(int i=0;i<N;i++){//访问当前节点的所有下一节点
            if(!visited_DFS[i]&&DFS_maze[cur][i]==1){
            s.push(i);
        }
    }
    cur=s.top();
    cout<<cur<<endl;
    visited_DFS[cur]=1;
    s.pop();
    }
    
}
int main(){
    // for(int i=0;i<5;i++){
    //     if(visited_BFS[i]){
    //         continue;
    //     }
    //     BFS(i);
    // }

    for(int i=0;i<5;i++){
        if(visited_DFS[i]){
            continue;
        }
        DFS(i);
    }
    return 0;
}