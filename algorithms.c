#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define MAX 100
#define INFINITY 9999


/*<----- GLOBAL VARAIBLES ----------->*/

int n,m; //no. of vertices
typdef int** Adjacency; //typdef for convinience
Adjacency AdjacencyMatrix[MAX][MAX]; //the Adjacency Matrix
struct {
    int weight;
    int src;
    int dest;
} edges[MAX];
int edgesCount = 0;  
/*<----- GLOBAL VARAIBLES ----------->*/
/*<----- FUNCTION DECLARATIONS ------>*/

void createMatrix(); //to create the adjacency matrix
void graph(); //for graph related algos
void MST(); //for MST related algos
void djikstra(); //for djikstra
void floydWarshall(); //for floydWarshall
void bellmanFord(); //for bellmanFord
void prims(); //for prims
void kruskal(); //for kruskal
int find(int);
int uni(int,int);
int minKey(int[], bool[]);

/*<----- FUNCTION DECLARATIONS ------>*/

/*<----- PLEASE NOTE ------>*/
//The system() function works only in Windows, make sure you're on a Windows system.
//The system() function is responsible for clearing and pausing (for user input).
/*<----- PLEASE NOTE ------>*/

void main() 
{
    system("clear");
    int quit = 0;
    int ch; char swallow;
    while(!quit)    // Infinite loop that terminates only when condition is met
    {
    printf("Choose :\n1.Input Graph\n2.Minimal Spanning Tree\n3.Exit\nEnter your choice: ");
    scanf("%d", &ch);
    system("clear");
    switch(ch)
    {
            case 1:
            createMatrix();
            timeout();
            break;

            case 2:
            graph();
            timeout();
            break;

            case 3:
            mst();
            timeout();
            break;

            case 4:
            printf("Thank You!");
            quit = 1;
            break; 
            //Loop terminates

            default: printf("Invalid choice");
    }
    }
}  

void createMatrix()
{
    printf("Enter no. of vertices:\n");
    scanf("%d",&n);
    printf("\nEnter the adjacency matrix:\n");
    for(i=0;i<n;i++)
        for(j=0;j<n;j++)
            scanf("%d",&adjacencyMatrix[i][j]);
}

void edge()
{
    edgesCount=0;
    for(int i=0;i<n;i++)
    {   
        for(int j=0;j<n;i++)
        {
            // for(int k=0;k<edgesCount;k++)
            // {
            //     if(edges[edgesCount].vertex2==i && edges[edgesCount].vertex1==j)
            //     continue;
            // }
            //check for duplicates

            if(AdjacencyMatrix[i][j]==0)
            {
                edges[edgesCount].src=i;
                edges[edgesCount].dest=j;
                edges[edgesCount].weight=AdjacencyMatrix[i][j];
                edgesCount++;
            }   
        }     
    }
    //to find all the edges in an undirected graph (considering undirected edges both times)
}

void graph()
{
    system("clear")
    char sel;
    printf("Select implementation\n");
    printf("a - Single source shortest path - Djikstra's algorithm\n");
    printf("b - Single source shortest path - Bellman Ford's algorithm\n");
    printf("c - All pairs shortest path - Floyd Warshall algorithm\n");
    scanf("%c",&sel);
    int i,j,ele;
    switch(sel)
    {
        case 'a':   djikstra();

        case 'b': //Bellman Ford

        case 'c': //Floyd Warshall
    }        
}

void mst()
{
    system("clear")
    char sel;
    printf("Select implementation\n");
    printf("a - Prim's algorithm\n");
    printf("b - Kruskal algorithm\n");
    scanf("%c",&sel);
    switch(sel)
    {
        case 'a': prims();

        case 'b': kruskal();
    }
}

void timeout()
{
    char swallow;
    printf("\n");
    system("pause");
    scanf("%c", &swallow);
    system("clear");
}

/*<------- GRAPH -------->*/
void djikstra()
{
    int cost[MAX][MAX],distance[MAX],pred[MAX];
    int visited[MAX],count,mindistance,nextnode;
    int i,j;

    printf("\nEnter the starting node:");
    scanf("%d",&u);

    for(i=0;i<n;i++)
        for(j=0;j<n;j++)
            cost[i][j]= AdjacencyMatrix[i][j]==0 ? INFINITY : G[i][j];
    //calculating the cost  

    for(i=0;i<n;i++)
    {
        distance[i]=cost[startnode][i];
        pred[i]=startnode;
        visited[i]=0;
    }

    distance[startnode]=0;
    visited[startnode]=1;
    count=1;

    while(count<n-1)
    {
        mindistance=INFINITY;
        //nextnode gives the node at minimum distance
        for(i=0;i<n;i++)
            if(distance[i]<mindistance&&!visited[i])
            {   
                mindistance=distance[i];
                nextnode=i;
            }
        //check if a better path exists through nextnode
        visited[nextnode]=1;
        for(i=0;i<n;i++)
            if(!visited[i])
                {
                if(mindistance+defuzzy(cost[nextnode][i],1)<distance[i])
                {
                    distance[i]=mindistance+defuzzy(cost[nextnode][i]);
                    pred[i]=nextnode;
                }
                count++;
                }
    }

    //print the path and distance of each node
    for(i=0;i<n;i++)
        if(i!=startnode)
            {
                printf("\nDistance of node %d = %d",i,distance[i]);
                printf("\nPath=%d",i);
                j=i;
                do
                    {
                        j=pred[j];
                        printf("<-%d",j);
                    }while(j!=startnode);
            }
}

void FloydWarshal()
{
    int k;//k=adjacency matrix for a vertex n;
    int i,j;//row and column
    for(k=0;k<n;k++)
    {
        for(i=0;i<n;i++)
        {
            for(j=0;j<n;j++)
            AdjacencyMatrix[i][j]=min(a[i][j],a[i][k]+a[k][j]);
        }
    }
}

void bellmanFord()
{   
    edge();
    int V = n;
    int E = edgesCount;
    int dist[V];
    
    for (int i = 0; i < n; i++)
        dist[i] = INFINITY;
    dist[src] = 0;
 
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < E; j++) {
            int u = edges[j].src;
            int v = edges[j].dest;
            int weight = edges[j].weight;
            if (dist[u] != INFINITY && dist[u] + weight < dist[v])
                dist[v] = dist[u] + weight;
        }
    }
    for (int i = 0; i < E; i++) {
        int u = edges[i].src;
        int v = edges[i].dest;
        int weight = edges[i].weight;
        if (dist[u] != INFINITY && dist[u] + weight < dist[v]) {
            printf("Graph contains negative weight cycle");
            return;
        }
    }
    
    printf("Vertex distance from Source\n");
    for (int i = 0; i < n; ++i)
        printf("%d \t\t %d\n", i, dist[i]);

}

/*<------- MST -------->*/

int minKey(int key[], bool mstSet[])
{
    // Initialize min value
    int min = MAX, min_index;
 
    for (int v = 0; v < V; v++)
        if (mstSet[v] == false && key[v] < min)
            min = key[v], min_index = v;
 
    return min_index;
}
 
// A utility function to print the
// constructed MST stored in parent[]
 
// Function to construct and print MST for
// a graph represented using adjacency
// matrix representation

void prims()
{
    // Array to store constructed MST
    int V=n;
    int parent[V];
    // Key values used to pick minimum weight edge in cut
    int key[V];
    // To represent set of vertices included in MST
    bool mstSet[V];
 
    // Initialize all keys as INFINITE
    for (int i = 0; i < V; i++)
        key[i] = INT_MAX, mstSet[i] = false;
 
    // Always include first 1st vertex in MST.
    // Make key 0 so that this vertex is picked as first vertex.
    key[0] = 0;
    parent[0] = -1; // First node is always root of MST
 
    // The MST will have V vertices
    for (int count = 0; count < V - 1; count++) {
        // Pick the minimum key vertex from the
        // set of vertices not yet included in MST
        int u = minKey(key, mstSet);
 
        // Add the picked vertex to the MST Set
        mstSet[u] = true;
 
        // Update key value and parent index of
        // the adjacent vertices of the picked vertex.
        // Consider only those vertices which are not
        // yet included in MST
        for (int v = 0; v < V; v++)
 
            // graph[u][v] is non zero only for adjacent vertices of m
            // mstSet[v] is false for vertices not yet included in MST
            // Update the key only if graph[u][v] is smaller than key[v]
            if (adjacencyMatrix[u][v] && mstSet[v] == false && adjacencyMatrix[u][v] < key[v])
                parent[v] = u, key[v] = graph[u][v];
    }
 
    // print the constructed MST
    printf("Edge \tWeight\n");
    for (int i = 1; i < V; i++)
        printf("%d - %d \t%d \n", parent[i], i, adjacencyMatrix[i][parent[i]]);
}

void kruskal()
{   
    int mincost=0, min;int parent[9];int ne=1,a,b,u,v,;
    	while(ne < n)
	{
		for(i=1,min=999;i<=n;i++)
		{
			for(j=1;j <= n;j++)
			{
				if(cost[i][j] < min)
				{
					min=cost[i][j];
					a=u=i;
					b=v=j;
				}
			}
		}
		u=find(u);
		v=find(v);
		if(uni(u,v))
		{
			printf("%d edge (%d,%d) =%d\n",ne++,a,b,min);
			mincost +=min;
		}
		adjacencyMatrix[a][b]=adjacencyMatrix[b][a]=999;
	}
	printf("\n\tMinimum cost = %d\n",mincost);
	getch();
}

int find(int i)
{
	while(parent[i])
	i=parent[i];
	return i;
}

int uni(int i,int j)
{
	if(i!=j)
	{
		parent[j]=i;
		return 1;
	}
	return 0;
}
