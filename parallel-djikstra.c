#include<stdlib.h>
#include<stdio.h>
#include<time.h>
#include<omp.h>

#define V 9

int main (int argc, char **argv);

int *dijkstra_distance (int distanceMatrix[V][V]);
void find_nearest_data (int s, int e, int minDistance[V], int connected[V], int *d, int *v);
void init (int distanceMatrix[V][V]);
void timestamp (void);
void updateMinDistance (int s, int e, int nearestNode, int connected[V], int distanceMatrix[V][V], int minDistance[V]);

int main (int argc, char **argv) {
  int i;
  int infinite = 2147483647;
  int j;
  int *minDistance;
  int distanceMatrix[V][V];
  init (distanceMatrix);

  minDistance = dijkstra_distance(distanceMatrix);

  for (i=0; i<V; i++)
    fprintf ( stdout, "  %2d  \t\t%2d\n", i, minDistance[i] );

  free ( minDistance );
  return 0;
}

int *dijkstra_distance (int distanceMatrix[V][V]) {
  int *connected;
  int i;
  int infinite = 2147483647;
  int minDist;
  int *minDistance;
  int nearestNode;
  int threadFirst;
  int threadID;
  int threadLast;
  int threadMinDistance;
  int threadNearestNode;
  int threadIterationCount;
  int numOfThreads;

  connected = (int *)malloc(V*sizeof(int));
  connected[0] = 1;
  for (i=1; i<V; i++)
    connected[i] = 0;

  minDistance = (int *)malloc(V*sizeof(int));

  for (i=0; i<V; i++)
    minDistance[i] = distanceMatrix[0][i];

  #pragma omp parallel private(threadFirst, threadID, threadLast, threadMinDistance, threadNearestNode,                         \
    threadIterationCount) shared(connected, minDist, minDistance, nearestNode, numOfThreads, distanceMatrix)
  {
    threadID = omp_get_thread_num();
    numOfThreads = omp_get_num_threads();
    threadFirst = (threadID*V)/numOfThreads;
    threadLast  = ((threadID+1)*V)/numOfThreads - 1;
    #pragma omp single
    {
      int TID = threadID;
      int threadsNum = numOfThreads;
    }
    for (threadIterationCount=1; threadIterationCount<V; threadIterationCount++)
    {
      #pragma omp single
      {
        minDist = infinite;
        nearestNode = -1;
      }
      find_nearest_data(threadFirst, threadLast, minDistance, connected, &threadMinDistance, &threadNearestNode);
      #pragma omp critical
      {
        if (threadMinDistance < minDist) {
          minDist = threadMinDistance;
          nearestNode = threadNearestNode;
        }
      }
      #pragma omp barrier
      #pragma omp single
      {
        if (nearestNode != -1)
          connected[nearestNode] = 1;
      }
      #pragma omp barrier
      if (nearestNode != -1)
        updateMinDistance(threadFirst, threadLast, nearestNode, connected, distanceMatrix, minDistance);
      #pragma omp barrier
    }
    #pragma omp single
    {
      int endID = threadID;
    }
  }
  free(connected);
  return minDistance;
}

void find_nearest_data(int s, int e, int minDistance[V], int connected[V], int *d, int *v) {
  int i;
  int infinite = 2147483647;
  *d = infinite;
  *v = -1;
  for (i=s; i<=e; i++) {
    if (!connected[i] && (minDistance[i] < *d)) {
      *d = minDistance[i];
      *v = i;
    }
  }
  return;
}

void init (int distanceMatrix[V][V]) {
  int i;
  int infinite = 2147483647;
  int j;
  for (i=0; i<V; i++) {
    for (j=0; j<V; j++) {
      if (i==j)
        distanceMatrix[i][i] = 0;
      else
        distanceMatrix[i][j] = infinite;
    }
  }
  distanceMatrix[0][1] = distanceMatrix[1][0] = 4;
  distanceMatrix[0][7] = distanceMatrix[7][0] = 8;
  distanceMatrix[1][2] = distanceMatrix[2][1] = 8;
  distanceMatrix[1][7] = distanceMatrix[7][1] = 11;
  distanceMatrix[2][3] = distanceMatrix[3][2] = 7;
  distanceMatrix[2][5] = distanceMatrix[5][2] = 4;
  distanceMatrix[2][8] = distanceMatrix[8][2] = 2;
  distanceMatrix[3][4] = distanceMatrix[4][3] = 9;
  distanceMatrix[3][5] = distanceMatrix[5][3] = 14;
  distanceMatrix[4][5] = distanceMatrix[5][4] = 10;
  distanceMatrix[5][6] = distanceMatrix[6][5] = 2;
  distanceMatrix[6][7] = distanceMatrix[7][6] = 1;
  distanceMatrix[6][8] = distanceMatrix[8][6] = 6;
  distanceMatrix[7][8] = distanceMatrix[8][7] = 7;
  return;
}

void timestamp(void) {

  #define TIME_SIZE 40
  static char time_buffer[TIME_SIZE];
  const struct tm *tm;
  time_t now;
  now = time ( NULL );
  tm = localtime ( &now );
  strftime ( time_buffer, TIME_SIZE, "%d %B %Y %I:%M:%S %p", tm );
  printf ( "%s\n", time_buffer );
  return;
  #undef TIME_SIZE
}

void updateMinDistance(int s, int e, int mv, int connected[V], int distanceMatrix[V][V], int minDistance[V]) {
  int i;
  int infinite = 2147483647;
  for (i=s; i<=e; i++) {
    if (!connected[i]) {
      if (distanceMatrix[mv][i] < infinite) {
        if (minDistance[mv] + distanceMatrix[mv][i] < minDistance[i]) {
          minDistance[i] = minDistance[mv] + distanceMatrix[mv][i];
        }
      }
    }
  }
  return;
}
