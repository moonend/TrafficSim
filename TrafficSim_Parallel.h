#define NUM_CCLANE 6
#define NUM_LANE 4
#define NUM_SECTION 4
#define MAX_VEC 20
#define dt          5

#define MIN(a,b) (((a)<(b)) ? (a):(b))
#define MAX(a,b) (((a)>(b)) ? (a):(b))

typedef struct {
	int vehType;       // type of vehicle
	int vehID;         // ID of vehicle

	int path[20];      // Array of Link IDs in the order in which vechicle passes
	int lenPath;       // Length of path, total number of links in path

	int minTargetLane[20]; // Minimum Target Lane
	int maxTargetLane[20]; // Maximum Target Lane

	int currLane;      // current Lane ID
	int currSection;   // current Section ID
	int currLink;      // current Link ID

} vehicle;


typedef struct {
	int linkID;
	
	int prevCCID[3];						//[Left,Right,straight]
	int nextCCID;							//각 Link마다 nextCC를 가짐. ex) Link 0 -> CC 0
	int nextNumVeh[2];						//[Left,Right]
	int nextVehIDArr[2][MAX_VEC];			//[Left,Right]
	int nextCurrLinkOrderArr[2][MAX_VEC];	//[Left,Right]

	int maxNumVeh[NUM_SECTION + 2][NUM_LANE];   	// 2D Array [NoCell	,NoLane]
	double maxNumCF[NUM_SECTION + 2][NUM_LANE];	// 2D Array [NoCell	,NoLane]
												//TODO : Dimension fix to +1
	double ffSpeed;	// Free flow speed 	
	double lenSection[NUM_SECTION + 2];

	int numVeh[NUM_SECTION + 2][NUM_LANE];  	  	// 

	int vehIDArr[NUM_SECTION + 2][NUM_LANE][MAX_VEC];	// vehID per each cell (include buffer cell)
	int currLinkOrderArr[NUM_SECTION + 2][NUM_LANE][MAX_VEC];
	int minTargetLaneArr[NUM_SECTION + 2][NUM_LANE][MAX_VEC]; 	// minimum Target Lane  EX) 2  타겟 레인의 하한값 설정
	int maxTargetLaneArr[NUM_SECTION + 2][NUM_LANE][MAX_VEC]; 	// max Target Lane  Ex) 3   타겟 레인 상한 값 설정 	

	int vehMLC[NUM_SECTION + 2][NUM_LANE][MAX_VEC];    	// 1이면 오른쪽으로차로변경, 0=> not right turn
	int vehOLC[NUM_SECTION + 2][NUM_LANE][MAX_VEC]; 		// if 1, right; if 0, stay(or MLC); if -1, left
	int vehCF[NUM_SECTION + 2][NUM_LANE][MAX_VEC];   //1이면 다음셀로 전진,0이면 현재셀에 머무르기

	int numMLCL[NUM_SECTION + 2][NUM_LANE];     	// 2D Array [NoCell	,NoLane]
	int numMLCR[NUM_SECTION + 2][NUM_LANE]; 		// 2D Array [NoCell	,NoLane]
	int numCF[NUM_SECTION + 1][NUM_LANE];

	double speed[NUM_SECTION + 2][NUM_LANE];

} link;


/*
typedef struct {

int nodeID;
int type; 	// intersection, source, sink
int speedlimit;
int intersection_type;
} node;
*/

typedef struct {
	int ccID;
	int prevLinkID;
	int nextLinkID;

	int numVeh[NUM_LANE];
	int vehIDArr[NUM_LANE][MAX_VEC];
	int currLinkOrderArr[NUM_LANE][MAX_VEC];

	int ccNumVeh[NUM_CCLANE];						// 0~3(Straight), 4(Left), 5(Right)
	int ccVehIDArr[NUM_CCLANE][MAX_VEC];			
	int ccCurrLinkOrderArr[NUM_CCLANE][MAX_VEC];	

} connection_cell;


void Setup_Veh(vehicle*, int);
void Setup_Link(link*, int, vehicle*, int);
void Setup_ConnectionCell(connection_cell*);

__device__ void Evaluate_MLC(link*);
__device__ void Evaluate_OLC(link *l);
__device__ int Evaluate_Prob(double);
__device__ void Select_Veh(link*, int, int, int, int);
__device__ void LCSim(link*);
__device__ void MoveLC(int*, int, int*, int, int);

__device__ void Evaluate_CF(link*);
__device__ void CFsim(link*);
__device__ void MoveCF(int*, int, int*, int, int);

__device__ void Update_NextCC(link*, connection_cell*);
__device__ void Update_PrevCC(link*, connection_cell*, connection_cell*, connection_cell*, vehicle*, int);

__device__ void Update_ConnectionCell(link*, connection_cell*);
__device__ void Update_Link(link*, connection_cell*, vehicle*);
__device__ void Reset_ConnectionCell(connection_cell*);
__device__ void Reset_Link(link*);

__device__ int rand();

__global__ void SimulationStep(link*, int, connection_cell*, int, vehicle*, int, float*, float*, int, int);

void PrintAll(link l[], int);
