//#ifndef __CUDACC__ 
//#define __CUDACC__
//#endif

#pragma once
#ifdef __INTELLISENSE__
void __syncthreads();
#endif

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <device_functions.h>
#include <curand_kernel.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "TrafficSim_Parallel.h"

/*--------------------------------------------------------------------*/
/// @fn      void Setup_Veh(vehicle*, int)
/// @brief   Function that setup the values of struct vehicle.
/// @param   vehicle* v, int numVeh
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_Veh(vehicle* v, int numVeh) {
	/// (1) Add inputs to vArr in order to create new vehicle.
	for (int i = 0; i < 20; i++) {
		v[i].vehType = 0;
		v[i].vehID = i;
		v[i].path[0] = 0;
		v[i].path[0] = 1;
		v[i].lenPath = 2;
		v[i].currLink = 0;
	}

	/// vTL[] = currLane, currSection
	int vPos[20][2] = {
		{ 0, 0 },{ 0, 1 },{ 0, 2 },{ 0, 3 },
		{ 1, 0 },{ 1, 1 },{ 1, 2 },{ 1, 3 },
		{ 2, 0 },{ 2, 1 },{ 2, 2 },{ 2, 3 },
		{ 3, 0 },{ 3, 1 },{ 3, 2 },{ 3, 3 },
		{ 0, 0 },{ 0, 1 },{ 0, 2 },{ 0, 3 } };

	/// 
	for (int i = 0; i < 20; i++) {
		v[i].currLane = vPos[i][0];
		v[i].currSection = vPos[i][1];
	}

	/// vTL[] = minTargetLane[0], maxTargetLane[0], minTargetLane[1], maxTargetLane[1]
	int vTL[20][4] = {
		{ 0, 0, 0, 0 },{ 1, 1, 1, 1 },{ 2, 2, 2, 2 },{ 3, 3, 3, 3 },
		{ 0, 1, 0, 0 },{ 0, 1, 0, 1 },{ 0, 1, 1, 2 },{ 0, 1, 2, 3 },
		{ 0, 1, 3, 3 },{ 1, 2, 0, 0 },{ 1, 2, 0, 1 },{ 1, 2, 1, 2 },
		{ 1, 2, 2, 3 },{ 1, 2, 3, 3 },{ 2, 3, 0, 0 },{ 2, 3, 0, 1 },
		{ 2, 3, 1, 2 },{ 2, 3, 2, 3 },{ 2, 3, 3, 3 },{ 0, 0, 3, 3 } };

	/// 
	for (int i = 0; i < 20; i++) {
		v[i].minTargetLane[0] = vTL[i][0];
		v[i].maxTargetLane[0] = vTL[i][1];
		v[i].minTargetLane[1] = vTL[i][2];
		v[i].maxTargetLane[1] = vTL[i][3];
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Setup_Link(link*, int, vehicle*, int)
/// @brief   Function that setup the values of struct link.
/// @param   link* l, int numLink, vehicle* v, int numVeh
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_Link(link* l, int numLink, vehicle* v, int numVeh) {
	/// (1) Add inputs to vArr in order to create new vehicle.
	/// lArr[] = [linkID, maxNumVeh, maxNumCF, ffSpeed, lenSection, prevCCID[L,R,S], nextCCID;
	int lArr[4][9] = { { 0, 20, 3, 27, 100, 2, 3, 1, 0 },{ 1, 20, 3, 27, 100, 0, 0, 0, 1 },
					    { 2, 20, 3, 27, 100, 0, 0, 0, 2 },{ 3, 20, 3, 27, 100, 0, 0, 0, 3 } };

	/// (2) Assign value of struct link using values of lArr and struct vehicle.
	for (int i = 0; i < 4; i++) {
		l[i].linkID = lArr[i][0];
		l[i].ffSpeed = lArr[i][3];
		l[i].prevCCID[0] = lArr[i][5];
		l[i].prevCCID[1] = lArr[i][6];
		l[i].prevCCID[2] = lArr[i][7];
		l[i].nextCCID = lArr[i][8];

		for (int cell = 0; cell < NUM_SECTION + 2; cell++) {
			l[i].lenSection[cell] = lArr[i][4];

			for (int lane = 0; lane < NUM_LANE; lane++) {
				l[i].maxNumVeh[cell][lane] = lArr[i][1];
				l[i].maxNumCF[cell][lane] = lArr[i][2];
			}
		}

		///
		for (int cell = 0; cell < NUM_SECTION + 2; cell++) {
			for (int lane = 0; lane < NUM_LANE; lane++) {
				l[i].numVeh[cell][lane] = 0;
				l[i].numMLCL[cell][lane] = 0;
				l[i].numMLCR[cell][lane] = 0;
				l[i].numCF[cell][lane] = 0;
				l[i].speed[cell][lane] = 0;

				for (int j = 0; j < numVeh; j++) {
					l[i].vehIDArr[cell][lane][j] = 0;
					l[i].currLinkOrderArr[cell][lane][j] = 0;
					l[i].minTargetLaneArr[cell][lane][j] = 0;
					l[i].maxTargetLaneArr[cell][lane][j] = 0;
					l[i].vehMLC[cell][lane][j] = 0;
					l[i].vehOLC[cell][lane][j] = 0;
					l[i].vehCF[cell][lane][j] = 0;
				}
			}
		}

		/// 
		for (int j = 0; j < numVeh; j++) {
			if (l[i].linkID == v[j].currLink) {
				int p = v[j].currSection;
				int q = v[j].currLane;
				int r = l[i].numVeh[p][q];

				l[i].vehIDArr[p][q][r] = v[j].vehID;
				l[i].currLinkOrderArr[p][q][r] = 0;
				l[i].minTargetLaneArr[p][q][r] = v[j].minTargetLane[0];
				l[i].maxTargetLaneArr[p][q][r] = v[j].maxTargetLane[0];
				l[i].speed[p][q] = l[i].ffSpeed;
				l[i].numVeh[p][q]++;
			}
		}
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Setup_ConnectionCell(connection_cell*)
/// @brief   Function that setup the values of struct connection_cell.
/// @param   connection_cell* cc
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_ConnectionCell(connection_cell* cc) {
	cc[0].ccID = 0;
	cc[0].prevLinkID = 0;
	cc[0].nextLinkID = 1;

	cc[1].ccID = 1;
	cc[1].prevLinkID = 1;
	cc[1].nextLinkID = 0;

	cc[2].ccID = 2;
	cc[2].prevLinkID = 2;
	cc[2].nextLinkID = 0;

	cc[3].ccID = 3;
	cc[3].prevLinkID = 3;
	cc[3].nextLinkID = 0;

	for (int lane = 0; lane < NUM_LANE - 1; lane++) {
		cc[0].numVeh[lane] = 0;
		for (int i = 0; i < MAX_VEC - 1; i++) {
			cc[0].vehIDArr[lane][i] = 0;
			cc[0].currLinkOrderArr[lane][i] = 0;
		}
	}

	for (int lane = 0; lane < NUM_LANE - 1; lane++) {
		cc[1].numVeh[lane] = 0;
		for (int i = 0; i < MAX_VEC - 1; i++) {
			cc[1].vehIDArr[lane][i] = 0;
			cc[1].currLinkOrderArr[lane][i] = 0;
		}
	}

	for (int lane = 0; lane < NUM_LANE - 1; lane++) {
		cc[2].numVeh[lane] = 0;
		for (int i = 0; i < MAX_VEC - 1; i++) {
			cc[1].vehIDArr[lane][i] = 0;
			cc[1].currLinkOrderArr[lane][i] = 0;
		}
	}
	for (int lane = 0; lane < NUM_LANE - 1; lane++) {
		cc[3].numVeh[lane] = 0;
		for (int i = 0; i < MAX_VEC - 1; i++) {
			cc[1].vehIDArr[lane][i] = 0;
			cc[1].currLinkOrderArr[lane][i] = 0;
		}
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void Evaluate_MLC(link, int)
/// @brief   Function that evaluate Mandatory Lane Change of a vehicle
///          and updates vehMLC Flag.
/// @param   link *l, int numLink
/// @return  None
/*--------------------------------------------------------------------*/
__device__ void Evaluate_MLC(link *l) {
	for (int cell = 0; cell < NUM_SECTION + 2; cell++) {
		for (int lane = 0; lane < NUM_LANE; lane++) {
			for (int i = 0; i < l->numVeh[cell][lane]; i++) {
				int minTL = l->minTargetLaneArr[cell][lane][i];
				int maxTL = l->maxTargetLaneArr[cell][lane][i];

				/// (1) Compare current lane with target lane and determine
				///     and update the value of vehMLC Flag.
				/// If vehicle should move left, set vehMLC Flag to -1
				if (lane > maxTL) {
					l->vehMLC[cell][lane][i] = -1;
					l->numMLCL[cell][lane]++;
				}
				/// If vehicle should move left, set vehMLC Flag to +1
				else if (lane < minTL) {
					l->vehMLC[cell][lane][i] = 1;
					l->numMLCR[cell][lane]++;
				}
				else {
					l->vehMLC[cell][lane][i] = 0;
				}
			}
		}
	}
}

/*--------------------------------------------------------------------*/
/// @fn      int Evaluate_Prob(double)
/// @brief   Function that randomly returns integer part or 
///          (integer part+1) of a rational number.
/// @param   double inputProb
/// @return  intPart+1 when random > probPart
///          intPart  when random <= probPart
/*--------------------------------------------------------------------*/
__device__ int Evaluate_Prob(double inputProb) {
	int intPart = (int)inputProb;
	double probPart = inputProb - (double)intPart;

	double random = ((rand() % 10) / 10.);

	return random > probPart ? (intPart + 1) : intPart;
}

/*--------------------------------------------------------------------*/
/// @fn      void Select_Veh()
/// @brief   Function that select vehicles of Optional Lane Change.
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void Select_Veh(link* l, int numOLC_L, int numOLC_R, int cell, int lane) {
	int numVeh = l->numVeh[cell][lane];
	int possOLC[MAX_VEC] = { 0 };

	/// (1) Exclude vehMLC from candidates that can OLC.
	for (int i = 0; i < numVeh; i++) {
		if (l->vehMLC[cell][lane][i] != 0) possOLC[i] = 0;
		else possOLC[i] = 2;
	}

	/// (2) Consider when current lane is either maximum or minimum target lane.
	for (int i = 0; i < numVeh; i++) {
		int minTL = l->minTargetLaneArr[cell][lane][i];
		int maxTL = l->maxTargetLaneArr[cell][lane][i];

		if (lane == minTL) {
			if (possOLC[i] == 2) possOLC[i] = 1;
			else if (possOLC[i] == -1) possOLC[i] = 0;
		}
		else if (lane == maxTL) {
			if (possOLC[i] == 2) possOLC[i] = -1;
			else if (possOLC[i] == 1) possOLC[i] = 0;
		}
	}

	/// (3) Calculate number of vehicles that can go left, right, or both.
	int possBoth = 0;
	int possLeft = 0;
	int possRight = 0;
	int possLeftArr[MAX_VEC] = { 0 };
	int possRightArr[MAX_VEC] = { 0 };
	for (int i = 0; i < numVeh; i++) {
		if (possOLC[i] == 2) {
			possLeftArr[possLeft] = i;
			possRightArr[possRight] = i;
			possLeft++;
			possRight++;
			possBoth++;
		}
		else if (possOLC[i] == -1) {
			possLeftArr[possLeft] = i;
			possLeft++;
		}
		else if (possOLC[i] == 1) {
			possRightArr[possRight] = i;
			possRight++;
		}
	}

	/// (4) Consider when number of OLC is larger than possible vehicle of OLC
	if (possLeft < numOLC_L) numOLC_L = possLeft;
	if (possRight < numOLC_R) numOLC_R = possRight;

	int possTotal = possLeft + possRight - possBoth;
	while (possTotal < numOLC_L + numOLC_R) {
		numOLC_L--;
		numOLC_R--;
	}

	/// (5) Update values of vehOLC flags.
	int count_R = numOLC_R;
	int count_L = numOLC_L;
	if (numOLC_L == 0 && numOLC_R == 0);

	else if (numOLC_L == 0) {
		while (count_R) {
			int randVeh = rand() % numOLC_R;
			if (l->vehOLC[cell][lane][possRightArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possRightArr[randVeh]] = 1;
				count_R--;
			}
		}
	}

	else if (numOLC_R == 0) {
		while (count_L) {
			int randVeh = rand() % numOLC_L;
			if (l->vehOLC[cell][lane][possLeftArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possLeftArr[randVeh]] = -1;
				count_L--;
			}
		}
	}

	else if ((possLeft / numOLC_L) > (possRight / numOLC_R)) {
		while (count_R) {
			int randVeh = rand() % numOLC_R;
			if (l->vehOLC[cell][lane][possRightArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possRightArr[randVeh]] = 1;
				count_R--;
			}
		}

		while (count_L) {
			int randVeh = rand() % numOLC_L;
			if (l->vehOLC[cell][lane][possLeftArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possLeftArr[randVeh]] = -1;
				count_L--;
			}
		}
	}

	else if ((possLeft / numOLC_L) <= (possRight / numOLC_R)) {
		while (count_L) {
			int randVeh = rand() % numOLC_L;
			if (l->vehOLC[cell][lane][possLeftArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possLeftArr[randVeh]] = -1;
				count_L--;
			}
		}

		while (count_R) {
			int randVeh = rand() % numOLC_R;
			if (l->vehOLC[cell][lane][possRightArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possRightArr[randVeh]] = 1;
				count_R--;
			}
		}
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void Evaluate_OLC(link*)
/// @brief   Function that evaluate Optional Lane Change of a vehicle
///          and updates vehOLC Flag.
/// @param   link *l
/// @return  None
/*--------------------------------------------------------------------*/
__device__ void Evaluate_OLC(link *l) {
	for (int cell = 0; cell < NUM_SECTION + 2; cell++) {
		for (int lane = 0; lane < NUM_LANE; lane++) {
			int numMLC_L = l->numMLCL[cell][lane];
			int numMLC_R = l->numMLCR[cell][lane];
			double probLC_L, probLC_R;
			int diffSpeed_L = l->speed[cell][lane - 1] - l->speed[cell][lane];
			int diffSpeed_R = l->speed[cell][lane + 1] - l->speed[cell][lane];

			/// (1) Set probLC to zero in special cases. 
			/// When current lane is leftmost, probLC_L = 0
			/// In other words, the vehicle cannot move left.
			if (lane == 0) probLC_L = 0;
			else probLC_L = (diffSpeed_L / l->ffSpeed) * l->numVeh[cell][lane];

			/// When current lane is rightmost, probLC_R = 0
			/// In other words, the vehicle cannot move right.
			if (lane == NUM_LANE - 1) probLC_R = 0;
			else probLC_R = (diffSpeed_R / l->ffSpeed) * l->numVeh[cell][lane];

			/// (2) Evaluate number of OLC by subtrating number of MLC
			///     from the total number of LC.
			int numOLC_L = Evaluate_Prob(probLC_L) - numMLC_L;
			int numOLC_R = Evaluate_Prob(probLC_R) - numMLC_R;

			/// numOLC cannot be smaller than zero.
			if (numOLC_L < 0) numOLC_L = 0;
			if (numOLC_R < 0) numOLC_R = 0;

			/// numOLC cannot be bigger than total number of vehicle.
			int numLC = numOLC_L + numOLC_R + numMLC_L + numMLC_R;
			if (numLC > l->numVeh[cell][lane]) numLC = l->numVeh[cell][lane];

			/// (3) Select vehicle to change lane.
			Select_Veh(l, numOLC_L, numOLC_R, cell, lane);
		}
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void MoveLC()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void MoveLC(int* fromArr, int fromArrSize, int* toArr, int toArrSize, int index) {
	double fromArrLoc = ((double)fromArrSize / (index + 1));
	int toArrLoc = Evaluate_Prob(toArrSize / fromArrLoc);

	for (int i = MAX_VEC - 1; i > toArrLoc; i--) {
		toArr[i] = toArr[i - 1];
	}
	toArr[toArrLoc] = fromArr[index];

	for (int i = index; i < MAX_VEC; i++) {
		fromArr[i] = fromArr[i + 1];
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void LCSim()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void LCSim(link* l) {
	for (int cell = 0; cell < NUM_SECTION + 2; cell++) {
		for (int lane = 0; lane < NUM_LANE; lane++) {
			for (int i = 0; i < MAX_VEC; i++) {
				if (l->vehMLC[cell][lane][i] == 1) {
					if (l->numVeh[cell][lane + 1] < MAX_VEC) {
						MoveLC(l->vehIDArr[cell][lane], l->numVeh[cell][lane],
							l->vehIDArr[cell][lane + 1], l->numVeh[cell][lane + 1], i + 1);
						MoveLC(l->currLinkOrderArr[cell][lane], l->numVeh[cell][lane],
							l->currLinkOrderArr[cell][lane + 1], l->numVeh[cell][lane + 1], i);
						MoveLC(l->minTargetLaneArr[cell][lane], l->numVeh[cell][lane],
							l->minTargetLaneArr[cell][lane + 1], l->numVeh[cell][lane + 1], i);
						MoveLC(l->maxTargetLaneArr[cell][lane], l->numVeh[cell][lane],
							l->maxTargetLaneArr[cell][lane + 1], l->numVeh[cell][lane + 1], i);
						l->numVeh[cell][lane + 1]++;
						l->numVeh[cell][lane]--;
					}
				}

				else if (l->vehMLC[cell][lane][i] == -1) {
					if (l->numVeh[cell][lane - 1] < MAX_VEC) {
						MoveLC(l->vehIDArr[cell][lane], l->numVeh[cell][lane],
							l->vehIDArr[cell][lane - 1], l->numVeh[cell][lane - 1], i);
						MoveLC(l->currLinkOrderArr[cell][lane], l->numVeh[cell][lane],
							l->currLinkOrderArr[cell][lane - 1], l->numVeh[cell][lane - 1], i);
						MoveLC(l->minTargetLaneArr[cell][lane], l->numVeh[cell][lane],
							l->minTargetLaneArr[cell][lane - 1], l->numVeh[cell][lane - 1], i);
						MoveLC(l->maxTargetLaneArr[cell][lane], l->numVeh[cell][lane],
							l->maxTargetLaneArr[cell][lane - 1], l->numVeh[cell][lane - 1], i);
						l->numVeh[cell][lane - 1]++;
						l->numVeh[cell][lane]--;
					}
				}
			}
		}
	}

	for (int cell = 0; cell < NUM_SECTION + 2; cell++) {
		for (int lane = 0; lane < NUM_LANE; lane++) {
			for (int i = 0; i < MAX_VEC; i++) {
				if (l->vehOLC[cell][lane][i] == 1 && l->numVeh[cell][lane] < MAX_VEC) {
					MoveLC(l->vehIDArr[cell][lane], l->numVeh[cell][lane],
						l->vehIDArr[cell][lane + 1], l->numVeh[cell][lane + 1], i);
					MoveLC(l->currLinkOrderArr[cell][lane], l->numVeh[cell][lane],
						l->currLinkOrderArr[cell][lane + 1], l->numVeh[cell][lane + 1], i);
					MoveLC(l->minTargetLaneArr[cell][lane], l->numVeh[cell][lane],
						l->minTargetLaneArr[cell][lane + 1], l->numVeh[cell][lane + 1], i);
					MoveLC(l->maxTargetLaneArr[cell][lane], l->numVeh[cell][lane],
						l->maxTargetLaneArr[cell][lane + 1], l->numVeh[cell][lane + 1], i);
					l->numVeh[cell][lane + 1]++;
					l->numVeh[cell][lane]--;
				}
				else if (l->vehOLC[cell][lane][i] == -1 && l->numVeh[cell][lane] < MAX_VEC) {
					MoveLC(l->vehIDArr[cell][lane], l->numVeh[cell][lane],
						l->vehIDArr[cell][lane - 1], l->numVeh[cell][lane - 1], i);
					MoveLC(l->currLinkOrderArr[cell][lane], l->numVeh[cell][lane],
						l->currLinkOrderArr[cell][lane - 1], l->numVeh[cell][lane - 1], i);
					MoveLC(l->minTargetLaneArr[cell][lane], l->numVeh[cell][lane],
						l->minTargetLaneArr[cell][lane - 1], l->numVeh[cell][lane - 1], i);
					MoveLC(l->maxTargetLaneArr[cell][lane], l->numVeh[cell][lane],
						l->maxTargetLaneArr[cell][lane - 1], l->numVeh[cell][lane - 1], i);
					l->numVeh[cell][lane - 1]++;
					l->numVeh[cell][lane]--;
				}
			}
		}
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void Evaluate_CF()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void Evaluate_CF(link* l) {
	double wSpeed = 15;
	//double length = 4;

	for (int cell = 0; cell < NUM_SECTION + 1; cell++) {
		for (int lane = 0; lane < NUM_LANE; lane++) {
			l->numCF[cell][lane] =
				//MIN(MIN((l->ffSpeed / 3.6 * dt) / l->lenSection[cell] * l->numVeh[cell][lane], l->maxNumCF[cell][lane]), 
				//	MIN(l->maxNumCF[cell+1][lane], wSpeed * dt / length * (l->maxNumCF[cell][lane] - l->numVeh[cell][lane])));   
				MIN(l->numVeh[cell][lane], MIN(l->maxNumCF[cell][lane], wSpeed / l->ffSpeed * (l->maxNumCF[cell + 1][lane] - l->numVeh[cell + 1][lane])));

			printf("numCF: %d\n", l->numCF[cell][lane]);

			for (int i = 0; i < l->numCF[cell][lane]; i++) {
				l->vehCF[cell][lane][i] = 1;
			}
		}
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void MoveCF()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void MoveCF(int* fromArr, int fromArrSize, int* toArr, int toArrSize, int index) {
	toArr[toArrSize] = fromArr[index];

	for (int i = MAX_VEC - 1; i >= 0; i--) {
		fromArr[i] = fromArr[i - 1];
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void CFsim()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void CFsim(link *l) {
	for (int cell = NUM_SECTION; cell >= 0; cell--) {
		for (int lane = 0; lane < NUM_LANE; lane++) {
			for (int i = 0; i < MAX_VEC; i++) {
				
				// Straight 처리만 있으므로 Left, Right에 대한 CF 처리 필요
				// Left, Right일 경우, 아래 변수들에 저장: Array 순서 [left, Right]
				// l->nextNumVeh[2], l->nextVehIDArr[2], l->nextCurrLinkOrderArr[2]

				if (l->vehCF[cell][lane][i] == 1 && l->numVeh[cell + 1][lane] < MAX_VEC) {
					MoveCF(l->vehIDArr[cell][lane], l->numVeh[cell][lane],
						l->vehIDArr[cell + 1][lane], l->numVeh[cell + 1][lane], i);
					MoveCF(l->currLinkOrderArr[cell][lane], l->numVeh[cell][lane],
						l->currLinkOrderArr[cell + 1][lane], l->numVeh[cell + 1][lane], i);
					MoveCF(l->minTargetLaneArr[cell][lane], l->numVeh[cell][lane],
						l->minTargetLaneArr[cell + 1][lane], l->numVeh[cell + 1][lane], i);
					MoveCF(l->maxTargetLaneArr[cell][lane], l->numVeh[cell][lane],
						l->maxTargetLaneArr[cell + 1][lane], l->numVeh[cell + 1][lane], i);
					l->numVeh[cell + 1][lane]++;
					l->numVeh[cell][lane]--;					
				}
			}
		}
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void Update_NextLink()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void Update_NextCC(link* l, connection_cell* cc) {
	for (int lane = 0; lane < NUM_CCLANE; lane++) {
		if (lane == 4)	//Left
			cc->ccNumVeh[lane] = l->nextNumVeh[0];
		else if (lane == 5)	//Right
			cc->ccNumVeh[lane] = l->nextNumVeh[1];
		else //straight
			cc->ccNumVeh[lane] = l->numVeh[NUM_SECTION + 1][lane];

		for (int i = 0; i < MAX_VEC; i++) {
			if (lane == 4) {	//Left
				cc->ccVehIDArr[lane][i] = l->nextVehIDArr[0][i];
				cc->ccCurrLinkOrderArr[lane][i] = l->nextCurrLinkOrderArr[0][i];
			}
			else if (lane == 5) {	//Right
				cc->ccVehIDArr[lane][i] = l->nextVehIDArr[1][i];
				cc->ccCurrLinkOrderArr[lane][i] = l->nextCurrLinkOrderArr[1][i];
			}
			else { //straight
				cc->ccVehIDArr[lane][i] = l->vehIDArr[NUM_SECTION + 1][lane][i];
				cc->ccCurrLinkOrderArr[lane][i] = l->currLinkOrderArr[NUM_SECTION + 1][lane][i];
			}
		}
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void Update_PrevLink()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void Update_PrevCC(link* l, connection_cell* ccl, connection_cell* ccr, connection_cell* ccs, vehicle* v, int numCC) {
	for (int lane = 0; lane < NUM_CCLANE; lane++) {
		
		// Left, Right에서 들어오는 차량의 경우, 직진 차량과의 합류(sum) 처리 필요
		// 현재 그냥 overwrite 처리
		
		for (int i = 0; i < MAX_VEC; i++) {
			if (lane == 4) {	//Left
				int currNumVeh = l->numVeh[0][0];
				int currOrder = ccl->ccCurrLinkOrderArr[lane][i];
				int currVehID = ccl->ccVehIDArr[lane][i];

				l->vehIDArr[0][0][currNumVeh + i] = ccl->ccVehIDArr[lane][i];
				l->minTargetLaneArr[0][0][currNumVeh + i] = v[currVehID].minTargetLane[currOrder + 1];
				l->maxTargetLaneArr[0][0][currNumVeh + i] = v[currVehID].maxTargetLane[currOrder + 1];
			}
			else if (lane == 5) {	//Right
				int currNumVeh = l->numVeh[0][3];
				int currOrder = ccr->ccCurrLinkOrderArr[lane][i];
				int currVehID = ccr->ccVehIDArr[lane][i];

				l->vehIDArr[0][3][currNumVeh + i] = ccr->ccVehIDArr[lane][i];
				l->minTargetLaneArr[0][3][currNumVeh + i] = v[currVehID].minTargetLane[currOrder + 1];
				l->maxTargetLaneArr[0][3][currNumVeh + i] = v[currVehID].maxTargetLane[currOrder + 1];
			}
			else { //straight
				int currNumVeh = l->numVeh[0][lane];
				int currOrder = ccs->ccCurrLinkOrderArr[lane][i];
				int currVehID = ccs->ccVehIDArr[lane][i];

				l->vehIDArr[0][lane][currNumVeh + i] = ccs->ccVehIDArr[lane][i];
				l->minTargetLaneArr[0][lane][currNumVeh + i] = v[currVehID].minTargetLane[currOrder + 1];
				l->maxTargetLaneArr[0][lane][currNumVeh + i] = v[currVehID].maxTargetLane[currOrder + 1];
			}
		}

		if (lane == 4) 	//Left
			l->numVeh[0][0] += ccl->ccNumVeh[lane];
		else if (lane == 5) //Right
			l->numVeh[0][3] += ccr->ccNumVeh[lane];
		else  //straight
			l->numVeh[0][lane] += ccs->ccNumVeh[lane];
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void Update_ConnectCell()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void Update_ConnectionCell(link* prevl, connection_cell* cc) {
	for (int lane = 0; lane < NUM_LANE; lane++) {
		cc->numVeh[lane] = prevl->numVeh[NUM_SECTION + 1][lane];

		for (int i = 0; i < MAX_VEC; i++) {
			cc->vehIDArr[lane][i] = prevl->vehIDArr[NUM_SECTION + 1][lane][i];
			cc->currLinkOrderArr[lane][i] = prevl->currLinkOrderArr[NUM_SECTION + 1][lane][i];
		}
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void Update_Link()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void Update_Link(link* nextl, connection_cell* cc, vehicle* v) {
	for (int lane = 0; lane < NUM_LANE; lane++) {
		for (int i = 0; i < MAX_VEC; i++) {
			int currNumVeh = nextl->numVeh[0][lane];
			int currOrder = cc->currLinkOrderArr[lane][i];
			int currVehID = cc->vehIDArr[lane][i];

			nextl->vehIDArr[0][lane][currNumVeh + i] = cc->vehIDArr[lane][i];
			nextl->minTargetLaneArr[0][lane][currNumVeh + i] = v[currVehID].minTargetLane[currOrder + 1];
			nextl->maxTargetLaneArr[0][lane][currNumVeh + i] = v[currVehID].maxTargetLane[currOrder + 1];
		}

		nextl->numVeh[0][lane] += cc->numVeh[lane];
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void Reset_ConnectionCell()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void Reset_ConnectionCell(connection_cell* cc) {
	for (int lane = 0; lane < NUM_LANE; lane++) {
		cc->numVeh[lane] = 0;

		for (int i = 0; i < MAX_VEC; i++) {
			cc->vehIDArr[lane][i] = 0;
			cc->currLinkOrderArr[lane][i] = 0;
		}
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Reset_Link()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__device__ void Reset_Link(link* l) {
	double wSpeed = 15;

	for (int cell = 0; cell < NUM_SECTION + 2; cell++) {
		for (int lane = 0; lane < NUM_LANE; lane++) {
			int jamdensity = l->maxNumVeh[cell][lane] / l->lenSection[cell];  //도로가 막히기 시작하는 density (veh/km), 링크 특성(고정값)
			int density = l->numVeh[cell][lane] / l->lenSection[cell];  //도로의 현재 density (veh/km), 시뮬레이션 스텝마다 업데이트

			l->speed[cell][lane] =
				MAX(0, MIN(l->ffSpeed, (-wSpeed + (wSpeed * jamdensity / density))));
			l->numMLCL[cell][lane] = 0;
			l->numMLCR[cell][lane] = 0;
			l->numCF[cell][lane] = 0;

			l->numVeh[NUM_SECTION + 1][lane] = 0;

			for (int i = 0; i < MAX_VEC; i++) {
				l->vehMLC[cell][lane][i] = 0;
				l->vehOLC[cell][lane][i] = 0;
				l->vehCF[cell][lane][i] = 0;

				l->vehIDArr[NUM_SECTION + 1][lane][i] = 0;
			}
		}
	}
}

__device__ int rand() {
	int data = 0;
	int size = 1;
		
	// Create a pseudo-random number generator
	curandGenerator_t generator;
	curandCreateGenerator(&generator, CURAND_RNG_PSEUDO_DEFAULT);

	// Set the seed for the random number generator using the system clock
	curandSetPseudoRandomGeneratorSeed(generator, 1234ULL);

	// Fill the array with random numbers on the device
	//int
	curandGenerate(generator, (unsigned int*)data, size);
	//Float
	//curandGenerateUniform(generator, data, size);

	return data;
}

void PrintAll(link l[], int numLink) {
	for (int link = 0; link < numLink; link++) {
		for (int cell = 0; cell < NUM_SECTION + 2; cell++) {
			for (int lane = 0; lane < NUM_LANE; lane++) {
				printf("link: %d, cell: %d, lane: %d, numVeh: %d \n",
					link, cell, lane, l[link].numVeh[cell][lane]);
			}
		}
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void SimulationStep()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
__global__ void SimulationStep
(link *l, int numLink, connection_cell *cc, int numCC, vehicle *v, int numVeh, float *n, float *y, int numElements, int loop_limit) {
	int tid = threadIdx.x;
	int idx = blockIdx.x * blockDim.x + tid;

	if (idx < numLink) {

		for (int count = 0; count < loop_limit; count++) {
			
			int link = idx;

			//for (int link = 0; link < numLink; link++) 
			{				
				//PrintAll(&l[link],myveh,vehn);
				Evaluate_MLC(&l[link]);
				Evaluate_OLC(&l[link]);

				LCSim(&l[link]);

				Evaluate_CF(&l[link]);

				CFsim(&l[link]);

				// Link에 연결된 CC에 Update 수행
				Update_NextCC(&l[link], &cc[l[link].nextCCID]);	

				// Left, Right, Straignt 각 방향의 CC로부터 Update 수행
				Update_PrevCC(&l[link], &cc[l[link].prevCCID[0]], &cc[l[link].prevCCID[1]], &cc[l[link].prevCCID[2]], v, numCC);

				// 추가 수정해야 함
				//Reset_ConnectionCell(&cc[cell])
			}
			
			/*
			for (int cell = 0; cell < numCC; cell++) {
				int prev = cc[cell].prevLinkID;
				int next = cc[cell].nextLinkID;

				printf("prev: %d\n", prev);
				printf("next: %d\n", next);

				Update_ConnectionCell(&l[prev], &cc[cell]);
				Update_Link(&l[next], &cc[cell], v);
				Reset_ConnectionCell(&cc[cell]);
			}
			*/
			//for (int link = 0; link < numLink; link++) 
			{
				Reset_Link(&l[link]);
			}			

			__syncthreads();
		}
	}
}

int main(int argc, char *argv[]) {

	srand(time(NULL));

	/////////////////////////////////////////////////////////////////////////////////////
	// HOST : initial data
	int numVeh = (int)atoi(argv[1]); // number of links
	int numLink = (int)atoi(argv[2]); // number of vehicle
	int numCC = (int)atoi(argv[3]);
	int loop_limit = (int)atoi(argv[4]); //number of periods

	printf("vehicle arguments: %s\n", argv[1]);
	printf("link arguments: %s\n", argv[2]);
	printf("connection cell arguments: %s\n", argv[3]);
	printf("loop count arguments: %s\n", argv[4]);

	vehicle* myveh = (vehicle*)calloc(numVeh, sizeof(vehicle));
	link* mylink = (link*)calloc(numLink, sizeof(link));
	//node mynode = (node*) calloc(n,sizeof(node));
	connection_cell* mycon = (connection_cell*)malloc(numLink * sizeof(connection_cell));

	Setup_Veh(myveh, numVeh);
	Setup_Link(mylink, numLink, myveh, numVeh);
	Setup_ConnectionCell(mycon);

	vehicle *dev_veh;
	link *dev_link;
	connection_cell *dev_con;
	float *dev_n, *host_n;
	float *dev_y, *host_y;

	host_n = (float *)malloc(sizeof(float) * numLink * 16);
	host_y = (float *)malloc(sizeof(float) * numLink * 16);

	/////////////////////////////////////////////////////////////////////////////////////
	// Copy data from host to device
	cudaMalloc((void**)&dev_veh, numVeh * sizeof(vehicle));
	cudaMalloc((void**)&dev_link, numLink * sizeof(link));
	cudaMalloc((void**)&dev_con, numLink * sizeof(connection_cell));
	cudaMalloc((void**)&dev_n, sizeof(float) * numLink * 16);
	cudaMalloc((void**)&dev_y, sizeof(float) * numLink * 16);

	cudaMemcpy(dev_veh, myveh, numVeh * sizeof(vehicle), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_link, mylink, numLink * sizeof(link), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_con, mycon, numLink * sizeof(connection_cell), cudaMemcpyHostToDevice);

	/////////////////////////////////////////////////////////////////////////////////////
	// Device Properties
	int nDevices;
	cudaGetDeviceCount(&nDevices);
	for (int i = 0; i < nDevices; i++) {
		cudaDeviceProp prop;
		cudaGetDeviceProperties(&prop, i);
		printf("\nDevice Number: %d\n", i);
		printf("  Device name: %s\n", prop.name);
		printf("  Memory Clock Rate (KHz): %d\n",
			prop.memoryClockRate);
		printf("  Memory Bus Width (bits): %d\n",
			prop.memoryBusWidth);
		printf("  Peak Memory Bandwidth (GB/s): %f\n",
			2.0*prop.memoryClockRate*(prop.memoryBusWidth / 8) / 1.0e6);
	}

	//float elapsed = 0;
	float Elapsed;
	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);

	// Number of Threads and Blocks 설정
	int numSMs;
	int devId = 0;
	cudaDeviceGetAttribute(&numSMs, cudaDevAttrMultiProcessorCount, devId);
	printf("\nnumSMs = %d \n", numSMs);//16 
	printf("\gridDim.x = %d \n", 32 * numSMs);//16
	int threadsPerBlock = 16;
	int blocksPerGird = (numLink + threadsPerBlock - 1) / threadsPerBlock;

	int numElements = NUM_LANE * NUM_SECTION * numLink;
	int threadsInBlock = 32;
	int blocksInGrid = (numElements + threadsPerBlock - 1) / threadsPerBlock;
	printf("\nthreadsInBlock = %d \n", threadsInBlock);
	printf("blocksInGrid = %d \n\n", blocksInGrid);

	/////////////////////////////////////////////////////////////////////////////////////
	// Call Kernel Function
	printf("Start Kernel...\n");
	cudaEventRecord(start, 0);
	SimulationStep << <blocksInGrid, threadsInBlock >> >(dev_link, numLink, dev_con, numCC, dev_veh, numVeh, dev_n, dev_y, numElements, loop_limit);
	//SimulationStep << <blocksPerGird, threadsPerBlock >> >(dev_link, dev_n, dev_y, n, loop_limit);
	//SimulationStep << <32 * numSMs, 16>> >(dev_link, dev_n, dev_y, n, loop_limit);
	cudaEventRecord(stop, 0);
	
	/////////////////////////////////////////////////////////////////////////////////////
	// Copy data from device to host
	cudaMemcpy(myveh, dev_veh, numVeh * sizeof(vehicle), cudaMemcpyHostToDevice);
	cudaMemcpy(mylink, dev_link, numLink * sizeof(link), cudaMemcpyDeviceToHost);
	cudaMemcpy(mycon, dev_con, numLink * sizeof(connection_cell), cudaMemcpyHostToDevice);
	cudaMemcpy(host_n, dev_n, numLink * sizeof(float) * 16, cudaMemcpyDeviceToHost);
	cudaMemcpy(host_y, dev_y, numLink * sizeof(float) * 16, cudaMemcpyDeviceToHost);
	
	cudaDeviceSynchronize();
	printf("End Kernel...\n");
	
	cudaEventElapsedTime(&Elapsed, start, stop);
	printf("\nElapsed Time(cuda):\t%.2f ms\n", Elapsed);

	cudaEventDestroy(start);
	cudaEventDestroy(stop);
	
	cudaFree(dev_veh);
	cudaFree(dev_link);
	cudaFree(dev_con);
	cudaFree(dev_n);
	cudaFree(dev_y);
	
	free(host_n);
	free(host_y);
	free(myveh);
	free(mylink);
	free(mycon);

	return 0;
}