/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <esp_planner/environments/environment_nav2D.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/utils/2Dgridsearch.h>

#include <chrono>
#include <thread>
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

using namespace std;

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

constexpr int kCollisionCheckFakeTime = 1000000; // microseconds;
constexpr bool kVisualize = false;

int kCount = 0;

//-------------------constructor--------------------------------------------
EnvironmentNAV2DProb::EnvironmentNAV2DProb() {
  EnvNav2DProbCfg.obsthresh = ENVNAV2D_DEFAULTOBSTHRESH;

  EnvNav2DProbCfg.numofdirs = 8;
  EnvNav2DProb.bInitialized = false;

  grid2Dsearchfromgoal = NULL;
}

//-------------------problem specific and local functions---------------------

static unsigned int inthash(unsigned int key) {
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X)
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvironmentNAV2DProb::GETHASHBIN(unsigned int X1,
                                              unsigned int X2) {
  return inthash(inthash(X1) + (inthash(X2) << 1)) & (EnvNav2DProb.HashTableSize
                                                      -
                                                      1);
}

void EnvironmentNAV2DProb::PrintHashTableHist() {
  int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

  for (int j = 0; j < EnvNav2DProb.HashTableSize; j++) {
    if ((int)EnvNav2DProb.Coord2StateIDHashTable[j].size() == 0) {
      s0++;
    } else if ((int)EnvNav2DProb.Coord2StateIDHashTable[j].size() < 50) {
      s1++;
    } else if ((int)EnvNav2DProb.Coord2StateIDHashTable[j].size() < 100) {
      s50++;
    } else if ((int)EnvNav2DProb.Coord2StateIDHashTable[j].size() < 200) {
      s100++;
    } else if ((int)EnvNav2DProb.Coord2StateIDHashTable[j].size() < 300) {
      s200++;
    } else if ((int)EnvNav2DProb.Coord2StateIDHashTable[j].size() < 400) {
      s300++;
    } else {
      slarge++;
    }
  }

  SBPL_PRINTF("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n",
              s0, s1, s50, s100,
              s200, s300, slarge);
}

void EnvironmentNAV2DProb::SetConfiguration(int width, int height,
                                            const unsigned char *mapdata,
                                            const double *probabilities,
                                            const unsigned char *edge_groups,
                                            int startx, int starty,
                                            int goalx, int goaly) {
  EnvNav2DProbCfg.EnvWidth_c = width;
  EnvNav2DProbCfg.EnvHeight_c = height;
  EnvNav2DProbCfg.StartX_c = startx;
  EnvNav2DProbCfg.StartY_c = starty;
  int x;

  if (EnvNav2DProbCfg.StartX_c < 0 ||
      EnvNav2DProbCfg.StartX_c >= EnvNav2DProbCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }

  if (EnvNav2DProbCfg.StartY_c < 0 ||
      EnvNav2DProbCfg.StartY_c >= EnvNav2DProbCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }

  EnvNav2DProbCfg.EndX_c = goalx;
  EnvNav2DProbCfg.EndY_c = goaly;

  //allocate the 2D environment
  EnvNav2DProbCfg.Grid2D = new unsigned char *[EnvNav2DProbCfg.EnvWidth_c];

  for (x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
    EnvNav2DProbCfg.Grid2D[x] = new unsigned char[EnvNav2DProbCfg.EnvHeight_c];
  }

  //environment:
  if (0 == mapdata) {
    for (int y = 0; y < EnvNav2DProbCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
        EnvNav2DProbCfg.Grid2D[x][y] = 0;
      }
    }
  } else {
    for (int y = 0; y < EnvNav2DProbCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
        unsigned char cval = mapdata[x + y * width];
        EnvNav2DProbCfg.Grid2D[x][y] = cval;
      }
    }
  }

  //allocate the 2D edge group IDs.
  EnvNav2DProbCfg.GridEdgeGroup2D = new unsigned char
  *[EnvNav2DProbCfg.EnvWidth_c];

  for (x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
    EnvNav2DProbCfg.GridEdgeGroup2D[x] = new unsigned
    char[EnvNav2DProbCfg.EnvHeight_c];
  }

  //edeg_groups
  if (0 == edge_groups) {
    for (int y = 0; y < EnvNav2DProbCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
        EnvNav2DProbCfg.GridEdgeGroup2D[x][y] = 0;
      }
    }
  } else {
    for (int y = 0; y < EnvNav2DProbCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
        unsigned char eval = edge_groups[x + y * width];
        EnvNav2DProbCfg.GridEdgeGroup2D[x][y] = eval;
      }
    }
  }

  //allocate the 2D probability grid.
  EnvNav2DProbCfg.GridProb2D = new double *[EnvNav2DProbCfg.EnvWidth_c];

  for (x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
    EnvNav2DProbCfg.GridProb2D[x] = new double[EnvNav2DProbCfg.EnvHeight_c];
  }

  //probabilities
  if (0 == probabilities) {
    for (int y = 0; y < EnvNav2DProbCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
        EnvNav2DProbCfg.GridProb2D[x][y] = 0;
      }
    }
  } else {
    for (int y = 0; y < EnvNav2DProbCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
        double pval = probabilities[x + y * width];
        EnvNav2DProbCfg.GridProb2D[x][y] = pval;
      }
    }
  }

  if (kVisualize) {
    visualizer_.SetGrid(EnvNav2DProbCfg.Grid2D, EnvNav2DProbCfg.EnvHeight_c,
                        EnvNav2DProbCfg.EnvWidth_c);
    visualizer_.AddSpecialState(startx, starty, 0, 255, 0);
    visualizer_.AddSpecialState(goalx, goaly, 255, 0, 0);
  }
}

void EnvironmentNAV2DProb::ReadConfiguration(FILE *fCfg) {
  //read in the configuration of environment and initialize  EnvNav2DProbCfg structure
  char sTemp[1024], sTemp1[1024];
  int dTemp;
  int x, y;

  //discretization(cells)
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  EnvNav2DProbCfg.EnvWidth_c = atoi(sTemp);

  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  EnvNav2DProbCfg.EnvHeight_c = atoi(sTemp);

  //obsthresh:
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  strcpy(sTemp1, "obsthresh:");

  if (strcmp(sTemp1, sTemp) != 0) {
    SBPL_ERROR("ERROR: configuration file has incorrect format\n");
    SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
    throw new SBPL_Exception();
  }

  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  EnvNav2DProbCfg.obsthresh = (int)(atof(sTemp));
  SBPL_PRINTF("obsthresh = %d\n", EnvNav2DProbCfg.obsthresh);

  //start(cells):
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  EnvNav2DProbCfg.StartX_c = atoi(sTemp);

  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  EnvNav2DProbCfg.StartY_c = atoi(sTemp);

  if (EnvNav2DProbCfg.StartX_c < 0 ||
      EnvNav2DProbCfg.StartX_c >= EnvNav2DProbCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }

  if (EnvNav2DProbCfg.StartY_c < 0 ||
      EnvNav2DProbCfg.StartY_c >= EnvNav2DProbCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }

  //end(cells):
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  EnvNav2DProbCfg.EndX_c = atoi(sTemp);

  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  EnvNav2DProbCfg.EndY_c = atoi(sTemp);

  if (EnvNav2DProbCfg.EndX_c < 0 ||
      EnvNav2DProbCfg.EndX_c >= EnvNav2DProbCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal end coordinates\n");
    throw new SBPL_Exception();
  }

  if (EnvNav2DProbCfg.EndY_c < 0 ||
      EnvNav2DProbCfg.EndY_c >= EnvNav2DProbCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal end coordinates\n");
    throw new SBPL_Exception();
  }

  //allocate the 2D environment
  EnvNav2DProbCfg.Grid2D = new unsigned char *[EnvNav2DProbCfg.EnvWidth_c];

  for (x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
    EnvNav2DProbCfg.Grid2D[x] = new unsigned char[EnvNav2DProbCfg.EnvHeight_c];
  }

  //environment:
  if (fscanf(fCfg, "%s", sTemp) != 1) {
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }

  for (y = 0; y < EnvNav2DProbCfg.EnvHeight_c; y++)
    for (x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
      if (fscanf(fCfg, "%d", &dTemp) != 1) {
        SBPL_ERROR("ERROR: incorrect format of config file\n");
        throw new SBPL_Exception();
      }

      EnvNav2DProbCfg.Grid2D[x][y] = dTemp;
    }

  SBPL_PRINTF("start has cost=%d goal has cost=%d\n",
              EnvNav2DProbCfg.Grid2D[EnvNav2DProbCfg.StartX_c][EnvNav2DProbCfg.StartY_c],
              EnvNav2DProbCfg.Grid2D[EnvNav2DProbCfg.EndX_c][EnvNav2DProbCfg.EndY_c]);
}

void EnvironmentNAV2DProb::InitializeEnvConfig() {
  //aditional to configuration file initialization of EnvNav2DProbCfg if necessary

  /*
   //actions
   EnvNav2DProbCfg.dXY[0][0] = -1;
   EnvNav2DProbCfg.dXY[0][1] = -1;
   EnvNav2DProbCfg.dXY[1][0] = -1;
   EnvNav2DProbCfg.dXY[1][1] = 0;
   EnvNav2DProbCfg.dXY[2][0] = -1;
   EnvNav2DProbCfg.dXY[2][1] = 1;
   EnvNav2DProbCfg.dXY[3][0] = 0;
   EnvNav2DProbCfg.dXY[3][1] = -1;
   EnvNav2DProbCfg.dXY[4][0] = 0;
   EnvNav2DProbCfg.dXY[4][1] = 1;
   EnvNav2DProbCfg.dXY[5][0] = 1;
   EnvNav2DProbCfg.dXY[5][1] = -1;
   EnvNav2DProbCfg.dXY[6][0] = 1;
   EnvNav2DProbCfg.dXY[6][1] = 0;
   EnvNav2DProbCfg.dXY[7][0] = 1;
   EnvNav2DProbCfg.dXY[7][1] = 1;
   */
  Computedxy();
}

void EnvironmentNAV2DProb::Computedxy() {
  //initialize some constants for 2D search
  EnvNav2DProbCfg.dx_[0] = 1;
  EnvNav2DProbCfg.dy_[0] = 1;
  EnvNav2DProbCfg.dxintersects_[0][0] = 0;
  EnvNav2DProbCfg.dyintersects_[0][0] = 1;
  EnvNav2DProbCfg.dxintersects_[0][1] = 1;
  EnvNav2DProbCfg.dyintersects_[0][1] = 0;

  EnvNav2DProbCfg.dx_[1] = 1;
  EnvNav2DProbCfg.dy_[1] = 0;
  EnvNav2DProbCfg.dxintersects_[1][0] = 0;
  EnvNav2DProbCfg.dyintersects_[1][0] = 0;
  EnvNav2DProbCfg.dxintersects_[1][1] = 0;
  EnvNav2DProbCfg.dyintersects_[1][1] = 0;

  EnvNav2DProbCfg.dx_[2] = 1;
  EnvNav2DProbCfg.dy_[2] = -1;
  EnvNav2DProbCfg.dxintersects_[2][0] = 0;
  EnvNav2DProbCfg.dyintersects_[2][0] = -1;
  EnvNav2DProbCfg.dxintersects_[2][1] = 1;
  EnvNav2DProbCfg.dyintersects_[2][1] = 0;

  EnvNav2DProbCfg.dx_[3] = 0;
  EnvNav2DProbCfg.dy_[3] = 1;
  EnvNav2DProbCfg.dxintersects_[3][0] = 0;
  EnvNav2DProbCfg.dyintersects_[3][0] = 0;
  EnvNav2DProbCfg.dxintersects_[3][1] = 0;
  EnvNav2DProbCfg.dyintersects_[3][1] = 0;

  EnvNav2DProbCfg.dx_[4] = 0;
  EnvNav2DProbCfg.dy_[4] = -1;
  EnvNav2DProbCfg.dxintersects_[4][0] = 0;
  EnvNav2DProbCfg.dyintersects_[4][0] = 0;
  EnvNav2DProbCfg.dxintersects_[4][1] = 0;
  EnvNav2DProbCfg.dyintersects_[4][1] = 0;

  EnvNav2DProbCfg.dx_[5] = -1;
  EnvNav2DProbCfg.dy_[5] = 1;
  EnvNav2DProbCfg.dxintersects_[5][0] = 0;
  EnvNav2DProbCfg.dyintersects_[5][0] = 1;
  EnvNav2DProbCfg.dxintersects_[5][1] = -1;
  EnvNav2DProbCfg.dyintersects_[5][1] = 0;

  EnvNav2DProbCfg.dx_[6] = -1;
  EnvNav2DProbCfg.dy_[6] = 0;
  EnvNav2DProbCfg.dxintersects_[6][0] = 0;
  EnvNav2DProbCfg.dyintersects_[6][0] = 0;
  EnvNav2DProbCfg.dxintersects_[6][1] = 0;
  EnvNav2DProbCfg.dyintersects_[6][1] = 0;

  EnvNav2DProbCfg.dx_[7] = -1;
  EnvNav2DProbCfg.dy_[7] = -1;
  EnvNav2DProbCfg.dxintersects_[7][0] = 0;
  EnvNav2DProbCfg.dyintersects_[7][0] = -1;
  EnvNav2DProbCfg.dxintersects_[7][1] = -1;
  EnvNav2DProbCfg.dyintersects_[7][1] = 0;

  //Note: these actions have to be starting at 8 and through 15, since they
  //get multiplied correspondingly in Dijkstra's search based on index
  EnvNav2DProbCfg.dx_[8] = 2;
  EnvNav2DProbCfg.dy_[8] = 1;
  EnvNav2DProbCfg.dxintersects_[8][0] = 1;
  EnvNav2DProbCfg.dyintersects_[8][0] = 0;
  EnvNav2DProbCfg.dxintersects_[8][1] = 1;
  EnvNav2DProbCfg.dyintersects_[8][1] = 1;

  EnvNav2DProbCfg.dx_[9] = 1;
  EnvNav2DProbCfg.dy_[9] = 2;
  EnvNav2DProbCfg.dxintersects_[9][0] = 0;
  EnvNav2DProbCfg.dyintersects_[9][0] = 1;
  EnvNav2DProbCfg.dxintersects_[9][1] = 1;
  EnvNav2DProbCfg.dyintersects_[9][1] = 1;

  EnvNav2DProbCfg.dx_[10] = -1;
  EnvNav2DProbCfg.dy_[10] = 2;
  EnvNav2DProbCfg.dxintersects_[10][0] = 0;
  EnvNav2DProbCfg.dyintersects_[10][0] = 1;
  EnvNav2DProbCfg.dxintersects_[10][1] = -1;
  EnvNav2DProbCfg.dyintersects_[10][1] = 1;

  EnvNav2DProbCfg.dx_[11] = -2;
  EnvNav2DProbCfg.dy_[11] = 1;
  EnvNav2DProbCfg.dxintersects_[11][0] = -1;
  EnvNav2DProbCfg.dyintersects_[11][0] = 0;
  EnvNav2DProbCfg.dxintersects_[11][1] = -1;
  EnvNav2DProbCfg.dyintersects_[11][1] = 1;

  EnvNav2DProbCfg.dx_[12] = -2;
  EnvNav2DProbCfg.dy_[12] = -1;
  EnvNav2DProbCfg.dxintersects_[12][0] = -1;
  EnvNav2DProbCfg.dyintersects_[12][0] = 0;
  EnvNav2DProbCfg.dxintersects_[12][1] = -1;
  EnvNav2DProbCfg.dyintersects_[12][1] = -1;

  EnvNav2DProbCfg.dx_[13] = -1;
  EnvNav2DProbCfg.dy_[13] = -2;
  EnvNav2DProbCfg.dxintersects_[13][0] = 0;
  EnvNav2DProbCfg.dyintersects_[13][0] = -1;
  EnvNav2DProbCfg.dxintersects_[13][1] = -1;
  EnvNav2DProbCfg.dyintersects_[13][1] = -1;

  EnvNav2DProbCfg.dx_[14] = 1;
  EnvNav2DProbCfg.dy_[14] = -2;
  EnvNav2DProbCfg.dxintersects_[14][0] = 0;
  EnvNav2DProbCfg.dyintersects_[14][0] = -1;
  EnvNav2DProbCfg.dxintersects_[14][1] = 1;
  EnvNav2DProbCfg.dyintersects_[14][1] = -1;

  EnvNav2DProbCfg.dx_[15] = 2;
  EnvNav2DProbCfg.dy_[15] = -1;
  EnvNav2DProbCfg.dxintersects_[15][0] = 1;
  EnvNav2DProbCfg.dyintersects_[15][0] = 0;
  EnvNav2DProbCfg.dxintersects_[15][1] = 1;
  EnvNav2DProbCfg.dyintersects_[15][1] = -1;

  //compute distances
  for (int dind = 0; dind < ENVNAV2D_MAXDIRS; dind++) {
    if (EnvNav2DProbCfg.dx_[dind] != 0 && EnvNav2DProbCfg.dy_[dind] != 0) {
      if (dind <= 7)
        //the cost of a diagonal move in millimeters
      {
        EnvNav2DProbCfg.dxy_distance_mm_[dind] = (int)ceil(ENVNAV2D_COSTMULT * 1.414);
      } else
        //the cost of a move to 1,2 or 2,1 or so on in millimeters
      {
        EnvNav2DProbCfg.dxy_distance_mm_[dind] = (int)ceil(ENVNAV2D_COSTMULT * 2.236);
      }
    } else {
      EnvNav2DProbCfg.dxy_distance_mm_[dind] =
        ENVNAV2D_COSTMULT;  //the cost of a horizontal move in millimeters
    }
  }
}

EnvNav2DProbHashEntry_t *EnvironmentNAV2DProb::GetHashEntry(int X, int Y) {
#if TIME_DEBUG
  clock_t currenttime = clock();
#endif

  int binid = GETHASHBIN(X, Y);

#if DEBUG

  if ((int)EnvNav2DProb.Coord2StateIDHashTable[binid].size() > 500) {
    SBPL_PRINTF("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %u\n",
                binid, X, Y, (unsigned)EnvNav2DProb.Coord2StateIDHashTable[binid].size());

    PrintHashTableHist();
  }

#endif

  //iterate over the states in the bin and select the perfect match
  for (int ind = 0; ind < (int)EnvNav2DProb.Coord2StateIDHashTable[binid].size();
       ind++) {
    if (EnvNav2DProb.Coord2StateIDHashTable[binid][ind]->X == X &&
        EnvNav2DProb.Coord2StateIDHashTable[binid][ind]->Y == Y) {
#if TIME_DEBUG
      time_gethash += clock() - currenttime;
#endif
      return EnvNav2DProb.Coord2StateIDHashTable[binid][ind];
    }
  }

#if TIME_DEBUG
  time_gethash += clock() - currenttime;
#endif

  return NULL;
}

EnvNav2DProbHashEntry_t *EnvironmentNAV2DProb::CreateNewHashEntry(int X,
                                                                  int Y) {
  int i;

#if TIME_DEBUG
  clock_t currenttime = clock();
#endif

  EnvNav2DProbHashEntry_t *HashEntry = new EnvNav2DProbHashEntry_t;

  HashEntry->X = X;
  HashEntry->Y = Y;

  HashEntry->stateID = EnvNav2DProb.StateID2CoordTable.size();

  //insert into the tables
  EnvNav2DProb.StateID2CoordTable.push_back(HashEntry);

  //get the hash table bin
  i = GETHASHBIN(HashEntry->X, HashEntry->Y);

  //insert the entry into the bin
  EnvNav2DProb.Coord2StateIDHashTable[i].push_back(HashEntry);

  //insert into and initialize the mappings
  int *entry = new int[NUMOFINDICES_STATEID2IND];
  StateID2IndexMapping.push_back(entry);

  for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
    StateID2IndexMapping[HashEntry->stateID][i] = -1;
  }

  if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
    SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
    throw new SBPL_Exception();
  }

#if TIME_DEBUG
  time_createhash += clock() - currenttime;
#endif

  return HashEntry;
}

bool EnvironmentNAV2DProb::IsValidCell(int X, int Y) {
  return (X >= 0 && X < EnvNav2DProbCfg.EnvWidth_c && Y >= 0 &&
          Y < EnvNav2DProbCfg.EnvHeight_c &&
          EnvNav2DProbCfg.Grid2D[X][Y] < EnvNav2DProbCfg.obsthresh);
}

bool EnvironmentNAV2DProb::IsWithinMapCell(int X, int Y) {
  return (X >= 0 && X < EnvNav2DProbCfg.EnvWidth_c && Y >= 0 &&
          Y < EnvNav2DProbCfg.EnvHeight_c);
}

EnvironmentNAV2DProb::~EnvironmentNAV2DProb() {

  if (grid2Dsearchfromgoal != NULL) {
    delete grid2Dsearchfromgoal;
  }

  grid2Dsearchfromgoal = NULL;

  if (EnvNav2DProb.Coord2StateIDHashTable != NULL) {
    delete[] EnvNav2DProb.Coord2StateIDHashTable;
  }

  for (unsigned int i = 0; i < EnvNav2DProb.StateID2CoordTable.size(); ++i) {
    if (EnvNav2DProb.StateID2CoordTable[i] != NULL) {
      delete EnvNav2DProb.StateID2CoordTable[i];
    }
  }

  if (EnvNav2DProbCfg.Grid2D != NULL) {
    for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
      if (EnvNav2DProbCfg.Grid2D[x] != NULL) {
        delete[] EnvNav2DProbCfg.Grid2D[x];
      }
    }

    delete[] EnvNav2DProbCfg.Grid2D;
  }

  if (EnvNav2DProbCfg.GridProb2D != NULL) {
    for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
      if (EnvNav2DProbCfg.GridProb2D[x] != NULL) {
        delete[] EnvNav2DProbCfg.GridProb2D[x];
      }
    }

    delete[] EnvNav2DProbCfg.GridProb2D;
  }

  if (EnvNav2DProbCfg.GridEdgeGroup2D != NULL) {
    for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
      if (EnvNav2DProbCfg.GridEdgeGroup2D[x] != NULL) {
        delete[] EnvNav2DProbCfg.GridEdgeGroup2D[x];
      }
    }

    delete[] EnvNav2DProbCfg.GridEdgeGroup2D;
  }
}

void EnvironmentNAV2DProb::InitializeEnvironment() {
  EnvNav2DProbHashEntry_t *HashEntry;

  //initialize the map from Coord to StateID
  EnvNav2DProb.HashTableSize = 64 * 1024; //should be power of two
  EnvNav2DProb.Coord2StateIDHashTable = new
  vector<EnvNav2DProbHashEntry_t *> [EnvNav2DProb.HashTableSize];

  //initialize the map from StateID to Coord
  EnvNav2DProb.StateID2CoordTable.clear();

  //create start state
  if ((HashEntry = GetHashEntry(EnvNav2DProbCfg.StartX_c,
                                EnvNav2DProbCfg.StartY_c)) == NULL) {
    HashEntry = CreateNewHashEntry(EnvNav2DProbCfg.StartX_c,
                                   EnvNav2DProbCfg.StartY_c);
  }

  EnvNav2DProb.startstateid = HashEntry->stateID;

  //create goal state
  if ((HashEntry = GetHashEntry(EnvNav2DProbCfg.EndX_c,
                                EnvNav2DProbCfg.EndY_c)) == NULL) {
    HashEntry = CreateNewHashEntry(EnvNav2DProbCfg.EndX_c, EnvNav2DProbCfg.EndY_c);
  }

  EnvNav2DProb.goalstateid = HashEntry->stateID;

  EnvNav2DProb.bInitialized = true;
}

static int EuclideanDistance(int X1, int Y1, int X2, int Y2) {
  int sqdist = ((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2));
  double dist = sqrt((double)sqdist);
  return (int)(ENVNAV2D_COSTMULT * dist);
}

//generates nNumofNeighs random neighbors of cell <X,Y> at distance nDist_c (measured in cells)
//it will also generate goal if within this distance as an additional neighbor
//bSuccs is set to true if we are computing successor states, otherwise it is Preds
void EnvironmentNAV2DProb::GetRandomNeighs(int stateID,
                                           std::vector<int> *NeighIDV,
                                           std::vector<int> *CLowV,
                                           int nNumofNeighs, int nDist_c, bool bSuccs) {
  //clear the successor array
  NeighIDV->clear();
  CLowV->clear();

  //get X, Y for the states
  EnvNav2DProbHashEntry_t *HashEntry = EnvNav2DProb.StateID2CoordTable[stateID];
  int X = HashEntry->X;
  int Y = HashEntry->Y;

  //iterate through random actions
  for (int i = 0, nAttempts = 0; i < nNumofNeighs &&
       nAttempts < 5 * nNumofNeighs; i++, nAttempts++) {
    int dX = 0;
    int dY = 0;

    //pick a direction
    float fDir = (float)(2 * PI_CONST * (((double)rand()) / RAND_MAX));

    //compute the successor that result from following this direction until
    //one of the coordinates reaches the desired distance
    //decide whether |dX| = dist or |dY| = dist
    float fRadius = 0;

    if (fabs(cos(fDir)) > fabs(sin(fDir))) {
      fRadius = (float)((nDist_c + 0.5) / fabs(cos(fDir)));
    } else {
      fRadius = (float)((nDist_c + 0.5) / fabs(sin(fDir)));
    }

    dX = (int)(fRadius * cos(fDir));
    dY = (int)(fRadius * sin(fDir));

    if ((fabs((float)dX) < nDist_c && fabs((float)dY) < nDist_c) ||
        fabs((float)dX) > nDist_c ||
        fabs((float)dY) > nDist_c) {
      SBPL_ERROR("ERROR in EnvNav2D genneighs function: dX=%d dY=%d\n", dX, dY);
      throw new SBPL_Exception();
    }

    //get the coords of the state
    int newX = X + dX;
    int newY = Y + dY;

    //skip the invalid cells
    if (!IsValidCell(newX, newY)) {
      i--;
      continue;
    }

    //get the state
    EnvNav2DProbHashEntry_t *OutHashEntry;

    if ((OutHashEntry = GetHashEntry(newX, newY)) == NULL) {
      //have to create a new entry
      OutHashEntry = CreateNewHashEntry(newX, newY);
    }

    //compute clow
    int clow;

    if (bSuccs) {
      clow = GetFromToHeuristic(stateID, OutHashEntry->stateID);
    } else {
      clow = GetFromToHeuristic(OutHashEntry->stateID, stateID);
    }

    //insert it into the list
    NeighIDV->push_back(OutHashEntry->stateID);
    CLowV->push_back(clow);
  }

  //see if the goal/start belongs to the inside area and if yes then add it to Neighs as well
  int desX_c = EnvNav2DProbCfg.EndX_c;
  int desY_c = EnvNav2DProbCfg.EndY_c;
  int desstateID = EnvNav2DProb.goalstateid;

  if (bSuccs == false) {
    desX_c = EnvNav2DProbCfg.StartX_c;
    desY_c = EnvNav2DProbCfg.StartY_c;
    desstateID = EnvNav2DProb.startstateid;
  }

  //add it if within the distance
  if (abs(desX_c - X) <= nDist_c && abs(desY_c - Y) <= nDist_c) {
    //compute clow
    int clow;

    if (bSuccs) {
      clow = GetFromToHeuristic(stateID, desstateID);
    } else {
      clow = GetFromToHeuristic(desstateID, stateID);
    }

    NeighIDV->push_back(desstateID);
    CLowV->push_back(clow);
  }
}

//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentNAV2DProb::ComputeHeuristicValues() {
  //whatever necessary pre-computation of heuristic values is done here
  printf("Precomputing heuristics...\n");
  grid2Dsearchfromgoal = new SBPL2DGridSearch(EnvNav2DProbCfg.EnvWidth_c,
                                              EnvNav2DProbCfg.EnvHeight_c,
                                              0.001);

  //set OPEN type to sliding buckets
  grid2Dsearchfromgoal->setOPENdatastructure(
    SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);


  unsigned char** heur_grid = new unsigned char *[EnvNav2DProbCfg.EnvWidth_c];

  for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
    heur_grid[x] = new unsigned char[EnvNav2DProbCfg.EnvHeight_c];
  }

  for (int y = 0; y < EnvNav2DProbCfg.EnvHeight_c; y++) {
    for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
      double prob = EnvNav2DProbCfg.GridProb2D[x][y];
      if (prob > 1e-3) {
        heur_grid[x][y] = 0;
      } else {
        heur_grid[x][y] = 1;
      }
    }
  }


  grid2Dsearchfromgoal->search(heur_grid, EnvNav2DProbCfg.obsthresh,
                               EnvNav2DProbCfg.EndX_c, EnvNav2DProbCfg.EndY_c,
                               EnvNav2DProbCfg.StartX_c, EnvNav2DProbCfg.StartY_c,
                               SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
  printf("2dsolcost_infullunits=%d\n",
         (int)(ENVNAV2D_COSTMULT *
               grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(
                 EnvNav2DProbCfg.StartX_c,
                 EnvNav2DProbCfg.StartY_c)));

  for (int x = 0; x < EnvNav2DProbCfg.EnvWidth_c; x++) {
      delete[] heur_grid[x];
  }
  delete[] heur_grid;

  printf("done\n");
}

//-----------interface with outside functions-----------------------------------

bool EnvironmentNAV2DProb::InitializeEnv(const char *sEnvFile) {
  FILE *fCfg = fopen(sEnvFile, "r");

  if (fCfg == NULL) {
    SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
    throw new SBPL_Exception();
  }

  ReadConfiguration(fCfg);
  fclose(fCfg);

  InitGeneral();

  return true;
}

bool EnvironmentNAV2DProb::InitializeEnv(int width, int height,
                                         const unsigned char *mapdata, const double *probabilities,
                                         const unsigned char *edge_groups,
                                         unsigned char obsthresh) {
  return InitializeEnv(width, height, mapdata, probabilities, edge_groups, 0, 0,
                       0,
                       0, // just use (0,0) for start and goal
                       obsthresh);
}

bool EnvironmentNAV2DProb::InitializeEnv(int width, int height,
                                         const unsigned char *mapdata, const double *probabilities,
                                         const unsigned char *edge_groups,
                                         int startx, int starty,
                                         int goalx, int goaly, unsigned char obsthresh) {
  SBPL_PRINTF("env: initialized with width=%d height=%d, start=%d %d, goal=%d %d, obsthresh=%d\n",
              width, height,
              startx, starty, goalx, goaly, obsthresh);

  EnvNav2DProbCfg.obsthresh = obsthresh;

  SetConfiguration(width, height, mapdata, probabilities, edge_groups, startx,
                   starty, goalx,
                   goaly);

  InitGeneral();

  return true;
}

bool EnvironmentNAV2DProb::InitGeneral() {
  //Initialize other parameters of the environment
  InitializeEnvConfig();

  //initialize Environment
  InitializeEnvironment();

  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

bool EnvironmentNAV2DProb::InitializeMDPCfg(MDPConfig *MDPCfg) {
  //initialize MDPCfg with the start and goal ids
  MDPCfg->goalstateid = EnvNav2DProb.goalstateid;
  MDPCfg->startstateid = EnvNav2DProb.startstateid;

  return true;
}

int EnvironmentNAV2DProb::GetFromToHeuristic(int FromStateID, int ToStateID) {
#if USE_HEUR==0
  return 0;
#endif

#if DEBUG

  if (FromStateID >= (int)EnvNav2DProb.StateID2CoordTable.size() ||
      ToStateID >= (int)EnvNav2DProb.StateID2CoordTable.size()) {
    SBPL_ERROR("ERROR in EnvNav2DProb... function: stateID illegal\n");
    throw new SBPL_Exception();
  }

#endif

  //get X, Y for the state
  EnvNav2DProbHashEntry_t *FromHashEntry =
    EnvNav2DProb.StateID2CoordTable[FromStateID];
  EnvNav2DProbHashEntry_t *ToHashEntry =
    EnvNav2DProb.StateID2CoordTable[ToStateID];

  return EuclideanDistance(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X,
                           ToHashEntry->Y);
}

int EnvironmentNAV2DProb::GetGoalHeuristic(int stateID) {
#if USE_HEUR==0
  return 0;
#endif
  EnvNav2DProbHashEntry_t *HashEntry = EnvNav2DProb.StateID2CoordTable[stateID];
  int h2D = ENVNAV2D_COSTMULT *
            grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X,
                                                                    HashEntry->Y);
  return h2D;

#if DEBUG

  if (stateID >= (int)EnvNav2DProb.StateID2CoordTable.size()) {
    SBPL_ERROR("ERROR in EnvNav2DProb... function: stateID illegal\n");
    throw new SBPL_Exception();
  }

#endif

  //define this function if it used in the planner (heuristic forward search would use it)
  return GetFromToHeuristic(stateID, EnvNav2DProb.goalstateid);
}

int EnvironmentNAV2DProb::GetStartHeuristic(int stateID) {
#if USE_HEUR==0
  return 0;
#endif

#if DEBUG

  if (stateID >= (int)EnvNav2DProb.StateID2CoordTable.size()) {
    SBPL_ERROR("ERROR in EnvNav2DProb... function: stateID illegal\n");
    throw new SBPL_Exception();
  }

#endif

  //define this function if it used in the planner (heuristic backward search would use it)
  return GetFromToHeuristic(EnvNav2DProb.startstateid, stateID);
}

void EnvironmentNAV2DProb::SetAllActionsandAllOutcomes(CMDPSTATE *state) {
  int cost;

#if DEBUG

  if (state->StateID >= (int)EnvNav2DProb.StateID2CoordTable.size()) {
    SBPL_ERROR("ERROR in Env... function: stateID illegal\n");
    throw new SBPL_Exception();
  }

  if ((int)state->Actions.size() != 0) {
    SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
    throw new SBPL_Exception();
  }

#endif

  //goal state should be absorbing
  if (state->StateID == EnvNav2DProb.goalstateid) {
    return;
  }

  //get X, Y for the state
  EnvNav2DProbHashEntry_t *HashEntry =
    EnvNav2DProb.StateID2CoordTable[state->StateID];

  //iterate through actions
  bool bTestBounds = false;

  if (HashEntry->X <= 1 || HashEntry->X >= EnvNav2DProbCfg.EnvWidth_c - 2 ||
      HashEntry->Y <= 1 ||
      HashEntry->Y >= EnvNav2DProbCfg.EnvHeight_c - 2) {
    bTestBounds = true;
  }

  for (int aind = 0; aind < EnvNav2DProbCfg.numofdirs; aind++) {
    int newX = HashEntry->X + EnvNav2DProbCfg.dx_[aind];
    int newY = HashEntry->Y + EnvNav2DProbCfg.dy_[aind];

    //skip the invalid cells
    if (bTestBounds) {
      if (!IsValidCell(newX, newY)) {
        continue;
      }
    }

    //compute cost multiplier
    int costmult = EnvNav2DProbCfg.Grid2D[newX][newY];

    //for diagonal move, take max over adjacent cells
    if (newX != HashEntry->X && newY != HashEntry->Y && aind <= 7) {
      //check two more cells through which the action goes
      costmult = __max(costmult, EnvNav2DProbCfg.Grid2D[HashEntry->X][newY]);
      costmult = __max(costmult, EnvNav2DProbCfg.Grid2D[newX][HashEntry->Y]);
    } else if (aind > 7) {
      //check two more cells through which the action goes
      costmult = __max(costmult,
                       EnvNav2DProbCfg.Grid2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][0]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][0]]);
      costmult = __max(costmult,
                       EnvNav2DProbCfg.Grid2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][1]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][1]]);
    }

    //check that it is valid
    if (costmult >= EnvNav2DProbCfg.obsthresh) {
      continue;
    }

    //otherwise compute the actual cost
    cost = (costmult + 1) * EnvNav2DProbCfg.dxy_distance_mm_[aind];

    //add the action
    CMDPACTION *action = state->AddAction(aind);

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    EnvNav2DProbHashEntry_t *OutHashEntry;

    if ((OutHashEntry = GetHashEntry(newX, newY)) == NULL) {
      //have to create a new entry
      OutHashEntry = CreateNewHashEntry(newX, newY);
    }

    action->AddOutcome(OutHashEntry->stateID, cost, 1.0);

#if TIME_DEBUG
    time3_addallout += clock() - currenttime;
#endif
  }
}

void EnvironmentNAV2DProb::SetAllPreds(CMDPSTATE *state) {
  //implement this if the planner needs access to predecessors

  SBPL_ERROR("ERROR in EnvNav2DProb... function: SetAllPreds is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentNAV2DProb::GetSuccs(int SourceStateID, vector<int> *SuccIDV,
                                    vector<int> *CostV) {
  int aind;

#if TIME_DEBUG
  clock_t currenttime = clock();
#endif

  //clear the successor array
  SuccIDV->clear();
  CostV->clear();
  SuccIDV->reserve(EnvNav2DProbCfg.numofdirs);
  CostV->reserve(EnvNav2DProbCfg.numofdirs);

  //goal state should be absorbing
  if (SourceStateID == EnvNav2DProb.goalstateid) {
    return;
  }

  //get X, Y for the state
  EnvNav2DProbHashEntry_t *HashEntry =
    EnvNav2DProb.StateID2CoordTable[SourceStateID];

  //iterate through actions
  bool bTestBounds = false;

  if (HashEntry->X <= 1 || HashEntry->X >= EnvNav2DProbCfg.EnvWidth_c - 2 ||
      HashEntry->Y <= 1 ||
      HashEntry->Y >= EnvNav2DProbCfg.EnvHeight_c - 2) {
    bTestBounds = true;
  }

  for (aind = 0; aind < EnvNav2DProbCfg.numofdirs; aind++) {
    int newX = HashEntry->X + EnvNav2DProbCfg.dx_[aind];
    int newY = HashEntry->Y + EnvNav2DProbCfg.dy_[aind];

    //skip the invalid cells
    if (bTestBounds) {
      if (!IsValidCell(newX, newY)) {
        continue;
      }
    }

    int costmult = EnvNav2DProbCfg.Grid2D[newX][newY];

    //for diagonal move, take max over adjacent cells
    if (newX != HashEntry->X && newY != HashEntry->Y && aind <= 7) {
      costmult = __max(costmult, EnvNav2DProbCfg.Grid2D[HashEntry->X][newY]);
      costmult = __max(costmult, EnvNav2DProbCfg.Grid2D[newX][HashEntry->Y]);
    } else if (aind > 7) {
      //check two more cells through which the action goes
      costmult = __max(costmult,
                       EnvNav2DProbCfg.Grid2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][0]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][0]]);
      costmult = __max(costmult,
                       EnvNav2DProbCfg.Grid2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][1]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][1]]);
    }

    //check that it is valid
    if (costmult >= EnvNav2DProbCfg.obsthresh) {
      continue;
    }

    //otherwise compute the actual cost
    int cost = (costmult + 1) * EnvNav2DProbCfg.dxy_distance_mm_[aind];

    EnvNav2DProbHashEntry_t *OutHashEntry;

    if ((OutHashEntry = GetHashEntry(newX, newY)) == NULL) {
      //have to create a new entry
      OutHashEntry = CreateNewHashEntry(newX, newY);
    }

    SuccIDV->push_back(OutHashEntry->stateID);
    CostV->push_back(cost);
  }

#if TIME_DEBUG
  time_getsuccs += clock() - currenttime;
#endif
}


void EnvironmentNAV2DProb::GetLazySuccs(int SourceStateID,
                                        std::vector<int> *SuccIDV,
                                        std::vector<int> *CostV, std::vector<bool> *isTrueCost) {
  vector<double> edge_probs;
  vector<double> edge_eval_times;
  vector<int> edge_groups;
  GetSuccs(SourceStateID, SuccIDV, CostV, &edge_probs, &edge_eval_times,
           &edge_groups);
  isTrueCost->resize(SuccIDV->size(), false);
}

void EnvironmentNAV2DProb::GetSuccs(int parent_id, std::vector<int> *succ_ids,
                                    std::vector<int> *costs, std::vector<double> *edge_probabilities,
                                    std::vector<double> *edge_eval_times,
                                    std::vector<int> *edge_groups) {
  int aind;

#if TIME_DEBUG
  clock_t currenttime = clock();
#endif

  //clear the successor array
  succ_ids->clear();
  costs->clear();
  edge_probabilities->clear();
  edge_eval_times->clear();
  succ_ids->reserve(EnvNav2DProbCfg.numofdirs);
  costs->reserve(EnvNav2DProbCfg.numofdirs);
  edge_probabilities->reserve(EnvNav2DProbCfg.numofdirs);
  edge_eval_times->reserve(EnvNav2DProbCfg.numofdirs);
  edge_groups->reserve(EnvNav2DProbCfg.numofdirs);

  //goal state should be absorbing
  if (parent_id == EnvNav2DProb.goalstateid) {
    return;
  }

  //get X, Y for the state
  EnvNav2DProbHashEntry_t *HashEntry =
    EnvNav2DProb.StateID2CoordTable[parent_id];

  // Visualize
  if (kVisualize) {
    kCount++;
    visualizer_.VisualizeState(HashEntry->X, HashEntry->Y);

    if (kCount % 100 == 0) {
      visualizer_.Display(1);
    }
  }

  //iterate through actions
  bool bTestBounds = false;

  if (HashEntry->X <= 1 || HashEntry->X >= EnvNav2DProbCfg.EnvWidth_c - 2 ||
      HashEntry->Y <= 1 ||
      HashEntry->Y >= EnvNav2DProbCfg.EnvHeight_c - 2) {
    bTestBounds = true;
  }

  for (aind = 0; aind < EnvNav2DProbCfg.numofdirs; aind++) {
    int newX = HashEntry->X + EnvNav2DProbCfg.dx_[aind];
    int newY = HashEntry->Y + EnvNav2DProbCfg.dy_[aind];

    //skip the invalid cells
    if (bTestBounds) {
      if (!IsValidCell(newX, newY)) {
        continue;
      }
    }

    int costmult = EnvNav2DProbCfg.Grid2D[newX][newY];
    double probability = EnvNav2DProbCfg.GridProb2D[newX][newY];
    unsigned char edge_group_id = EnvNav2DProbCfg.GridEdgeGroup2D[newX][newY];
    double time = kCollisionCheckFakeTime;

    //for diagonal move, take max over adjacent cells
    //for edge group id, we assume that deterministic cells have the largest ID
    //possible, and that an edge does not cells with multiple stochastic group
    //ids.
    if (newX != HashEntry->X && newY != HashEntry->Y && aind <= 7) {
      costmult = __max(costmult, EnvNav2DProbCfg.Grid2D[HashEntry->X][newY]);
      costmult = __max(costmult, EnvNav2DProbCfg.Grid2D[newX][HashEntry->Y]);
      probability = std::min(probability, EnvNav2DProbCfg.GridProb2D[HashEntry->X][newY]);
      probability = std::min(probability, EnvNav2DProbCfg.GridProb2D[newX][HashEntry->Y]);
      edge_group_id = std::min(edge_group_id, EnvNav2DProbCfg.GridEdgeGroup2D[HashEntry->X][newY]);
      edge_group_id = std::min(edge_group_id, EnvNav2DProbCfg.GridEdgeGroup2D[newX][HashEntry->Y]);
    } else if (aind > 7) {
      //check two more cells through which the action goes
      costmult = __max(costmult,
                       EnvNav2DProbCfg.Grid2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][0]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][0]]);
      costmult = __max(costmult,
                       EnvNav2DProbCfg.Grid2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][1]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][1]]);
      probability = std::min(probability,
                       EnvNav2DProbCfg.GridProb2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][0]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][0]]);
      probability = std::min(probability,
                       EnvNav2DProbCfg.GridProb2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][1]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][1]]);
      edge_group_id = std::min(edge_group_id,
                       EnvNav2DProbCfg.GridEdgeGroup2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][0]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][0]]);
      edge_group_id = std::min(edge_group_id,
                       EnvNav2DProbCfg.GridEdgeGroup2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][1]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][1]]);
    }

    // Don't do collision checking here. This will be handled by GetTrueCost or
    // EvaluateEdge.
    // if (costmult >= EnvNav2DProbCfg.obsthresh) continue;

    // We still need to return the cost of the nominal action, if the edge were
    // to exist.
    // int cost = (costmult + 1) * EnvNav2DProbCfg.dxy_distance_mm_[aind];
    int cost = EnvNav2DProbCfg.dxy_distance_mm_[aind];

    EnvNav2DProbHashEntry_t *OutHashEntry;

    if ((OutHashEntry = GetHashEntry(newX, newY)) == NULL) {
      //have to create a new entry
      OutHashEntry = CreateNewHashEntry(newX, newY);
    }

    succ_ids->push_back(OutHashEntry->stateID);
    edge_probabilities->push_back(probability);
    edge_eval_times->push_back(time);
    edge_groups->push_back(static_cast<int>(edge_group_id));
    costs->push_back(cost);
  }

#if TIME_DEBUG
  time_getsuccs += clock() - currenttime;
#endif
}

int EnvironmentNAV2DProb::GetTrueCost(int parent_id, int child_id) {
  int aind = -1;
  bool valid = EvaluateEdge(parent_id, child_id, &aind);

  if (!valid) {
    return -1;
  }

  if (aind == -1) {
    printf("Action Index was not found for %d -> %d when getting true cost\n",
           parent_id, child_id);
    return -1;
  }

  return EnvNav2DProbCfg.dxy_distance_mm_[aind];
}

bool EnvironmentNAV2DProb::EvaluateEdge(int parent_id, int child_id) {
  int aind = -1;
  return EvaluateEdge(parent_id, child_id, &aind);
}

bool EnvironmentNAV2DProb::EvaluateEdge(int parent_id, int child_id,
                                        int *aind) {

  EnvNav2DProbHashEntry_t *HashEntry =
    EnvNav2DProb.StateID2CoordTable[parent_id];
  EnvNav2DProbHashEntry_t *ChildHashEntry =
    EnvNav2DProb.StateID2CoordTable[child_id];

  // Find aind
  int newX = ChildHashEntry->X;
  int newY = ChildHashEntry->Y;
  *aind = -1;

  for (int temp_aind = 0; temp_aind < EnvNav2DProbCfg.numofdirs; temp_aind++) {
    int expX = HashEntry->X + EnvNav2DProbCfg.dx_[temp_aind];
    int expY = HashEntry->Y + EnvNav2DProbCfg.dy_[temp_aind];

    if (expX == newX && expY == newY) {
      *aind = temp_aind;
      break;
    }
  }

  if (*aind == -1) {
    printf("Action Index was not found for %d -> %d\n", parent_id, child_id);
    return false;
  }

  int costmult = EnvNav2DProbCfg.Grid2D[newX][newY];
  double probability = EnvNav2DProbCfg.GridProb2D[newX][newY];
  unsigned char edge_group_id = EnvNav2DProbCfg.GridEdgeGroup2D[newX][newY];
  double time = kCollisionCheckFakeTime;

  //for diagonal move, take max over adjacent cells
  //for edge group id, we assume that deterministic cells have the largest ID
  //possible, and that an edge does not cells with multiple stochastic group
  //ids.
  if (newX != HashEntry->X && newY != HashEntry->Y && *aind <= 7) {
    costmult = __max(costmult, EnvNav2DProbCfg.Grid2D[HashEntry->X][newY]);
    costmult = __max(costmult, EnvNav2DProbCfg.Grid2D[newX][HashEntry->Y]);
    probability = std::min(probability, EnvNav2DProbCfg.GridProb2D[HashEntry->X][newY]);
    probability = std::min(probability, EnvNav2DProbCfg.GridProb2D[newX][HashEntry->Y]);
    edge_group_id = std::min(edge_group_id, EnvNav2DProbCfg.GridEdgeGroup2D[HashEntry->X][newY]);
    edge_group_id = std::min(edge_group_id, EnvNav2DProbCfg.GridEdgeGroup2D[newX][HashEntry->Y]);
  } else if (*aind > 7) {
    //check two more cells through which the action goes
    costmult = __max(costmult,
                     EnvNav2DProbCfg.Grid2D[HashEntry->X +
                                            EnvNav2DProbCfg.dxintersects_[*aind][0]][HashEntry->Y
                                                                                    + EnvNav2DProbCfg.dyintersects_[*aind][0]]);
    costmult = __max(costmult,
                     EnvNav2DProbCfg.Grid2D[HashEntry->X +
                                            EnvNav2DProbCfg.dxintersects_[*aind][1]][HashEntry->Y
                                                                                    + EnvNav2DProbCfg.dyintersects_[*aind][1]]);
    probability = std::min(probability,
                     EnvNav2DProbCfg.GridProb2D[HashEntry->X +
                                            EnvNav2DProbCfg.dxintersects_[*aind][0]][HashEntry->Y
                                                                                    + EnvNav2DProbCfg.dyintersects_[*aind][0]]);
    probability = std::min(probability,
                     EnvNav2DProbCfg.GridProb2D[HashEntry->X +
                                            EnvNav2DProbCfg.dxintersects_[*aind][1]][HashEntry->Y
                                                                                    + EnvNav2DProbCfg.dyintersects_[*aind][1]]);
    edge_group_id = std::min(edge_group_id,
                     EnvNav2DProbCfg.GridEdgeGroup2D[HashEntry->X +
                                            EnvNav2DProbCfg.dxintersects_[*aind][0]][HashEntry->Y
                                                                                    + EnvNav2DProbCfg.dyintersects_[*aind][0]]);
    edge_group_id = std::min(edge_group_id,
                     EnvNav2DProbCfg.GridEdgeGroup2D[HashEntry->X +
                                            EnvNav2DProbCfg.dxintersects_[*aind][1]][HashEntry->Y
                                                                                    + EnvNav2DProbCfg.dyintersects_[*aind][1]]);
  }

  auto it = true_cost_cache_.find(edge_group_id);
  if (it != true_cost_cache_.end()) {
    return it->second;
  }

  // Add a delay here to fake collision checking time for probabilitstic edges
  // only.
  if (probability < 1.0) {
    sleep_for(microseconds(kCollisionCheckFakeTime));
  }

  //check that it is valid
  bool is_valid = costmult < EnvNav2DProbCfg.obsthresh;
  true_cost_cache_[edge_group_id] = is_valid;
  return is_valid;
}

bool EnvironmentNAV2DProb::EdgesInSameGroup(const std::pair<int, int> &edge_1,
                                            const std::pair<int, int> &edge_2) {
  EnvNav2DProbHashEntry_t *e1_source =
    EnvNav2DProb.StateID2CoordTable[edge_1.first];
  EnvNav2DProbHashEntry_t *e1_target =
    EnvNav2DProb.StateID2CoordTable[edge_1.second];
  EnvNav2DProbHashEntry_t *e2_source =
    EnvNav2DProb.StateID2CoordTable[edge_2.first];
  EnvNav2DProbHashEntry_t *e2_target =
    EnvNav2DProb.StateID2CoordTable[edge_2.second];

  int source_dist = abs(e1_source->X - e2_source->X) + abs(
                      e1_source->Y - e2_source->Y);
  int target_dist = abs(e1_target->X - e2_target->X) + abs(
                      e1_target->Y - e2_target->Y);

  return true;

  if (source_dist < 2000 && target_dist < 200000) {
    return true;
  }

  return false;
}

void EnvironmentNAV2DProb::GetPreds(int TargetStateID, vector<int> *PredIDV,
                                    vector<int> *CostV) {
  int aind;

#if TIME_DEBUG
  clock_t currenttime = clock();
#endif

  //clear the successor array
  PredIDV->clear();
  CostV->clear();
  PredIDV->reserve(EnvNav2DProbCfg.numofdirs);
  CostV->reserve(EnvNav2DProbCfg.numofdirs);

  //get X, Y for the state
  EnvNav2DProbHashEntry_t *HashEntry =
    EnvNav2DProb.StateID2CoordTable[TargetStateID];

  //no predecessors if obstacle
  if (EnvNav2DProbCfg.Grid2D[HashEntry->X][HashEntry->Y] >=
      EnvNav2DProbCfg.obsthresh) {
    return;
  }

  int targetcostmult = EnvNav2DProbCfg.Grid2D[HashEntry->X][HashEntry->Y];

  //iterate through actions
  bool bTestBounds = false;

  if (HashEntry->X <= 1 || HashEntry->X >= EnvNav2DProbCfg.EnvWidth_c - 2 ||
      HashEntry->Y <= 1 ||
      HashEntry->Y >= EnvNav2DProbCfg.EnvHeight_c - 2) {
    bTestBounds = true;
  }

  for (aind = 0; aind < EnvNav2DProbCfg.numofdirs; aind++) {
    // the actions are undirected, so we can use the same array of actions as in getsuccs case
    int predX = HashEntry->X + EnvNav2DProbCfg.dx_[aind];
    int predY = HashEntry->Y + EnvNav2DProbCfg.dy_[aind];

    // skip the invalid cells
    if (bTestBounds) {
      if (!IsValidCell(predX, predY)) {
        continue;
      }
    }

    // compute costmult
    int costmult = targetcostmult;

    // for diagonal move, take max over adjacent cells
    if (predX != HashEntry->X && predY != HashEntry->Y && aind <= 7) {
      costmult = __max(costmult, EnvNav2DProbCfg.Grid2D[HashEntry->X][predY]);
      costmult = __max(costmult, EnvNav2DProbCfg.Grid2D[predX][HashEntry->Y]);
    } else if (aind > 7) {
      // check two more cells through which the action goes since actions
      // are undirected, we don't have to figure out what are the
      // intersecting cells on moving from <predX,predY> to <X,Y>.  it is
      // the same cells as moving from <X,Y> to <predX,predY>, which is
      // action aind
      costmult = __max(costmult,
                       EnvNav2DProbCfg.Grid2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][0]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][0]]);
      costmult = __max(costmult,
                       EnvNav2DProbCfg.Grid2D[HashEntry->X +
                                              EnvNav2DProbCfg.dxintersects_[aind][1]][HashEntry->Y
                                                                                      + EnvNav2DProbCfg.dyintersects_[aind][1]]);
    }

    // check that it is valid
    if (costmult >= EnvNav2DProbCfg.obsthresh) {
      continue;
    }

    // otherwise compute the actual cost (once again we use the fact that
    // actions are undirected to determine the cost)
    int cost = (costmult + 1) * EnvNav2DProbCfg.dxy_distance_mm_[aind];

    EnvNav2DProbHashEntry_t *OutHashEntry;

    if ((OutHashEntry = GetHashEntry(predX, predY)) == NULL) {
      // have to create a new entry
      OutHashEntry = CreateNewHashEntry(predX, predY);
    }

    PredIDV->push_back(OutHashEntry->stateID);
    CostV->push_back(cost);
  }

#if TIME_DEBUG
  time_getsuccs += clock() - currenttime;
#endif
}

int EnvironmentNAV2DProb::SizeofCreatedEnv() {
  return (int)EnvNav2DProb.StateID2CoordTable.size();
}

void EnvironmentNAV2DProb::PrintState(int stateID, bool bVerbose,
                                      FILE *fOut /*=NULL*/) {
#if DEBUG

  if (stateID >= (int)EnvNav2DProb.StateID2CoordTable.size()) {
    SBPL_ERROR("ERROR in EnvNav2DProb... function: stateID illegal (2)\n");
    throw new SBPL_Exception();
  }

#endif

  if (fOut == NULL) {
    fOut = stdout;
  }

  EnvNav2DProbHashEntry_t *HashEntry = EnvNav2DProb.StateID2CoordTable[stateID];

  if (stateID == EnvNav2DProb.goalstateid && bVerbose) {
    SBPL_FPRINTF(fOut, "the state is a goal state\n");
  }

  if (bVerbose) {
    SBPL_FPRINTF(fOut, "X=%d Y=%d\n", HashEntry->X, HashEntry->Y);
  } else {
    SBPL_FPRINTF(fOut, "%d %d\n", HashEntry->X, HashEntry->Y);
  }
}

void EnvironmentNAV2DProb::GetCoordFromState(int stateID, int &x,
                                             int &y) const {
  EnvNav2DProbHashEntry_t *HashEntry = EnvNav2DProb.StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
}

int EnvironmentNAV2DProb::GetStateFromCoord(int x, int y) {
  EnvNav2DProbHashEntry_t *OutHashEntry;

  if ((OutHashEntry = GetHashEntry(x, y)) == NULL) {
    //have to create a new entry
    OutHashEntry = CreateNewHashEntry(x, y);
  }

  return OutHashEntry->stateID;
}

const EnvNav2DProbConfig_t *EnvironmentNAV2DProb::GetEnvNavConfig() {
  return &EnvNav2DProbCfg;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV2DProb::SetGoal(int x, int y) {
  if (!IsWithinMapCell(x, y)) {
    SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n",
               x, y);
    return -1;
  }

  if (!IsValidCell(x, y)) {
    SBPL_PRINTF("WARNING: goal cell is invalid\n");
  }

  EnvNav2DProbHashEntry_t *OutHashEntry;

  if ((OutHashEntry = GetHashEntry(x, y)) == NULL) {
    //have to create a new entry
    OutHashEntry = CreateNewHashEntry(x, y);
  }

  EnvNav2DProb.goalstateid = OutHashEntry->stateID;
  EnvNav2DProbCfg.EndX_c = x;
  EnvNav2DProbCfg.EndY_c = y;

  return EnvNav2DProb.goalstateid;
}

void EnvironmentNAV2DProb::SetGoalTolerance(double tol_x, double tol_y,
                                            double tol_theta) {
  // not used yet
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV2DProb::SetStart(int x, int y) {
  if (!IsWithinMapCell(x, y)) {
    SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n",
               x, y);
    return -1;
  }

  if (!IsValidCell(x, y)) {
    SBPL_PRINTF("WARNING: start cell is invalid\n");
  }

  EnvNav2DProbHashEntry_t *OutHashEntry;

  if ((OutHashEntry = GetHashEntry(x, y)) == NULL) {
    //have to create a new entry
    OutHashEntry = CreateNewHashEntry(x, y);
  }

  EnvNav2DProb.startstateid = OutHashEntry->stateID;
  EnvNav2DProbCfg.StartX_c = x;
  EnvNav2DProbCfg.StartY_c = y;

  return EnvNav2DProb.startstateid;
}

bool EnvironmentNAV2DProb::UpdateCost(int x, int y, unsigned char newcost) {
  EnvNav2DProbCfg.Grid2D[x][y] = newcost;

  return true;
}

void EnvironmentNAV2DProb::PrintEnv_Config(FILE *fOut) {
  //implement this if the planner needs to print out EnvNav2DProb. configuration

  SBPL_ERROR("ERROR in EnvNav2DProb... function: PrintEnv_Config is undefined\n");
  throw new SBPL_Exception();
}

void EnvironmentNAV2DProb::PrintTimeStat(FILE *fOut) {
#if TIME_DEBUG
  SBPL_FPRINTF(fOut,
               "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
               time3_addallout / (double)CLOCKS_PER_SEC,
               time_gethash / (double)CLOCKS_PER_SEC,
               time_createhash / (double)CLOCKS_PER_SEC,
               time_getsuccs / (double)CLOCKS_PER_SEC);
#endif
}

void EnvironmentNAV2DProb::GetPredsofChangedEdges(vector<nav2dcell_t> const
                                                  *changedcellsV,
                                                  vector<int> *preds_of_changededgesIDV) {
  nav2dcell_t cell;
  int aind;

  for (int i = 0; i < (int)changedcellsV->size(); i++) {
    cell = changedcellsV->at(i);
    preds_of_changededgesIDV->push_back(GetStateFromCoord(cell.x, cell.y));

    for (aind = 0; aind < EnvNav2DProbCfg.numofdirs; aind++) {
      //the actions are undirected, so we can use the same array of actions as in getsuccs case
      int affx = cell.x + EnvNav2DProbCfg.dx_[aind];
      int affy = cell.y + EnvNav2DProbCfg.dy_[aind];

      if (affx < 0 || affx >= EnvNav2DProbCfg.EnvWidth_c || affy < 0 ||
          affy >= EnvNav2DProbCfg.EnvHeight_c) {
        continue;
      }

      preds_of_changededgesIDV->push_back(GetStateFromCoord(affx, affy));
    }
  }
}

// identical to GetPredsofChangedEdges except for changing "preds"
// into "succs"... can probably have just one method.
void EnvironmentNAV2DProb::GetSuccsofChangedEdges(vector<nav2dcell_t> const
                                                  *changedcellsV,
                                                  vector<int> *succs_of_changededgesIDV) {
  nav2dcell_t cell;
  int aind;

  for (int i = 0; i < (int)changedcellsV->size(); i++) {
    cell = changedcellsV->at(i);
    succs_of_changededgesIDV->push_back(GetStateFromCoord(cell.x, cell.y));

    for (aind = 0; aind < EnvNav2DProbCfg.numofdirs; aind++) {
      int affx = cell.x + EnvNav2DProbCfg.dx_[aind];
      int affy = cell.y + EnvNav2DProbCfg.dy_[aind];

      if (affx < 0 || affx >= EnvNav2DProbCfg.EnvWidth_c || affy < 0 ||
          affy >= EnvNav2DProbCfg.EnvHeight_c) {
        continue;
      }

      succs_of_changededgesIDV->push_back(GetStateFromCoord(affx, affy));
    }
  }
}

bool EnvironmentNAV2DProb::IsObstacle(int x, int y) {
  return (EnvNav2DProbCfg.Grid2D[x][y] >= EnvNav2DProbCfg.obsthresh);
}

unsigned char EnvironmentNAV2DProb::GetMapCost(int x, int y) {
  return EnvNav2DProbCfg.Grid2D[x][y];
}

void EnvironmentNAV2DProb::GetEnvParms(int *size_x, int *size_y, int *startx,
                                       int *starty, int *goalx, int *goaly,
                                       unsigned char *obsthresh) {
  *size_x = EnvNav2DProbCfg.EnvWidth_c;
  *size_y = EnvNav2DProbCfg.EnvHeight_c;

  *startx = EnvNav2DProbCfg.StartX_c;
  *starty = EnvNav2DProbCfg.StartY_c;
  *goalx = EnvNav2DProbCfg.EndX_c;
  *goaly = EnvNav2DProbCfg.EndY_c;

  *obsthresh = EnvNav2DProbCfg.obsthresh;
}

bool EnvironmentNAV2DProb::SetEnvParameter(const char *parameter, int value) {
  if (EnvNav2DProb.bInitialized == true) {
    SBPL_ERROR("ERROR: all parameters must be set before initialization of the environment\n");
    return false;
  }

  SBPL_PRINTF("setting parameter %s to %d\n", parameter, value);

  if (strcmp(parameter, "is16connected") == 0) {
    if (value != 0) {
      EnvNav2DProbCfg.numofdirs = 16;
    } else {
      EnvNav2DProbCfg.numofdirs = 8;
    }
  } else {
    SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
    return false;
  }

  return true;
}

//returns true if two states meet the same condition - see environment.h for more info
bool EnvironmentNAV2DProb::AreEquivalent(int StateID1, int StateID2) {
#if DEBUG

  if (StateID1 >= (int)EnvNav2DProb.StateID2CoordTable.size() ||
      StateID2 >= (int)EnvNav2DProb.StateID2CoordTable.size()) {
    SBPL_ERROR("ERROR in EnvNav2DProb... function: stateID illegal (2)\n");
    throw new SBPL_Exception();
  }

#endif

  //get X, Y for the states
  EnvNav2DProbHashEntry_t *HashEntry1 =
    EnvNav2DProb.StateID2CoordTable[StateID1];
  EnvNav2DProbHashEntry_t *HashEntry2 =
    EnvNav2DProb.StateID2CoordTable[StateID2];

  if (HashEntry1->X == HashEntry2->X && HashEntry1->Y == HashEntry2->Y) {
    return true;
  }

  return false;
}

//generate succs at some domain-dependent distance - see environment.h for more info
void EnvironmentNAV2DProb::GetRandomSuccsatDistance(int SourceStateID,
                                                    std::vector<int> *SuccIDV, std::vector<int> *CLowV) {
  //number of random neighbors
  int nNumofNeighs = 10;
  //distance at which the neighbors are generated
  int nDist_c = 100;

#if DEBUG

  if (SourceStateID >= (int)EnvNav2DProb.StateID2CoordTable.size()) {
    SBPL_ERROR("ERROR in EnvNav2DProbGetRandSuccs... function: stateID illegal\n");
    throw new SBPL_Exception();
  }

#endif

  //goal state should be absorbing
  if (SourceStateID == EnvNav2DProb.goalstateid) {
    return;
  }

  //get the successors
  bool bSuccs = true;
  GetRandomNeighs(SourceStateID, SuccIDV, CLowV, nNumofNeighs, nDist_c, bSuccs);
}

//generate preds at some domain-dependent distance - see environment.h for more info
void EnvironmentNAV2DProb::GetRandomPredsatDistance(int TargetStateID,
                                                    std::vector<int> *PredIDV, std::vector<int> *CLowV) {
  //number of random neighbors
  int nNumofNeighs = 10;
  //distance at which the neighbors are generated
  int nDist_c = 5; //TODO-was100;

#if DEBUG

  if (TargetStateID >= (int)EnvNav2DProb.StateID2CoordTable.size()) {
    SBPL_ERROR("ERROR in EnvNav2DProbGetRandSuccs... function: stateID illegal\n");
    throw new SBPL_Exception();
  }

#endif

  //start state does not have start state
  if (TargetStateID == EnvNav2DProb.startstateid) {
    return;
  }

  //get the predecessors
  bool bSuccs = false;
  GetRandomNeighs(TargetStateID, PredIDV, CLowV, nNumofNeighs, nDist_c, bSuccs);
}

//------------------------------------------------------------------------------
