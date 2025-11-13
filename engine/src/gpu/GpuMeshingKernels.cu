#include "engine/gpu/GpuMeshingKernels.h"
#include <nanovdb/NanoVDB.h>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/system/cuda/execution_policy.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <chrono>
#include <thread>

namespace Urbaxio::Engine {

// Marching Cubes edge table (256 entries)
// Indicates which edges are intersected for each cube configuration
__constant__ int d_edgeTable[256];

// Marching Cubes triangle table (256 x 16 entries)
// Defines triangles for each cube configuration
__constant__ int d_triTable[256][16];

// Host-side lookup tables (initialized once)
static bool s_tablesInitialized = false;

// Complete Marching Cubes edge table (256 entries)
static const int h_edgeTable[256] = {
    0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
    0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
    0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
    0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
    0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
    0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
    0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
    0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
    0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
    0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
    0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
    0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
    0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
    0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
    0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
    0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
    0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
    0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
    0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
    0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
    0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
    0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0   
};

// Complete Marching Cubes triangle table (256 x 16 entries)
static const int h_triTable[256][16] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
};

// ============================================================================
// NEW: Extract leaf origins kernel
// ============================================================================
__global__ void ExtractOriginsKernel(
    const nanovdb::FloatGrid* grid,
    nanovdb::Coord* outOrigins,
    uint64_t leafCount)
{
    uint64_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= leafCount) return;

    auto* leaf = grid->tree().getFirstNode<0>() + idx;
    outOrigins[idx] = leaf->origin();
}

// ============================================================================
// SPARSE MARCHING CUBES: Step 1 - Count triangles per CELL, then sum per leaf
// ============================================================================
__global__ void CountTrianglesPerCellKernel(
    const nanovdb::FloatGrid* grid,
    float isoValue,
    uint64_t leafCount,
    int* triangleCountsPerCell,  // Output: size = leafCount * 343
    const nanovdb::Coord* leafOrigins  // NEW!
)
{
    // Each thread processes ONE cell (same as Generate kernel)
    uint64_t globalCellIdx = blockIdx.x * blockDim.x + threadIdx.x;
    uint64_t totalCells = leafCount * 343;
    
    if (globalCellIdx >= totalCells) return;
    
    uint64_t leafIdx = globalCellIdx / 343;
    int cellIdxInLeaf = globalCellIdx % 343;
    
    nanovdb::Coord leafOrigin = leafOrigins[leafIdx];
    auto acc = grid->tree().getAccessor();
    
    int cx = cellIdxInLeaf % 7;
    int cy = (cellIdxInLeaf / 7) % 7;
    int cz = cellIdxInLeaf / 49;
    
    // Sample 8 corners with standard Marching Cubes vertex order
    const int vertexOrder[8][3] = {
        {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
        {0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}
    };
    float v[8];
    for (int i = 0; i < 8; ++i) {
        int gx = leafOrigin[0] + cx + vertexOrder[i][0];
        int gy = leafOrigin[1] + cy + vertexOrder[i][1];
        int gz = leafOrigin[2] + cz + vertexOrder[i][2];
        
        v[i] = acc.getValue(nanovdb::Coord(gx, gy, gz));
    }
    
    // Calculate cube index
    int cubeIndex = 0;
    if (v[0] < isoValue) cubeIndex |= 1;
    if (v[1] < isoValue) cubeIndex |= 2;
    if (v[2] < isoValue) cubeIndex |= 4;
    if (v[3] < isoValue) cubeIndex |= 8;
    if (v[4] < isoValue) cubeIndex |= 16;
    if (v[5] < isoValue) cubeIndex |= 32;
    if (v[6] < isoValue) cubeIndex |= 64;
    if (v[7] < isoValue) cubeIndex |= 128;
    
    // Count triangles for this cell
    int numTriangles = 0;
    if (cubeIndex != 0 && cubeIndex != 255) {
        for (int i = 0; i < 16 && d_triTable[cubeIndex][i] != -1; i += 3) {
            numTriangles++;
        }
    }
    
    triangleCountsPerCell[globalCellIdx] = numTriangles;
}

// ============================================================================
// SPARSE MARCHING CUBES: Step 1.5 - Sum per-cell counts to per-leaf counts
// ============================================================================
__global__ void SumCellsToLeavesKernel(
    const int* countsPerCell,
    int* countsPerLeaf,
    uint64_t leafCount)
{
    // Each block reduces one leaf's cell counts.
    uint64_t leafIdx = blockIdx.x;
    if (leafIdx >= leafCount) return;

    extern __shared__ int s_data[];

    const int* my_cells = countsPerCell + leafIdx * 343;
    
    int thread_sum = 0;
    // Each thread in the block sums a portion of the 343 cells for this leaf.
    for (int i = threadIdx.x; i < 343; i += blockDim.x) {
        thread_sum += my_cells[i];
    }
    s_data[threadIdx.x] = thread_sum;

    __syncthreads();

    // Standard parallel reduction in shared memory
    for (unsigned int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (threadIdx.x < s) {
            s_data[threadIdx.x] += s_data[threadIdx.x + s];
        }
        __syncthreads();
    }

    // Thread 0 writes the final sum for this leaf.
    if (threadIdx.x == 0) {
        countsPerLeaf[leafIdx] = s_data[0];
    }
}

// ============================================================================
// SPARSE MARCHING CUBES: Step 3 - Generate triangles
// ============================================================================
__device__ float3 VertexInterp(
    float isoValue,
    float3 p1, float valp1,
    float3 p2, float valp2)
{
    if (fabsf(isoValue - valp1) < 0.00001f) return p1;
    if (fabsf(isoValue - valp2) < 0.00001f) return p2;
    if (fabsf(valp1 - valp2) < 0.00001f) return p1;
    
    float mu = (isoValue - valp1) / (valp2 - valp1);
    return make_float3(
        p1.x + mu * (p2.x - p1.x),
        p1.y + mu * (p2.y - p1.y),
        p1.z + mu * (p2.z - p1.z)
    );
}

// ============================================================================
// SPARSE MARCHING CUBES: Step 3 - Generate triangles (ATOMIC VERSION - CORRECT)
// ============================================================================
__global__ void GenerateTrianglesKernel(
    nanovdb::FloatGrid* grid,
    float isoValue,
    uint64_t leafCount,
    const int* triangleOffsetsPerLeaf,
    const int* triangleCountsPerLeaf,
    int* atomicCountersPerLeaf,
    float* outVertices,
    float* outNormals,
    const nanovdb::Coord* leafOrigins  // NEW!
)
{
    uint64_t globalCellIdx = blockIdx.x * blockDim.x + threadIdx.x;
    uint64_t totalCells = leafCount * 343;
    
    if (globalCellIdx >= totalCells) return;
    
    uint64_t leafIdx = globalCellIdx / 343;
    int cellIdxInLeaf = globalCellIdx % 343;
    
    nanovdb::Coord leafOrigin = leafOrigins[leafIdx];
    auto acc = grid->tree().getAccessor();
    
    // DEBUG: Print first 5 leaves
    if (globalCellIdx < 5 && threadIdx.x == 0) {
        printf("[KERNEL] Cell %llu: leafIdx=%llu, leafOrigin=(%d,%d,%d)\n",
               globalCellIdx, leafIdx, 
               leafOrigin[0], leafOrigin[1], leafOrigin[2]);
    }
    
    int cx = cellIdxInLeaf % 7;
    int cy = (cellIdxInLeaf / 7) % 7;
    int cz = cellIdxInLeaf / 49;
    
    // Sample 8 corners with standard Marching Cubes vertex order
    const int vertexOrder[8][3] = {
        {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
        {0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}
    };
    float v[8];
    float3 p[8];
    
    for (int i = 0; i < 8; ++i) {
        int gx = leafOrigin[0] + cx + vertexOrder[i][0];
        int gy = leafOrigin[1] + cy + vertexOrder[i][1];
        int gz = leafOrigin[2] + cz + vertexOrder[i][2];
        
        // DEBUG: Print coordinates for cell 0
        if (globalCellIdx == 0 && i == 0) {
            printf("[KERNEL] Cell 0, corner 0: voxel=(%d,%d,%d)\n", gx, gy, gz);
        }
        
        // FIX: Use grid transform instead of manual voxelSize multiplication
        nanovdb::Vec3f worldPos = grid->indexToWorldF(nanovdb::Vec3f(gx, gy, gz));
        
        if (globalCellIdx == 0 && i == 0) {
            printf("[KERNEL] Cell 0, corner 0: world=(%f,%f,%f)\n", 
                   worldPos[0], worldPos[1], worldPos[2]);
        }
        
        p[i] = make_float3(worldPos[0], worldPos[1], worldPos[2]);
        
        v[i] = acc.getValue(nanovdb::Coord(gx, gy, gz));
    }
    
    // Calculate cube index
    int cubeIndex = 0;
    if (v[0] < isoValue) cubeIndex |= 1;
    if (v[1] < isoValue) cubeIndex |= 2;
    if (v[2] < isoValue) cubeIndex |= 4;
    if (v[3] < isoValue) cubeIndex |= 8;
    if (v[4] < isoValue) cubeIndex |= 16;
    if (v[5] < isoValue) cubeIndex |= 32;
    if (v[6] < isoValue) cubeIndex |= 64;
    if (v[7] < isoValue) cubeIndex |= 128;
    
    if (cubeIndex == 0 || cubeIndex == 255) return;
    
    // Count triangles for this cell BEFORE reserving space
    int numTriangles = 0;
    for (int i = 0; d_triTable[cubeIndex][i] != -1; i += 3) {
        numTriangles++;
    }
    
    if (numTriangles == 0) return;

    // Edge vertices
    float3 vertList[12];
    if (d_edgeTable[cubeIndex] & 1) vertList[0] = VertexInterp(isoValue, p[0], v[0], p[1], v[1]);
    if (d_edgeTable[cubeIndex] & 2) vertList[1] = VertexInterp(isoValue, p[1], v[1], p[2], v[2]);
    if (d_edgeTable[cubeIndex] & 4) vertList[2] = VertexInterp(isoValue, p[2], v[2], p[3], v[3]);
    if (d_edgeTable[cubeIndex] & 8) vertList[3] = VertexInterp(isoValue, p[3], v[3], p[0], v[0]);
    if (d_edgeTable[cubeIndex] & 16) vertList[4] = VertexInterp(isoValue, p[4], v[4], p[5], v[5]);
    if (d_edgeTable[cubeIndex] & 32) vertList[5] = VertexInterp(isoValue, p[5], v[5], p[6], v[6]);
    if (d_edgeTable[cubeIndex] & 64) vertList[6] = VertexInterp(isoValue, p[6], v[6], p[7], v[7]);
    if (d_edgeTable[cubeIndex] & 128) vertList[7] = VertexInterp(isoValue, p[7], v[7], p[4], v[4]);
    if (d_edgeTable[cubeIndex] & 256) vertList[8] = VertexInterp(isoValue, p[0], v[0], p[4], v[4]);
    if (d_edgeTable[cubeIndex] & 512) vertList[9] = VertexInterp(isoValue, p[1], v[1], p[5], v[5]);
    if (d_edgeTable[cubeIndex] & 1024) vertList[10] = VertexInterp(isoValue, p[2], v[2], p[6], v[6]);
    if (d_edgeTable[cubeIndex] & 2048) vertList[11] = VertexInterp(isoValue, p[3], v[3], p[7], v[7]);
    
    // FIX: Atomically reserve a CONTIGUOUS BLOCK for all triangles from this cell
    int localOffset = atomicAdd(&atomicCountersPerLeaf[leafIdx], numTriangles);
    int baseOffset = triangleOffsetsPerLeaf[leafIdx];
    
    // Bounds check for the entire block
    if (localOffset + numTriangles > triangleCountsPerLeaf[leafIdx]) return;
    
    // Generate triangles
    int triCounter = 0;
    for (int i = 0; d_triTable[cubeIndex][i] != -1; i += 3) {
        int globalOffset = baseOffset + localOffset + triCounter;
        
        float3 v0 = vertList[d_triTable[cubeIndex][i]];
        float3 v1 = vertList[d_triTable[cubeIndex][i+1]];
        float3 v2 = vertList[d_triTable[cubeIndex][i+2]];
        
        // Write vertices (swap v1/v2 to fix winding for SDF)
        outVertices[globalOffset * 9 + 0] = v0.x;
        outVertices[globalOffset * 9 + 1] = v0.y;
        outVertices[globalOffset * 9 + 2] = v0.z;
        outVertices[globalOffset * 9 + 3] = v2.x;
        outVertices[globalOffset * 9 + 4] = v2.y;
        outVertices[globalOffset * 9 + 5] = v2.z;
        outVertices[globalOffset * 9 + 6] = v1.x;
        outVertices[globalOffset * 9 + 7] = v1.y;
        outVertices[globalOffset * 9 + 8] = v1.z;
        
        // Calculate normal (swap edges to match winding)
        float3 edge1 = make_float3(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        float3 edge2 = make_float3(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        float3 normal = make_float3(
            edge1.y * edge2.z - edge1.z * edge2.y,
            edge1.z * edge2.x - edge1.x * edge2.z,
            edge1.x * edge2.y - edge1.y * edge2.x
        );
        float len = sqrtf(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
        if (len > 0.0001f) {
            normal.x /= len; normal.y /= len; normal.z /= len;
        }
        
        // Write normals
        for (int j = 0; j < 3; ++j) {
            outNormals[globalOffset * 9 + j*3 + 0] = normal.x;
            outNormals[globalOffset * 9 + j*3 + 1] = normal.y;
            outNormals[globalOffset * 9 + j*3 + 2] = normal.z;
        }
        
        triCounter++;
    }
}

void GpuMeshingKernels::InitializeLookupTables() {
    if (s_tablesInitialized) return;

    std::cout << "[GpuMeshingKernels] Initializing lookup tables..." << std::endl;
    
    // Check table sizes
    std::cout << "  h_edgeTable size: " << sizeof(h_edgeTable) << " bytes (expected: " << (256 * sizeof(int)) << ")" << std::endl;
    std::cout << "  h_triTable size: " << sizeof(h_triTable) << " bytes (expected: " << (256 * 16 * sizeof(int)) << ")" << std::endl;
    
    // Copy lookup tables to GPU constant memory
    cudaError_t err1 = cudaMemcpyToSymbol(d_edgeTable, h_edgeTable, sizeof(h_edgeTable));
    cudaError_t err2 = cudaMemcpyToSymbol(d_triTable, h_triTable, sizeof(h_triTable));
    
    if (err1 != cudaSuccess) {
        std::cerr << "[GpuMeshingKernels] ❌ Failed to copy edgeTable: " << cudaGetErrorString(err1) << std::endl;
    }
    if (err2 != cudaSuccess) {
        std::cerr << "[GpuMeshingKernels] ❌ Failed to copy triTable: " << cudaGetErrorString(err2) << std::endl;
    }

    s_tablesInitialized = true;
    std::cout << "[GpuMeshingKernels] ✅ Lookup tables initialized" << std::endl;
}

bool GpuMeshingKernels::MarchingCubes(
    void* deviceGridPtr,
    uint64_t leafCount,  // NEW: Pass as parameter
    float isoValue,
    float voxelSize,
    CadKernel::MeshBuffers& outMesh)
{
    if (!deviceGridPtr || leafCount == 0) {
        std::cerr << "[GpuMeshingKernels] Invalid parameters!" << std::endl;
        return false;
    }

    InitializeLookupTables();

    auto* grid = reinterpret_cast<nanovdb::FloatGrid*>(deviceGridPtr);
    std::cout << "[GpuMeshingKernels] Processing " << leafCount << " leaves..." << std::endl;
    // ========================================================================
    // STEP 0: Extract leaf origins (one-time cost)
    // ========================================================================
    std::cout << "[DEBUG] Step 0: Extracting leaf origins..." << std::endl;

    thrust::device_vector<nanovdb::Coord> d_leafOrigins(leafCount);

    dim3 extractBlockSize(256);
    dim3 extractGridSize((leafCount + 255) / 256);

    ExtractOriginsKernel<<<extractGridSize, extractBlockSize>>>(
        grid,
        thrust::raw_pointer_cast(d_leafOrigins.data()),
        leafCount
    );

    cudaError_t extractErr = cudaGetLastError();
    if (extractErr != cudaSuccess) {
        std::cerr << "[GpuMeshingKernels] Extract origins error: " 
                  << cudaGetErrorString(extractErr) << std::endl;
        return false;
    }
    cudaDeviceSynchronize();

    std::cout << "[DEBUG] Leaf origins extracted successfully" << std::endl;

    // ========================================================================
    // STEP 1: Count triangles per CELL
    // ========================================================================
    const uint64_t numCells = leafCount * 343;  // FIX: Renamed to avoid conflict
    thrust::device_vector<int> d_triangleCountsPerCell(numCells);
    
    dim3 countBlockSize(256);
    dim3 countGridSize((numCells + 255) / 256);
    
    std::cout << "[DEBUG] Step 1: Counting triangles per cell..." << std::endl;
    std::cout << "  numCells = " << numCells << std::endl;
    
    CountTrianglesPerCellKernel<<<countGridSize, countBlockSize>>>(
        grid,
        isoValue,
        leafCount,
        thrust::raw_pointer_cast(d_triangleCountsPerCell.data()),
        thrust::raw_pointer_cast(d_leafOrigins.data())  // NEW!
    );
    
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[GpuMeshingKernels] Count kernel error: " 
                  << cudaGetErrorString(err) << std::endl;
        return false;
    }
    cudaDeviceSynchronize();
    
    // ========================================================================
    // STEP 1.5: Sum per-cell counts to get per-leaf counts (GPU KERNEL)
    // ========================================================================
    std::cout << "[DEBUG] Step 1.5: Summing per-cell to per-leaf (GPU kernel)..." << std::endl;
    
    thrust::device_vector<int> d_triangleCounts(leafCount);
    dim3 sumBlockSize(256); // A reasonable number of threads for reduction
    dim3 sumGridSize(leafCount); // One block per leaf
    size_t sharedMemSize = sumBlockSize.x * sizeof(int);
    
    SumCellsToLeavesKernel<<<sumGridSize, sumBlockSize, sharedMemSize>>>(
        thrust::raw_pointer_cast(d_triangleCountsPerCell.data()),
        thrust::raw_pointer_cast(d_triangleCounts.data()),
        leafCount
    );
    
    cudaError_t sumErr = cudaGetLastError();
    if (sumErr != cudaSuccess) {
        std::cerr << "[GpuMeshingKernels] Summation kernel error: " 
                  << cudaGetErrorString(sumErr) << std::endl;
        return false;
    }
    cudaDeviceSynchronize();
    std::cout << "[DEBUG] Per-leaf counts computed via GPU kernel" << std::endl;

    // ========================================================================
    // STEP 2: Prefix sum to get triangle offsets
    // ========================================================================
    thrust::device_vector<int> d_triangleOffsets(leafCount);
    thrust::exclusive_scan(
        d_triangleCounts.begin(), 
        d_triangleCounts.end(), 
        d_triangleOffsets.begin()
    );
    
    // Get total triangle count
    int lastCount = 0;
    int lastOffset = 0;
    if (leafCount > 0) {
        cudaMemcpy(&lastCount, 
                   thrust::raw_pointer_cast(d_triangleCounts.data()) + leafCount - 1,
                   sizeof(int), cudaMemcpyDeviceToHost);
        cudaMemcpy(&lastOffset,
                   thrust::raw_pointer_cast(d_triangleOffsets.data()) + leafCount - 1,
                   sizeof(int), cudaMemcpyDeviceToHost);
    }
    
    int totalTriangles = lastOffset + lastCount;
    
    if (totalTriangles == 0) {
        std::cout << "[GpuMeshingKernels] No surface found (no zero crossings)" << std::endl;
        return true;
    }
    
    // NEW: Create atomic counter array (one per leaf, initialized to 0)
    thrust::device_vector<int> d_atomicCounters(leafCount, 0);
    
    std::cout << "[DEBUG] Created atomic counters for " << leafCount << " leaves" << std::endl;
    
    std::cout << "[GpuMeshingKernels] Generating " << totalTriangles << " triangles..." << std::endl;
    // ========================================================================
    // STEP 3: Allocate output buffers and generate triangles
    // ========================================================================
    std::cout << "[DEBUG] Allocating output buffers..." << std::endl;
    std::cout << "  totalTriangles = " << totalTriangles << std::endl;
    std::cout << "  Buffer size = " << (totalTriangles * 9) << " floats = " 
              << (totalTriangles * 9 * sizeof(float) / 1024.0 / 1024.0) << " MB" << std::endl;
    
    thrust::device_vector<float> d_vertices(totalTriangles * 9);
    thrust::device_vector<float> d_normals(totalTriangles * 9);
    
    std::cout << "[DEBUG] Buffers allocated successfully" << std::endl;
    
    // FIX: Launch with enough threads to cover all cells
    dim3 genBlockSize(256);
    dim3 genGridSize((numCells + 255) / 256);
    
    std::cout << "[DEBUG] Launching GenerateTrianglesKernel:" << std::endl;
    std::cout << "  numCells = " << numCells << std::endl;
    std::cout << "  genGridSize = " << genGridSize.x << std::endl;
    std::cout << "  genBlockSize = " << genBlockSize.x << std::endl;
    std::cout << "  Total threads = " << (genGridSize.x * genBlockSize.x) << std::endl;
    std::cout << "  Output buffer size (vertices) = " << (totalTriangles * 9) << " floats" << std::endl;
    std::cout << "  Output buffer size (normals) = " << (totalTriangles * 9) << " floats" << std::endl;
    
    GenerateTrianglesKernel<<<genGridSize, genBlockSize>>>(
        grid,
        isoValue,
        leafCount,
        thrust::raw_pointer_cast(d_triangleOffsets.data()),
        thrust::raw_pointer_cast(d_triangleCounts.data()),
        thrust::raw_pointer_cast(d_atomicCounters.data()),  // NEW
        thrust::raw_pointer_cast(d_vertices.data()),
        thrust::raw_pointer_cast(d_normals.data()),
        thrust::raw_pointer_cast(d_leafOrigins.data())  // NEW!
    );
    
    std::cout << "[DEBUG] Kernel launched, checking for errors..." << std::endl;
    
    // Check for launch error FIRST
    cudaError_t launchErr = cudaGetLastError();
    if (launchErr != cudaSuccess) {
        std::cerr << "[GpuMeshingKernels] ❌ Kernel LAUNCH error: " 
                  << cudaGetErrorString(launchErr) << std::endl;
        return false;
    }
    
    std::cout << "[DEBUG] No launch error, syncing..." << std::endl;
    
    // Set a timeout using cudaStreamQuery instead of blocking sync
    auto start = std::chrono::high_resolution_clock::now();
    cudaError_t syncErr;
    while (true) {
        syncErr = cudaStreamQuery(0);  // Check default stream
        if (syncErr == cudaSuccess) {
            std::cout << "[DEBUG] Kernel completed!" << std::endl;
            break;  // Kernel finished
        }
        if (syncErr != cudaErrorNotReady) {
            std::cerr << "[GpuMeshingKernels] ❌ Kernel execution error: " 
                      << cudaGetErrorString(syncErr) << std::endl;
            return false;
        }
        
        // Check timeout (5 seconds)
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
        if (elapsed > 5) {
            std::cerr << "[GpuMeshingKernels] ❌ TIMEOUT: Kernel running >5 seconds, likely infinite loop!" << std::endl;
            cudaDeviceReset();  // Force kill
            return false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // ========================================================================
    // STEP 4: Copy results to host (temporary - will be replaced by D2D)
    // ========================================================================
    thrust::host_vector<float> h_vertices = d_vertices;
    thrust::host_vector<float> h_normals = d_normals;
    
    // Pack into MeshBuffers
    outMesh.vertices.resize(totalTriangles * 9);
    outMesh.normals.resize(totalTriangles * 9);
    outMesh.indices.resize(totalTriangles * 3);
    
    std::memcpy(outMesh.vertices.data(), h_vertices.data(), 
                totalTriangles * 9 * sizeof(float));
    std::memcpy(outMesh.normals.data(), h_normals.data(), 
                totalTriangles * 9 * sizeof(float));
    
    // Generate sequential indices (unindexed mesh)
    for (int i = 0; i < totalTriangles * 3; ++i) {
        outMesh.indices[i] = i;
    }
    
    // Placeholder UVs
    outMesh.uvs.resize(totalTriangles * 3 * 2, 0.0f);
    
    std::cout << "[GpuMeshingKernels] ✅ Mesh generated: " 
              << (totalTriangles * 3) << " vertices, " 
              << totalTriangles << " triangles" << std::endl;
    return true;
}

bool GpuMeshingKernels::IsAvailable() {
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    return (error == cudaSuccess && deviceCount > 0);
}

size_t GpuMeshingKernels::EstimateTriangleCount(void* deviceGridPtr, float isoValue) {
    // Simplified estimation - return a conservative estimate
    // In production, run classification kernel to get exact count
    return 100000; // Placeholder
}

// ============================================================================
// ASYNC VERSION: Returns device pointers for direct VBO upload
// ============================================================================
bool GpuMeshingKernels::MarchingCubesAsync(
    void* deviceGridPtr,
    uint64_t leafCount,
    float isoValue,
    float voxelSize,
    float** d_vertices_out,
    float** d_normals_out,
    int* triangleCount_out,
    void* cudaStream)  // ✅ FIX: Match header
{
    if (!deviceGridPtr || leafCount == 0) return false;
    
    InitializeLookupTables();
    auto* grid = reinterpret_cast<nanovdb::FloatGrid*>(deviceGridPtr);
    
    cudaStream_t stream = static_cast<cudaStream_t>(cudaStream);
    
    nanovdb::Coord* d_leafOrigins = nullptr;
    cudaMalloc(&d_leafOrigins, leafCount * sizeof(nanovdb::Coord));

    dim3 extractBlockSize(256);
    dim3 extractGridSize((leafCount + 255) / 256);

    ExtractOriginsKernel<<<extractGridSize, extractBlockSize, 0, stream>>>(
        grid,
        d_leafOrigins,
        leafCount
    );

    // Step 1: Count per cell
    const uint64_t numCells = leafCount * 343;  // FIX: Renamed
    int* d_countsPerCell = nullptr;
    cudaMalloc(&d_countsPerCell, numCells * sizeof(int));
    
    dim3 countBlockSize(256);
    dim3 countGridSize((numCells + 255) / 256);
    
    CountTrianglesPerCellKernel<<<countGridSize, countBlockSize, 0, stream>>>(
        grid, isoValue, leafCount, d_countsPerCell, d_leafOrigins
    );
    
    // Step 1.5: Sum per-cell to per-leaf (GPU kernel)
    int* d_counts = nullptr;
    cudaMalloc(&d_counts, leafCount * sizeof(int));
    
    dim3 sumBlockSize(256);
    dim3 sumGridSize(leafCount);
    size_t sharedMemSize = sumBlockSize.x * sizeof(int);
    
    SumCellsToLeavesKernel<<<sumGridSize, sumBlockSize, sharedMemSize, stream>>>(
        d_countsPerCell,
        d_counts,
        leafCount
    );
    
    // Free per-cell counts
    cudaFree(d_countsPerCell);
    
    // Step 2: Scan
    int* d_offsets = nullptr;
    cudaMalloc(&d_offsets, leafCount * sizeof(int));
    
    // Note: thrust doesn't support custom streams well, use CUB instead for production
    thrust::device_ptr<int> counts_ptr(d_counts);
    thrust::device_ptr<int> offsets_ptr(d_offsets);
    thrust::exclusive_scan(thrust::cuda::par.on(stream), 
                          counts_ptr, counts_ptr + leafCount, offsets_ptr);
    
    // Get total count
    int lastCount, lastOffset;
    cudaMemcpyAsync(&lastCount, d_counts + leafCount - 1, 
                    sizeof(int), cudaMemcpyDeviceToHost, stream);
    cudaMemcpyAsync(&lastOffset, d_offsets + leafCount - 1, 
                    sizeof(int), cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);
    
    int totalTriangles = lastOffset + lastCount;
    *triangleCount_out = totalTriangles;
    
    if (totalTriangles == 0) {
        cudaFree(d_leafOrigins);
        cudaFree(d_counts);
        cudaFree(d_offsets);
        return true;
    }
    
    // Step 3: Allocate output (caller must free!)
    cudaMalloc(d_vertices_out, totalTriangles * 9 * sizeof(float));
    cudaMalloc(d_normals_out, totalTriangles * 9 * sizeof(float));
    
    // Allocate atomic counters
    int* d_atomicCounters = nullptr;
    cudaMalloc(&d_atomicCounters, leafCount * sizeof(int));
    cudaMemsetAsync(d_atomicCounters, 0, leafCount * sizeof(int), stream);
    
    dim3 genBlockSize(256);
    dim3 genGridSize((numCells + 255) / 256);
    
    GenerateTrianglesKernel<<<genGridSize, genBlockSize, 0, stream>>>(
        grid, isoValue, leafCount,
        d_offsets, d_counts, d_atomicCounters, *d_vertices_out, *d_normals_out, d_leafOrigins
    );
    
    // Cleanup atomic counters
    cudaFree(d_atomicCounters);
    cudaFree(d_leafOrigins);
    
    // Cleanup temp buffers
    cudaFree(d_counts);
    cudaFree(d_offsets);
    
    return true;
}

} // namespace Urbaxio::Engine




