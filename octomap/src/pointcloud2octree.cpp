/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
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

/*
 * convert point cloud to octree
 * Kanzhi Wu
 */


#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <octomap/octomap.h>
#include <octomap/octomap_timing.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace octomap;


void printUsage(char* self){
  cerr << "USAGE: " << self << " <InputFile.pcd> <OutputFile.bt>\n";
  cerr << "This tool creates occupied cells from a point cloud\n";
  exit(0);
}


int main(int argc, char** argv) {
  if (argc != 3)
    printUsage(argv[0]);

  string inputFilename = argv[1];
  string outputFilename = argv[2];

  // pcl point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclcloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::io::loadPCDFile( inputFilename, *pclcloud );

  // data conversion
  Pointcloud * cloud = new Pointcloud;
  for ( size_t i = 0; i < pclcloud->size(); ++ i ) {
    point3d pt(pclcloud->points[i].x, pclcloud->points[i].y, pclcloud->points[i].z);
    cloud->push_back( pt );
  }
  point3d sensor_origin(0,0,0);
  OcTree* tree = new OcTree(0.1);
  tree->insertPointCloud( cloud, sensor_origin );
  tree->writeBinary(outputFilename);







}
