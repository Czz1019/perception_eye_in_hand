// SPDX-License-Identifier: Apache-2.0
#include "nvblox_in_hand/esdf_pointcloud_exporter.hpp"
#include <glog/logging.h>
#include <nvblox/mapper/mapper.h>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
void require(bool condition,const char * what) {
  if(!condition) throw std::runtime_error(what);
}
int main(int argc,char **argv) {
  google::InitGoogleLogging(argv[0]);
  auto stream=std::make_shared<nvblox::CudaStreamOwning>();
  // Let the installed Mapper allocate its blocks. Do not instantiate its private
  // GPU hash insertion ABI from this test's potentially different CUDA arch list.
  nvblox::Mapper mapper(.03f,nvblox::MemoryType::kUnified,
      nvblox::ProjectiveLayerType::kTsdf,stream);
  nvblox::DepthImage image(nvblox::MemoryType::kDevice);
  std::vector<float> depths(80*60,1.f);
  image.copyFromAsync(60,80,depths.data(),*stream);
  nvblox::Camera camera(70,70,40,30,80,60);
  nvblox::Transform pose=nvblox::Transform::Identity();pose.translation().x()=-.3f;
  mapper.integrateDepth(image,pose,camera);mapper.updateEsdf();stream->synchronize();
  auto &layer=mapper.esdf_layer();
  const auto ids=layer.getAllBlockIndices();
  require(!ids.empty(),"Reference Mapper allocated no ESDF blocks");
  nvblox::Index3D index=ids.front();
  for(const auto &id:ids) {
    auto block=layer.getBlockAtIndex(id);
    for(int x=0;x<8;++x) for(int y=0;y<8;++y) for(int z=0;z<8;++z)
      block->voxels[x][y][z]=nvblox::EsdfVoxel();
    if(id.x()<0) index=id;
  }
  require(index.x()<0,"Fixture must exercise negative block indices");
  auto block=layer.getBlockAtIndex(index);
  auto & inside=block->voxels[1][2][3];
  inside.observed=true;inside.is_inside=true;inside.squared_distance_vox=1.f;
  auto & outside=block->voxels[4][5][6];
  outside.observed=true;outside.squared_distance_vox=4.f;
  auto & far=block->voxels[0][0][0];
  far.observed=true;far.squared_distance_vox=100.f;
  auto & invalid=block->voxels[7][7][7];
  invalid.observed=true;invalid.squared_distance_vox=std::numeric_limits<float>::quiet_NaN();
  in_hand::EsdfPointCloudExporter exporter;
  using Mode=in_hand::EsdfPointSelection;
  auto surface=exporter.run(layer,Mode::kInside,.12f,*stream);
  require(surface.points.size()==1,"Inside diagnostic must match observed && inside");
  auto p=surface.points[0];
  require(std::abs(p.x-(8*index.x()+1+.5f)*.03f)<1e-6 &&
          std::abs(p.y-(8*index.y()+2+.5f)*.03f)<1e-6 &&
          std::abs(p.z-(8*index.z()+3+.5f)*.03f)<1e-6,"Incorrect native centre");
  require(std::abs(p.distance+.03f)<1e-6,"Incorrect signed metric distance");
  require(p.observed==1 && p.is_inside==1 && p.is_site==0,
          "Explicit observed/inside/site fields must preserve native flags");
  require(surface.observed_voxels==4 && surface.inside_voxels==1 &&
          surface.allocated_voxels==ids.size()*512,"Incorrect native flags/counts");
  auto band=exporter.run(layer,Mode::kBand,.12f,*stream);
  require(band.points.size()==2,"Band must omit unknown, invalid and far voxels");
  auto observed=exporter.run(layer,Mode::kObserved,.12f,*stream);
  require(observed.points.size()==3,"Observed mode must keep finite far distance");
  nvblox::EsdfLayer empty(.03f,nvblox::MemoryType::kDevice);
  require(exporter.run(empty,Mode::kSurface,.12f,*stream).points.empty(),"Empty layer");
  bool rejected=false;
  try { exporter.run(layer,Mode::kBand,0.f,*stream); }
  catch(const std::invalid_argument &) { rejected=true; }
  require(rejected,"Invalid band accepted");
  std::cout << "GPU native ESDF selection/centres/signs/empty-layer tests passed\n";
}
