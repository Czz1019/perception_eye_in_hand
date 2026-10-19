"""Local optimizations; the original realtime_nvblox backends are not modified.

Sparse export follows realtime_nvblox_2's Apache-2.0 implementation: positive
TSDF weights select candidates, but values ALWAYS come from the native ESDF.
Prepared queries retain the caller's exact grid order (including unknowns).
"""
import numpy as np
from realtime_nvblox.nvblox_backend import NvbloxBackend


class EfficientBackend(NvbloxBackend):
    def prepare_query(self, points):
        points=np.asarray(points,dtype=np.float32)
        if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():
            raise ValueError('Prepared query requires finite Nx3 positions')
        with self.lock:
            self.query_points=points.copy()
            self.query_gpu=self.torch.zeros((len(points),4),device='cuda',dtype=self.torch.float32)
            self.query_gpu[:,:3].copy_(self.torch.as_tensor(points,device='cuda'))
            self.output_gpu=self.torch.empty((len(points),1),device='cuda',dtype=self.torch.float32)

    def query_prepared(self):
        with self.lock:
            # Unknown queries must not retain the previous contents of a reused buffer.
            self.output_gpu.fill_(self.esdf_unknown_distance)
            output=self.mapper.query_layer(self.QueryType.ESDF,self.query_gpu,output=self.output_gpu,mapper_id=0)
            # Read back under the lock. CUDA->CPU creates an owned array; no extra GPU clone.
            return output.detach().cpu().numpy().reshape(-1)

    def prepare_fixed_grid(self, center, grid):
        # Preserve the existing dense topic's sampling/order for its consumers.
        self.prepare_esdf_grid(grid)
        self.prepare_query(self._grid_offsets_cpu+np.asarray(center,dtype=np.float32))

    def observed_esdf_numpy(self):
        from nvblox_torch.indexing import get_local_voxel_center_grid
        torch=self.torch
        with self.lock,torch.no_grad():
            layer=self.mapper.tsdf_layer_view(0)
            blocks,indices=layer.get_all_blocks()
            if not blocks:return np.empty((0,3),np.float32),np.empty(0,np.float32)
            local=get_local_voxel_center_grid(layer.voxel_size())
            exported=[]
            for start in range(0,len(blocks),128):
                values=torch.stack(blocks[start:start+128])
                origins=torch.stack(indices[start:start+128]).to(device='cuda',dtype=torch.float32)
                centers=origins[:,None,None,None,:]*(8*layer.voxel_size())+local
                points=centers[torch.isfinite(values[...,1]) & (values[...,1]>0)]
                if not len(points):continue
                query=torch.cat((points,torch.zeros_like(points[:,:1])),dim=1).contiguous()
                distance=self.mapper.query_layer(self.QueryType.ESDF,query,mapper_id=0).reshape(-1)
                known=torch.isfinite(distance) & ~torch.isclose(distance.abs(),torch.full_like(distance,abs(self.esdf_unknown_distance)),rtol=1e-5,atol=1e-3)
                exported.append(torch.cat((points[known],distance[known,None]),dim=1))
            if not exported:return np.empty((0,3),np.float32),np.empty(0,np.float32)
            owned=torch.cat(exported).detach().cpu().numpy()
            return owned[:,:3],owned[:,3]
