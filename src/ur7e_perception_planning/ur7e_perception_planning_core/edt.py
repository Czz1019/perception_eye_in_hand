"""Exact separable Euclidean distance transform on the already available CUDA GPU.

Each pass minimizes f(i)+(i-j)^2 along one axis. Squared distances on grids
with axis length <=256 are exactly representable in float32. A bounded scratch
batch avoids a world-sized pairwise tensor. Large axes use SciPy instead.
"""
import numpy as np
from scipy.ndimage import distance_transform_edt


class CudaEDT:
    def __init__(self):
        import torch
        self.torch=torch;self.costs={};self.last_backend='cuda'

    def __call__(self, volume):
        torch=self.torch
        if max(volume.shape)>256 or np.all(volume):
            self.last_backend='scipy_fallback'
            return distance_transform_edt(volume)
        self.last_backend='cuda'
        from .runtime import GPU_LOCK
        with GPU_LOCK.mapping(), torch.no_grad():
            mask=torch.as_tensor(np.ascontiguousarray(volume),device='cuda',dtype=torch.bool)
            value=torch.where(mask,float('inf'),0.)
            for axis in range(3):
                n=value.shape[axis]
                if n not in self.costs:
                    indices=torch.arange(n,device='cuda',dtype=torch.float32)
                    self.costs[n]=(indices[:,None]-indices[None,:]).square()
                cost=self.costs[n]
                moved=value.movedim(axis,-1);shape=moved.shape
                rows=moved.reshape(-1,n);output=torch.empty_like(rows)
                batch=max(1,8_000_000//(n*n))  # <=32 MB temporary per reduction
                for start in range(0,len(rows),batch):
                    output[start:start+batch]=(rows[start:start+batch,:,None]+cost).amin(dim=1)
                value=output.reshape(shape).movedim(-1,axis)
            return np.sqrt(value.cpu().numpy().astype(np.float64))
