"""Mask measured robot pixels, including the fixed base excluded from moving balls."""
import numpy as np

def robot_depth_mask(depth,pose,intr,centers,radii,link_ids,padding=.025,depth_tolerance=.06):
    """Same ray/sphere rule as the shared filter, evaluated in array batches.

    Hundreds of tiny NumPy calls per frame are costly under concurrent Python
    ROS callbacks. Batching keeps the exact padding and ray intersection rule,
    and bounds temporary allocation to 32 spheres at a time.
    """
    expanded_centers=list(centers);expanded_radii=list(radii)
    for link in np.unique(link_ids):
        ids=np.flatnonzero(link_ids==link)
        for a,b in zip(ids,ids[1:]):
            count=int(np.ceil(np.linalg.norm(centers[b]-centers[a])/.04))
            for t in np.linspace(0.,1.,count+1)[1:-1]:
                expanded_centers.append((1-t)*centers[a]+t*centers[b])
                expanded_radii.append(min(radii[a],radii[b]))
    camera=(np.asarray(expanded_centers)-pose[:3,3])@pose[:3,:3]
    radius=np.asarray(expanded_radii)+padding
    visible=camera[:,2]+radius>=.2
    camera=camera[visible];radius=radius[visible]
    v,u=np.indices(depth.shape)
    rays=np.stack([(u-intr.cx)/intr.fx,(v-intr.cy)/intr.fy,np.ones_like(u)],-1).reshape(-1,3)
    aa=np.sum(rays*rays,axis=1)[:,None]
    measured=depth.reshape(-1,1)
    result=np.zeros(depth.size,bool)
    for start in range(0,len(camera),32):
        c=camera[start:start+32];r=radius[start:start+32]
        b=rays@c.T
        discriminant=b*b-aa*(np.sum(c*c,axis=1)-r*r)
        root=np.sqrt(np.maximum(discriminant,0.))
        near=(b-root)/aa;far=(b+root)/aa
        result|=np.any((discriminant>=0.)&(far>=.2)&(measured>=near-depth_tolerance)&(measured>0.),axis=1)
    return result.reshape(depth.shape)

def mask(depth,pose,intr,centers,radii,link_ids,extra):
    centers=np.concatenate([centers,np.array(extra['centers'])])
    radii=np.r_[radii,extra['radii']]
    link_ids=np.r_[link_ids,np.arange(len(extra['radii']))+max(link_ids)+1]
    # Invalidating depth never inserts free rays or clears TSDF voxels.
    return robot_depth_mask(depth,pose,intr,centers,radii,link_ids)
