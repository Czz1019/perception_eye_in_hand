"""Ray/sphere self filtering robust to depth noise along the viewing ray."""
import numpy as np


def robot_depth_mask(depth, pose, intr, centers, radii, link_ids, padding=.025, depth_tolerance=.06):
    # Fill gaps between collision spheres on the same rigid link. This only
    # invalidates pixels; it never supplies evidence of free space.
    expanded_centers, expanded_radii = list(centers), list(radii)
    for link in np.unique(link_ids):
        ids = np.flatnonzero(link_ids == link)
        for a, b in zip(ids, ids[1:]):
            count = int(np.ceil(np.linalg.norm(centers[b]-centers[a])/.04))
            for t in np.linspace(0., 1., count+1)[1:-1]:
                expanded_centers.append((1-t)*centers[a]+t*centers[b])
                expanded_radii.append(min(radii[a], radii[b]))
    camera = (np.asarray(expanded_centers)-pose[:3, 3]) @ pose[:3, :3]
    v, u = np.indices(depth.shape)
    rx, ry = (u-intr.cx)/intr.fx, (v-intr.cy)/intr.fy
    a = rx*rx+ry*ry+1.
    mask = np.zeros(depth.shape, bool)
    for c, radius in zip(camera, expanded_radii):
        radius += padding
        if c[2]+radius < .2:
            continue
        left, right, top, bottom = 0, depth.shape[1], 0, depth.shape[0]
        if c[2] > radius:
            xs = [(c[0]+dx)/(c[2]+dz) for dx in (-radius, radius) for dz in (-radius, radius)]
            ys = [(c[1]+dy)/(c[2]+dz) for dy in (-radius, radius) for dz in (-radius, radius)]
            left = max(0, int(np.floor(intr.fx*min(xs)+intr.cx))-1)
            right = min(right, int(np.ceil(intr.fx*max(xs)+intr.cx))+2)
            top = max(0, int(np.floor(intr.fy*min(ys)+intr.cy))-1)
            bottom = min(bottom, int(np.ceil(intr.fy*max(ys)+intr.cy))+2)
        if left >= right or top >= bottom:
            continue
        region = np.s_[top:bottom, left:right]
        aa = a[region]
        b = rx[region]*c[0]+ry[region]*c[1]+c[2]
        discriminant = b*b-aa*(np.dot(c, c)-radius**2)
        root = np.sqrt(np.maximum(discriminant, 0.))
        near, far = (b-root)/aa, (b+root)/aa
        # The coarse camera-body sphere can enclose the optical centre. Ignore
        # intersections entirely inside the camera's 0.2 m blind zone.
        mask[region] |= ((discriminant >= 0.) & (far >= .2)
                         & (depth[region] >= near-depth_tolerance) & (depth[region] > 0.))
    return mask
