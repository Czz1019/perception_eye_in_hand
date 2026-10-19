"""Bounded joint-space search using only the perceived planning distance field."""
import heapq
import time
import numpy as np
from .demo import goal_candidates, guide_is_clear, goal_ik


def joint_route(geometry, snapshot, start, goals, anchors=(), budget=2., allow_rrt=True):
    """Checked roadmap first, then bidirectional RRT; None means hold/observe.

    Anchors are observation joint poses, not obstacle coordinates. Every edge,
    including edges through a commissioned pose, is checked on this snapshot.
    """
    goals = [np.asarray(g) for g in goals]
    if not goals:
        return None
    deadline = time.monotonic()+budget
    def clear(a, b):
        return guide_is_clear(geometry, snapshot, a, b)
    for goal in goals:
        if clear(start, goal):
            return [np.asarray(start), goal]
    vertices = [np.asarray(start), *goals]
    for a in anchors:
        if clear(a, a):
            vertices.append(np.asarray(a))
    # Lazy Dijkstra over the small observation roadmap.
    queue, visited, parent, cost = [(0., 0)], set(), {}, {0: 0.}
    while queue and time.monotonic() < deadline:
        value, i = heapq.heappop(queue)
        if i in visited:
            continue
        visited.add(i)
        if 1 <= i <= len(goals):
            path = [vertices[i]]
            while i:
                i = parent[i]; path.append(vertices[i])
            return shortcut(geometry, snapshot, path[::-1])
        for j in sorted(set(range(len(vertices)))-visited,
                        key=lambda j: np.linalg.norm(vertices[j]-vertices[i])):
            length = np.linalg.norm(vertices[j]-vertices[i])
            if value+length < cost.get(j, np.inf) and clear(vertices[i], vertices[j]):
                cost[j] = value+length; parent[j] = i
                heapq.heappush(queue, (cost[j], j))
    if not allow_rrt:
        return None
    # Reserve a fresh bounded budget for the continuous search. Seed is stable
    # for repeatable tests; no path is accepted on sampling alone.
    deadline = time.monotonic()+budget
    rng = np.random.default_rng(17)
    goal = goals[0]
    lower = np.maximum(geometry.lower, np.minimum(start, goal)-[.9, .7, 1., 1., .8, .6])
    upper = np.minimum(geometry.upper, np.maximum(start, goal)+[.9, .7, 1., 1., .8, .6])
    trees = [([np.asarray(start)], [-1]), ([goal], [-1])]
    def extend(tree, sample):
        points, parents = tree
        nearest = int(np.argmin(np.linalg.norm(np.asarray(points)-sample, axis=1)))
        delta = sample-points[nearest]; length = np.linalg.norm(delta)
        if length < 1e-8:
            return nearest, True
        q = points[nearest]+delta*min(1., .22/length)
        if not clear(points[nearest], q):
            return None, False
        points.append(q); parents.append(nearest)
        return len(points)-1, length <= .22
    def branch(tree, index):
        result = []
        while index >= 0:
            result.append(tree[0][index]); index = tree[1][index]
        return result[::-1]
    for iteration in range(2000):
        if time.monotonic() >= deadline:
            break
        side = iteration % 2
        sample = trees[1-side][0][0] if rng.random() < .2 else rng.uniform(lower, upper)
        ia, _ = extend(trees[side], sample)
        if ia is None:
            continue
        for _ in range(40):
            ib, reached = extend(trees[1-side], trees[side][0][ia])
            if ib is None or time.monotonic() >= deadline:
                break
            if reached:
                a, b = branch(trees[side], ia), branch(trees[1-side], ib)
                path = a+b[-2::-1] if side == 0 else b+a[-2::-1]
                return shortcut(geometry, snapshot, path)
    return None


def shortcut(geometry, snapshot, path):
    result, i = [path[0]], 0
    while i < len(path)-1:
        j = len(path)-1
        while j > i+1 and not guide_is_clear(geometry, snapshot, path[i], path[j]):
            j -= 1
        result.append(path[j]); i = j
    return result


def goal_route(geometry, snapshot, start, xyz, anchors=(), budget=2., allow_rrt=True):
    # Preserve the useful redundant-posture refinement before invoking search.
    direct = goal_ik(geometry, snapshot, start, xyz)
    if direct is not None:
        return [np.asarray(start), direct]
    candidates = goal_candidates(geometry, snapshot, start, xyz)
    return joint_route(geometry, snapshot, start, candidates, anchors, budget, allow_rrt)


def recovery_step(geometry, snapshot, start, margin=.015):
    """Restore the extra planning buffer without violating the execution margin.

    A new observation can reduce clearance below the 2 cm route buffer while
    remaining above the 1.5 cm execution limit. Only admit a short, fully checked
    retreat with monotonically improving clearance; never escape a collision.
    """
    from .planner import check_segment
    initial = float(np.min(snapshot.query(geometry.spheres(start))-geometry.radii))
    if not margin+.001 < initial < .026:
        return None
    candidates = np.array([start+sign*step*np.eye(6)[axis]
                           for step in (.02, .05, .1) for axis in range(6) for sign in (-1., 1.)])
    centers = geometry.spheres_batch(candidates)
    clearance = np.min(snapshot.query(centers)-geometry.radii, axis=-1)
    order = np.argsort(np.linalg.norm(candidates-start, axis=1))
    for i in order:
        if clearance[i] < .028:
            continue
        q = candidates[i]
        if not check_segment(geometry, snapshot, start, q, margin)[0]:
            continue
        path_q = np.linspace(start, q, 21)
        if hasattr(geometry, 'camera_axis_z') and np.max(geometry.camera_axis_z(path_q)) > -.25-np.sum(abs(q-start))/40:
            continue
        samples = geometry.spheres_batch(path_q)
        values = np.min(snapshot.query(samples)-geometry.radii, axis=-1)
        if np.all(np.diff(values) >= -1e-5):
            return q
    return None
