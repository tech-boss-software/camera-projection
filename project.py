"""Calibrated pinhole projection with camera pose and image-frustum clipping."""
import argparse
import json
import math
from pathlib import Path


def camera_coordinates(point, position=(0, 0, 0), yaw=0):
    """World-to-camera transform: inverse translation, then inverse yaw.

    Coordinates are right-handed: x right, y down, z forward. Positive yaw
    rotates the camera's forward axis toward positive world x.
    """
    if len(point) != 3 or len(position) != 3 or any(not math.isfinite(v) for v in (*point, *position, yaw)):
        raise ValueError('Pose and point must be finite 3D coordinates')
    x, y, z = (a-b for a, b in zip(point, position))
    c, s = math.cos(yaw), math.sin(yaw)
    return c*x-s*z, y, s*x+c*z


def validate_camera(camera):
    if any(not math.isfinite(camera[k]) for k in ('fx','fy','cx','cy','near','width','height')):
        raise ValueError('Camera values must be finite')
    if camera['fx'] <= 0 or camera['fy'] <= 0 or camera['near'] <= 0 or camera['width'] <= 0 or camera['height'] <= 0:
        raise ValueError('Focal lengths, dimensions and near plane must be positive')


def project(point, camera):
    validate_camera(camera)
    if any(not math.isfinite(v) for v in point):
        raise ValueError('Point must be finite')
    x, y, z = point
    if z < camera['near']:
        # Half-space interpolation can land a few ulps below the boundary.
        # Accept only roundoff-sized differences, not genuinely hidden points.
        if not math.isclose(z, camera['near'], rel_tol=1e-12, abs_tol=0):
            return None
        z = camera['near']
    return camera['fx']*x/z+camera['cx'], camera['fy']*y/z+camera['cy']


def clip_segment(a, b, camera):
    """Clip a 3D segment against five convex frustum half-spaces.

    Clipping before perspective division avoids exploding screen coordinates
    when an edge crosses the camera plane. Bounds are continuous image edges.
    """
    validate_camera(camera)
    if any(not math.isfinite(v) for v in (*a, *b)):
        raise ValueError('Segment coordinates must be finite')
    fx, fy, cx, cy = (camera[k] for k in ('fx','fy','cx','cy'))
    w, h, near = (camera[k] for k in ('width','height','near'))
    planes = [lambda p: p[2]-near,
              lambda p: fx*p[0]+cx*p[2],
              lambda p: -fx*p[0]+(w-cx)*p[2],
              lambda p: fy*p[1]+cy*p[2],
              lambda p: -fy*p[1]+(h-cy)*p[2]]
    low, high = 0.0, 1.0
    for plane in planes:
        fa, fb = plane(a), plane(b)
        if fa < 0 and fb < 0:
            return None
        if fa < 0 or fb < 0:
            crossing = fa/(fa-fb)
            if fa < 0:
                low = max(low, crossing)
            else:
                high = min(high, crossing)
        if low > high:
            return None
    def endpoint(t):
        # Preserve unclipped endpoints exactly: interpolating t=1 can round
        # differently and falsely label a fully visible edge as clipped.
        if t == 0.0:
            return a
        if t == 1.0:
            return b
        x, y, z = (x+t*(y-x) for x,y in zip(a,b))
        # The half-space calculation establishes visibility analytically;
        # snapping its boundary roundoff keeps the returned endpoint usable.
        if z < near and math.isclose(z, near, rel_tol=1e-12, abs_tol=0):
            z = near
        return x, y, z
    return endpoint(low), endpoint(high)


def cube(center, size):
    x,y,z = center
    vertices = [(x+dx*size/2,y+dy*size/2,z+dz*size/2) for dz in (-1,1) for dy in (-1,1) for dx in (-1,1)]
    edges = [(i,j) for i in range(8) for j in range(i+1,8) if bin(i^j).count('1')==1]
    return vertices, edges


def run(output):
    output.mkdir(parents=True, exist_ok=True)
    camera = {'fx':420.0,'fy':420.0,'cx':320.0,'cy':220.0,'near':.25,'width':640,'height':440}
    position, yaw = (.25,-.1,0), math.radians(8)
    # Include off-screen and behind-camera geometry, not just happy paths.
    scene = [((0,0,4),1.6), ((-3,0,3),1.8), ((2,-.5,5),1.1), ((0,0,-2),1)]
    lines, total, visible, clipped_count = [], 0, 0, 0
    colors = ['#155e75','#b45309','#6d28d9','#374151']
    for (center,size),color in zip(scene,colors):
        vertices,edges = cube(center,size)
        vertices = [camera_coordinates(v,position,yaw) for v in vertices]
        for i,j in edges:
            total += 1
            segment = clip_segment(vertices[i],vertices[j],camera)
            if segment is None:
                continue
            visible += 1
            clipped_count += segment != (vertices[i],vertices[j])
            a,b = (project(p,camera) for p in segment)
            lines.append(f'<line x1="{a[0]:.3f}" y1="{a[1]:.3f}" x2="{b[0]:.3f}" y2="{b[1]:.3f}" stroke="{color}" stroke-width="2"/>')
    svg = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 440"><title>Calibrated pinhole projection of synthetic cubes</title><rect width="640" height="440" fill="#f8fafc"/><path d="M310 220h20M320 210v20" stroke="#94a3b8"/>'+''.join(lines)+'<text x="12" y="425" font-size="12" fill="#334155">Synthetic geometry · x right, y down, z forward · no lens distortion</text></svg>'
    (output/'projection.svg').write_text(svg,encoding='utf-8')
    report = {'camera':camera,'camera_position_world':position,'camera_yaw_degrees':8,
              'scene':'Four generated cubes, including off-screen and behind-camera geometry',
              'total_edges':total,'visible_edges':visible,'partially_clipped_edges':clipped_count,
              'rejected_edges':total-visible,'metric_note':'Visibility accounting, not a prediction-accuracy benchmark'}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('examples'))
    run(parser.parse_args().output)
