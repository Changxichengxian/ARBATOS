"""按真实 STEP 平面分解 RMUL 场地；不把整个凹场地变成一个凸包。"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull


ASSETS = Path(__file__).with_name("assets")
ROTATION = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float)


def repartition_horizontal(parts: list) -> tuple[list, dict]:
    """水平薄板内部用严格包含的矩形，边角约束三角化；并集和孔洞保持原样。"""
    import shapely
    from shapely.geometry import Polygon, box
    from shapely.ops import unary_union

    groups, untouched = {}, []
    for part in parts:
        count = len(part)//2
        top, bottom = part[:count], part[count:]
        if (len(part) % 2 or np.ptp(top[:, 2]) > 1e-12 or np.ptp(bottom[:, 2]) > 1e-12 or
                not np.allclose(top[:, :2], bottom[:, :2], atol=1e-12, rtol=0)):
            untouched.append(part)
            continue
        key = (round(float(top[0, 2]), 12), round(float(bottom[0, 2]), 12))
        groups.setdefault(key, []).append(part)
    result = list(untouched)
    records = []
    for (upper, lower), original in groups.items():
        polygons = [Polygon(part[:len(part)//2, :2]) for part in original]
        surface = unary_union(polygons)
        if surface.area < 1.0:
            result.extend(original)
            continue
        x0, y0, x1, y1 = surface.bounds
        # 格线避开入口的轮子；整格必须由原多边形覆盖，不能用容差扩张来填孔。
        cell_size, grid_origin = 1.0, .25
        cells = []
        if abs(upper) <= 1e-12:
            for x in range(int(np.floor(x0-grid_origin)), int(np.ceil(x1-grid_origin))):
                for y in range(int(np.floor(y0-grid_origin)), int(np.ceil(y1-grid_origin))):
                    cell = box(x+grid_origin, y+grid_origin, x+grid_origin+cell_size,
                               y+grid_origin+cell_size)
                    if surface.covers(cell):
                        cells.append(cell)
        # 相邻完整格只在矩形并集内合并，减少轮子跨内部格线时的重复候选。
        occupied = {(round(p.bounds[0]-grid_origin), round(p.bounds[1]-grid_origin)) for p in cells}
        rectangles = []
        while occupied:
            x, y = min(occupied)
            width = 0
            best = (1, 1)
            while (x+width, y) in occupied:
                width += 1
                height = 1
                while all((x+dx, y+height) in occupied for dx in range(width)):
                    height += 1
                if width*height > best[0]*best[1]:
                    best = (width, height)
            width, height = best
            occupied.difference_update((x+dx, y+dy) for dx in range(width) for dy in range(height))
            rectangles.append(box(x+grid_origin, y+grid_origin, x+grid_origin+width,
                                  y+grid_origin+height))
        cells = rectangles
        remainder = surface.difference(unary_union(cells))
        triangles = list(shapely.constrained_delaunay_triangles(remainder).geoms)
        # 其他高度只消除严格重叠/内部接缝，块数不减少则保留已有原生盒体。
        if not cells and len(triangles) >= len(original):
            result.extend(original)
            continue
        rebuilt = unary_union(cells+triangles)
        error_area = surface.symmetric_difference(rebuilt).area
        boundary_error = surface.boundary.hausdorff_distance(rebuilt.boundary)
        if error_area > max(1e-10, surface.area*1e-11) or boundary_error > 1e-9:
            raise ValueError(f"水平地面剖分改变边界：z={upper}, area={error_area}, boundary={boundary_error}")
        if abs(sum(p.area for p in cells+triangles)-surface.area) > max(1e-10, surface.area*1e-11):
            raise ValueError("水平地面剖分重叠或遗漏")
        for polygon in cells+triangles:
            xy = np.asarray(polygon.exterior.coords[:-1])
            result.append(np.vstack((np.column_stack((xy, np.full(len(xy), upper))),
                                     np.column_stack((xy, np.full(len(xy), lower))))))
        records.append(dict(top_m=upper, bottom_m=lower, source_parts=len(original),
            area_m2=surface.area, boxes=len(cells), edge_triangles=len(triangles),
            symmetric_difference_area_m2=error_area, boundary_error_m=boundary_error))
    return result, dict(method="exact_horizontal_union_grid_and_constrained_edges",
        grid_size_m=1.0, grid_origin_m=.25, shapely_version=shapely.__version__, groups=records,
        original_collision_parts=len(parts), output_collision_parts=len(result))


def convert(source: Path, destination: Path) -> dict:
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    text = source.read_text(encoding="utf-8")
    if "SI_UNIT(.MILLI.,.METRE.)" not in text:
        raise ValueError("此转换器要求源文件明确声明毫米单位，不能猜缩放")
    reader = STEPControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise ValueError("STEP 读取失败")
    reader.TransferRoots()
    shape = reader.OneShape()
    # 源模型全是平面，此处只为裁剪面和带洞多边形建立三角面，不降低场地尺寸精度。
    BRepMesh_IncrementalMesh(shape, .5, False, .1, True)
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    parts, source_triangles = [], []
    face_count = degenerate_count = split_count = 0
    while explorer.More():
        face = TopoDS.Face(explorer.Current())
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation_s(face, location)
        face_count += 1
        if triangulation is None:
            raise ValueError(f"第 {face_count} 个面没有三角化结果")
        transform = location.Transformation()
        vertices = np.array([[point.X(), point.Y(), point.Z()] for point in (
            triangulation.Node(index).Transformed(transform)
            for index in range(1, triangulation.NbNodes()+1))])
        vertices = vertices @ ROTATION.T * .001
        triangles = np.array([triangulation.Triangle(index).Get()
            for index in range(1, triangulation.NbTriangles()+1)], dtype=int)-1
        surfaces = vertices[triangles]
        cross = np.cross(surfaces[:, 1]-surfaces[:, 0], surfaces[:, 2]-surfaces[:, 0])
        areas = np.linalg.norm(cross, axis=1)/2
        valid = areas > 1e-12
        surfaces, areas, cross = surfaces[valid], areas[valid], cross[valid]
        if not len(surfaces):
            degenerate_count += 1
            explorer.Next()
            continue
        source_triangles.extend(surfaces)
        normal = cross[np.argmax(areas)] / (2*areas.max())
        if normal[2] < -1e-8 or (abs(normal[2]) < 1e-8 and normal[np.argmax(np.abs(normal))] < 0):
            normal = -normal
        axis = (surfaces[0, 1]-surfaces[0, 0])
        axis /= np.linalg.norm(axis)
        other = np.cross(normal, axis)
        projection = np.column_stack((vertices @ axis, vertices @ other))
        hull = ConvexHull(projection)
        # 面面积等于凸包面积才允许合并；凹面或带洞面保留原三角分解。
        convex = abs(hull.volume-areas.sum()) <= max(1e-10, areas.sum()*1e-7)
        polygons = [vertices[hull.vertices]] if convex else list(surfaces)
        if not convex:
            split_count += 1
        for polygon in polygons:
            # STEP 是薄壳面模型：承载面向下加2 mm，竖面两侧各1 mm。
            # 原始台顶、坡面和凹陷高度保留，不跨面补出隐藏地板。
            if abs(normal[2]) > .05:
                prism = np.vstack((polygon, polygon-normal*.002))
            else:
                prism = np.vstack((polygon+normal*.001, polygon-normal*.001))
            parts.append(prism)
        explorer.Next()
    parts, repartition = repartition_horizontal(parts)
    offsets = np.cumsum([0] + [len(part) for part in parts])
    vertices = np.vstack(parts)
    source_triangles = np.asarray(source_triangles)
    bounds = np.array([source_triangles.min(axis=(0, 1)), source_triangles.max(axis=(0, 1))])
    metadata = dict(source_name=source.name, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        source_length_unit="mm", scale_to_m=.001, axis_transform="world (x,y,z) = CAD (X,Z,-Y) / 1000",
        local_bounds_m=bounds.tolist(), size_m=(bounds[1]-bounds[0]).tolist(),
        faces=face_count, convex_collision_parts=len(parts), concave_faces_split=split_count,
        source_triangles=len(source_triangles), degenerate_faces=degenerate_count,
        collision_shell_thickness_m=.002,
        horizontal_repartition=repartition,
        collision_note="逐平面凸分解；承载面向下2mm，竖面两侧1mm；零高地面严格等价矩形剖分，边角与孔洞约束三角化",
        converter="cadquery-ocp 8.0.1.0.0 / OpenCascade", orientation_reason="CAD Y跨度0.52m，地面Y=0，高台Y为负")
    destination.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(destination, vertices=vertices, offsets=offsets, source_triangles=source_triangles)
    destination.with_suffix(".json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, default=ASSETS / "Rmul2026.npz")
    args = parser.parse_args()
    print(json.dumps(convert(args.source.resolve(), args.output.resolve()), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
