#!/usr/bin/env python3
"""
Compute MuJoCo <inertial> fields for Feixi arm links from collision/*.stl meshes.

- Uniform density default: 1.4 g/cm^3 == 1400 kg/m^3 (MuJoCo uses SI).
- Volume / center of mass / inertia at COM use the surface integral formulation from
  David Eberly, "Polyhedral Mass Properties Revisited" (as implemented in trimesh
  `triangles.mass_properties`).
"""

from __future__ import annotations

import argparse
import re
import struct
import sys
from pathlib import Path
from typing import Iterable

import numpy as np

# --- Polyhedral mass properties (GeometricTools / trimesh style) -----------------

_TOL_ZERO = 1e-12


def _tri_cross(triangles: np.ndarray) -> np.ndarray:
    """Cross of edges (v1-v0) x (v2-v0) for each triangle; shape (n, 3)."""
    vectors = triangles[:, 1:, :] - triangles[:, :2, :]
    return np.cross(vectors[:, 0], vectors[:, 1])


def mesh_mass_properties(
    triangles: np.ndarray, density: float
) -> tuple[float, np.ndarray, np.ndarray, float]:
    """
    Returns (mass, com (3,), inertia3x3_at_com, signed_volume).
    """
    triangles = np.asanyarray(triangles, dtype=np.float64)
    if triangles.ndim != 3 or triangles.shape[1:] != (3, 3):
        raise ValueError(f"triangles must be (n, 3, 3), got {triangles.shape}")

    crosses = _tri_cross(triangles)

    f1 = triangles[:, 0, :] + triangles[:, 1, :] + triangles[:, 2, :]
    f2 = (
        triangles[:, 0, :] ** 2
        + triangles[:, 1, :] ** 2
        + triangles[:, 0, :] * triangles[:, 1, :]
        + triangles[:, 2, :] * f1
    )
    f3 = (
        (triangles[:, 0, :] ** 3)
        + (triangles[:, 0, :] ** 2) * (triangles[:, 1, :])
        + (triangles[:, 0, :]) * (triangles[:, 1, :] ** 2)
        + (triangles[:, 1, :] ** 3)
        + (triangles[:, 2, :] * f2)
    )
    g0 = f2 + (triangles[:, 0, :] + f1) * triangles[:, 0, :]
    g1 = f2 + (triangles[:, 1, :] + f1) * triangles[:, 1, :]
    g2 = f2 + (triangles[:, 2, :] + f1) * triangles[:, 2, :]

    integral = np.zeros((10, len(triangles)))
    integral[0] = crosses[:, 0] * f1[:, 0]
    integral[1:4] = (crosses * f2).T
    integral[4:7] = (crosses * f3).T
    for i in range(3):
        tri_i = (i + 1) % 3
        integral[i + 7] = crosses[:, i] * (
            (triangles[:, 0, tri_i] * g0[:, i])
            + (triangles[:, 1, tri_i] * g1[:, i])
            + (triangles[:, 2, tri_i] * g2[:, i])
        )

    integrated = integral.sum(axis=1) / np.array(
        [6, 24, 24, 24, 60, 60, 60, 120, 120, 120], dtype=np.float64
    )
    signed_volume = float(integrated[0])

    if abs(signed_volume) < _TOL_ZERO:
        raise ValueError(
            "STL volume ~0 (mesh may be open, degenerate, or badly scaled). "
            "Check collision mesh winding and watertightness."
        )

    center_mass = integrated[1:4] / signed_volume

    inertia = np.zeros((3, 3), dtype=np.float64)
    inertia[0, 0] = integrated[5] + integrated[6] - (
        signed_volume * (center_mass[[1, 2]] ** 2).sum()
    )
    inertia[1, 1] = integrated[4] + integrated[6] - (
        signed_volume * (center_mass[[0, 2]] ** 2).sum()
    )
    inertia[2, 2] = integrated[4] + integrated[5] - (
        signed_volume * (center_mass[[0, 1]] ** 2).sum()
    )
    inertia[0, 1] = -(integrated[7] - (signed_volume * np.prod(center_mass[[0, 1]])))
    inertia[1, 2] = -(integrated[8] - (signed_volume * np.prod(center_mass[[1, 2]])))
    inertia[0, 2] = -(integrated[9] - (signed_volume * np.prod(center_mass[[0, 2]])))
    inertia[2, 0] = inertia[0, 2]
    inertia[2, 1] = inertia[1, 2]
    inertia[1, 0] = inertia[0, 1]

    inertia *= density
    mass = float(density * signed_volume)
    return mass, center_mass, inertia, signed_volume


# --- STL I/O ----------------------------------------------------------------------


def load_stl_triangles(path: Path) -> np.ndarray:
    """Return (n, 3, 3 float) vertex triples per triangle (v0, v1, v2)."""
    data = path.read_bytes()
    if data[:5].lower() == b"solid" and b"facet normal" in data[:200].lower():
        return _read_ascii_stl(data)
    return _read_binary_stl(data)


def _read_binary_stl(data: bytes) -> np.ndarray:
    if len(data) < 84:
        raise ValueError("STL too small")
    n = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + n * 50
    if len(data) < expected:
        raise ValueError("Binary STL truncated")
    triangles = np.empty((n, 3, 3), dtype=np.float64)
    offset = 84
    for i in range(n):
        # skip normal (3 floats) + read 9 floats for vertices
        off = offset + 12
        v0 = struct.unpack_from("<fff", data, off)
        v1 = struct.unpack_from("<fff", data, off + 12)
        v2 = struct.unpack_from("<fff", data, off + 24)
        triangles[i, 0] = v0
        triangles[i, 1] = v1
        triangles[i, 2] = v2
        offset += 50
    return triangles


def _read_ascii_stl(data: bytes) -> np.ndarray:
    text = data.decode("latin-1", errors="replace")
    verts: list[tuple[float, float, float]] = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("vertex"):
            parts = line.split()
            verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
    if len(verts) % 3 != 0:
        raise ValueError("ASCII STL vertex count is not a multiple of 3")
    arr = np.array(verts, dtype=np.float64).reshape(-1, 3, 3)
    return arr


def format_inertial_tag(
    mass: float, com: np.ndarray, inertia: np.ndarray, precision: int = 8
) -> str:
    ixx, iyy, izz = float(inertia[0, 0]), float(inertia[1, 1]), float(inertia[2, 2])
    ixy, ixz, iyz = float(inertia[0, 1]), float(inertia[0, 2]), float(inertia[1, 2])
    px, py, pz = float(com[0]), float(com[1]), float(com[2])
    fmt = f"{{:.{precision}g}}"
    return (
        f'<inertial pos="{fmt} {fmt} {fmt}" mass="{fmt}" '
        f'fullinertia="{fmt} {fmt} {fmt} {fmt} {fmt} {fmt}"/>'
    ).format(px, py, pz, mass, ixx, iyy, izz, ixy, ixz, iyz)


_ARM_BODIES_STL: tuple[tuple[str, str], ...] = (
    ("base_link", "link0.stl"),
    ("link1", "link1.stl"),
    ("link2", "link2.stl"),
    ("link3", "link3.stl"),
    ("link4", "link4.stl"),
    ("link5", "link5.stl"),
    ("link6", "link6.stl"),
    ("link7", "link7.stl"),
)


def compute_arm(
    collision_dir: Path, density_kg_m3: float
) -> dict[str, tuple[float, np.ndarray, np.ndarray, float]]:
    out: dict[str, tuple[float, np.ndarray, np.ndarray, float]] = {}
    for body, stl_name in _ARM_BODIES_STL:
        tris = load_stl_triangles(collision_dir / stl_name)
        m, com, I, vol = mesh_mass_properties(tris, density_kg_m3)
        out[body] = (m, com.copy(), I.copy(), vol)
    return out


_BODY_OPEN_RE = re.compile(r'^(?P<indent>\s*)<body name="(?P<name>[^"]+)"[^>]*>\s*$')


def apply_inertials_to_file(
    path: Path,
    body_to_tag: dict[str, str],
) -> str:
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    out: list[str] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.rstrip("\n")
        m = _BODY_OPEN_RE.match(stripped)
        if m:
            name = m.group("name")
            indent = m.group("indent")
            next_indent = indent + "  "
            tag = body_to_tag.get(name)
            out.append(line)
            if tag is None:
                i += 1
                continue
            # Next lines: optional existing inertial to replace; else insert after body line
            j = i + 1
            if j < len(lines) and lines[j].lstrip().startswith("<inertial"):
                # Replace first inertial after this body
                out.append(next_indent + tag + "\n")
                i = j + 1
                continue
            out.append(next_indent + tag + "\n")
            i += 1
            continue
        out.append(line)
        i += 1
    new_text = "".join(out)
    return new_text


def main(argv: Iterable[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--model-dir",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="Directory containing collision/*.stl (default: this script's directory).",
    )
    p.add_argument(
        "--density-g-per-cm3",
        type=float,
        default=1.4,
        help="Uniform density in g/cm^3 (default: 1.4).",
    )
    p.add_argument(
        "--write",
        action="store_true",
        help="Write <inertial/> into feixi_model.xml and feixi_with_gripper.xml in model-dir.",
    )
    args = p.parse_args(list(argv) if argv is not None else None)

    model_dir: Path = args.model_dir
    collision_dir = model_dir / "collision"
    if not collision_dir.is_dir():
        print(f"missing collision directory: {collision_dir}", file=sys.stderr)
        return 2

    # 1 g/cm^3 = 1000 kg/m^3
    density_kg_m3 = float(args.density_g_per_cm3) * 1000.0

    results = compute_arm(collision_dir, density_kg_m3)

    body_to_tag: dict[str, str] = {}
    print(f"density = {args.density_g_per_cm3} g/cm^3 = {density_kg_m3:g} kg/m^3\n")
    for body, stl_name in _ARM_BODIES_STL:
        m, com, I, vol = results[body]
        tag = format_inertial_tag(m, com, I)
        body_to_tag[body] = tag
        print(f"[{body}] {stl_name}")
        print(f"  volume(m^3)={vol:.6g}  mass(kg)={m:.6g}")
        print(f"  com(m)=({com[0]:.8g}, {com[1]:.8g}, {com[2]:.8g})")
        print(f"  {tag}")
        print()

    if args.write:
        for rel in ("feixi_model.xml", "feixi_with_gripper.xml"):
            path = model_dir / rel
            if not path.is_file():
                print(f"skip missing file: {path}", file=sys.stderr)
                continue
            new_text = apply_inertials_to_file(path, body_to_tag)
            path.write_text(new_text, encoding="utf-8")
            print(f"updated {path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
