#!/usr/bin/env python3
"""Build the reviewable CAD-envelope contact variant without changing dynamics.

Uses NumPy/SciPy from the existing geometry environment. Validation/rendering
also use its MuJoCo install. The legacy SDK XML is deliberately unchanged.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import struct
from pathlib import Path
import xml.etree.ElementTree as ET

DEMO = Path(__file__).resolve().parents[1]
ASSETS = DEMO.parent / "sdk/assets/khan_mini"
ENABLED_BODIES = (
    "base_link", "Left_knee_Link", "Right_knee_Link",
    "Left_ankle_pitch_Link", "Right_ankle_pitch_Link",
)
CONTACT_BITS = {
    "external": (1, 31),
    "left_foot": (2, 23),
    "right_foot": (4, 15),
    "left_ankle_joint_envelope": (8, 29),
    "right_ankle_joint_envelope": (16, 27),
}


def contact_truth_table() -> dict:
    table = {}
    excluded = {frozenset(("left_foot", "left_ankle_joint_envelope")),
                frozenset(("right_foot", "right_ankle_joint_envelope"))}
    for a, (at, aa) in CONTACT_BITS.items():
        for b, (bt, ba) in CONTACT_BITS.items():
            enabled = bool((at & ba) or (bt & aa))
            expected = frozenset((a, b)) not in excluded
            if enabled != expected:
                raise AssertionError((a, b, enabled, expected))
            table[f"{a} / {b}"] = enabled
    return table


def binary_stl_triangles(path: Path):
    import numpy as np
    blob=path.read_bytes();count=struct.unpack("<I",blob[80:84])[0]
    dtype=np.dtype([("normal","<f4",3),("vertices","<f4",(3,3)),("attribute","<u2")])
    if len(blob)!=84+50*count:raise ValueError(f"Expected binary STL: {path}")
    return np.frombuffer(blob,dtype=dtype,count=count,offset=84)["vertices"].astype(float)


def write_clipped_convex_mesh(source: Path, output: Path, cut_z: float, above: bool) -> dict:
    """Hull of the CAD triangles clipped by a knee-local Z plane.

    Plane intersections are obtained from triangle edges; no new geometry or
    decomposition package is needed. Each part remains rigidly on knee_Link.
    """
    import numpy as np
    from scipy.spatial import ConvexHull
    tri=binary_stl_triangles(source);verts=tri.reshape(-1,3)
    keep=verts[:,2]>=cut_z if above else verts[:,2]<=cut_z
    points=[verts[keep]]
    for i,j in [(0,1),(1,2),(2,0)]:
        a,b=tri[:,i],tri[:,j];mask=(a[:,2]-cut_z)*(b[:,2]-cut_z)<0
        t=((cut_z-a[mask,2])/(b[mask,2]-a[mask,2]))[:,None]
        points.append(a[mask]+t*(b[mask]-a[mask]))
    points=np.unique(np.round(np.concatenate(points),10),axis=0)
    hull=ConvexHull(points);center=points[hull.vertices].mean(axis=0)
    faces=[]
    for face in hull.simplices:
        v=points[face].copy();normal=np.cross(v[1]-v[0],v[2]-v[0])
        if np.dot(normal,v.mean(axis=0)-center)<0:v[[1,2]]=v[[2,1]];normal=-normal
        normal/=np.linalg.norm(normal)
        faces.append(struct.pack("<12fH",*(normal.tolist()+v.ravel().tolist()),0))
    output.write_bytes(b"CAD knee contact envelope; no inertial contribution".ljust(80,b"\0")+
                       struct.pack("<I",len(faces))+b"".join(faces))
    return {"source_stl":str(source.resolve()),"output_stl":str(output.resolve()),
            "sha256":hashlib.sha256(output.read_bytes()).hexdigest(),
            "cut_z_m":cut_z,"region":"proximal" if above else "distal ankle-joint envelope",
            "bounds_m":[points.min(axis=0).tolist(),points.max(axis=0).tolist()],
            "hull_triangles":len(faces)}


def generate(source: Path, output: Path, manifest: Path | None = None, calf_cut_z: float = -.25) -> dict:
    if source.parent.resolve() != output.parent.resolve():
        raise ValueError("Write the contact XML beside its legacy XML so both use the same portable mesh directory")
    root = ET.parse(source).getroot()
    root.set("model", "Khan_mini_contact")
    # Both files live beside the same unchanged STL directory.
    root.find("compiler").set("meshdir", "./meshes")
    root.insert(0, ET.Comment(
        " CAD-envelope review model. Base, lower legs and ankle brackets have "
        "collision enabled; feet retain their original whole-mesh convex hull. "
        f"Lower-leg hulls are split at knee-local Z={calf_cut_z:g}m. Only distal joint "
        "envelope vs same-side foot is masked to avoid their nested-joint "
        "convex-hull overlap; main lower leg and all opposite-side collisions remain. "
        "Internal parallel linkage meshes remain non-colliding. Body inertias, "
        "joints, actuators, equality constraints and contact material parameters "
        "are inherited unchanged. Geometry is not a hardware contact calibration. "))
    changed = []
    bodies={b.get("name"):b for b in root.findall(".//body")}
    for body in root.findall(".//body"):
        if body.get("name") not in ENABLED_BODIES:
            continue
        geoms = body.findall("geom")
        if len(geoms) != 1:
            raise ValueError(f"Expected one external CAD geom on {body.get('name')}")
        geom = geoms[0]
        changed.append({"body": body.get("name"), "mesh": geom.get("mesh"),
                        "old_contype": geom.get("contype", "1"),
                        "old_conaffinity": geom.get("conaffinity", "1")})
        geom.set("contype", "1")
        geom.set("conaffinity", "1")
    if {x["body"] for x in changed} != set(ENABLED_BODIES):
        raise ValueError("Missing expected CAD contact body")
    # Every active external geom uses the common external bit. Inactive internal
    # rods keep 0/0, so reserving these bits does not enable linkage self-contact.
    for body in bodies.values():
        for geom in body.findall("geom"):
            if geom.get("contype","1")!="0" or geom.get("conaffinity","1")!="0":
                geom.set("contype","1");geom.set("conaffinity","31")
    mesh_assets=root.find("asset");parts=[]
    for side,label in [("Left","left"),("Right","right")]:
        foot=bodies[f"{side}_ankle_roll_Link"].find("geom")
        typ,aff=CONTACT_BITS[f"{label}_foot"]
        foot.set("contype",str(typ));foot.set("conaffinity",str(aff))
        body=bodies[f"{side}_knee_Link"];visual=body.find("geom")
        visual.set("contype","0");visual.set("conaffinity","0")
        # One full lower-leg hull would fill the ankle fork and overlap the
        # foot hull by ~32mm even at neutral. Keep the contact on the proximal
        # shaft, while only the distal mechanical package has the local mask.
        for above,suffix,mask_name in [(True,"main","external"),
                                       (False,"ankle_joint",f"{label}_ankle_joint_envelope")]:
            name=f"{side}_knee_{suffix}_contact"
            dest=output.parent/"meshes"/(name+".STL")
            parts.append(write_clipped_convex_mesh(source.parent/"meshes"/(f"{side}_knee_Link.STL"),
                                                  dest,calf_cut_z,above))
            ET.SubElement(mesh_assets,"mesh",name=name,file=dest.name)
            typ,aff=CONTACT_BITS[mask_name]
            ET.SubElement(body,"geom",name=name,type="mesh",mesh=name,density="0",
                          contype=str(typ),conaffinity=str(aff),group="3",rgba="0.2 0.8 0.35 0")
    contact=root.find("contact")
    if contact is None:
        contact=ET.Element("contact")
        root.insert(list(root).index(root.find("equality")),contact)
    contact.append(ET.Comment(
        " Pelvis and hip-roll convex envelopes cover a nested two-axis hip "
        "gimbal. Exclude only those internal mechanical pairs; base/hip-yaw, "
        "base/legs/feet and opposite-side contacts remain enabled. "))
    internal_excludes=[]
    for side in ["Left","Right"]:
        ET.SubElement(contact,"exclude",name=f"base_{side.lower()}_hip_roll_internal",
                      body1="base_link",body2=f"{side}_hip_roll_Link")
        internal_excludes.append({"body1":"base_link","body2":f"{side}_hip_roll_Link",
                                  "reason":"Nested hip gimbal convex-envelope overlap, up to ~14.5mm in reference geometry check; not an allowed external penetration."})
    ET.indent(root, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="unicode")
    output.write_text(output.read_text() + "\n")
    result = {
        "source": str(source.resolve()), "output": str(output.resolve()),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "output_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
        "enabled_external_cad_geoms": changed,
        "foot_model": "Original Left/Right_ankle_roll_Link.STL convex hull, including upper foot structure",
        "material_parameters": "Unchanged SDK defaults; not measured hardware calibration",
        "mechanical_pair_excludes": internal_excludes,
        "collision_bits":CONTACT_BITS,"collision_truth_table":contact_truth_table(),
        "lower_leg_partition":parts,
        "mechanical_mask_reason":"Whole knee convex hull fills ankle fork; same-side distal envelope/foot hull overlaps by ~32mm at neutral. Only that local joint package pair is masked. Main lower leg/own foot, every opposite-side pair and foot/foot remain enabled.",
        "unchanged": ["all inertial definitions", "all joints and limits", "armature and passive damping/frictionloss",
                      "actuators", "parallel linkage equality constraints", "ground", "internal linkage collision masks"],
    }
    if manifest:
        manifest.parent.mkdir(parents=True, exist_ok=True)
        manifest.write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ASSETS / "Khan_mini_sym.xml")
    parser.add_argument("--output", type=Path, default=ASSETS / "Khan_mini_contact.xml")
    parser.add_argument("--manifest", type=Path, default=ASSETS / "Khan_mini_contact.manifest.json")
    parser.add_argument("--calf-cut-z", type=float, default=-.25,
                        help="Knee-local partition plane, default ankle center (-.30m) plus .05m")
    args = parser.parse_args()
    print(json.dumps(generate(args.source, args.output, args.manifest,args.calf_cut_z), indent=2))
