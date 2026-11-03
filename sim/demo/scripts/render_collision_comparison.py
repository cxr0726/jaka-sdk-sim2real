#!/usr/bin/env python3
"""Render actual compiled SDK/training collision geometry without simulation.

Uses an existing MuJoCo/Pillow installation. Source models are never changed.
Mesh collisions are rendered with mjVIS_CONVEXHULL, primitives as compiled.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DEMO=Path(__file__).resolve().parents[1]
DEFAULT_TRAIN=Path('/home/cxr0726/workspace/jaka_supermimic/SuperMimic-Reproduced-jaka/assets/robots/khan_mini/robot.xml')
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if not Path(FONT).exists():FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'

def font(size):return ImageFont.truetype(FONT,size)
def name(m,kind,i):return mujoco.mj_id2name(m,kind,int(i)) or str(i)

def prepare(path,pose):
    m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m)
    d.qpos[:]=m.qpos0
    free=np.flatnonzero(m.jnt_type==mujoco.mjtJoint.mjJNT_FREE)
    assert len(free)==1
    qadr=int(m.jnt_qposadr[free[0]])
    d.qpos[qadr:qadr+7]=[0,0,.65,1,0,0,0]
    jid={name(m,mujoco.mjtObj.mjOBJ_JOINT,j).casefold():j for j in range(m.njnt)}
    for joint,q in pose.items():d.qpos[m.jnt_qposadr[jid[joint.casefold()]]]=q
    # Forward kinematics only; do not enforce equality or integrate dynamics.
    mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d)
    active=((m.geom_contype!=0)|(m.geom_conaffinity!=0))&(m.geom_bodyid!=0)
    visual=(m.geom_type==mujoco.mjtGeom.mjGEOM_MESH)&(m.geom_bodyid!=0)
    # Synthetic collision-only meshes are not the CAD visual silhouette.
    for g in np.flatnonzero(visual):
        gn=name(m,mujoco.mjtObj.mjOBJ_GEOM,g)
        if 'contact' in gn or 'collision' in gn:visual[g]=False
    types={}
    records=[]
    for g in np.flatnonzero(active):
        typ=mujoco.mjtGeom(int(m.geom_type[g])).name.removeprefix('mjGEOM_').lower()
        types[typ]=types.get(typ,0)+1
        records.append({'body':name(m,mujoco.mjtObj.mjOBJ_BODY,m.geom_bodyid[g]),
                        'geom':name(m,mujoco.mjtObj.mjOBJ_GEOM,g),'type':typ,
                        'contype':int(m.geom_contype[g]),'conaffinity':int(m.geom_conaffinity[g])})
    m.vis.global_.offwidth=1200;m.vis.global_.offheight=1200
    m.vis.headlight.ambient[:]=[.55,.55,.55];m.vis.headlight.diffuse[:]=[.65,.65,.65]
    m.vis.headlight.specular[:]=[.12,.12,.12]
    return m,d,active,visual,{'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'collision_geom_count_excluding_world':int(active.sum()),'collision_types':types,'collision_geoms':records}

def render(model,camera,color,size,visual_context=None):
    m,d,active,visual,_=model
    opt=mujoco.MjvOption();opt.geomgroup[:]=0;opt.geomgroup[3]=1
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL]=True
    opt.flags[mujoco.mjtVisFlag.mjVIS_STATIC]=False
    # Opaque collision-only pass. Everything outside the rendered object white.
    m.geom_group[:]=5;m.geom_group[active]=3
    m.geom_rgba[active]=[*color,1]
    renderer=mujoco.Renderer(m,height=size[1],width=size[0])
    renderer.update_scene(d,camera=camera,scene_option=opt)
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX]=False
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
    raw=renderer.render().copy()
    mask=(raw.max(axis=2)-raw.min(axis=2)>15)&(raw.max(axis=2)>20)
    output=np.full(raw.shape,255,dtype=np.uint8)
    output[mask]=raw[mask]
    renderer.close()
    if visual_context is not None:
        # A separate common CAD-only pass supplies a faint positional outline.
        cm,cd,_,cv,_=visual_context
        cm.geom_group[:]=5;cm.geom_group[cv]=2;cm.geom_rgba[cv]=[.58,.60,.63,1]
        vopt=mujoco.MjvOption();vopt.geomgroup[:]=0;vopt.geomgroup[2]=1
        vopt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL]=False
        rr=mujoco.Renderer(cm,height=size[1],width=size[0]);rr.update_scene(cd,camera=camera,scene_option=vopt)
        rr.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX]=False;rr.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
        vv=rr.render().copy();rr.close();vm=vv.max(axis=2)>10
        # This context is intentionally very light; only colour means collision.
        output[vm&~mask]=(.22*vv[vm&~mask]+.78*255).astype(np.uint8)
    return Image.fromarray(output)

def cam(azimuth,elevation=0,lookat=(0,0,.65),distance=2.05):
    c=mujoco.MjvCamera();c.type=mujoco.mjtCamera.mjCAMERA_FREE;c.orthographic=True
    c.lookat[:]=lookat;c.distance=distance;c.azimuth=azimuth;c.elevation=elevation
    return c

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sdk',type=Path,default=DEMO.parent/'sdk/assets/khan_mini/Khan_mini_contact.xml')
    p.add_argument('--training',type=Path,default=DEFAULT_TRAIN)
    p.add_argument('--output-dir',type=Path,default=DEMO/'docs/images/collision_comparison_new100k')
    args=p.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    # Use the actual new actor's default pose, not two different keyframes.
    profile=json.loads(args.training.with_name('profile.json').read_text())
    defaults=profile['default_joint_pos']
    pose=defaults if isinstance(defaults,dict) else dict(zip(profile['joint_names'],defaults))
    sdk=prepare(args.sdk.resolve(),pose);train=prepare(args.training.resolve(),pose)
    green=(.12,.68,.36);blue=(.14,.42,.88)
    size=(650,850);views=[('正面',0,0),('侧面',90,0),('背面',180,0),('斜前方',35,-12)]
    sheet=Image.new('RGB',(2600,2050),'white');draw=ImageDraw.Draw(sheet)
    draw.text((32,20),'全身碰撞体对比｜当前 SDK 与新 100k 策略训练模型',font=font(42),fill='#182b43')
    draw.text((32,82),'同一关节姿态、同一相机和比例；着色为实际启用的碰撞几何，浅灰仅为共同 CAD 外观定位。',font=font(26),fill='#516176')
    for row,(model,color,title) in enumerate([(sdk,green,'SDK：完整 CAD 接触模型'),(train,blue,'训练：khan_lafan1_delay_covex（整脚凸包 + 非足部简化碰撞）')]):
        y=140+row*920
        draw.text((32,y),title,font=font(31),fill='#166640' if row==0 else '#2358a1')
        for col,(label,az,el) in enumerate(views):
            image=render(model,cam(az,el),color,size,visual_context=train)
            sheet.paste(image,(col*650,y+50));draw=ImageDraw.Draw(sheet)
            draw.text((col*650+20,y+65),label,font=font(25),fill='#526176')
    draw.text((32,1990),'说明：训练碰撞包含 box / capsule / 凸包 mesh；图中空缺表示该处没有独立启用的碰撞体，不表示没有刚体或 visual。',font=font(24),fill='#516176')
    sheet.save(args.output_dir/'whole_body_collision_comparison.png')

    # Additional torso/lower-leg closeups make the different envelopes readable.
    details=Image.new('RGB',(2400,1220),'white');draw=ImageDraw.Draw(details)
    draw.text((25,15),'非足部碰撞放大｜绿色 SDK，蓝色训练；每对视图保持同姿态与同尺度',font=font(35),fill='#182b43')
    panels=[('躯干 / base · 侧面',cam(90,0,(0,0,.88),.97)),
            ('小腿 / 踝 / 脚 · 斜侧',cam(30,-8,(.02,0,.26),.93))]
    for group,(label,camera) in enumerate(panels):
        for col,(model,color,short) in enumerate([(sdk,green,'SDK'),(train,blue,'训练')]):
            x=group*1200+col*600
            image=render(model,camera,color,(600,1000),visual_context=train)
            details.paste(image,(x,135));draw=ImageDraw.Draw(details)
            draw.text((x+20,85),label+' · '+short,font=font(25),fill='#26465f')
    draw.text((25,1150),'脚部两侧均为同一整脚 CAD 凸包；明显差别集中于躯干、base、小腿、髋部及关节支架。',font=font(25),fill='#516176')
    details.save(args.output_dir/'nonfoot_collision_closeup.png')
    manifest={'renderer_mujoco_version':mujoco.__version__,'method':'Actual compiled collision geoms + mjVIS_CONVEXHULL; kinematics only, no mj_step; orthographic common cameras.',
              'sdk':sdk[-1],'training':train[-1],'pose':pose,'base_pose':[0,0,.65,1,0,0,0],
              'images':['whole_body_collision_comparison.png','nonfoot_collision_closeup.png'],
              'note':'Geometry visibility is based on contype/conaffinity; per-pair exclusions/material/forces are not encoded by the colours.'}
    (args.output_dir/'render_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({k:manifest[k] for k in ['renderer_mujoco_version','images']}));print(args.output_dir.resolve())

if __name__=='__main__':main()
