#!/usr/bin/env python3
"""Render close-up CAD/contact screenshots using an existing MuJoCo EGL install.

The grey visual pass and coloured collision pass use one camera. Only the latter
enables mjVIS_CONVEXHULL, so the overlay shows both visual cavities and the actual
compiled mesh collision hull. This script does not simulate or run a policy.
"""
from __future__ import annotations
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
import yaml

DEMO=Path(__file__).resolve().parents[1]
ASSETS=DEMO.parent/'sdk/assets/khan_mini'
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if not Path(FONT).exists():FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'

def font(size):return ImageFont.truetype(FONT,size)

def make_review_model(source: Path, target: Path, isolated_foot=False):
    original=ET.parse(source).getroot()
    if isolated_foot:
        root=ET.Element('mujoco',model='left_foot_collision_review')
        ET.SubElement(root,'compiler',angle='radian',meshdir=str(source.parent/'meshes'))
        asset=ET.SubElement(root,'asset')
        foot_mesh=next(x for x in original.findall('./asset/mesh') if x.get('name')=='Left_ankle_roll_Link')
        asset.append(deepcopy(foot_mesh));world=ET.SubElement(root,'worldbody')
        body=ET.SubElement(world,'body',name='Left_ankle_roll_Link')
        ET.SubElement(body,'geom',type='mesh',mesh='Left_ankle_roll_Link',group='2',
                      contype='0',conaffinity='0',rgba='.64 .67 .71 1')
        ET.SubElement(body,'geom',type='mesh',mesh='Left_ankle_roll_Link',group='3',
                      contype='1',conaffinity='1',rgba='.08 .72 .30 1')
    else:
        root=deepcopy(original);root.find('compiler').set('meshdir',str(source.parent/'meshes'))
        for body in root.findall('.//body'):
            bodyname=body.get('name','')
            for geom in list(body.findall('geom')):
                synthetic=geom.get('name','').endswith('_contact')
                if synthetic:
                    geom.set('group','3')
                    geom.set('rgba','.95 .52 .09 1' if 'ankle_joint' in geom.get('name','') else '.08 .72 .30 1')
                    continue
                active=geom.get('contype','1')!='0' or geom.get('conaffinity','1')!='0'
                if active and bodyname in {'base_link','Left_ankle_pitch_Link','Right_ankle_pitch_Link',
                                           'Left_ankle_roll_Link','Right_ankle_roll_Link'}:
                    collision=deepcopy(geom);collision.set('name',bodyname+'_review_collision')
                    collision.set('group','3');collision.set('rgba','.08 .72 .30 1');collision.set('density','0')
                    body.append(collision)
                geom.set('contype','0');geom.set('conaffinity','0');geom.set('group','2')
                geom.set('rgba','.70 .73 .77 1')
        world=root.find('worldbody')
        # Hide the ground in the review model; no simulation takes place.
        for geom in root.findall('./worldbody/geom'):geom.set('rgba','1 1 1 0')
    vis=root.find('visual')
    if vis is None:vis=ET.SubElement(root,'visual')
    head=vis.find('headlight')
    if head is None:head=ET.SubElement(vis,'headlight')
    head.set('ambient','.5 .5 .5');head.set('diffuse','.7 .7 .7');head.set('specular','.2 .2 .2')
    global_=vis.find('global')
    if global_ is None:global_=ET.SubElement(vis,'global')
    global_.set('offwidth','1200');global_.set('offheight','1200')
    # Remove the original black skybox and replace it with an unobtrusive one.
    assets=root.findall('asset')
    for a in assets:
        for tex in list(a.findall('texture')):
            if tex.get('type')=='skybox':a.remove(tex)
    a=assets[0] if assets else ET.SubElement(root,'asset')
    ET.SubElement(a,'texture',name='review_sky',type='skybox',builtin='flat',
                  rgb1='.98 .985 .995',rgb2='.98 .985 .995',width='512',height='512')
    ET.indent(root);ET.ElementTree(root).write(target,encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(target))

def image(m,d,lookat,distance,azimuth,elevation,size=(900,780)):
    renderer=mujoco.Renderer(m,height=size[1],width=size[0])
    cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:]=lookat;cam.distance=distance;cam.azimuth=azimuth;cam.elevation=elevation
    opt=mujoco.MjvOption();opt.geomgroup[:]=0;opt.geomgroup[2]=1
    renderer.update_scene(d,camera=cam,scene_option=opt);visual=renderer.render().copy()
    opt.geomgroup[:]=0;opt.geomgroup[3]=1;opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL]=True
    renderer.update_scene(d,camera=cam,scene_option=opt);hull=renderer.render().copy();renderer.close()
    # Colour identifies the collision pass; no recomputed contact data is drawn.
    active=(hull[:,:,1].astype(float)-hull[:,:,0].astype(float)>15) | (hull[:,:,0].astype(float)-hull[:,:,2].astype(float)>35)
    output=visual.astype(float);output[active]=.52*hull[active]+.48*visual[active]
    return Image.fromarray(np.uint8(np.clip(output,0,255)))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,default=ASSETS/'Khan_mini_contact.xml')
    p.add_argument('--output-dir',type=Path,default=DEMO/'docs/images/contact_model')
    a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=True)
    scratch=Path(tempfile.mkdtemp(prefix='jaka_contact_render_'))
    foot=make_review_model(a.model,scratch/'foot_review.xml',True);fd=mujoco.MjData(foot);mujoco.mj_forward(foot,fd)
    visual_geom=int(np.flatnonzero(foot.geom_group==2)[0]);mesh=int(foot.geom_dataid[visual_geom])
    vertices=foot.mesh_vert[foot.mesh_vertadr[mesh]:foot.mesh_vertadr[mesh]+foot.mesh_vertnum[mesh]]
    rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,foot.geom_quat[visual_geom])
    points=(vertices@rotation.reshape(3,3).T+foot.geom_pos[visual_geom])*1000
    dimensions=np.ptp(points,axis=0);bottom=points[:,2].min()
    views=[('斜侧视图：包含脚背与连接结构的凸包',135,-22),('侧视图',90,0),('俯视图',90,-89),('底视图',90,89)]
    sheet=Image.new('RGB',(1800,1740),'white');draw=ImageDraw.Draw(sheet)
    draw.text((30,15),'整脚碰撞凸包核对：灰色 visual，透明绿色为编译后的凸包',font=font(31),fill='#18392a')
    for i,(label,az,el) in enumerate(views):
        im=image(foot,fd,[.025,0,-.016],.34,az,el);x=(i%2)*900;y=70+(i//2)*800
        sheet.paste(im,(x,y));draw=ImageDraw.Draw(sheet);draw.text((x+20,y+10),label,font=font(25),fill='#173b2b')
    draw.text((25,1680),f'整脚包围盒：长 {dimensions[0]:.2f} mm × 宽 {dimensions[1]:.2f} mm × 高 {dimensions[2]:.2f} mm；最低点 Z ≈ {bottom:.0f} mm',font=font(24),fill='#18392a')
    draw.text((25,1710),'静态 CAD 核对图；62 mm 包含连接结构，不代表鞋底厚度。并非实机测量或动作测试。',font=font(20),fill='#4b5360')
    sheet.save(a.output_dir/'foot_hull_zoom.png')
    whole=make_review_model(a.model,scratch/'body_review.xml',False);wd=mujoco.MjData(whole)
    cfg=yaml.safe_load((DEMO/'config/KpiConfig.yaml').read_text())
    names={mujoco.mj_id2name(whole,mujoco.mjtObj.mjOBJ_JOINT,j).casefold():j for j in range(whole.njnt)
           if mujoco.mj_id2name(whole,mujoco.mjtObj.mjOBJ_JOINT,j)}
    wd.qpos[:3]=[0,0,.65];wd.qpos[3:7]=[1,0,0,0]
    for name,q in zip(cfg['joint_names'],cfg['init_target']):wd.qpos[whole.jnt_qposadr[names[name.casefold()]]]=q
    mujoco.mj_forward(whole,wd)
    overview=Image.new('RGB',(1800,1320),'white');draw=ImageDraw.Draw(overview)
    draw.text((25,15),'base / 小腿 / 脚部 CAD 接触外包络',font=font(34),fill='#18392a')
    for x,az,label in [(0,135,'斜前视角'),(900,45,'斜后视角')]:
        im=image(whole,wd,[0,0,.38],1.23,az,-15,(900,1120));overview.paste(im,(x,60))
        draw=ImageDraw.Draw(overview);draw.text((x+25,75),label,font=font(27),fill='#18392a')
    draw.text((25,1195),'绿色：主小腿、base、脚及踝支架；橙色：小腿末端机械关节包络',font=font(27),fill='#18392a')
    manifest_path=a.model.with_suffix('.manifest.json')
    partition=json.loads(manifest_path.read_text())['lower_leg_partition'][0]['cut_z_m'] if manifest_path.exists() else -.25
    draw.text((25,1235),f'小腿在 knee 局部 Z={partition*1000:.0f} mm 分区（踝中心上方 {(partition+.30)*1000:.0f} mm）。仅末端与同侧脚的内部配合被屏蔽。',font=font(23),fill='#4b5360')
    draw.text((25,1272),'主小腿↔同侧脚、所有异侧脚-腿、脚-脚碰撞保留；髋部仅排除 base↔同侧 hip_roll 内部包络。',font=font(22),fill='#4b5360')
    overview.save(a.output_dir/'body_contact_overview.png')
    (a.output_dir/'render_manifest.json').write_text(json.dumps({'source_model':str(a.model.resolve()),
        'source_model_sha256':hashlib.sha256(a.model.read_bytes()).hexdigest(),
        'renderer_mujoco_version':mujoco.__version__,'method':'Static visual pass plus compiled convex-hull collision pass; no policy or mj_step.',
        'images':['foot_hull_zoom.png','body_contact_overview.png'],'note':'CAD-envelope approximation for review, not calibrated hardware contact material/geometry.'},indent=2)+'\n')
    shutil.rmtree(scratch)
    print(str(a.output_dir.resolve()))

if __name__=='__main__':main()
