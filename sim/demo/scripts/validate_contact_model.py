#!/usr/bin/env python3
"""Compile and inspect the CAD contact variant using an existing MuJoCo install.

Reference-pose contacts are kinematic geometry checks, not closed-loop forces.
"""
from pathlib import Path
from collections import defaultdict
import argparse
import json
import numpy as np
import mujoco
import yaml
from generate_contact_model import CONTACT_BITS,contact_truth_table

DEMO=Path(__file__).resolve().parents[1]
ASSETS=DEMO.parent/'sdk/assets/khan_mini'
ADDED={'base_link','Left_knee_Link','Right_knee_Link','Left_ankle_pitch_Link','Right_ankle_pitch_Link'}

def compare(a,b):
    fields=['body_mass','body_inertia','body_ipos','body_iquat','body_pos','body_quat',
            'jnt_range','jnt_axis','jnt_pos','dof_armature','dof_damping','dof_frictionloss',
            'actuator_gear','eq_data','qpos0']
    result={}
    for key in fields:
        av,bv=getattr(a,key),getattr(b,key)
        if av.shape!=bv.shape:raise ValueError((key,av.shape,bv.shape))
        result[key]=float(np.max(abs(av-bv))) if av.size else 0.
    if any(v!=0. for v in result.values()):raise ValueError(result)
    # New collision pieces change geom count. Match all old visual/ground geoms
    # by body/type/original mesh id and prove their geometric/material data did
    # not change, instead of comparing shifted array indices.
    geom_fields=['geom_pos','geom_quat','geom_size','geom_friction','geom_solref',
                 'geom_solimp','geom_margin','geom_gap']
    geom_diff={key:0. for key in geom_fields}
    for g in range(a.ngeom):
        matches=np.flatnonzero((b.geom_bodyid==a.geom_bodyid[g]) &
                               (b.geom_type==a.geom_type[g]) &
                               (b.geom_dataid==a.geom_dataid[g]))
        if len(matches)!=1:raise ValueError(('ambiguous original geom',g,matches))
        for key in geom_fields:
            delta=float(np.max(abs(np.asarray(getattr(a,key)[g])-getattr(b,key)[matches[0]])))
            geom_diff[key]=max(geom_diff[key],delta)
    if any(geom_diff.values()):raise ValueError(geom_diff)
    result.update({'original_'+key:v for key,v in geom_diff.items()})
    return result

def body_name(m,i):return mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,i)

def contacts(m,d):
    result=[]
    for c in d.contact[:d.ncon]:
        bodies=[body_name(m,int(m.geom_bodyid[g])) for g in c.geom]
        if not any(b in ADDED for b in bodies):continue
        result.append({'bodies':bodies,'distance_m':float(c.dist),'geoms':list(map(int,c.geom))})
    return result

def reference_scan(m,paths):
    jid={mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,i).casefold():i
         for i in range(m.njnt) if mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,i)}
    d=mujoco.MjData(m);result={}
    cfg=yaml.safe_load((DEMO/'config/KpiConfig.yaml').read_text())
    d.qpos[:]=m.qpos0;d.qpos[:3]=[0,0,.65];d.qpos[3:7]=[1,0,0,0]
    for name,q in zip(cfg['joint_names'],cfg['init_target']):d.qpos[m.jnt_qposadr[jid[name.casefold()]]]=q
    mujoco.mj_forward(m,d)
    result['default_standing']={'contacts_involving_new_geoms':contacts(m,d),'all_contact_count':d.ncon}
    for path in paths:
        if not path.exists():continue
        data=np.load(path,allow_pickle=False);mapping=[m.jnt_qposadr[jid[n.casefold()]] for n in data['joint_names'].tolist()]
        fps=float(data['fps'][0]);samples=[];pairs=defaultdict(lambda:{'frames':0,'minimum_distance_m':0.})
        for index in range(0,len(data['joint_pos']),max(1,int(fps))):
            d.qpos[:]=m.qpos0;d.qpos[:3]=data['body_pos_w'][index,0];d.qpos[3:7]=data['body_quat_w'][index,0]
            d.qpos[mapping]=data['joint_pos'][index];mujoco.mj_forward(m,d)
            seen=set()
            for c in contacts(m,d):
                pair=tuple(sorted(c['bodies']));key=' / '.join(pair)
                pairs[key]['minimum_distance_m']=min(pairs[key]['minimum_distance_m'],c['distance_m'])
                if key not in seen:pairs[key]['frames']+=1;seen.add(key)
            if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qacc)):
                raise ValueError(f'Nonfinite reference forward result: {path}:{index}')
            samples.append(index)
        result[path.name]={'sampled_frames':len(samples),'pairs_involving_new_geoms':dict(pairs),
                           'note':'Direct reference poses, not actual controlled penetration or contact forces.'}
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,default=ASSETS/'Khan_mini_contact.xml')
    p.add_argument('--legacy',type=Path,default=ASSETS/'Khan_mini_sym.xml')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();old=mujoco.MjModel.from_xml_path(str(a.legacy));new=mujoco.MjModel.from_xml_path(str(a.model))
    compiled_masks={}
    for side,label in [('Left','left'),('Right','right')]:
        bid=mujoco.mj_name2id(new,mujoco.mjtObj.mjOBJ_BODY,side+'_ankle_roll_Link')
        foot=np.flatnonzero((new.geom_bodyid==bid)&((new.geom_contype!=0)|(new.geom_conaffinity!=0)))
        if len(foot)!=1:raise ValueError(('expected whole-mesh foot',side,foot))
        for role,gid in [(label+'_foot',int(foot[0])),
                         (label+'_ankle_joint_envelope',mujoco.mj_name2id(new,mujoco.mjtObj.mjOBJ_GEOM,side+'_knee_ankle_joint_contact')),
                         ('external',mujoco.mj_name2id(new,mujoco.mjtObj.mjOBJ_GEOM,side+'_knee_main_contact'))]:
            actual=(int(new.geom_contype[gid]),int(new.geom_conaffinity[gid]))
            if actual!=CONTACT_BITS[role]:raise ValueError((role,actual,CONTACT_BITS[role]))
            compiled_masks[side+'/'+role]=actual
    result={'compiler_version':mujoco.__version__,'old_dimensions':[old.nq,old.nv,old.nu,old.neq],
            'new_dimensions':[new.nq,new.nv,new.nu,new.neq],
            'unchanged_numeric_fields_max_abs_difference':compare(old,new),
            'old_active_collision_geoms':int(np.sum((old.geom_contype!=0)|(old.geom_conaffinity!=0))),
            'new_active_collision_geoms':int(np.sum((new.geom_contype!=0)|(new.geom_conaffinity!=0))),
            'compiled_collision_masks':compiled_masks,'collision_mask_truth_table':contact_truth_table(),
            'reference_pose_checks':reference_scan(new,[DEMO/'scripts'/n for n in
                ['walk2_subject1.npz','dance1_subject2.npz','fallAndGetUp2_subject2.npz']])}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
