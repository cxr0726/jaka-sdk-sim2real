#!/usr/bin/env python3
"""Compare actual sim/real SuperMimic FSM tapes under identical supplied inputs.

This is an OFFLINE diagnostic: the optional binaries must be the dedicated math
replay drivers, never KpiDeployReal or KpiMujocoSimDemo. No live control is used.
Both drivers receive the same model path and recorded feedback/reference tape.
Real YAML physical target clipping is checked independently rather than hidden.
Requires NumPy and PyYAML; it does not inspect ONNX metadata or start ROS/ZMQ.
"""
from __future__ import annotations
import argparse
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

BLOCKS=(('reference_root_displacement_b',0,15),('reference_world_z',15,20),
        ('reference_joint_pos',20,155),('future_anchor_ori_b',155,185),
        ('projected_gravity_history',185,200),('base_ang_vel_history',200,215),
        ('joint_pos_history',215,350),('joint_vel_history',350,485),
        ('raw_action_history',485,620))


def sha(path):
    with Path(path).open('rb') as f:
        h=hashlib.sha256()
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()


def rows(path):
    with Path(path).open() as f:
        for line in f:
            if line.strip():yield json.loads(line)


def compare(sim,real,config,tolerance):
    config=yaml.safe_load(Path(config).read_text())
    if config['angle_map']!=0:raise ValueError('This comparison checks physical PR targets with angle_map=0')
    sdk=[n.casefold() for n in config['joint_names']]
    policy=[n.casefold() for n in config['jakaminimimic_superzmq']['joint_names']]
    if len(sdk)!=27 or len(policy)!=27 or len(set(sdk))!=27 or len(set(policy))!=27 or set(sdk)!=set(policy):
        raise ValueError('YAML requires two complete unique 27-joint name orders')
    permutation=np.array([sdk.index(n) for n in policy])
    default=np.asarray(config['default_angles'],np.float32).astype(np.float64)[permutation]
    scale=np.asarray(config['jakaminimimic_superzmq'].get('action_scale',config['action_scale']),np.float32)
    if scale.ndim==0:scale=np.full(27,scale,np.float32)
    scale=scale.astype(np.float64)
    lower=np.array(config['down_limit'],float);upper=np.array(config['up_limit'],float)
    lower[[4,5,16,17]]=config.get('pitch_roll_down_limit',[-.52,-.35,-.52,-.35])
    upper[[4,5,16,17]]=config.get('pitch_roll_up_limit',[.61,.35,.61,.35])
    maxima={name:0. for name,_,_ in BLOCKS}
    maxima.update(actions=0.,sim_unclipped_target_formula=0.,real_clipped_target_formula=0.,real_vs_clipped_sim_target=0.,
                  same_sensor_input=0.,same_reference_input=0.,same_alignment=0.)
    clipped=np.zeros(27,np.int64);clip_max=np.zeros(27);resets=0;count=0;statuses={};failures=[]
    sentinel=object()
    for step,(a,b) in enumerate(itertools.zip_longest(rows(sim),rows(real),fillvalue=sentinel)):
        if a is sentinel or b is sentinel:raise ValueError('Tape lengths differ')
        for row in [a,b]:
            if (len(row['obs'])!=620 or len(row['actions'])!=27 or
                len(row['targets_deploy'])!=27 or len(row['frames'])!=5):
                raise ValueError(f'Invalid observation/action/reference dimensions at {step}')
        if bool(a['reset_history'])!=bool(b['reset_history']):raise ValueError(f'reset mismatch at {step}')
        # Different receiver implementations do not share stream-generation
        # semantics. Numeric parity still requires identical explicit history
        # resets, supplied frames and alignment (checked below).
        if a.get('buffer_backend')==b.get('buffer_backend'):
            if a['generation']!=b['generation'] or a['status']!=b['status']:
                raise ValueError(f'status/generation mismatch at {step}')
        resets+=bool(a['reset_history']);statuses[a['status']]=statuses.get(a['status'],0)+1
        def measure(name,x,y):
            x,y=np.asarray(x,float),np.asarray(y,float)
            if x.shape!=y.shape or not np.isfinite(x).all() or not np.isfinite(y).all():
                raise ValueError(f'Invalid shape/nonfinite {name} at step {step}')
            error=float(np.max(abs(x-y))) if x.size else 0.
            maxima[name]=max(maxima[name],error)
            if error>tolerance and len(failures)<20:failures.append({'step':step,'name':name,'error':error})
        for field in ['q_deploy','qd_deploy','omega','robot_quat']:
            measure('same_sensor_input',a[field],b[field])
        for f in range(5):
            for field in ['root_pos','root_quat','joint_pos_buffer']:
                measure('same_reference_input',a['frames'][f][field],b['frames'][f][field])
        measure('same_alignment',a['align_quat'],b['align_quat'])
        for name,lo,hi in BLOCKS:measure(name,a['obs'][lo:hi],b['obs'][lo:hi])
        measure('actions',a['actions'],b['actions'])
        action=np.asarray(b['actions'],np.float32).astype(float)
        expected=np.empty(27);expected[permutation]=default+scale*action
        expected_clip=np.clip(expected,lower,upper)
        measure('sim_unclipped_target_formula',a['targets_deploy'],expected)
        measure('real_clipped_target_formula',b['targets_deploy'],expected_clip)
        measure('real_vs_clipped_sim_target',b['targets_deploy'],np.clip(a['targets_deploy'],lower,upper))
        delta=abs(expected-expected_clip);clipped+=delta>0;clip_max=np.maximum(clip_max,delta)
        count+=1
    if not count:raise ValueError('No matched policy ticks')
    return {'passed':not failures,'steps':count,'reset_count':resets,'status_counts':statuses,'max_abs_errors':maxima,
            'physical_target_clipped_components':int(clipped.sum()),
            'clipping_by_sdk_joint':{n:{'components':int(k),'max_clip_rad':float(v)} for n,k,v in zip(sdk,clipped,clip_max) if k},
            'policy_to_sdk_by_independent_name_match':permutation.tolist(),'failures':failures,
            'sim_tape':str(Path(sim).resolve()),'real_tape':str(Path(real).resolve()),
            'scope':'same supplied frames/state/history contract; different receiver timeline metadata is not compared'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--sim-tape',type=Path)
    p.add_argument('--real-tape',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--tolerance',type=float,default=1e-6)
    p.add_argument('--sim-binary',type=Path)
    p.add_argument('--real-binary',type=Path)
    p.add_argument('--model',type=Path)
    p.add_argument('--input',type=Path,action='append',default=[],help='Recorded supplied inputs; replay using both offline math drivers')
    p.add_argument('--run-dir',type=Path)
    p.add_argument('--max-steps',type=int,default=1000000)
    a=p.parse_args()
    if not math.isfinite(a.tolerance) or a.tolerance<=0 or a.max_steps<1:
        p.error('tolerance must be finite/positive and max-steps must be positive')
    result={'passed':False,'scope':'Same supplied inputs, actual sim and real FSM numerical helpers; no closed-loop/hardware validation',
            'hardware_control_executed':False,'tolerance':a.tolerance,'config':str(a.config.resolve()),
            'config_sha256':sha(a.config),'cases':[]}
    if a.input:
        if not all([a.sim_binary,a.real_binary,a.model,a.run_dir]):p.error('Replay requires --sim-binary --real-binary --model --run-dir')
        if a.sim_binary.name!='SimSuperMimicPolicyReplay' or a.real_binary.name!='RealSuperMimicPolicyTest':
            p.error('Only the dedicated offline math replay executables are accepted')
        a.run_dir.mkdir(parents=True,exist_ok=True)
        result['model_sha256']=sha(a.model)
        result['replay_binaries']={side:{'path':str(binary.resolve()),'sha256':sha(binary)}
                                  for side,binary in [('sim',a.sim_binary),('real',a.real_binary)]}
        for i,source in enumerate(a.input):
            dest=a.run_dir/f'case_{i:02d}';dest.mkdir(exist_ok=True)
            command=[]
            for side,binary in [('sim',a.sim_binary),('real',a.real_binary)]:
                tape=dest/f'{side}.jsonl'
                cmd=[str(binary.resolve()),str(a.config.resolve()),str(a.model.resolve()),str(source.resolve()),str(tape.resolve()),str(a.max_steps)]
                with (dest/f'{side}.log').open('w') as log:subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT,env=os.environ)
                command.append(cmd)
            row=compare(dest/'sim.jsonl',dest/'real.jsonl',a.config,a.tolerance)
            row.update(source=str(source.resolve()),source_sha256=sha(source),commands=command)
            result['cases'].append(row)
            print(f'{source}: steps={row["steps"]} passed={row["passed"]} max={max(row["max_abs_errors"].values()):.3g}',flush=True)
    else:
        if not a.sim_tape or not a.real_tape:p.error('Supply --sim-tape and --real-tape, or replay --input arguments')
        result['cases'].append(compare(a.sim_tape,a.real_tape,a.config,a.tolerance))
    result['passed']=bool(result['cases']) and all(x['passed'] for x in result['cases'])
    result['total_steps']=sum(x['steps'] for x in result['cases'])
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(a.output.resolve())
    return 0 if result['passed'] else 1


if __name__=='__main__':sys.exit(main())
