#!/usr/bin/env python3
"""Offline audit of real YAML versus a trusted SuperMimic ONNX export.

This test may inspect metadata to validate the manually configured policy
contract. Deployment does not read these joint parameters from metadata.
No RobotIntf, ROS node, ZMQ socket, or hardware process is constructed.
Run with an existing Python installation containing NumPy, PyYAML and ORT.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
import onnxruntime as ort
import yaml


def require(condition, message):
    if not condition:
        raise ValueError(message)


def numeric_csv(metadata, key):
    result=np.asarray([float(x) for x in metadata[key].split(',')],dtype=np.float64)
    require(result.shape==(27,) and np.isfinite(result).all(),f'invalid export metadata {key}')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--baseline-ref',help='Optional Git revision to prove all hardware gains/defaults are unchanged')
    parser.add_argument('--require-reference-residual',action='store_true',
                        help='Also verify the embedded reference skip computation and dedicated PR gains')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    config_path=args.config.resolve()
    cfg=yaml.safe_load(config_path.read_text())
    scfg=cfg['jakaminimimic_superzmq']
    sdk_names=[x.strip().casefold() for x in cfg['joint_names']]
    policy_names=[x.strip().casefold() for x in scfg['joint_names']]
    require(len(sdk_names)==27 and len(set(sdk_names))==27,'SDK order must have 27 unique joints')
    require(len(policy_names)==27 and len(set(policy_names))==27,'policy order must have 27 unique joints')
    require(set(policy_names)==set(sdk_names),'SDK/policy joint sets differ')
    options=ort.SessionOptions()
    options.intra_op_num_threads=1;options.inter_op_num_threads=1
    session=ort.InferenceSession(str(args.model),sess_options=options,providers=['CPUExecutionProvider'])
    inputs,outputs=session.get_inputs(),session.get_outputs()
    require(len(inputs)==len(outputs)==1,'expected one actor input/output')
    require(inputs[0].name=='obs' and inputs[0].type=='tensor(float)' and inputs[0].shape[1]==620,
            'expected float32 obs[batch,620]')
    require(outputs[0].name=='actions' and outputs[0].type=='tensor(float)' and outputs[0].shape[1]==27,
            'expected float32 actions[batch,27]')
    metadata=session.get_modelmeta().custom_metadata_map
    model_names=[x.strip().casefold() for x in metadata['joint_names'].split(',')]
    require(policy_names==model_names,'manual YAML policy order differs from actual ONNX training order')
    schema=json.loads(metadata['onnx_deployment_schema'])
    require(schema['dimensions']=={'obs':620,'goal':155,'policy_obs':465,'actions':27},'incorrect actor dimensions')
    require(schema['packed_order']==['goal','policy_obs'],'incorrect packed actor order')
    require(schema['future_offsets']==[0,1,2,3,4] and schema['history']==5,'incorrect window/history')
    require(metadata['anchor_body_name']=='base_link','this deployment requires base_link anchor')
    permutation=[sdk_names.index(x) for x in policy_names]
    defaults=np.asarray(cfg['default_angles'],dtype=float)
    require(defaults.shape==(27,) and np.isfinite(defaults).all(),'invalid SDK default_angles')
    model_defaults=numeric_csv(metadata,'default_joint_pos')
    np.testing.assert_allclose(defaults[permutation],model_defaults,atol=1e-6,rtol=0,
                               err_msg='YAML q0 differs from training; affects both q-q0 observation and action target')
    scale=np.asarray(scfg.get('action_scale',cfg['action_scale']),dtype=float)
    if scale.ndim==0:scale=np.full(27,float(scale))
    require(scale.shape==(27,) and np.isfinite(scale).all() and np.all(scale>0),'invalid YAML action_scale')
    np.testing.assert_allclose(scale,numeric_csv(metadata,'action_scale'),atol=1e-7,rtol=0)
    terms={x['name']:x for x in schema['policy_terms']}
    require(np.isclose(scfg['ang_vel_scale'],terms['base_ang_vel']['caller_scale']),'YAML angular velocity scale differs from training')
    require(np.isclose(scfg['dof_vel_scale'],terms['joint_vel']['caller_scale']),'YAML joint velocity scale differs from training')
    require(np.isclose(scfg['dt_s'],.02) and np.isclose(cfg['pln_Ts'],.02),'real control/window dt must be 20ms')
    require(scfg['future_steps']==[0,1,2,3,4],'future steps must be consecutive five frames')
    gains={k:cfg[k] for k in ['kp','kd','kp_pitch_roll','kd_pitch_roll']}
    for key,values in gains.items():
        v=np.asarray(values,dtype=float);expected=4 if 'pitch_roll' in key else 27
        require(v.shape==(expected,) and np.isfinite(v).all() and np.all(v>=0),f'invalid hardware {key}')
    baseline_checked=False
    if args.baseline_ref:
        repo=Path(subprocess.check_output(['git','-C',str(config_path.parent),'rev-parse','--show-toplevel'],text=True).strip())
        relative=config_path.relative_to(repo).as_posix()
        baseline=yaml.safe_load(subprocess.check_output(['git','-C',str(repo),'show',f'{args.baseline_ref}:{relative}'],text=True))
        for key in ['kp','kd','kp_pitch_roll','kd_pitch_roll','default_angles','init_target','angle_map']:
            require(cfg[key]==baseline[key],f'hardware YAML {key} changed from authorized baseline {args.baseline_ref}')
        baseline_checked=True
    residual_check=None
    if args.require_reference_residual:
        # Metadata is an offline assertion only; runtime values stay YAML-owned.
        import copy
        import onnx
        action=schema['action']
        require(action.get('parameterization')=='reference_residual' and
                action.get('reference_skip_embedded') is True and
                action.get('reference_frame_offset')==1 and
                action.get('reference_residual_beta')==1.0,'unsupported residual export')
        require(action['pd_target']=='default_joint_pos + action_scale * actions',
                'use the full ONNX action without a second reference add')
        require(cfg['angle_map']==0,'residual real deployment requires PR PD')
        require(float(scfg.get('reference_height_offset',0))==0,'reference height must remain unshifted')
        ankle_names=['left_ankle_pitch_joint','left_ankle_roll_joint',
                     'right_ankle_pitch_joint','right_ankle_roll_joint']
        ankle_indices=[policy_names.index(n) for n in ankle_names]
        np.testing.assert_allclose(cfg['kp_pitch_roll'],numeric_csv(metadata,'joint_stiffness')[ankle_indices],atol=1e-6,rtol=0)
        np.testing.assert_allclose(cfg['kd_pitch_roll'],numeric_csv(metadata,'joint_damping')[ankle_indices],atol=1e-6,rtol=0)
        model=onnx.load(str(args.model))
        producers={name:node for node in model.graph.node for name in node.output}
        final=producers['actions']
        require(final.op_type=='Add','expected exported decoder + reference skip')
        decoder=[name for name in final.input if producers.get(name) is not None and producers[name].op_type=='Gemm']
        require(len(decoder)==1,'cannot identify the exported decoder residual')
        probe=copy.deepcopy(model)
        probe.graph.output.append(onnx.helper.make_tensor_value_info(decoder[0],onnx.TensorProto.FLOAT,['batch',27]))
        onnx.checker.check_model(probe)
        probe_session=ort.InferenceSession(probe.SerializeToString(),sess_options=options,providers=['CPUExecutionProvider'])
        rng=np.random.default_rng(20261111)
        obs=rng.normal(0,.2,(128,620)).astype(np.float32)
        q0=model_defaults.astype(np.float32);alpha=scale.astype(np.float32)
        obs[:,47:74]=q0+rng.normal(0,.35,(128,27)).astype(np.float32)
        total,delta=probe_session.run(['actions',decoder[0]],{'obs':obs})
        raw_error=float(np.max(abs(total-(delta+(obs[:,47:74]-q0)/alpha))))
        target_error=float(np.max(abs((q0+alpha*total)-(obs[:,47:74]+alpha*delta))))
        require(raw_error<2e-6 and target_error<2e-6,'embedded residual computation differs from deployment')
        residual_check={'samples':128,'reference_frame':1,'obs_reference_slice':[47,74],
                        'raw_action_error':raw_error,'target_equivalence_error_rad':target_error,
                        'dedicated_PR_gains_match':True,'last_action':'full raw ONNX output'}
    result={'passed':True,'model':str(args.model.resolve()),'model_sha256':hashlib.sha256(args.model.read_bytes()).hexdigest(),
            'config':str(config_path),'policy_to_deploy':permutation,'default_joint_pos_max_error':float(np.max(abs(defaults[permutation]-model_defaults))),
            'hardware_pd_source':'YAML, intentionally not required to numerically equal training PD metadata',
            'hardware_gains':gains,'baseline_ref':args.baseline_ref,'baseline_preserved':baseline_checked,
            'runtime_metadata_parameter_reads':False,'reference_residual_check':residual_check,
            'note':'Metadata is inspected by this offline audit only; no hardware code is executed.'}
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
