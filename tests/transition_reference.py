"""Offline oracle: compare the real Python manager's whole post-step schedule."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import numpy as np


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--reference-root",required=True,type=Path)
    parser.add_argument("--out",required=True,type=Path)
    parser.add_argument("--cpp-driver",required=True,type=Path)
    args=parser.parse_args()
    sys.path.insert(0,str(args.reference_root))
    from box import Box
    from robojudo.config.g1.env.g1_env_cfg import G1_29DoF
    from robojudo.config.g1.policy.g1_asap_policy_cfg import G1AsapLocoPolicyCfg,G1AsapPolicyCfg
    from robojudo.pipeline.rl_loco_mimic_pipeline import PolicyInterpManager
    dof=G1_29DoF()
    defaults=np.asarray(dof.default_pos,dtype=float)
    loco=defaults.copy();loco[16]=.3;loco[18]=1;loco[23]=-.3;loco[25]=1
    class Env:
        def __init__(self):
            self.dof_cfg=dof;self.default_pos=defaults.copy();self.dof_pos=defaults.copy()
        def reset(self):
            pass
        def update_dof_cfg(self,**kwargs):
            pass
    args.out.mkdir(parents=True,exist_ok=True)
    summary={}
    for direction in ["enter","return"]:
        env=Env()
        manager=PolicyInterpManager(G1AsapLocoPolicyCfg(),[G1AsapPolicyCfg()],env,loco_dof_pos=loco)
        init=manager.policy_by_id(1).get_init_dof_pos()
        manager._current_policy_id=0 if direction=="enter" else 1
        manager.override_dof_pos=(loco if direction=="enter" else init).copy()
        if direction=="enter":manager.switch_to_mimic()
        else:manager.switch_to_loco()
        records=[];inputs=[]
        for frame in range(112):
            measured=defaults+.001*np.arange(29)+.0001*frame
            env.dof_pos=measured.copy()
            target_id=1 if direction=="enter" else 0
            complete=manager.interp_state==manager.InterpState.IDLE
            records.append(np.concatenate([[int(manager.current_policy_id==target_id),int(complete)],manager.override_dof_pos]))
            inputs.append(measured)
            state=Box(dof_pos=measured,dof_vel=np.zeros(29),base_quat=np.array([0.,0.,0.,1.]),base_ang_vel=np.zeros(3))
            ctrl=Box()
            if manager.current_policy_id==0:ctrl["ref_dof_pos"]=manager.override_dof_pos.copy()
            manager.step(state,ctrl)
        input_path=args.out/f"{direction}.input.txt";output_path=args.out/f"{direction}.cpp.txt"
        np.savetxt(input_path,inputs,fmt="%.17g")
        np.savetxt(args.out/f"{direction}.python.txt",records,fmt="%.17g")
        subprocess.run([str(args.cpp_driver),direction,str(input_path),str(output_path)],check=True)
        actual=np.loadtxt(output_path);expected=np.asarray(records)
        error=np.abs(actual-expected)
        summary[direction]={"frames":112,"max_abs":float(error.max()),"pass":bool(np.all(error<=1e-12)),"first_target_frame":int(np.flatnonzero(actual[:,0])[0]),"completed_frame":int(np.flatnonzero(actual[:,1])[0])}
        print(direction,json.dumps(summary[direction]))
    (args.out/"summary.json").write_text(json.dumps(summary,indent=2))
    if not all(v["pass"] for v in summary.values()):raise SystemExit(1)


if __name__=="__main__":main()
