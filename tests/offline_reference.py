"""Offline-only numerical oracle using the unchanged Python research implementation."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-root", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--cpp-driver", required=True, type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.reference_root))
    from box import Box
    import onnxruntime
    from robojudo.config.g1.env.g1_env_cfg import G1_29DoF
    from robojudo.config.g1.policy.g1_asap_policy_cfg import G1AsapLocoPolicyCfg, G1AsapPolicyCfg
    from robojudo.pipeline.rl_pipeline import PolicyWrapper
    import robojudo.policy.asap_policy as implementation

    args.out.mkdir(parents=True, exist_ok=True)
    summary = {"python_ort": onnxruntime.__version__, "results": {}}
    defaults = np.asarray(G1_29DoF().default_pos, dtype=np.float64)
    for model_id, cfg in [("asap_loco", G1AsapLocoPolicyCfg()), ("asap_cr7", G1AsapPolicyCfg())]:
        wrapper = PolicyWrapper(cfg, G1_29DoF(), "cpu")
        policy = wrapper.policy
        fixture, expected = [], []
        for frame in range(224):
            wall = 1000 + frame * 0.02
            implementation.time.time = lambda wall=wall: wall
            joints = np.arange(29)
            q = defaults + 0.02 * np.sin(joints * 0.37 + frame * 0.19)
            dq = 0.13 * np.cos(joints * 0.23 + frame * 0.17)
            xyz = np.array([0.07 * np.sin(frame * 0.1), -0.11 * np.cos(frame * 0.05), 0.03 * np.sin(frame * 0.07)])
            quat = np.append(xyz, np.sqrt(1 - np.dot(xyz, xyz)))
            ang = np.array([0.1 + frame * 0.01, -0.15, 0.07 * np.sin(frame * 0.1)])
            command = np.array([0.14 * np.sin(frame * 0.13), -0.07, 0.11])
            stand, height = int(frame % 3 != 0), 0.78 + 0.01 * np.sin(frame * 0.1)
            upper = defaults.copy()
            upper[12:] = 0.03 * np.cos(joints[12:] * 0.21 + frame * 0.1)
            if model_id == "asap_loco":
                policy.lin_vel_command = command[:2].copy()
                policy.ang_vel_command = command[2:].copy()
                policy.stand_command = np.array([stand])
                policy.base_height_command = np.array([height])
            state = Box(dof_pos=q, dof_vel=dq, base_quat=quat, base_ang_vel=ang)
            obs, extras = wrapper.get_observation(state, Box(ref_dof_pos=upper))
            raw = policy.session.run(["action"], {"actor_obs": np.expand_dims(obs, 0).astype(np.float32)})[0].reshape(-1)
            q_ref = wrapper.get_pd_target(obs)
            expected.append(np.concatenate([np.asarray(obs, dtype=np.float32), raw, q_ref, [int("[MOTION_DONE]" in extras.get("CALLBACK", []))]]))
            fixture.append(np.concatenate([[wall], q, dq, quat, ang, command, [stand, height], upper]))
            policy.post_step_callback([])
        input_path = args.out / f"{model_id}.input.txt"
        expected_path = args.out / f"{model_id}.python.txt"
        output_path = args.out / f"{model_id}.cpp.txt"
        np.savetxt(input_path, fixture, fmt="%.17g")
        np.savetxt(expected_path, expected, fmt="%.17g")
        subprocess.run([str(args.cpp_driver), model_id, cfg.policy_file, str(input_path), str(output_path)], check=True)
        actual, expected = np.loadtxt(output_path), np.asarray(expected)
        obs_n, act_n = (500, 12) if model_id == "asap_loco" else (380, 23)
        slices = {"observation_and_grouped_history": (0, obs_n, 3e-6), "raw_action": (obs_n, obs_n + act_n, 5e-5), "canonical_q_ref": (obs_n + act_n, obs_n + act_n + 29, 1.3e-5), "motion_done_before_callback": (obs_n + act_n + 29, obs_n + act_n + 30, 0)}
        results = {}
        for name, (start, end, tolerance) in slices.items():
            delta = np.abs(actual[:, start:end] - expected[:, start:end])
            results[name] = {"max_abs": float(delta.max()), "tolerance": tolerance, "pass": bool(np.all(delta <= tolerance))}
        summary["results"][model_id] = {"frames": len(fixture), "model_path": cfg.policy_file, "model_sha256": hashlib.sha256(Path(cfg.policy_file).read_bytes()).hexdigest(), "comparisons": results}
        print(model_id, json.dumps(results))
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2))
    if not all(result["pass"] for m in summary["results"].values() for result in m["comparisons"].values()):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
