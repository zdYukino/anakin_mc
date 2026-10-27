"""Replay recorded observations through unchanged Python inference/postprocessing.

This verifies computation on recorded observations. It deliberately makes no claim
about feedback acquisition, dynamics, initialization or floating stability.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-root", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(args.reference_root.resolve()))
    import onnxruntime
    from robojudo.config.g1.env.g1_env_cfg import G1_29DoF
    from robojudo.config.g1.policy.g1_asap_policy_cfg import G1AsapLocoPolicyCfg, G1AsapPolicyCfg
    from robojudo.pipeline.rl_pipeline import PolicyWrapper

    if onnxruntime.__version__ != "1.20.1":
        raise RuntimeError("This matched-runtime replay requires isolated Python ORT 1.20.1")
    rows = [json.loads(line) for line in args.trace.read_text().splitlines()]
    if not rows:
        raise RuntimeError("No inference rows to replay")
    configs = {"asap_loco": G1AsapLocoPolicyCfg, "asap_cr7": G1AsapPolicyCfg}
    wrappers = {name: PolicyWrapper(configs[name](), G1_29DoF(), "cpu")
                for name in {row["model"] for row in rows}}
    raw_max, q_max = 0., 0.
    for row in rows:
        wrapper = wrappers[row["model"]]
        observation = np.asarray(row["observation"], dtype=np.float32)
        raw = wrapper.policy.session.run(["action"], {"actor_obs": observation[None, :]})[0].reshape(-1)
        q_ref = wrapper.get_pd_target(observation)
        raw_max = max(raw_max, float(np.max(np.abs(raw-np.asarray(row["raw_action"])))))
        q_max = max(q_max, float(np.max(np.abs(q_ref-np.asarray(row["policy_q_ref"])))))
    report = {"pass": raw_max == 0 and q_max == 0, "python_ort": onnxruntime.__version__,
              "frames": len(rows), "trace": str(args.trace), "max_raw_action_abs": raw_max,
              "max_canonical_q_ref_abs": q_max, "sdk_channels": False,
              "scope": "Recorded-observation inference/postprocessing only; no acquisition or dynamics claim"}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report))
    if not report["pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
