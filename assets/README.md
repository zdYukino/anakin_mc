# Default deployment models

Copied unchanged from the existing Anakin research assets. Both files are installed under `share/anakin_mc/assets/`; their directory layout is preserved for `ANAKIN_ASSETS` compatibility.

| Model | SHA256 |
| --- | --- |
| `models/g1/asap/dec_loco/20250109_231507-noDR_rand_history_loco_stand_height_noise-decoupled_locomotion-g1_29dof/model_6600.onnx` | `188aea28fac59adc561b6c5effabc8add608aab3955368755e322831de9af652` |
| `models/g1/asap/mimic/CR7_level1/model_191500.onnx` | `100b853d5f66a0bf53c03d2bea1064430b52723d2a1bdd5ba19748c60497a67e` |

These match the numerical reference recorded in `docs/numerical-reference-ort120.json`. Source attribution is retained in `NOTICE.md` and `LICENSE-Anakin`. Runtime overrides remain available through `ANAKIN_LOCO_MODEL` and `ANAKIN_MIMIC_MODEL`.
