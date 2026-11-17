# Robot asset preparation

These scripts convert explicitly supplied robot descriptions into portable ManiMux
assets. They do not download sources, install SDKs or connect hardware. Each
component README records its source revision, license and geometry conventions.

| Script | Required local input | Generated component |
| --- | --- | --- |
| `prepare_aloha_agilex.py` | `--source` RoboTwin `aloha-agilex` directory | [ALOHA follower arms](../../manimux/embodiments/arm/aloha_agilex/README.md): two URDFs and nine DAE meshes |
| `prepare_piper.py` | `--upstream` pinned `piper_ros` checkout | [Standard PiPER](../../manimux/embodiments/arm/piper/README.md): visual/kinematic URDF, ten unchanged STL meshes and MIT notice |
| `prepare_arx_x5.py` | `--urdf` official `x5.urdf` and `--license` upstream notice | [X5 (2023)](../../manimux/embodiments/arm/arx_x5/README.md): kinematic-only URDF and BSD notice |
| `prepare_tianji_wuji.py` | `--urdf` selected Marvin description and `--package-root` mesh package | [Tianji](../../manimux/embodiments/arm/tianji/README.md): two local bare-arm descriptions and unchanged STL meshes |

Run from the repository root with a ManiMux interpreter. Use `--help` for arguments.
By default, output goes to the corresponding bundled arm component; `--output`
selects a separate directory for comparison. Preserve the documented source pin
and license when regenerating. Review changes to joints, TCP and gripper mapping
before replacing existing models.

The ALOHA generator writes geometry under `assets/robotwin/`; its MIT notice is
retained separately in `assets/LICENSE.txt`. Keep that notice with redistributed
assets. PiPER and X5 generators also copy their supplied upstream license.

Keep upstream checkouts, generated validation reports and temporary comparisons
in ignored local workspaces. SDK source and compiled libraries are not asset output.

Tianji output stays in ignored `assets/marvin/`, separately from the existing
runtime visual paths. Its local manifest retains source hashes, joint names and
the absence of a Wuji mounting transform. It never changes SDK configuration or
station bindings. Existing output is not overwritten. Run the matching geometry
audit before selecting a description for control or display; see the
[owning contract](../../.agents/skills/manimux-development/references/tianji-wuji.md).
