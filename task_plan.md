# UIKA Deployment/Training Review Plan

## Goal
Find likely deployment bugs relative to `/home/saiki/project/UIKA_lab`, with emphasis on observations, actions, scales, joint order, and joint mapping.

## Phases
- [complete] Phase 1: Locate deployment policy/config and training UIKA env definitions.
- [complete] Phase 2: Extract training observation/action contract.
- [complete] Phase 3: Extract deployment observation/action contract.
- [complete] Phase 4: Compare scales, joint names, mappings, defaults, and dimensions.
- [complete] Phase 5: Report findings with file/line references and risk level.

## Errors Encountered
- `python -m pytest src/rl_sar/test/test_uika_integration.py -q` failed because the active Python environment has no `pytest` module.
