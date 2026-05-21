# UIKA Deployment/Training Review Findings

## Training Contract
- UIKA training joint order is SDK order: `FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf`.
- Policy single-frame observation order is `commands(3), ang_vel(3), gravity_vec(3), dof_pos(12), dof_vel(12), actions(12)`, total 45.
- HimLoco history length is 5 past frames plus current, so deployed TorchScript expects 270 input values.
- Action scale is hip 0.125, all thigh/calf 0.25.
- Training actuator gains are stiffness 30.0 and damping 1.0.

## Deployment Matches
- `policy/uika/himloco/config.yaml` has the correct observation order, 45 one-step dimension, 6 history frames, action scale, obs scales, default policy posture, and identity joint mapping for current training source.
- `policy.pt` accepts input 270 and outputs 12 actions.
- Observation history order is newest to oldest, matching the HimLoco wrapper.

## Findings
- Deployment `rl_kp` is 20.0 while training actuator stiffness is 30.0. This changes action execution dynamics.
- Navigation mode `/cmd_vel` values bypass command range clamping before entering observations.
- Real UIKA topic path hardcodes MotorFeedback12/MotorCommand12 order and does not apply `joint_mapping`; this is only safe while hardware message field order is exactly SDK/training order.
- `base.yaml` default hip posture is +/-0.75 while training and policy config use +/-0.78; this affects get-up/pre-policy posture.
- `test_uika_integration.py` is stale relative to current config and references a non-existent `/home/esd/.../deploy.yaml` path.
