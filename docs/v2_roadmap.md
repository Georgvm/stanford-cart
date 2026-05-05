# v2 roadmap (after v1 ships)

After the closed-course MVP works 10/10. None of this is on the critical
path for shipping.

## Localization
- BerryIMU integration: re-enable cuVSLAM `enable_imu_fusion: true` and EKF imu0
- Hierarchical 3DGS map of the route on H200 (modal_pipeline/build_map.py)
- ACE / GLACE scene-coordinate-regression for sub-meter relocalization,
  <10ms inference on Thor
- GS-CPR test-time pose refinement against the 3DGS map

## Perception
- Real depth source for nvblox (Depth Anything V2 published as
  `/front_wide/depth`, OR proper stereo from front_narrow+front_wide)
- Multi-camera nvblox fusion (use all 4 cams instead of front_wide only)
- Pedestrian intent prediction (PIE / JAAD pretrained)

## Planning
- TEB or MPPI controller for sharper turns
- Pedestrian-aware behavior tree branches
- Lane-graph layer for intersections

## Safety
- Voice/horn output ("self-driving cart, please excuse me")
- Visible warning lights
- Telemetry to a remote dashboard (Foxglove)
- CARLA replay tests against recorded bags before public-area drives

## Productionization
- CI for cart_ws packages
- Replay tests in CI against the recorded bag dataset
- One-button rollback if a config change regresses behavior
- udev rules committed (`docs/jetson_setup.md` template)
- camera intrinsics / extrinsics in version-controlled YAML
