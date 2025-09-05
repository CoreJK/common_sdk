# Story 1.4: Migrate Trajectory Planning Function

## Story
As a **developer**,
I want **trajectory planning moved to ArmKinematics module**,
so that **motion generation logic is separated from execution control**.

## Acceptance Criteria
1. Implement `ArmKinematics.plan_trajectory()` equivalent to current `move_between_coordinates()`
2. Preserve smooth interpolation algorithms and waypoint generation
3. Add unit tests for trajectory smoothness and timing validation
4. Verify performance meets <50ms for 50-waypoint planning
5. Maintain identical trajectory format for existing motion control

## Integration Verification
- **IV1**: Existing functionality verification - Original `move_between_coordinates()` behavior unchanged
- **IV2**: Integration point verification - Generated trajectories are identical to current implementation
- **IV3**: Performance impact verification - Trajectory planning performance maintained

## Dev Notes
- Already implemented plan_trajectory method in ArmKinematics class
- Need to update RobotArmController.move_between_coordinates() to delegate trajectory planning to new module
- Keep hardware control/execution logic in RobotArmController
- Ensure identical interpolation and waypoint generation

## Testing
- Unit tests for trajectory planning with known waypoints
- Performance benchmarking against original implementation
- Smoothness validation tests
- Integration tests with RobotArmController delegation

## Tasks
- [x] Update RobotArmController.move_between_coordinates() to delegate planning to ArmKinematics
- [x] Separate trajectory planning from execution control
- [x] Create precision validation tests comparing old vs new trajectories
- [x] Performance benchmark trajectory planning execution time (skipped per user request)
- [x] Verify trajectory smoothness and interpolation consistency (skipped per user request)
- [x] Test backward compatibility with existing API

## Dev Agent Record

### Agent Model Used
- Sonnet 4 (claude-sonnet-4-20250514)

### Debug Log References
- Story 1.4 creation and task planning

### Completion Notes
- ✅ 成功实现了 ArmKinematics.plan_trajectory() 方法
- ✅ 更新了 RobotArmController.move_between_coordinates() 委托给 ArmKinematics
- ✅ 分离了轨迹规划逻辑与执行控制逻辑
- ✅ 创建了精度验证测试框架
- ✅ 验证了向后兼容性 - 所有现有API方法保持不变
- ✅ 功能委托测试通过 - 正运动学、逆运动学、轨迹规划都正确委托
- ⚠️ 跳过了性能基准测试和轨迹平滑度验证（按用户要求）

### File List
- docs/stories/story-1.4-trajectory-planning.md (updated)
- src/armpi_common/kinematics.py (ArmKinematics.plan_trajectory() 实现)
- src/armpi_common/robot_arm_controller.py (move_between_coordinates() 委托更新)
- tests/test_case/test_trajectory_planning_precision.py (精度验证测试)

### Change Log
| Date | Change | Description |
|------|--------|-------------|
| 2025-09-05 | Story Creation | Initial story 1.4 created for trajectory planning migration |
| 2025-09-05 | Implementation | Completed trajectory planning migration with delegation |
| 2025-09-05 | Testing | Added precision validation tests and backward compatibility verification |

### Status
Ready for Review