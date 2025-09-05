# Common SDK Brownfield Enhancement PRD

## Intro Project Analysis and Context

### SCOPE ASSESSMENT REQUIRED

This is clearly a **SIGNIFICANT enhancement** requiring comprehensive planning. The requirements document shows:
- **2500+ line file** needs modular restructuring
- **Multiple methods** need migration (`get_joint_fkine()`, `get_joint_ikine()`, etc.)
- **Backward compatibility** requirements 
- **Performance benchmarks** needed
- **Extensive testing** requirements (90% coverage)

This justifies the full PRD process rather than simpler epic/story creation.

### Existing Project Overview

**Analysis Source**: IDE-based fresh analysis

**Current Project State**: 
The `common_sdk` project is a robotics control system for an ArmPi robotic arm. The core `RobotArmController` class (2504 lines) currently handles:
- Serial communication management
- Servo motor control logic  
- **Kinematics calculations** (the functions to be separated)
- Visual calibration systems
- State management functionality

### Available Documentation Analysis

✓ Requirements Documentation (kinematics-module-separation-requirements.md)
✓ Hand-eye Calibration Guides
✓ Motor Enable Fix Documentation  
✓ OpenCV Window Management Docs
✗ Tech Stack Documentation (missing)
✗ Source Tree/Architecture (missing)
✗ API Documentation (missing)
✗ Coding Standards (missing)

### Enhancement Scope Definition

**Enhancement Type**: ✓ Major Feature Modification (architectural restructuring)

**Enhancement Description**: 
Separate kinematics calculation functions from the monolithic `RobotArmController` class into a dedicated `ArmKinematics` module while maintaining full backward compatibility and improving code maintainability.

**Impact Assessment**: ✓ Significant Impact (substantial existing code changes)

### Goals and Background Context

**Goals**:
• Reduce coupling between kinematics calculations and hardware control
• Enable independent testing of kinematics algorithms  
• Improve maintainability through separation of concerns
• Support future multi-robot-model extensibility
• Maintain 100% backward compatibility with existing APIs

**Background Context**:
The current `RobotArmController` violates single responsibility principle by mixing serial communication, motor control, and mathematical kinematics calculations. This architectural debt makes testing difficult and creates tight coupling between distinct concerns. The separation will enable better testing, clearer code organization, and future extensibility while ensuring existing user code continues working unchanged.

### Change Log

| Change | Date | Version | Description | Author |
|--------|------|---------|-------------|--------|
| Initial PRD Creation | 2025-09-05 | 1.0 | Created comprehensive PRD for kinematics module separation | John (PM Agent) |

## Requirements

### Functional

**FR1**: The new `ArmKinematics` class shall provide forward kinematics calculation equivalent to existing `RobotArmController.get_joint_fkine()` method, accepting joint angles in radians and returning end-effector pose as [x,y,z,rx,ry,rz] vector.

**FR2**: The new `ArmKinematics` class shall provide inverse kinematics calculation equivalent to existing `RobotArmController.get_joint_ikine()` method, accepting target pose and returning joint angles in radians.

**FR3**: The new `ArmKinematics` class shall provide trajectory planning functionality equivalent to existing `RobotArmController.move_between_coordinates()` method, generating smooth interpolated waypoints between start and end poses.

**FR4**: The `RobotArmController` class shall maintain all existing public API methods (`get_joint_fkine()`, `get_joint_ikine()`, `set_joint_move_with_coordinate()`, `move_between_coordinates()`) with identical signatures and return formats for backward compatibility.

**FR5**: The `RobotArmController` class shall internally delegate kinematics calculations to the new `ArmKinematics` module while preserving existing behavior for hardware control logic.

**FR6**: The `ArmKinematics` class shall be instantiable independently of `RobotArmController` to enable standalone kinematics calculations for simulation and planning scenarios.

### Non Functional

**NFR1**: Forward kinematics calculation performance shall not exceed 1ms execution time, maintaining current system responsiveness.

**NFR2**: Inverse kinematics calculation performance shall not exceed 10ms execution time, preserving real-time control capabilities.

**NFR3**: Trajectory planning for 50 waypoints shall complete within 50ms, ensuring smooth motion execution.

**NFR4**: Memory footprint of `ArmKinematics` module instantiation shall not exceed 10MB to maintain system resource efficiency.

**NFR5**: Unit test coverage for the new kinematics module shall achieve minimum 90% code coverage with comprehensive edge case testing.

**NFR6**: All kinematics calculations shall maintain numerical precision equivalent to current implementation, with maximum deviation of 0.001 units for position and 0.001 radians for orientation.

### Compatibility Requirements

**CR1**: **Existing API Compatibility**: All public methods in `RobotArmController` related to kinematics (`get_joint_fkine()`, `get_joint_ikine()`, `set_joint_move_with_coordinate()`, `move_between_coordinates()`) must maintain identical function signatures, parameter types, and return value formats.

**CR2**: **Dependency Compatibility**: Integration with existing `armipi_module.py` DH parameters and `roboticstoolbox`/`spatialmath` libraries must remain unchanged to preserve current mathematical accuracy.

**CR3**: **Error Handling Consistency**: New kinematics module must replicate existing error handling behavior and exception types to ensure client code error handling remains functional.

**CR4**: **Configuration Integration**: Kinematics module must seamlessly integrate with existing robot configuration parameters and coordinate system conventions without requiring user code modifications.

## Technical Constraints and Integration Requirements

### Existing Technology Stack

**Languages**: Python 3.10.12  
**Frameworks**: roboticstoolbox-python (≥1.1.1) for kinematics algorithms, spatialmath for spatial transformations  
**Hardware Interface**: pyserial (≥3.5) for robot communication  
**Database**: None - configuration stored in Python modules  
**Infrastructure**: File-based system with modular Python packages  
**External Dependencies**: NumPy for numerical computations, OpenCV for vision systems

### Integration Approach

**Database Integration Strategy**: No database changes required - current file-based configuration system maintained

**API Integration Strategy**: Preserve existing method signatures in `RobotArmController` while adding internal delegation to new `ArmKinematics` class. New direct access API through controller's `kinematics` property.

**Frontend Integration Strategy**: N/A - this is a backend library with programmatic API only

**Testing Integration Strategy**: Extend existing test infrastructure in `tests/test_case/test_robot_kinematics.py` with comprehensive unit tests for separated kinematics module

### Code Organization and Standards

**File Structure Approach**: New `src/armpi_common/kinematics.py` follows existing package structure. Maintain current import patterns and module organization.

**Naming Conventions**: Follow existing Python conventions - snake_case for functions/variables, PascalCase for classes. Preserve existing method names for backward compatibility.

**Coding Standards**: Maintain existing patterns - detailed docstrings, type hints, comprehensive error handling as seen in current codebase

**Documentation Standards**: Follow existing docstring style with detailed parameter descriptions and return value specifications

### Deployment and Operations

**Build Process Integration**: No changes to existing build pipeline - new module integrates seamlessly into current package structure

**Deployment Strategy**: In-place update approach - maintain backward compatibility to allow gradual rollout without breaking existing deployments

**Monitoring and Logging**: Leverage existing logging infrastructure from `_log.py` module for consistent error reporting and debugging

**Configuration Management**: Reuse existing configuration approach through `armipi_module.py` DH parameters and robot specifications

### Risk Assessment and Mitigation

**Technical Risks**: 
- Numerical precision loss during separation could affect robot accuracy
- Performance regression from additional abstraction layers
- Complex dependency management with roboticstoolbox integration

**Integration Risks**:  
- Breaking changes to existing API contracts during refactoring
- Thread safety issues if kinematics module shared between control contexts
- Memory leaks from improper roboticstoolbox resource management

**Deployment Risks**:
- Existing user code breakage despite backward compatibility efforts  
- Version conflicts with roboticstoolbox dependencies
- Testing coverage gaps for edge cases in real robot scenarios

**Mitigation Strategies**:
- Comprehensive unit tests with numerical precision validation
- Performance benchmarking against current implementation  
- Phased rollout with feature flags for gradual adoption
- Extensive integration testing with actual hardware

## Epic and Story Structure

### Epic Approach

**Epic Structure Decision**: **Single comprehensive epic** with rationale - This brownfield enhancement involves tightly coupled changes across multiple files and components that must be coordinated to maintain system integrity. Separating into multiple epics would create unnecessary integration complexity and deployment risk. A single epic ensures all changes are developed, tested, and deployed cohesively while maintaining backward compatibility throughout the process.

## Epic 1: Kinematics Module Separation

**Epic Goal**: Separate kinematics calculation functions from RobotArmController into a dedicated ArmKinematics module while maintaining 100% backward compatibility and improving code maintainability for future robot model extensions.

**Integration Requirements**: Ensure existing user code continues working unchanged while providing new direct access to kinematics functionality through clean modular interfaces.

### Story 1.1: Create ArmKinematics Module Foundation

As a **developer**,
I want **a new ArmKinematics class with core infrastructure**,
so that **I have a solid foundation for migrating kinematics functions**.

#### Acceptance Criteria
1. Create `src/armpi_common/kinematics.py` with `ArmKinematics` class
2. Implement initialization method accepting RobotArmModule instance
3. Add comprehensive type hints and docstrings following existing patterns
4. Create basic unit test structure in `tests/test_case/test_kinematics.py`
5. Ensure module imports successfully without breaking existing functionality

#### Integration Verification
- **IV1**: Existing functionality verification - All current RobotArmController tests pass unchanged
- **IV2**: Integration point verification - New module integrates cleanly with existing import structure
- **IV3**: Performance impact verification - Module loading adds <10ms to startup time

### Story 1.2: Migrate Forward Kinematics Function

As a **developer**,
I want **forward kinematics calculation moved to ArmKinematics module**,
so that **mathematical computation is separated from hardware control**.

#### Acceptance Criteria
1. Implement `ArmKinematics.forward_kinematics()` method equivalent to current `get_joint_fkine()`
2. Migrate exact algorithm logic preserving numerical precision
3. Add comprehensive unit tests with known input/output validation
4. Verify performance meets <1ms execution requirement
5. Maintain identical return format and error handling behavior

#### Integration Verification
- **IV1**: Existing functionality verification - Original `get_joint_fkine()` behavior unchanged
- **IV2**: Integration point verification - New method produces identical results to original
- **IV3**: Performance impact verification - No performance regression in forward kinematics timing

### Story 1.3: Migrate Inverse Kinematics Function

As a **developer**,
I want **inverse kinematics calculation moved to ArmKinematics module**,
so that **complex mathematical solving is isolated and testable**.

#### Acceptance Criteria
1. Implement `ArmKinematics.inverse_kinematics()` method equivalent to current `get_joint_ikine()`
2. Preserve roboticstoolbox integration patterns and convergence behavior
3. Add comprehensive unit tests including edge cases and failure scenarios
4. Verify performance meets <10ms execution requirement
5. Maintain identical error handling for unsolvable configurations

#### Integration Verification
- **IV1**: Existing functionality verification - Original `get_joint_ikine()` behavior unchanged
- **IV2**: Integration point verification - New method handles all current use cases correctly
- **IV3**: Performance impact verification - Inverse kinematics performance maintained or improved

### Story 1.4: Migrate Trajectory Planning Function

As a **developer**,
I want **trajectory planning moved to ArmKinematics module**,
so that **motion generation logic is separated from execution control**.

#### Acceptance Criteria
1. Implement `ArmKinematics.plan_trajectory()` equivalent to current `move_between_coordinates()`
2. Preserve smooth interpolation algorithms and waypoint generation
3. Add unit tests for trajectory smoothness and timing validation
4. Verify performance meets <50ms for 50-waypoint planning
5. Maintain identical trajectory format for existing motion control

#### Integration Verification
- **IV1**: Existing functionality verification - Original `move_between_coordinates()` behavior unchanged
- **IV2**: Integration point verification - Generated trajectories are identical to current implementation
- **IV3**: Performance impact verification - Trajectory planning performance maintained

### Story 1.5: Integrate ArmKinematics into RobotArmController

As a **developer**,
I want **RobotArmController to use ArmKinematics internally**,
so that **existing APIs delegate to the new modular implementation**.

#### Acceptance Criteria
1. Add ArmKinematics instance as RobotArmController attribute during initialization
2. Update existing methods to delegate to kinematics module while preserving signatures
3. Implement direct access property for users wanting standalone kinematics
4. Ensure thread safety for concurrent kinematics operations
5. Maintain identical error propagation and logging behavior

#### Integration Verification
- **IV1**: Existing functionality verification - All existing RobotArmController functionality works identically
- **IV2**: Integration point verification - Internal delegation is transparent to users
- **IV3**: Performance impact verification - No measurable performance degradation from delegation

### Story 1.6: Refactor Coordinate Movement Method

As a **developer**,
I want **set_joint_move_with_coordinate split between kinematics and control**,
so that **mathematical computation and hardware control are properly separated**.

#### Acceptance Criteria
1. Extract kinematics computation portion to ArmKinematics module
2. Keep hardware control logic in RobotArmController
3. Maintain identical method signature and behavior for backward compatibility
4. Add integration tests covering full coordinate movement scenarios
5. Ensure error handling spans both kinematics and control components properly

#### Integration Verification
- **IV1**: Existing functionality verification - Coordinate movement behavior is identical to current
- **IV2**: Integration point verification - Kinematics-control handoff works seamlessly
- **IV3**: Performance impact verification - Overall coordinate movement timing unchanged

### Story 1.7: Comprehensive Testing and Documentation Update

As a **developer**,
I want **complete test coverage and updated documentation**,
so that **the separated architecture is reliable and maintainable**.

#### Acceptance Criteria
1. Achieve ≥90% unit test coverage for ArmKinematics module
2. Add integration tests covering all backward compatibility scenarios
3. Update docstrings and type hints throughout codebase
4. Create usage examples for both backward-compatible and new direct access patterns
5. Add performance benchmarking tests comparing old vs new implementation

#### Integration Verification
- **IV1**: Existing functionality verification - All existing tests pass without modification
- **IV2**: Integration point verification - New tests validate both old and new usage patterns
- **IV3**: Performance impact verification - Benchmarks confirm no performance regression

---

**Document Version**: v1.0  
**Created Date**: 2025-09-05  
**Created By**: John (Product Manager Agent)  
**Status**: Ready for Development