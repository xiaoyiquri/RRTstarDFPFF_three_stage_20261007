# Plan: Repair the Differential Flatness Layer
_Locked via grill — by Claude + user_

## Goal
Update the `df` layer in `RRTstarDFPFF_next` so Unicycle, Quad2D, and Quad3D use explicit flat-state/control boundaries, system-specific lifting data, trajectory-time inverse mapping, and validated physical reconstruction. Preserve Unicycle forward/reverse branch metadata, use the DynoBench Quad2D no-drag convention, and replace Quad3D finite-difference angular reconstruction with analytic derivatives. Keep the generic RRT*, PFF, SIMD, and DynoBench layers untouched.

## Approach
1. Add compile-time `FlatControl<Dim>` and extend `FlatTrajectory<Dim, Order, Metadata>` with default `std::monostate` metadata, plus `evaluateState()` and `evaluateControl()` aliases.
2. Define common DF result/lifting helpers and update the CRTP adapter to expose explicit lifting, state/control trajectory-time methods, and feasibility configuration without silently filling missing derivatives.
3. Implement Unicycle lifting from explicit signed velocity, branch metadata, fixed-branch inverse mapping, signed controls, zero-speed rejection, and configurable feasibility checks.
4. Implement Quad2D lifting from explicit acceleration/jerk, no-drag force and torque mapping using FlatState plus FlatControl, trajectory-time reconstruction, and limit checks.
5. Implement Quad3D lifting from explicit acceleration/jerk/yaw derivatives, analytic attitude derivative recovery from acceleration/jerk/snap and yaw derivatives, DynoBench-compatible quaternion/control mapping, and singularity/limit checks.
6. Add focused compile-time and runtime tests for metadata retention, state/control order boundaries, Unicycle reverse semantics, Quad2D control extraction, and Quad3D analytic trajectory reconstruction where the current test harness permits.
7. Build or run the available checks and report any environment limitations separately.

## Key decisions & tradeoffs
- `FlatState<Dim, Order>` stores derivatives `0 ... Order-1`; `FlatControl<Dim>` stores derivative `Order`.
- `FlatTrajectory` metadata is a template parameter defaulting to `std::monostate`; Unicycle uses `VelocityBranch` and `signed_start_velocity`.
- Every Unicycle edge has one fixed branch. It cannot cross zero speed or change branch implicitly.
- Physical-state-to-flat lifting requires explicit system-specific `LiftData`; missing high-order data returns failure.
- DF inverse operations use state and control evaluated at the same trajectory time.
- Quad2D uses the no-drag model requested by the user.
- Quad3D uses analytic geometric derivatives and rejects force/yaw-frame singularities; finite differences are not used for final control recovery.
- This change remains limited to the DF/common trajectory interfaces needed by the DF contracts; RRT*, PFF, SIMD, and DynoBench adapters are not redesigned here.

## Risks / open questions
- Existing system model aliases and PFF result types may need small source-compatible updates because Unicycle's trajectory type now carries metadata.
- The repository may not have Eigen/CMake available in the current Windows/WSL environment, so compilation may be unavailable.
- Quad3D actuator bounds depend on the adapter's exact motor convention; the implementation will preserve the audited B-matrix convention and expose configuration values.

## Out of scope
- Rewriting RRT* or PFF mathematics.
- Implementing SIMD kernels.
- Adding or changing DynoBench source code or environment files.
- Introducing drag into Quad2D.
- Replacing the existing benchmark architecture.
