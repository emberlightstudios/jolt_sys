//! Raw `extern "C"` bindings to Jolt Physics via the shim in
//! `wrapper/wrapper.cpp`.
//!
//! This crate owns the CMake build of the Jolt static library and the raw
//! function declarations. Everything here is `unsafe`. Safe wrappers live in
//! `bevy_jolt`.

use std::ffi::{c_float, c_int};

/// Opaque handle to the C++ `BJoltWorld` (PhysicsSystem + TempAllocator +
/// ThreadPool job system + layer filters).
#[repr(C)]
pub struct BJoltWorld {
    _opaque: [u8; 0],
}

/// Opaque handle to the C++ `BJoltRagdollBuild` (`RagdollSettings` +
/// `Skeleton` under construction).
#[repr(C)]
pub struct BJoltRagdollBuild {
    _opaque: [u8; 0],
}

/// Opaque handle to a live C++ `BJoltRagdoll` (created bodies +
/// constraints in the system).
#[repr(C)]
pub struct BJoltRagdoll {
    _opaque: [u8; 0],
}

/// Largest object-layer count the shim supports (one broadphase bucket each).
pub const MAX_OBJECT_LAYERS: usize = 16;

/// Which frame joint anchors/axes live in. Matches Jolt's
/// `EConstraintSpace`: 0 = world, 1 = local to each body's center of mass.
/// `LocalToBodyCOM` positions are relative to the body's center of mass,
/// NOT the Bevy `Transform` origin: subtract the shape's center-of-mass
/// offset first.
pub const JOINT_SPACE_WORLD: u8 = 0;
pub const JOINT_SPACE_LOCAL_TO_BODY_COM: u8 = 1;

/// Caps for the vehicle/path FFI arrays. Must match `BJOLT_MAX_*` in
/// `wrapper/wrapper.cpp`.
pub const MAX_VEHICLE_WHEELS: usize = 8;
pub const MAX_VEHICLE_DIFFS: usize = 4;
pub const MAX_VEHICLE_ROLL_BARS: usize = 2;
pub const MAX_VEHICLE_GEARS: usize = 8;
pub const MAX_CURVE_POINTS: usize = 8;
pub const MAX_PATH_POINTS: usize = 16;

/// One `(x, y)` knot for a Jolt `LinearCurve`. Matches `BJoltCurvePoint`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CurvePointFfi {
    pub knot_x: c_float,
    pub knot_y: c_float,
}

/// One Hermite spline knot: position + tangent + normal. Matches
/// `BJoltPathPoint`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PathPointFfi {
    pub pos_x: c_float,
    pub pos_y: c_float,
    pub pos_z: c_float,
    pub tan_x: c_float,
    pub tan_y: c_float,
    pub tan_z: c_float,
    pub nrm_x: c_float,
    pub nrm_y: c_float,
    pub nrm_z: c_float,
}
/// One wheel crossing the FFI: `kind` 0 = wheeled (slip-curve tires),
/// 1 = tracked (plain friction pair). Empty curve counts keep Jolt's
/// tire-profile defaults. `#[repr(C)]` so field order matches
/// `BJoltWheelConfig`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VehicleWheelFfi {
    pub pos_x: c_float,
    pub pos_y: c_float,
    pub pos_z: c_float,
    pub susp_force_x: c_float,
    pub susp_force_y: c_float,
    pub susp_force_z: c_float,
    pub susp_min_length: c_float,
    pub susp_max_length: c_float,
    pub susp_preload_length: c_float,
    pub susp_frequency: c_float,
    pub susp_damping: c_float,
    pub wheel_radius: c_float,
    pub wheel_width: c_float,
    pub max_steer_angle_rad: c_float,
    pub wheel_inertia: c_float,
    pub wheel_damping: c_float,
    pub max_brake_torque: c_float,
    pub max_hand_brake_torque: c_float,
    pub track_longitudinal_friction: c_float,
    pub track_lateral_friction: c_float,
    pub kind: u8,
    pub long_curve_count: u8,
    pub lat_curve_count: u8,
    pub long_curve: [CurvePointFfi; MAX_CURVE_POINTS],
    pub lat_curve: [CurvePointFfi; MAX_CURVE_POINTS],
}

/// Engine crossing the FFI: scalars plus the normalized torque curve
/// (empty keeps Jolt's 0.8 / 1.0 / 0.8 default).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VehicleEngineFfi {
    pub max_torque: c_float,
    pub min_rpm: c_float,
    pub max_rpm: c_float,
    pub engine_inertia: c_float,
    pub engine_damping: c_float,
    pub torque_curve_count: u8,
    pub torque_curve: [CurvePointFfi; MAX_CURVE_POINTS],
}

/// Gearbox crossing the FFI: fixed-size ratio tables plus the used counts.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VehicleTransmissionFfi {
    pub auto_mode: u8,
    pub gear_count: u8,
    pub reverse_gear_count: u8,
    pub gear_ratios: [c_float; MAX_VEHICLE_GEARS],
    pub reverse_gear_ratios: [c_float; MAX_VEHICLE_GEARS],
    pub switch_time: c_float,
    pub clutch_release_time: c_float,
    pub switch_latency: c_float,
    pub shift_up_rpm: c_float,
    pub shift_down_rpm: c_float,
    pub clutch_strength: c_float,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VehicleDifferentialFfi {
    pub left_wheel: i32,
    pub right_wheel: i32,
    pub differential_ratio: c_float,
    pub left_right_split: c_float,
    pub limited_slip_ratio: c_float,
    pub engine_torque_ratio: c_float,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VehicleRollBarFfi {
    pub left_wheel: i32,
    pub right_wheel: i32,
    pub stiffness: c_float,
}

/// One tank track side: member wheel indices plus the driven one.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VehicleTrackFfi {
    pub wheel_count: u8,
    pub wheel_indices: [u8; MAX_VEHICLE_WHEELS],
    pub driven_wheel: u8,
    pub track_inertia: c_float,
    pub track_damping: c_float,
    pub max_brake_torque: c_float,
    pub differential_ratio: c_float,
}

/// Motorcycle lean-spring balance tuning.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VehicleLeanFfi {
    pub max_lean_angle_rad: c_float,
    pub lean_spring_constant: c_float,
    pub lean_spring_damping: c_float,
    pub lean_integration_coefficient: c_float,
    pub lean_integration_decay: c_float,
    pub lean_smoothing: c_float,
}

/// Max parts in one compound body. Fixed-size FFI arrays keep the call one
/// pointer per field; raise it if a body needs more chunks.
pub const MAX_COMPOUND_PARTS: usize = 16;

/// One compound part crossing the FFI: `part_kind` 0 = box (half extents),
/// 1 = sphere (`part_half_x` is the radius), 2 = capsule (`part_half_x` is
/// the cylinder half height, `part_half_y` the radius). Offset + quaternion
/// pose the part relative to the body origin. `#[repr(C)]` matches
/// `BJoltCompoundPart`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CompoundPartFfi {
    pub part_kind: u32,
    pub part_half_x: c_float,
    pub part_half_y: c_float,
    pub part_half_z: c_float,
    pub offset_x: c_float,
    pub offset_y: c_float,
    pub offset_z: c_float,
    pub rot_x: c_float,
    pub rot_y: c_float,
    pub rot_z: c_float,
    pub rot_w: c_float,
}

unsafe extern "C" {
    pub fn bjolt_init() -> bool;
    pub fn bjolt_world_create_with_layers(
        layer_count: u32,
        collide_matrix: *const u8,
    ) -> *mut BJoltWorld;
    pub fn bjolt_world_destroy(world_ptr: *mut BJoltWorld);
    pub fn bjolt_create_floor(
        world_ptr: *mut BJoltWorld,
        half_x: c_float,
        half_y: c_float,
        half_z: c_float,
        floor_height: c_float,
    ) -> u32;
    pub fn bjolt_create_sphere(
        world_ptr: *mut BJoltWorld,
        sphere_radius: c_float,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        object_layer: u16,
        density_kg_per_m3: c_float,
        gravity_factor: c_float,
    ) -> u32;
    pub fn bjolt_world_update(
        world_ptr: *mut BJoltWorld,
        delta_time: c_float,
        collision_steps: i32,
    );
    pub fn bjolt_body_is_active(world_ptr: *mut BJoltWorld, body_id_raw: u32) -> bool;
    pub fn bjolt_body_state(
        world_ptr: *mut BJoltWorld,
        body_id_raw: u32,
        out_position: *mut c_float,
        out_velocity: *mut c_float,
        out_angular_velocity: *mut c_float,
    );
    pub fn bjolt_body_remove_destroy(world_ptr: *mut BJoltWorld, body_id_raw: u32);
    pub fn bjolt_create_box(
        world_ptr: *mut BJoltWorld,
        half_x: c_float,
        half_y: c_float,
        half_z: c_float,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        object_layer: u16,
        motion: u8,
        density_kg_per_m3: c_float,
        gravity_factor: c_float,
    ) -> u32;
    /// One rigid body from up to 16 box/sphere/capsule parts. Parts pose
    /// relative to the body origin; the body spawns at the position with
    /// identity rotation. Returns 0 on failure.
    pub fn bjolt_create_compound(
        world_ptr: *mut BJoltWorld,
        parts: *const CompoundPartFfi,
        part_count: u32,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        object_layer: u16,
        motion: u8,
        density_kg_per_m3: c_float,
        gravity_factor: c_float,
    ) -> u32;
    pub fn bjolt_move_kinematic(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        rot_x: c_float,
        rot_y: c_float,
        rot_z: c_float,
        rot_w: c_float,
        delta_time: c_float,
    );
    pub fn bjolt_create_capsule(
        world_ptr: *mut BJoltWorld,
        half_height: c_float,
        capsule_radius: c_float,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        object_layer: u16,
        density_kg_per_m3: c_float,
        gravity_factor: c_float,
    ) -> u32;
    pub fn bjolt_set_gravity_factor(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        gravity_factor: c_float,
    );
    pub fn bjolt_gravity_factor(world_ptr: *mut BJoltWorld, body_raw: u32) -> c_float;
    pub fn bjolt_set_gravity(
        world_ptr: *mut BJoltWorld,
        gravity_x: c_float,
        gravity_y: c_float,
        gravity_z: c_float,
    );
    pub fn bjolt_world_gravity(world_ptr: *mut BJoltWorld, out_gravity: *mut c_float);
    pub fn bjolt_create_cylinder(
        world_ptr: *mut BJoltWorld,
        half_height: c_float,
        cylinder_radius: c_float,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        object_layer: u16,
        density_kg_per_m3: c_float,
        gravity_factor: c_float,
    ) -> u32;
    pub fn bjolt_create_tapered_cylinder(
        world_ptr: *mut BJoltWorld,
        half_height: c_float,
        top_radius: c_float,
        bottom_radius: c_float,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        object_layer: u16,
        density_kg_per_m3: c_float,
        gravity_factor: c_float,
    ) -> u32;
    pub fn bjolt_create_tapered_capsule(
        world_ptr: *mut BJoltWorld,
        half_height: c_float,
        top_radius: c_float,
        bottom_radius: c_float,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        object_layer: u16,
        density_kg_per_m3: c_float,
        gravity_factor: c_float,
    ) -> u32;
    pub fn bjolt_body_transform(
        world_ptr: *mut BJoltWorld,
        body_id_raw: u32,
        out_position: *mut c_float,
        out_rotation: *mut c_float,
        out_velocity: *mut c_float,
        out_angular_velocity: *mut c_float,
    );
    pub fn bjolt_cast_ray(
        world_ptr: *mut BJoltWorld,
        origin_x: c_float,
        origin_y: c_float,
        origin_z: c_float,
        dir_x: c_float,
        dir_y: c_float,
        dir_z: c_float,
        out_hit_body: *mut u32,
        out_hit_fraction: *mut c_float,
    ) -> bool;
    /// All bodies a ray passes through, nearest first. Writes at most
    /// `hit_capacity` pairs into `out_hit_bodies`/`out_hit_fractions` and
    /// returns the total count found (which may exceed capacity: entries
    /// past capacity are dropped).
    pub fn bjolt_cast_ray_all(
        world_ptr: *mut BJoltWorld,
        origin_x: c_float,
        origin_y: c_float,
        origin_z: c_float,
        dir_x: c_float,
        dir_y: c_float,
        dir_z: c_float,
        out_hit_bodies: *mut u32,
        out_hit_fractions: *mut c_float,
        hit_capacity: u32,
    ) -> u32;
    /// Every body containing a point. Same batch layout as `bjolt_cast_ray_all`
    /// (body ids only; fractions unused, left zeroed).
    pub fn bjolt_collide_point_all(
        world_ptr: *mut BJoltWorld,
        point_x: c_float,
        point_y: c_float,
        point_z: c_float,
        out_hit_bodies: *mut u32,
        out_hit_fractions: *mut c_float,
        hit_capacity: u32,
    ) -> u32;
    /// Every body overlapping a box or sphere probe at a center-of-mass
    /// pose: `probe_kind` 0 = box (`probe_half_extents` used), 1 = sphere
    /// (`probe_half_extents.x` is the radius). Writes body id, penetration
    /// depth, and contact point into the three output arrays.
    pub fn bjolt_overlap_shape_all(
        world_ptr: *mut BJoltWorld,
        probe_kind: u32,
        center_x: c_float,
        center_y: c_float,
        center_z: c_float,
        probe_half_x: c_float,
        probe_half_y: c_float,
        probe_half_z: c_float,
        out_hit_bodies: *mut u32,
        out_hit_depths: *mut c_float,
        out_contact_points: *mut c_float,
        hit_capacity: u32,
    ) -> u32;
    /// Sweeps a box or sphere probe along a direction (`probe_kind` as in
    /// `bjolt_overlap_shape_all`). Writes body id, hit fraction, and contact
    /// point for every body touched, nearest first.
    pub fn bjolt_cast_shape_all(
        world_ptr: *mut BJoltWorld,
        probe_kind: u32,
        center_x: c_float,
        center_y: c_float,
        center_z: c_float,
        probe_half_x: c_float,
        probe_half_y: c_float,
        probe_half_z: c_float,
        cast_dir_x: c_float,
        cast_dir_y: c_float,
        cast_dir_z: c_float,
        out_hit_bodies: *mut u32,
        out_hit_fractions: *mut c_float,
        out_contact_points: *mut c_float,
        hit_capacity: u32,
    ) -> u32;
    pub fn bjolt_create_plane(
        world_ptr: *mut BJoltWorld,
        normal_x: c_float,
        normal_y: c_float,
        normal_z: c_float,
        plane_constant: c_float,
        half_extent: c_float,
        object_layer: u16,
    ) -> u32;
    pub fn bjolt_create_fixed_constraint(
        world_ptr: *mut BJoltWorld,
        body1_raw: u32,
        body2_raw: u32,
        joint_space: u8,
    ) -> u32;
    pub fn bjolt_create_distance_constraint(
        world_ptr: *mut BJoltWorld,
        body1_raw: u32,
        body2_raw: u32,
        point1_x: c_float,
        point1_y: c_float,
        point1_z: c_float,
        point2_x: c_float,
        point2_y: c_float,
        point2_z: c_float,
        min_distance: c_float,
        max_distance: c_float,
        joint_space: u8,
    ) -> u32;
    pub fn bjolt_create_hinge_constraint(
        world_ptr: *mut BJoltWorld,
        body1_raw: u32,
        body2_raw: u32,
        point_x: c_float,
        point_y: c_float,
        point_z: c_float,
        hinge_axis1_x: c_float,
        hinge_axis1_y: c_float,
        hinge_axis1_z: c_float,
        normal_axis1_x: c_float,
        normal_axis1_y: c_float,
        normal_axis1_z: c_float,
        hinge_axis2_x: c_float,
        hinge_axis2_y: c_float,
        hinge_axis2_z: c_float,
        normal_axis2_x: c_float,
        normal_axis2_y: c_float,
        normal_axis2_z: c_float,
        limits_min: c_float,
        limits_max: c_float,
        motor_frequency: c_float,
        motor_damping: c_float,
        motor_force_limit: c_float,
        joint_space: u8,
    ) -> u32;
    pub fn bjolt_create_point_constraint(
        world_ptr: *mut BJoltWorld,
        body1_raw: u32,
        body2_raw: u32,
        point_x: c_float,
        point_y: c_float,
        point_z: c_float,
        joint_space: u8,
    ) -> u32;
    pub fn bjolt_create_slider_constraint(
        world_ptr: *mut BJoltWorld,
        body1_raw: u32,
        body2_raw: u32,
        slider_axis1_x: c_float,
        slider_axis1_y: c_float,
        slider_axis1_z: c_float,
        normal_axis1_x: c_float,
        normal_axis1_y: c_float,
        normal_axis1_z: c_float,
        slider_axis2_x: c_float,
        slider_axis2_y: c_float,
        slider_axis2_z: c_float,
        normal_axis2_x: c_float,
        normal_axis2_y: c_float,
        normal_axis2_z: c_float,
        limits_min: c_float,
        limits_max: c_float,
        motor_frequency: c_float,
        motor_damping: c_float,
        motor_force_limit: c_float,
        joint_space: u8,
    ) -> u32;
    pub fn bjolt_create_cone_constraint(
        world_ptr: *mut BJoltWorld,
        body1_raw: u32,
        body2_raw: u32,
        point_x: c_float,
        point_y: c_float,
        point_z: c_float,
        twist_axis1_x: c_float,
        twist_axis1_y: c_float,
        twist_axis1_z: c_float,
        twist_axis2_x: c_float,
        twist_axis2_y: c_float,
        twist_axis2_z: c_float,
        half_cone_angle: c_float,
        joint_space: u8,
    ) -> u32;
    pub fn bjolt_create_swing_twist_constraint(
        world_ptr: *mut BJoltWorld,
        body1_raw: u32,
        body2_raw: u32,
        position_x: c_float,
        position_y: c_float,
        position_z: c_float,
        twist_axis1_x: c_float,
        twist_axis1_y: c_float,
        twist_axis1_z: c_float,
        plane_axis1_x: c_float,
        plane_axis1_y: c_float,
        plane_axis1_z: c_float,
        twist_axis2_x: c_float,
        twist_axis2_y: c_float,
        twist_axis2_z: c_float,
        plane_axis2_x: c_float,
        plane_axis2_y: c_float,
        plane_axis2_z: c_float,
        normal_half_cone_angle: c_float,
        plane_half_cone_angle: c_float,
        twist_min_angle: c_float,
        twist_max_angle: c_float,
        joint_space: u8,
    ) -> u32;
    pub fn bjolt_create_six_dof(
        world_ptr: *mut BJoltWorld,
        body1_raw: u32,
        body2_raw: u32,
        position1_x: c_float,
        position1_y: c_float,
        position1_z: c_float,
        axis_x1_x: c_float,
        axis_x1_y: c_float,
        axis_x1_z: c_float,
        axis_y1_x: c_float,
        axis_y1_y: c_float,
        axis_y1_z: c_float,
        position2_x: c_float,
        position2_y: c_float,
        position2_z: c_float,
        axis_x2_x: c_float,
        axis_x2_y: c_float,
        axis_x2_z: c_float,
        axis_y2_x: c_float,
        axis_y2_y: c_float,
        axis_y2_z: c_float,
        limit_min_tx: c_float,
        limit_max_tx: c_float,
        limit_min_ty: c_float,
        limit_max_ty: c_float,
        limit_min_tz: c_float,
        limit_max_tz: c_float,
        limit_min_rx: c_float,
        limit_max_rx: c_float,
        limit_min_ry: c_float,
        limit_max_ry: c_float,
        limit_min_rz: c_float,
        limit_max_rz: c_float,
        motor_axes: u8,
        motor_frequency: c_float,
        motor_damping: c_float,
        motor_force_limit: c_float,
        joint_space: u8,
    ) -> u32;
    pub fn bjolt_create_pulley_constraint(
        world_ptr: *mut BJoltWorld,
        body1_raw: u32,
        body2_raw: u32,
        body_point1_x: c_float,
        body_point1_y: c_float,
        body_point1_z: c_float,
        fixed_point1_x: c_float,
        fixed_point1_y: c_float,
        fixed_point1_z: c_float,
        body_point2_x: c_float,
        body_point2_y: c_float,
        body_point2_z: c_float,
        fixed_point2_x: c_float,
        fixed_point2_y: c_float,
        fixed_point2_z: c_float,
        ratio: c_float,
        min_length: c_float,
        max_length: c_float,
        joint_space: u8,
    ) -> u32;
    pub fn bjolt_create_gear_constraint(
        world_ptr: *mut BJoltWorld,
        body1_raw: u32,
        body2_raw: u32,
        hinge_axis_x: c_float,
        hinge_axis_y: c_float,
        hinge_axis_z: c_float,
        ratio: c_float,
        hinge_id1: u32,
        hinge_id2: u32,
        joint_space: u8,
    ) -> u32;
    pub fn bjolt_create_rack_pinion_constraint(
        world_ptr: *mut BJoltWorld,
        body1_raw: u32,
        body2_raw: u32,
        hinge_axis_x: c_float,
        hinge_axis_y: c_float,
        hinge_axis_z: c_float,
        slider_axis_x: c_float,
        slider_axis_y: c_float,
        slider_axis_z: c_float,
        ratio: c_float,
        pinion_hinge_id: u32,
        rack_slider_id: u32,
        joint_space: u8,
    ) -> u32;
    pub fn bjolt_create_path_cart(
        world_ptr: *mut BJoltWorld,
        static_body_raw: u32,
        cart_body_raw: u32,
        points: *const PathPointFfi,
        point_count: u8,
        looping: u8,
        motor_frequency: c_float,
        motor_damping: c_float,
        motor_force_limit: c_float,
        rotation_mode: u8,
    ) -> u32;
    pub fn bjolt_create_wheeled_vehicle(
        world_ptr: *mut BJoltWorld,
        object_layer: u16,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        half_x: c_float,
        half_y: c_float,
        half_z: c_float,
        com_off_x: c_float,
        com_off_y: c_float,
        com_off_z: c_float,
        mass_kg: c_float,
        max_pitch_roll_angle_rad: c_float,
        tester_radius: c_float,
        wheels: *const VehicleWheelFfi,
        wheel_count: u8,
        engine: *const VehicleEngineFfi,
        gearbox: *const VehicleTransmissionFfi,
        diffs: *const VehicleDifferentialFfi,
        diff_count: u8,
        roll_bars: *const VehicleRollBarFfi,
        roll_bar_count: u8,
        limited_slip_ratio: c_float,
        out_body_raw: *mut u32,
        out_constraint_id: *mut u32,
    ) -> u32;
    pub fn bjolt_create_tracked_vehicle(
        world_ptr: *mut BJoltWorld,
        object_layer: u16,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        half_x: c_float,
        half_y: c_float,
        half_z: c_float,
        com_off_x: c_float,
        com_off_y: c_float,
        com_off_z: c_float,
        mass_kg: c_float,
        max_pitch_roll_angle_rad: c_float,
        tester_radius: c_float,
        wheels: *const VehicleWheelFfi,
        wheel_count: u8,
        engine: *const VehicleEngineFfi,
        gearbox: *const VehicleTransmissionFfi,
        tracks: *const VehicleTrackFfi,
        out_body_raw: *mut u32,
        out_constraint_id: *mut u32,
    ) -> u32;
    pub fn bjolt_create_motorcycle(
        world_ptr: *mut BJoltWorld,
        object_layer: u16,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        half_x: c_float,
        half_y: c_float,
        half_z: c_float,
        com_off_x: c_float,
        com_off_y: c_float,
        com_off_z: c_float,
        mass_kg: c_float,
        max_pitch_roll_angle_rad: c_float,
        tester_radius: c_float,
        wheels: *const VehicleWheelFfi,
        wheel_count: u8,
        engine: *const VehicleEngineFfi,
        gearbox: *const VehicleTransmissionFfi,
        diffs: *const VehicleDifferentialFfi,
        diff_count: u8,
        lean: *const VehicleLeanFfi,
        out_body_raw: *mut u32,
        out_constraint_id: *mut u32,
    ) -> u32;
    pub fn bjolt_vehicle_drive(
        world_ptr: *mut BJoltWorld,
        constraint_id: u32,
        forward: c_float,
        right: c_float,
        brake: c_float,
        hand_brake: c_float,
    );
    pub fn bjolt_tracked_drive(
        world_ptr: *mut BJoltWorld,
        constraint_id: u32,
        forward: c_float,
        left_ratio: c_float,
        right_ratio: c_float,
        brake: c_float,
    );
    pub fn bjolt_vehicle_shift(
        world_ptr: *mut BJoltWorld,
        constraint_id: u32,
        gear: i32,
        clutch_friction: c_float,
    );
    pub fn bjolt_apply_impulse(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        lin_x: c_float,
        lin_y: c_float,
        lin_z: c_float,
        ang_x: c_float,
        ang_y: c_float,
        ang_z: c_float,
    );
    pub fn bjolt_apply_force(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        force_x: c_float,
        force_y: c_float,
        force_z: c_float,
        torque_x: c_float,
        torque_y: c_float,
        torque_z: c_float,
    );
    pub fn bjolt_set_velocity(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        lin_x: c_float,
        lin_y: c_float,
        lin_z: c_float,
        ang_x: c_float,
        ang_y: c_float,
        ang_z: c_float,
    );
    pub fn bjolt_set_linear_velocity(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        lin_x: c_float,
        lin_y: c_float,
        lin_z: c_float,
    );
    pub fn bjolt_set_angular_velocity(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        ang_x: c_float,
        ang_y: c_float,
        ang_z: c_float,
    );
    pub fn bjolt_set_rotation(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        rot_x: c_float,
        rot_y: c_float,
        rot_z: c_float,
        rot_w: c_float,
    );
    pub fn bjolt_set_position(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
    );
    pub fn bjolt_set_friction(world_ptr: *mut BJoltWorld, body_raw: u32, friction: c_float);
    pub fn bjolt_set_restitution(world_ptr: *mut BJoltWorld, body_raw: u32, restitution: c_float);
    pub fn bjolt_set_ccd(world_ptr: *mut BJoltWorld, body_raw: u32, use_ccd: bool);
    pub fn bjolt_sleep_body(world_ptr: *mut BJoltWorld, body_raw: u32);
    pub fn bjolt_wake_body(world_ptr: *mut BJoltWorld, body_raw: u32);
    pub fn bjolt_set_motion_type(world_ptr: *mut BJoltWorld, body_raw: u32, motion_type: u8);
    pub fn bjolt_drain_slept(
        world_ptr: *mut BJoltWorld,
        out_ids: *mut u32,
        id_capacity: u32,
    ) -> u32;
    pub fn bjolt_drain_woke(world_ptr: *mut BJoltWorld, out_ids: *mut u32, id_capacity: u32)
        -> u32;
    pub fn bjolt_create_heightfield(
        world_ptr: *mut BJoltWorld,
        sample_heights: *const c_float,
        sample_count: u32,
        grid_width: u32,
        cell_size: c_float,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        object_layer: u16,
    ) -> u32;
    pub fn bjolt_create_hull(
        world_ptr: *mut BJoltWorld,
        points_xyz: *const c_float,
        point_count: u32,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        object_layer: u16,
        motion: u8,
        density_kg_per_m3: c_float,
        gravity_factor: c_float,
    ) -> u32;
    pub fn bjolt_create_mesh(
        world_ptr: *mut BJoltWorld,
        vertices_xyz: *const c_float,
        vertex_count: u32,
        triangle_indices: *const u32,
        triangle_count: u32,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        object_layer: u16,
    ) -> u32;
    pub fn bjolt_apply_buoyancy(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        surface_y: c_float,
        buoyancy: c_float,
        linear_drag: c_float,
        angular_drag: c_float,
        fluid_vel_x: c_float,
        fluid_vel_y: c_float,
        fluid_vel_z: c_float,
        gravity_x: c_float,
        gravity_y: c_float,
        gravity_z: c_float,
        delta_time: c_float,
    );
    pub fn bjolt_create_shared_settings(
        vertex_positions: *const c_float,
        vertex_velocities: *const c_float,
        vertex_inv_masses: *const c_float,
        vertex_count: u32,
        face_indices: *const u32,
        face_count: u32,
        edge_pairs: *const u32,
        edge_compliances: *const c_float,
        edge_count: u32,
        volume_quads: *const u32,
        volume_compliances: *const c_float,
        volume_count: u32,
        edge_compliance: c_float,
        shear_compliance: c_float,
        bend_compliance: c_float,
        bend_type: u8,
    ) -> u64;
    pub fn bjolt_destroy_shared_settings(shared_handle: u64);
    pub fn bjolt_create_cube_settings(grid_size: u32, grid_spacing: c_float) -> u64;
    pub fn bjolt_create_cloth_settings(
        grid_nx: u32,
        grid_nz: u32,
        grid_spacing: c_float,
        pinned_rows: u32,
        bend_type: u8,
    ) -> u64;
    pub fn bjolt_create_sphere_settings(
        sphere_radius: c_float,
        theta_segments: u32,
        phi_segments: u32,
        bend_type: u8,
    ) -> u64;
    pub fn bjolt_shared_vertex_count(shared_handle: u64) -> u32;
    pub fn bjolt_shared_face_count(shared_handle: u64) -> u32;
    pub fn bjolt_shared_faces(shared_handle: u64, out_triangles: *mut u32, capacity: u32) -> u32;
    pub fn bjolt_create_soft_body(
        world_ptr: *mut BJoltWorld,
        shared_handle: u64,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        rot_x: c_float,
        rot_y: c_float,
        rot_z: c_float,
        rot_w: c_float,
        object_layer: u16,
        num_iterations: u32,
        linear_damping: c_float,
        max_linear_velocity: c_float,
        restitution: c_float,
        friction: c_float,
        pressure: c_float,
        gravity_factor: c_float,
        vertex_radius: c_float,
        update_position: bool,
        make_rotation_identity: bool,
        allow_sleeping: bool,
        faces_double_sided: bool,
        user_data: u64,
    ) -> u32;
    pub fn bjolt_soft_vertex_count(world_ptr: *mut BJoltWorld, body_raw: u32) -> u32;
    pub fn bjolt_soft_vertices(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        out_positions: *mut c_float,
        capacity: u32,
    ) -> u32;
    pub fn bjolt_soft_velocities(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        out_velocities: *mut c_float,
        capacity: u32,
    ) -> u32;
    pub fn bjolt_soft_inv_masses(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        out_inv_masses: *mut c_float,
        capacity: u32,
    ) -> u32;
    pub fn bjolt_soft_set_inv_masses(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        inv_masses: *const c_float,
        count: u32,
    );
    pub fn bjolt_soft_contacts(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        out_contacted: *mut u8,
        capacity: u32,
    ) -> u32;
    pub fn bjolt_soft_pressure(world_ptr: *mut BJoltWorld, body_raw: u32) -> c_float;
    pub fn bjolt_soft_set_pressure(world_ptr: *mut BJoltWorld, body_raw: u32, pressure: c_float);
    pub fn bjolt_soft_iterations(world_ptr: *mut BJoltWorld, body_raw: u32) -> u32;
    pub fn bjolt_soft_set_iterations(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        num_iterations: u32,
    );
    pub fn bjolt_soft_vertex_radius(world_ptr: *mut BJoltWorld, body_raw: u32) -> c_float;
    pub fn bjolt_soft_set_vertex_radius(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        vertex_radius: c_float,
    );
    pub fn bjolt_soft_volume(world_ptr: *mut BJoltWorld, body_raw: u32) -> c_float;
    pub fn bjolt_soft_destroy(world_ptr: *mut BJoltWorld, body_raw: u32);
    pub fn bjolt_soft_push(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        force_x: c_float,
        force_y: c_float,
        force_z: c_float,
    );
    pub fn bjolt_set_position_rotation(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        rot_x: c_float,
        rot_y: c_float,
        rot_z: c_float,
        rot_w: c_float,
    );
    pub fn bjolt_constraint_drive_at(
        world_ptr: *mut BJoltWorld,
        constraint_id: u32,
        target_velocity: c_float,
    ) -> bool;
    pub fn bjolt_remove_constraint(world_ptr: *mut BJoltWorld, constraint_id: u32);
    pub fn bjolt_constraint_path_fraction(
        world_ptr: *mut BJoltWorld,
        constraint_id: u32,
    ) -> c_float;
    pub fn bjolt_constraint_path_looping(world_ptr: *mut BJoltWorld, constraint_id: u32) -> c_int;
    /// Creates a virtual character capsule. Bottom of the capsule sits at
    /// the spawn position; returns the character id (0 on failure).
    pub fn bjolt_character_create(
        world_ptr: *mut BJoltWorld,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        capsule_half_height: c_float,
        capsule_radius: c_float,
        object_layer: u16,
        mass_kg: c_float,
        max_strength: c_float,
        max_slope_degrees: c_float,
        character_padding: c_float,
        penetration_recovery: c_float,
    ) -> u32;
    pub fn bjolt_character_destroy(world_ptr: *mut BJoltWorld, character_id: u32);
    /// One movement step: sets the velocity, runs ExtendedUpdate (move +
    /// stick-to-floor + walk-stairs), and reports the new position, velocity,
    /// ground state (0 air, 1 ground, 2 steep), ground normal, and whether
    /// the character stands on something.
    pub fn bjolt_character_move(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        delta_time: c_float,
        velocity_x: c_float,
        velocity_y: c_float,
        velocity_z: c_float,
        gravity_x: c_float,
        gravity_y: c_float,
        gravity_z: c_float,
        step_up_height: c_float,
        stick_to_floor_distance: c_float,
        out_position: *mut c_float,
        out_velocity: *mut c_float,
        out_ground_normal: *mut c_float,
        out_ground_state: *mut u32,
        out_is_supported: *mut u32,
    );
    /// Teleport: drops the character at a position with a velocity and
    /// refreshes contacts. No-op on bad ids.
    pub fn bjolt_character_teleport(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        velocity_x: c_float,
        velocity_y: c_float,
        velocity_z: c_float,
    );
    /// Current capsule dimensions (half height, radius). No-op on bad ids.
    pub fn bjolt_character_stance(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        capsule_half_height: c_float,
        capsule_radius: c_float,
    ) -> bool;
    pub fn bjolt_character_set_rotation(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        rot_x: c_float,
        rot_y: c_float,
        rot_z: c_float,
        rot_w: c_float,
    );
    pub fn bjolt_character_rotation(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        out_rotation: *mut c_float,
    );
    pub fn bjolt_character_set_mass(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        mass_kg: c_float,
        max_strength: c_float,
    );
    pub fn bjolt_character_set_padding(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        character_padding: c_float,
        penetration_recovery: c_float,
    );
    pub fn bjolt_character_set_up(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        up_x: c_float,
        up_y: c_float,
        up_z: c_float,
        max_slope_degrees: c_float,
    );
    pub fn bjolt_character_set_shape_offset(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        offset_x: c_float,
        offset_y: c_float,
        offset_z: c_float,
    );
    pub fn bjolt_character_set_user_data(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        user_data: u64,
    );
    pub fn bjolt_character_update(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        delta_time: c_float,
        velocity_x: c_float,
        velocity_y: c_float,
        velocity_z: c_float,
        gravity_x: c_float,
        gravity_y: c_float,
        gravity_z: c_float,
        out_position: *mut c_float,
        out_velocity: *mut c_float,
        out_ground_normal: *mut c_float,
        out_ground_state: *mut u32,
        out_is_supported: *mut u32,
    );
    pub fn bjolt_character_can_walk_stairs(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        velocity_x: c_float,
        velocity_y: c_float,
        velocity_z: c_float,
    ) -> bool;
    pub fn bjolt_character_walk_stairs(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        delta_time: c_float,
        step_up_height: c_float,
        step_forward: c_float,
        step_forward_test: c_float,
        step_down_extra: c_float,
    ) -> bool;
    pub fn bjolt_character_stick_to_floor(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        stick_down_distance: c_float,
    ) -> bool;
    pub fn bjolt_character_refresh_contacts(world_ptr: *mut BJoltWorld, character_id: u32);
    pub fn bjolt_character_ground(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        out_ground_position: *mut c_float,
        out_ground_normal: *mut c_float,
        out_ground_velocity: *mut c_float,
        out_ground_body: *mut u32,
        out_ground_user_data: *mut u64,
    );
    pub fn bjolt_rigid_character_create(
        world_ptr: *mut BJoltWorld,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        rot_x: c_float,
        rot_y: c_float,
        rot_z: c_float,
        rot_w: c_float,
        capsule_half_height: c_float,
        capsule_radius: c_float,
        object_layer: u16,
        mass_kg: c_float,
        friction: c_float,
        gravity_factor: c_float,
        allowed_dofs: u8,
        user_data: u64,
    ) -> u32;
    pub fn bjolt_rigid_character_destroy(world_ptr: *mut BJoltWorld, character_id: u32);
    pub fn bjolt_rigid_character_post(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        max_separation: c_float,
    );
    pub fn bjolt_rigid_character_set_velocity(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        lin_x: c_float,
        lin_y: c_float,
        lin_z: c_float,
        ang_x: c_float,
        ang_y: c_float,
        ang_z: c_float,
    );
    pub fn bjolt_rigid_character_add_velocity(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        lin_x: c_float,
        lin_y: c_float,
        lin_z: c_float,
    );
    pub fn bjolt_rigid_character_add_impulse(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        imp_x: c_float,
        imp_y: c_float,
        imp_z: c_float,
    );
    pub fn bjolt_rigid_character_pose(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        out_position: *mut c_float,
        out_rotation: *mut c_float,
        out_velocity: *mut c_float,
        out_ground_normal: *mut c_float,
        out_ground_state: *mut u32,
        out_is_supported: *mut u32,
    );
    pub fn bjolt_rigid_character_set_pose(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        rot_x: c_float,
        rot_y: c_float,
        rot_z: c_float,
        rot_w: c_float,
    );
    pub fn bjolt_rigid_character_body(world_ptr: *mut BJoltWorld, character_id: u32) -> u32;
    pub fn bjolt_rigid_character_set_layer(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        object_layer: u16,
    );
    pub fn bjolt_rigid_character_stance(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        capsule_half_height: c_float,
        capsule_radius: c_float,
        max_penetration: c_float,
    ) -> bool;
    pub fn bjolt_rigid_character_ground(
        world_ptr: *mut BJoltWorld,
        character_id: u32,
        out_ground_position: *mut c_float,
        out_ground_normal: *mut c_float,
        out_ground_velocity: *mut c_float,
        out_ground_body: *mut u32,
        out_ground_user_data: *mut u64,
    );
    /// Stops two bodies colliding (ragdoll parent-child pairs). Builds one
    /// shared group table per pair: fine for a handful of links, not for
    /// crowds. No-op on bad ids.
    pub fn bjolt_bodies_no_collide(world_ptr: *mut BJoltWorld, body_a_raw: u32, body_b_raw: u32);
    /// Drains contact begin pairs since the last step into flat body-id
    /// arrays. Returns events kept (capped at capacity); resets the queue.
    pub fn bjolt_drain_contact_added(
        world_ptr: *mut BJoltWorld,
        out_a: *mut u32,
        out_b: *mut u32,
        pair_capacity: u32,
    ) -> u32;
    /// Drains contact end pairs since the last step. Same layout as added.
    pub fn bjolt_drain_contact_removed(
        world_ptr: *mut BJoltWorld,
        out_a: *mut u32,
        out_b: *mut u32,
        pair_capacity: u32,
    ) -> u32;
    /// Marks a body as a sensor: overlaps report but push nothing (trigger
    /// volumes, pickups). No-op on bad ids.
    pub fn bjolt_body_set_sensor(world_ptr: *mut BJoltWorld, body_raw: u32, is_sensor: bool);
    /// Linear + angular damping on a live body. No-op on bad or static ids.
    pub fn bjolt_body_set_damping(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        linear_damping: c_float,
        angular_damping: c_float,
    );
    /// Live density: rescales mass + inertia to density × shape volume.
    /// No-op on bad or static ids.
    pub fn bjolt_body_set_density(
        world_ptr: *mut BJoltWorld,
        body_raw: u32,
        density_kg_per_m3: c_float,
    );
    /// Ragdoll builder: opaque `BJoltRagdollBuild` holding Jolt
    /// `RagdollSettings` + `Skeleton`. Parts added parents-first; joint,
    /// part, and collision-subgroup indices coincide.
    pub fn bjolt_ragdoll_build_create() -> *mut BJoltRagdollBuild;
    /// Adds one part: `shape_kind` 0 = capsule (`dim_x` cylinder half
    /// height, `dim_y` radius), 1 = box (`dim`s half extents), 2 = sphere
    /// (`dim_x` radius). Position + quaternion pose the body origin.
    /// Returns the part index, or -1 on a bad shape.
    pub fn bjolt_ragdoll_build_add_part(
        build: *mut BJoltRagdollBuild,
        parent_index: c_int,
        shape_kind: u8,
        dim_x: c_float,
        dim_y: c_float,
        dim_z: c_float,
        pos_x: c_float,
        pos_y: c_float,
        pos_z: c_float,
        rot_x: c_float,
        rot_y: c_float,
        rot_z: c_float,
        rot_w: c_float,
        object_layer: u16,
        density_kg_per_m3: c_float,
        motion_type: u8,
    ) -> c_int;
    /// Hinge limit between a part and its parent, about `anchor` (world
    /// space), within [`limits_min`, `limits_max`], seated pose reads zero.
    pub fn bjolt_ragdoll_build_set_hinge(
        build: *mut BJoltRagdollBuild,
        part_index: c_int,
        anchor_x: c_float,
        anchor_y: c_float,
        anchor_z: c_float,
        hinge_axis1_x: c_float,
        hinge_axis1_y: c_float,
        hinge_axis1_z: c_float,
        normal_axis1_x: c_float,
        normal_axis1_y: c_float,
        normal_axis1_z: c_float,
        hinge_axis2_x: c_float,
        hinge_axis2_y: c_float,
        hinge_axis2_z: c_float,
        normal_axis2_x: c_float,
        normal_axis2_y: c_float,
        normal_axis2_z: c_float,
        limits_min: c_float,
        limits_max: c_float,
    ) -> bool;
    /// Swing-twist limit between a part and its parent, about `anchor`
    /// (world space): cone swing about `twist_axis` plus bounded twist.
    /// Per-side frames (`1` parent, `2` child).
    pub fn bjolt_ragdoll_build_set_swing_twist(
        build: *mut BJoltRagdollBuild,
        part_index: c_int,
        anchor_x: c_float,
        anchor_y: c_float,
        anchor_z: c_float,
        twist_axis1_x: c_float,
        twist_axis1_y: c_float,
        twist_axis1_z: c_float,
        plane_axis1_x: c_float,
        plane_axis1_y: c_float,
        plane_axis1_z: c_float,
        twist_axis2_x: c_float,
        twist_axis2_y: c_float,
        twist_axis2_z: c_float,
        plane_axis2_x: c_float,
        plane_axis2_y: c_float,
        plane_axis2_z: c_float,
        normal_half_cone_angle: c_float,
        plane_half_cone_angle: c_float,
        twist_min_angle: c_float,
        twist_max_angle: c_float,
    ) -> bool;
    /// Jolt mass stabilization (ratio clamp + parent-inertia boost), in
    /// place. Run after all parts, before create. False on failure.
    pub fn bjolt_ragdoll_build_stabilize(build: *mut BJoltRagdollBuild) -> bool;
    /// Constraint priorities + shared parent-child no-collide filter.
    pub fn bjolt_ragdoll_build_finalize(build: *mut BJoltRagdollBuild);
    /// Creates bodies + constraints and adds them to the system in one
    /// shot. `group_id` unique per ragdoll. Null on failure.
    pub fn bjolt_ragdoll_create(
        world_ptr: *mut BJoltWorld,
        build: *mut BJoltRagdollBuild,
        group_id: u32,
        user_data: u64,
    ) -> *mut BJoltRagdoll;
    /// Part count (= body count) of a live ragdoll.
    pub fn bjolt_ragdoll_body_count(handle: *mut BJoltRagdoll) -> u32;
    /// Body ids in part order. Returns ids written (capped at capacity).
    pub fn bjolt_ragdoll_body_ids(
        handle: *mut BJoltRagdoll,
        out_ids: *mut u32,
        id_capacity: u32,
    ) -> u32;
    /// Flips every body in the ragdoll to one motion: 0 static,
    /// 1 kinematic (follow bones, hitbox mode), 2 dynamic (simulate).
    pub fn bjolt_ragdoll_set_motion(
        world_ptr: *mut BJoltWorld,
        handle: *mut BJoltRagdoll,
        motion_type: u8,
    );
    /// Removes bodies + constraints and frees the ragdoll. Never mix with
    /// per-body remove/destroy on these ids.
    pub fn bjolt_ragdoll_destroy(world_ptr: *mut BJoltWorld, handle: *mut BJoltRagdoll);
    /// Frees the builder (settings only, after create).
    pub fn bjolt_ragdoll_build_destroy(build: *mut BJoltRagdollBuild);
}
