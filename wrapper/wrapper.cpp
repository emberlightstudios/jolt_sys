// Minimal extern "C" shim over Jolt Physics for bevy_jolt.
// Deliberately small: mirrors HelloWorld (floor + sphere, ThreadPool job system).
// New subsystems (character, vehicle, soft body, ...) get their own sections here.

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/PulleyConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Character/Character.h>
 #include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <cstdint>
#include <thread>
#include <vector>

JPH_SUPPRESS_WARNINGS
using namespace JPH;

// Largest object-layer count the table filters support. Jolt itself allows
// up to 65536 object layers, but each object layer gets its own broadphase
// bucket here, and buckets are separate trees: keep this small.
static constexpr uint MAX_OBJECT_LAYERS = 16;
// Joint anchor frame: 0 = world, anything else = local to each body's
// center of mass. Matches JOINT_SPACE_* in src/lib.rs.
static inline EConstraintSpace bjolt_decode_joint_space(uint8_t joint_space)
{
	return joint_space == 0 ? EConstraintSpace::WorldSpace : EConstraintSpace::LocalToBodyCOM;
}


// Collision yes/no answers, owned by Rust and copied in at world creation.
// collide_pairs[a][b] answers "can object layer a hit object layer b?".
// Rust always writes it symmetric, so only one triangle is ever read.
// Entries are bytes (0/1), never C++ bool: Rust bool layout is unspecified.
struct BevyJoltCollisionTable {
	uint layer_count = 0;
	uint8_t collide_pairs[MAX_OBJECT_LAYERS][MAX_OBJECT_LAYERS] = {};
};

class BevyJoltBroadPhaseLayerInterface final : public BroadPhaseLayerInterface {
public:
	void SetLayerCount(uint inCount) { layer_count = inCount; }

	uint GetNumBroadPhaseLayers() const override { return layer_count; }

	BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override
	{
		JPH_ASSERT(inLayer < layer_count);
		return BroadPhaseLayer((BroadPhaseLayer::Type)inLayer);
	}

private:
	uint layer_count = 0;
};

class BevyJoltObjectVsBroadPhaseLayerFilter final : public ObjectVsBroadPhaseLayerFilter {
public:
	void SetTable(const BevyJoltCollisionTable *inTable) { table = inTable; }

	bool ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
	{
		uint layer2 = inLayer2.GetValue();
		if (inLayer1 >= table->layer_count || layer2 >= table->layer_count)
			return false;
		return table->collide_pairs[inLayer1][layer2];
	}

private:
	const BevyJoltCollisionTable *table = nullptr;
};

class BevyJoltObjectLayerPairFilter final : public ObjectLayerPairFilter {
public:
	void SetTable(const BevyJoltCollisionTable *inTable) { table = inTable; }

	bool ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override
	{
		if (inObject1 >= table->layer_count || inObject2 >= table->layer_count)
			return false;
		return table->collide_pairs[inObject1][inObject2];
	}

private:
	const BevyJoltCollisionTable *table = nullptr;
};

// Records contact begin/end + sensor overlap pairs during the step. Fixed
// caps (no allocation under solver locks); overflow drops the newest. Rust
// drains after each step.
static constexpr uint BJOLT_MAX_CONTACT_EVENTS = 256;

class BevyJoltContactListener final : public ContactListener {
public:
	uint32_t added_count = 0;
	uint32_t removed_count = 0;
	uint32_t sensor_begin_count = 0;
	uint32_t sensor_end_count = 0;
	uint32_t added_a[BJOLT_MAX_CONTACT_EVENTS] = {};
	uint32_t added_b[BJOLT_MAX_CONTACT_EVENTS] = {};
	uint32_t removed_a[BJOLT_MAX_CONTACT_EVENTS] = {};
	uint32_t removed_b[BJOLT_MAX_CONTACT_EVENTS] = {};
	uint32_t sensor_begin_a[BJOLT_MAX_CONTACT_EVENTS] = {};
	uint32_t sensor_begin_b[BJOLT_MAX_CONTACT_EVENTS] = {};
	uint32_t sensor_end_a[BJOLT_MAX_CONTACT_EVENTS] = {};
	uint32_t sensor_end_b[BJOLT_MAX_CONTACT_EVENTS] = {};

	void OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &, ContactSettings &) override
	{
		if (added_count < BJOLT_MAX_CONTACT_EVENTS)
		{
			added_a[added_count] = inBody1.GetID().GetIndexAndSequenceNumber();
			added_b[added_count] = inBody2.GetID().GetIndexAndSequenceNumber();
			++added_count;
		}
	}

	void OnContactRemoved(const SubShapeIDPair &inPair) override
	{
		if (removed_count < BJOLT_MAX_CONTACT_EVENTS)
		{
			removed_a[removed_count] = inPair.GetBody1ID().GetIndexAndSequenceNumber();
			removed_b[removed_count] = inPair.GetBody2ID().GetIndexAndSequenceNumber();
			++removed_count;
		}
	}

	void OnContactPersisted(const Body &, const Body &, const ContactManifold &, ContactSettings &) override
	{
	}
};

// Sleep/wake transitions during the step. Same fixed-cap, drain-after-step
// rule as contacts: activations are rare, so 64 never fills in practice.
static constexpr uint BJOLT_MAX_ACTIVATION_EVENTS = 64;

class BevyJoltActivationListener final : public BodyActivationListener {
public:
	uint32_t slept_count = 0;
	uint32_t woke_count = 0;
	uint32_t slept_ids[BJOLT_MAX_ACTIVATION_EVENTS] = {};
	uint32_t woke_ids[BJOLT_MAX_ACTIVATION_EVENTS] = {};

	void OnBodyActivated(const BodyID &inBodyID, uint64) override
	{
		if (woke_count < BJOLT_MAX_ACTIVATION_EVENTS)
			woke_ids[woke_count++] = inBodyID.GetIndexAndSequenceNumber();
	}

	void OnBodyDeactivated(const BodyID &inBodyID, uint64) override
	{
		if (slept_count < BJOLT_MAX_ACTIVATION_EVENTS)
			slept_ids[slept_count++] = inBodyID.GetIndexAndSequenceNumber();
	}
};
struct BJoltWorld {
	// Declared first: filters below borrow it, members destroy in reverse
	// order so the table outlives every PhysicsSystem query.
	BevyJoltCollisionTable collision_table;
	BevyJoltBroadPhaseLayerInterface broad_phase_interface;
	BevyJoltObjectVsBroadPhaseLayerFilter object_vs_broad_phase_filter;
	BevyJoltObjectLayerPairFilter object_pair_filter;
	TempAllocatorImpl *temp_allocator = nullptr;
	// Budgets for the guard below: set once at creation, read at every
	// create site. Jolt never grows the pool, so this is the final word.
	uint32_t max_bodies = 0;
	uint32_t max_body_pairs = 0;
	uint32_t max_contact_constraints = 0;
	JobSystemThreadPool *job_system = nullptr;
	PhysicsSystem *physics_system = nullptr;
	// Constraint registry: ids handed to Rust are 1-based indices.
	// Entries are nulled on removal but never reused, so ids stay stable.
	// Ref keeps each live constraint alive if Jolt releases its own ref.
	std::vector<Ref<Constraint>> constraint_registry;
	// Character registry: same 1-based id scheme. CharacterVirtual is a
	// RefTarget, so Ref keeps each live character alive; destroyed slots
	// hold null and are never reused.
	std::vector<Ref<CharacterVirtual>> character_registry;
	// Object layer per character slot (Jolt characters carry no layer
	// themselves; the movement/teleport/stance filters need it).
	std::vector<ObjectLayer> character_layers;
	// Rigid character registry: same 1-based id scheme. Character owns a
	// real body in the physics system; destroyed slots null out and never
	// recycle.
	std::vector<Ref<Character>> rigid_character_registry;
	// Ragdoll registry: live `Ragdoll` per slot, 1-based Rust ids. `Ref`
	// keeps each ragdoll alive; destroyed slots hold null and are never
	// reused, so ids stay stable for the world's lifetime.
	std::vector<Ref<Ragdoll>> ragdoll_registry;
	// Contact + sensor event queues: the listener records raw body ids
	// during the step (no Bevy access under solver locks); Rust drains them
	// after the step and resolves entities itself.
	BevyJoltContactListener *contact_listener = nullptr;
	// Sleep/wake transitions, same record-then-drain rule.
	BevyJoltActivationListener *activation_listener = nullptr;
};
// Body-budget guard: Jolt hands back a null body with no message when the
// pool is dry, which used to surface as a bare assert pages away. Every
// create site calls this first so the log names the limit and the fix.
static inline bool bjolt_bodies_available(BJoltWorld *world)
{
	uint32_t live_bodies = world->physics_system->GetNumBodies();
	if (live_bodies < world->max_bodies)
		return true;
	fprintf(stderr,
		"[bevy_jolt] out of Jolt bodies: %u live of %u max. "
		"Raise the world budget (max bodies) and retry.\n",
		live_bodies, world->max_bodies);
	return false;
}

// Files the constraint and returns its 1-based Rust id.
static uint32_t bjolt_register_constraint(BJoltWorld *world, Constraint *constraint)
{
	world->physics_system->AddConstraint(constraint);
	world->constraint_registry.push_back(constraint);
	return (uint32_t)world->constraint_registry.size();
}

static bool g_jolt_initialized = false;

extern "C" {

bool bjolt_init()
{
	if (g_jolt_initialized)
		return true;
	RegisterDefaultAllocator();
	Factory::sInstance = new Factory();
	RegisterTypes();
	g_jolt_initialized = true;
	return true;
}

BJoltWorld *bjolt_world_create_with_layers(uint layer_count, const uint8_t *collide_matrix,
	uint32_t max_bodies, uint32_t max_body_pairs, uint32_t max_contact_constraints,
	uint64_t temp_allocator_bytes)
{
	BJoltWorld *world = new BJoltWorld();
	if (layer_count == 0 || layer_count > MAX_OBJECT_LAYERS || collide_matrix == nullptr)
	{
		delete world;
		return nullptr;
	}
	// Budgets are fixed for the world's life: Jolt preallocates, never
	// grows. Zero means "pick the default" so older callers stay valid.
	if (max_bodies == 0)
		max_bodies = 10240;
	if (max_body_pairs == 0)
		max_body_pairs = 65536;
	if (max_contact_constraints == 0)
		max_contact_constraints = 20480;
	if (temp_allocator_bytes == 0)
		temp_allocator_bytes = 32ULL * 1024ULL * 1024ULL;
	world->collision_table.layer_count = layer_count;
	for (uint row = 0; row < layer_count; ++row)
		for (uint col = 0; col < layer_count; ++col)
			world->collision_table.collide_pairs[row][col] = collide_matrix[row * MAX_OBJECT_LAYERS + col] ? 1 : 0;
	world->broad_phase_interface.SetLayerCount(layer_count);
	world->object_vs_broad_phase_filter.SetTable(&world->collision_table);
	world->object_pair_filter.SetTable(&world->collision_table);
	world->temp_allocator = new TempAllocatorImpl((unsigned int)temp_allocator_bytes);
	const int thread_count = (int)std::thread::hardware_concurrency() - 1;
	world->job_system = new JobSystemThreadPool(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread_count > 0 ? thread_count : 1);
	world->physics_system = new PhysicsSystem();
	world->max_bodies = max_bodies;
	world->max_body_pairs = max_body_pairs;
	world->max_contact_constraints = max_contact_constraints;
	world->physics_system->Init(world->max_bodies, 0, world->max_body_pairs,
		world->max_contact_constraints,
		world->broad_phase_interface,
		world->object_vs_broad_phase_filter,
		world->object_pair_filter);
	world->contact_listener = new BevyJoltContactListener();
	world->physics_system->SetContactListener(world->contact_listener);
	world->activation_listener = new BevyJoltActivationListener();
	world->physics_system->SetBodyActivationListener(world->activation_listener);
	return world;
}
// Live body count for budget telemetry (pairs/contacts have no Jolt-side
// counter: size those by rule, watch bodies live).
uint32_t bjolt_world_body_count(BJoltWorld *world, uint32_t *out_max_bodies)
{
	if (world == nullptr)
		return 0;
	if (out_max_bodies != nullptr)
		*out_max_bodies = world->max_bodies;
	return world->physics_system->GetNumBodies();
}

void bjolt_world_destroy(BJoltWorld *world)
{
	if (world == nullptr)
		return;
	// Characters own bodies in the physics system: release them first so
	// their destructors run against a live system, not a deleted one.
	world->rigid_character_registry.clear();
	world->character_registry.clear();
	// Same for ragdolls: each live Ragdoll destroys its bodies through the
	// physics system when its last Ref drops. Remove and clear the registry
	// now, while the system is alive; after the system is deleted that
	// destructor touches freed memory and the process dies on exit.
	for (Ref<Ragdoll> &ragdoll_slot : world->ragdoll_registry)
	{
		if (ragdoll_slot != nullptr)
			ragdoll_slot->RemoveFromPhysicsSystem();
	}
	world->ragdoll_registry.clear();
	delete world->contact_listener;
	delete world->activation_listener;
	delete world->physics_system;
	delete world->job_system;
	delete world->temp_allocator;
	delete world;
}

uint32_t bjolt_create_floor(BJoltWorld *world, float half_x, float half_y, float half_z, float pos_y)
{
	if (!bjolt_bodies_available(world))
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BoxShapeSettings shape_settings(Vec3(half_x, half_y, half_z));
	shape_settings.SetEmbedded();
	Shape::ShapeResult shape_result = shape_settings.Create();
	Ref<Shape> shape = shape_result.Get();
	BodyCreationSettings settings(shape, RVec3(0, pos_y, 0), Quat::sIdentity(), EMotionType::Static, (ObjectLayer)0);
	Body *body = body_interface.CreateBody(settings);
	BodyID body_id = body->GetID();
	body_interface.AddBody(body_id, EActivation::DontActivate);
	return body_id.GetIndexAndSequenceNumber();
}

uint32_t bjolt_create_sphere(BJoltWorld *world, float radius,
	float pos_x, float pos_y, float pos_z, uint16_t object_layer, float density_kg_per_m3, float gravity_factor)
{
	if (!bjolt_bodies_available(world))
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	SphereShapeSettings shape_settings(radius);
	shape_settings.mDensity = density_kg_per_m3;
	shape_settings.SetEmbedded();
	Shape::ShapeResult shape_result = shape_settings.Create();
	BodyCreationSettings settings(shape_result.Get(),
		RVec3(pos_x, pos_y, pos_z), Quat::sIdentity(), EMotionType::Dynamic, object_layer);
	Body *body = body_interface.CreateBody(settings);
	BodyID body_id = body->GetID();
	body_interface.AddBody(body_id, EActivation::Activate);
	return body_id.GetIndexAndSequenceNumber();
}

void bjolt_world_update(BJoltWorld *world, float delta_time, int collision_steps)
{
	world->physics_system->Update(delta_time, collision_steps, world->temp_allocator, world->job_system);
}

bool bjolt_body_is_active(BJoltWorld *world, uint32_t body_id_raw)
{
	BodyID body_id(body_id_raw);
	return world->physics_system->GetBodyInterface().IsActive(body_id);
}

void bjolt_body_state(BJoltWorld *world, uint32_t body_id_raw, float *out_position, float *out_velocity, float *out_angular_velocity)
{
	BodyID body_id(body_id_raw);
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	RVec3 position = body_interface.GetCenterOfMassPosition(body_id);
	Vec3 velocity = body_interface.GetLinearVelocity(body_id);
	Vec3 spin = body_interface.GetAngularVelocity(body_id);
	out_position[0] = position.GetX();
	out_position[1] = position.GetY();
	out_position[2] = position.GetZ();
	out_velocity[0] = velocity.GetX();
	out_velocity[1] = velocity.GetY();
	out_velocity[2] = velocity.GetZ();
	out_angular_velocity[0] = spin.GetX();
	out_angular_velocity[1] = spin.GetY();
	out_angular_velocity[2] = spin.GetZ();
}

void bjolt_body_remove_destroy(BJoltWorld *world, uint32_t body_id_raw)
{
	BodyID body_id(body_id_raw);
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	body_interface.RemoveBody(body_id);
	body_interface.DestroyBody(body_id);
}

uint32_t bjolt_create_box(BJoltWorld *world, float half_x, float half_y, float half_z,
	float pos_x, float pos_y, float pos_z, uint16_t object_layer, uint8_t motion, float density_kg_per_m3, float gravity_factor)
{
	if (!bjolt_bodies_available(world))
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BoxShapeSettings shape_settings(Vec3(half_x, half_y, half_z));
	shape_settings.mDensity = density_kg_per_m3;
	shape_settings.SetEmbedded();
	Shape::ShapeResult shape_result = shape_settings.Create();
	Ref<Shape> shape = shape_result.Get();
	// 0 = static, 1 = kinematic, anything else = dynamic. Matches JoltMotion order.
	EMotionType motion_type = motion == 0 ? EMotionType::Static :
		(motion == 1 ? EMotionType::Kinematic : EMotionType::Dynamic);
	BodyCreationSettings settings(shape, RVec3(pos_x, pos_y, pos_z), Quat::sIdentity(), motion_type, object_layer);
	settings.mGravityFactor = gravity_factor;
	// Kinematic platforms driven every tick must never sleep, or MoveKinematic
	// stops producing velocity and riders fall off.
	settings.mAllowSleeping = (motion_type != EMotionType::Kinematic);
	BodyID body_id = body_interface.CreateAndAddBody(settings,
		motion_type == EMotionType::Static ? EActivation::DontActivate : EActivation::Activate);
	return body_id.GetIndexAndSequenceNumber();
}

// Drives a kinematic body toward a target pose: Jolt derives the velocity
// from the delta so it pushes dynamics on contact. No-op on other motions.
void bjolt_move_kinematic(BJoltWorld *world, uint32_t body_raw,
	float pos_x, float pos_y, float pos_z,
	float rot_x, float rot_y, float rot_z, float rot_w,
	float delta_time)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	body_interface.MoveKinematic(body_id, RVec3(pos_x, pos_y, pos_z),
		Quat(rot_x, rot_y, rot_z, rot_w), delta_time);
	// MoveKinematic only auto-activates on nonzero derived velocity; at sine
	// extremes the delta vanishes, so keep it awake explicitly.
	body_interface.ActivateBody(body_id);
}

// Manual sleep: freezes the body where it stands. Still solid, still in the
// broadphase, wakes on contact. No-op on bad ids.
void bjolt_sleep_body(BJoltWorld *world, uint32_t body_raw)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	body_interface.DeactivateBody(BodyID(body_raw));
}

// Manual wake: rejoins the sim next step with velocities intact. No-op on
// bad ids.
void bjolt_wake_body(BJoltWorld *world, uint32_t body_raw)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	body_interface.ActivateBody(BodyID(body_raw));
}

// Live motion-type flip: 0 static (solid, unwakeable), 1 kinematic,
// 2 dynamic (rejoins with velocities intact). Static deactivates inside
// Jolt; non-static activates. No-op on bad ids.
void bjolt_set_motion_type(BJoltWorld *world, uint32_t body_raw, uint8_t motion_type)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	EMotionType jolt_motion = motion_type == 0 ? EMotionType::Static :
		(motion_type == 1 ? EMotionType::Kinematic : EMotionType::Dynamic);
	EActivation wake = jolt_motion == EMotionType::Static ?
		EActivation::DontActivate : EActivation::Activate;
	body_interface.SetMotionType(body_id, jolt_motion, wake);
}

uint32_t bjolt_create_capsule(BJoltWorld *world, float half_height, float radius,
	float pos_x, float pos_y, float pos_z, uint16_t object_layer, float density_kg_per_m3, float gravity_factor)
{
	if (!bjolt_bodies_available(world))
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	CapsuleShapeSettings shape_settings(half_height, radius);
	shape_settings.mDensity = density_kg_per_m3;
	shape_settings.SetEmbedded();
	Shape::ShapeResult shape_result = shape_settings.Create();
	BodyCreationSettings settings(shape_result.Get(),
		RVec3(pos_x, pos_y, pos_z), Quat::sIdentity(), EMotionType::Dynamic, object_layer);
	settings.mGravityFactor = gravity_factor;
	BodyID body_id = body_interface.CreateAndAddBody(settings, EActivation::Activate);
	return body_id.GetIndexAndSequenceNumber();
}

// Solid cylinder along Y: half_height to each flat face, radius around the
// axis. Matches Bevy's Cylinder primitive (radius, half height). Convex
// radius rounds the rim slightly; keep Jolt's default so rolling stays
// stable instead of catching on a sharp edge.
uint32_t bjolt_create_cylinder(BJoltWorld *world, float half_height, float radius,
	float pos_x, float pos_y, float pos_z, uint16_t object_layer, float density_kg_per_m3, float gravity_factor)
{
	if (!bjolt_bodies_available(world))
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	CylinderShapeSettings shape_settings(half_height, radius);
	shape_settings.mDensity = density_kg_per_m3;
	shape_settings.SetEmbedded();
	Shape::ShapeResult shape_result = shape_settings.Create();
	if (shape_result.HasError())
		return 0;
	BodyCreationSettings settings(shape_result.Get(),
		RVec3(pos_x, pos_y, pos_z), Quat::sIdentity(), EMotionType::Dynamic, object_layer);
	settings.mGravityFactor = gravity_factor;
	BodyID body_id = body_interface.CreateAndAddBody(settings, EActivation::Activate);
	return body_id.GetIndexAndSequenceNumber();
}

// Tapered cylinder (cone/frustum) along Y: half_height to each face,
// separate top/bottom radii. Equal radii behave like a plain cylinder.
uint32_t bjolt_create_tapered_cylinder(BJoltWorld *world, float half_height,
	float top_radius, float bottom_radius,
	float pos_x, float pos_y, float pos_z, uint16_t object_layer, float density_kg_per_m3, float gravity_factor)
{
	if (!bjolt_bodies_available(world))
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	TaperedCylinderShapeSettings shape_settings(half_height, top_radius, bottom_radius);
	shape_settings.mDensity = density_kg_per_m3;
	shape_settings.SetEmbedded();
	Shape::ShapeResult shape_result = shape_settings.Create();
	if (shape_result.HasError())
		return 0;
	BodyCreationSettings settings(shape_result.Get(),
		RVec3(pos_x, pos_y, pos_z), Quat::sIdentity(), EMotionType::Dynamic, object_layer);
	settings.mGravityFactor = gravity_factor;
	BodyID body_id = body_interface.CreateAndAddBody(settings, EActivation::Activate);
	return body_id.GetIndexAndSequenceNumber();
}

// Tapered capsule along Y: like a capsule but the two end spheres may have
// different radii. Equal radii behave like a plain capsule.
uint32_t bjolt_create_tapered_capsule(BJoltWorld *world, float half_height,
	float top_radius, float bottom_radius,
	float pos_x, float pos_y, float pos_z, uint16_t object_layer, float density_kg_per_m3, float gravity_factor)
{
	if (!bjolt_bodies_available(world))
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	TaperedCapsuleShapeSettings shape_settings(half_height, top_radius, bottom_radius);
	shape_settings.mDensity = density_kg_per_m3;
	shape_settings.SetEmbedded();
	Shape::ShapeResult shape_result = shape_settings.Create();
	if (shape_result.HasError())
		return 0;
	BodyCreationSettings settings(shape_result.Get(),
		RVec3(pos_x, pos_y, pos_z), Quat::sIdentity(), EMotionType::Dynamic, object_layer);
	settings.mGravityFactor = gravity_factor;
	BodyID body_id = body_interface.CreateAndAddBody(settings, EActivation::Activate);
	return body_id.GetIndexAndSequenceNumber();
}

void bjolt_body_transform(BJoltWorld *world, uint32_t body_id_raw, float *out_position, float *out_rotation, float *out_velocity, float *out_angular_velocity)
{
	BodyID body_id(body_id_raw);
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	RVec3 position;
	Quat rotation;
	body_interface.GetPositionAndRotation(body_id, position, rotation);
	Vec3 velocity = body_interface.GetLinearVelocity(body_id);
	Vec3 spin = body_interface.GetAngularVelocity(body_id);
	out_position[0] = position.GetX();
	out_position[1] = position.GetY();
	out_position[2] = position.GetZ();
	out_rotation[0] = rotation.GetX();
	out_rotation[1] = rotation.GetY();
	out_rotation[2] = rotation.GetZ();
	out_rotation[3] = rotation.GetW();
	out_velocity[0] = velocity.GetX();
	out_velocity[1] = velocity.GetY();
	out_velocity[2] = velocity.GetZ();
	out_angular_velocity[0] = spin.GetX();
	out_angular_velocity[1] = spin.GetY();
	out_angular_velocity[2] = spin.GetZ();
}

bool bjolt_cast_ray(BJoltWorld *world,
	float origin_x, float origin_y, float origin_z,
	float dir_x, float dir_y, float dir_z,
	uint32_t *out_hit_body, float *out_hit_fraction)
{
	const NarrowPhaseQuery &query = world->physics_system->GetNarrowPhaseQuery();
	RRayCast ray(RVec3(origin_x, origin_y, origin_z), Vec3(dir_x, dir_y, dir_z));
	RayCastResult hit;
	if (!query.CastRay(ray, hit))
		return false;
	*out_hit_body = hit.mBodyID.GetIndexAndSequenceNumber();
	*out_hit_fraction = hit.mFraction;
	return true;
}

// Batch query plumbing: `AllHitCollisionCollector` gathers every hit (cap
// the report, not the search: the count tells Rust the buffer overflowed).
// Contact-point arrays are flat xyz triplets; point queries leave the
// fraction array zeroed.
static uint32_t bjolt_write_ray_hits(const AllHitCollisionCollector<CastRayCollector> &collector,
	uint32_t *out_hit_bodies, float *out_hit_fractions, uint32_t hit_capacity)
{
	uint32_t hit_total = (uint32_t)collector.mHits.size();
	uint32_t hit_kept = hit_total < hit_capacity ? hit_total : hit_capacity;
	for (uint32_t hit_index = 0; hit_index < hit_kept; ++hit_index)
	{
		out_hit_bodies[hit_index] = collector.mHits[hit_index].mBodyID.GetIndexAndSequenceNumber();
		out_hit_fractions[hit_index] = collector.mHits[hit_index].mFraction;
	}
	return hit_total;
}

uint32_t bjolt_cast_ray_all(BJoltWorld *world,
	float origin_x, float origin_y, float origin_z,
	float dir_x, float dir_y, float dir_z,
	uint32_t *out_hit_bodies, float *out_hit_fractions, uint32_t hit_capacity)
{
	if (out_hit_bodies == nullptr || out_hit_fractions == nullptr || hit_capacity == 0)
		return 0;
	const NarrowPhaseQuery &query = world->physics_system->GetNarrowPhaseQuery();
	RRayCast ray(RVec3(origin_x, origin_y, origin_z), Vec3(dir_x, dir_y, dir_z));
	AllHitCollisionCollector<CastRayCollector> collector;
	RayCastSettings ray_settings;
	query.CastRay(ray, ray_settings, collector);
	collector.Sort();
	return bjolt_write_ray_hits(collector, out_hit_bodies, out_hit_fractions, hit_capacity);
}

uint32_t bjolt_collide_point_all(BJoltWorld *world,
	float point_x, float point_y, float point_z,
	uint32_t *out_hit_bodies, float *out_hit_fractions, uint32_t hit_capacity)
{
	if (out_hit_bodies == nullptr || out_hit_fractions == nullptr || hit_capacity == 0)
		return 0;
	const NarrowPhaseQuery &query = world->physics_system->GetNarrowPhaseQuery();
	AllHitCollisionCollector<CollidePointCollector> collector;
	query.CollidePoint(RVec3(point_x, point_y, point_z), collector);
	// Point hits carry no fraction: report ids only, zero the fractions.
	uint32_t hit_total = (uint32_t)collector.mHits.size();
	uint32_t hit_kept = hit_total < hit_capacity ? hit_total : hit_capacity;
	for (uint32_t hit_index = 0; hit_index < hit_kept; ++hit_index)
	{
		out_hit_bodies[hit_index] = collector.mHits[hit_index].mBodyID.GetIndexAndSequenceNumber();
		out_hit_fractions[hit_index] = 0.0f;
	}
	return hit_total;
}

// Probe shape for overlap/sweep: box or sphere centered on its center of
// mass, identity rotation. Heap-own the shape for the call; the query never
// outlives it.
static Ref<Shape> bjolt_probe_shape(uint32_t probe_kind, float half_x, float half_y, float half_z)
{
	if (probe_kind == 1)
		return new SphereShape(half_x);
	return new BoxShape(Vec3(half_x, half_y, half_z));
}

uint32_t bjolt_overlap_shape_all(BJoltWorld *world,
	uint32_t probe_kind,
	float center_x, float center_y, float center_z,
	float probe_half_x, float probe_half_y, float probe_half_z,
	uint32_t *out_hit_bodies, float *out_hit_depths, float *out_contact_points, uint32_t hit_capacity)
{
	if (out_hit_bodies == nullptr || out_hit_depths == nullptr || out_contact_points == nullptr || hit_capacity == 0)
		return 0;
	const NarrowPhaseQuery &query = world->physics_system->GetNarrowPhaseQuery();
	Ref<Shape> probe_shape = bjolt_probe_shape(probe_kind, probe_half_x, probe_half_y, probe_half_z);
	RMat44 probe_pose = RMat44::sTranslation(RVec3(center_x, center_y, center_z));
	CollideShapeSettings collide_settings;
	AllHitCollisionCollector<CollideShapeCollector> collector;
	query.CollideShape(probe_shape, Vec3::sOne(), probe_pose, collide_settings, RVec3::sZero(), collector);
	uint32_t hit_total = (uint32_t)collector.mHits.size();
	uint32_t hit_kept = hit_total < hit_capacity ? hit_total : hit_capacity;
	for (uint32_t hit_index = 0; hit_index < hit_kept; ++hit_index)
	{
		const CollideShapeResult &hit = collector.mHits[hit_index];
		out_hit_bodies[hit_index] = hit.mBodyID2.GetIndexAndSequenceNumber();
		out_hit_depths[hit_index] = hit.mPenetrationDepth;
		out_contact_points[3 * hit_index] = hit.mContactPointOn2.GetX();
		out_contact_points[3 * hit_index + 1] = hit.mContactPointOn2.GetY();
		out_contact_points[3 * hit_index + 2] = hit.mContactPointOn2.GetZ();
	}
	return hit_total;
}

uint32_t bjolt_cast_shape_all(BJoltWorld *world,
	uint32_t probe_kind,
	float center_x, float center_y, float center_z,
	float probe_half_x, float probe_half_y, float probe_half_z,
	float cast_dir_x, float cast_dir_y, float cast_dir_z,
	uint32_t *out_hit_bodies, float *out_hit_fractions, float *out_contact_points, uint32_t hit_capacity)
{
	if (out_hit_bodies == nullptr || out_hit_fractions == nullptr || out_contact_points == nullptr || hit_capacity == 0)
		return 0;
	const NarrowPhaseQuery &query = world->physics_system->GetNarrowPhaseQuery();
	Ref<Shape> probe_shape = bjolt_probe_shape(probe_kind, probe_half_x, probe_half_y, probe_half_z);
	RMat44 probe_start = RMat44::sTranslation(RVec3(center_x, center_y, center_z));
	RShapeCast shape_cast(probe_shape, Vec3::sOne(), probe_start, Vec3(cast_dir_x, cast_dir_y, cast_dir_z));
	ShapeCastSettings cast_settings;
	AllHitCollisionCollector<CastShapeCollector> collector;
	query.CastShape(shape_cast, cast_settings, RVec3::sZero(), collector);
	collector.Sort();
	uint32_t hit_total = (uint32_t)collector.mHits.size();
	uint32_t hit_kept = hit_total < hit_capacity ? hit_total : hit_capacity;
	for (uint32_t hit_index = 0; hit_index < hit_kept; ++hit_index)
	{
		const ShapeCastResult &hit = collector.mHits[hit_index];
		out_hit_bodies[hit_index] = hit.mBodyID2.GetIndexAndSequenceNumber();
		out_hit_fractions[hit_index] = hit.mFraction;
		out_contact_points[3 * hit_index] = hit.mContactPointOn2.GetX();
		out_contact_points[3 * hit_index + 1] = hit.mContactPointOn2.GetY();
		out_contact_points[3 * hit_index + 2] = hit.mContactPointOn2.GetZ();
	}
	return hit_total;
}

uint32_t bjolt_create_plane(BJoltWorld *world,
	float normal_x, float normal_y, float normal_z, float constant,
	float half_extent, uint16_t object_layer)
{
	if (!bjolt_bodies_available(world))
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	Plane plane(Vec3(normal_x, normal_y, normal_z), constant);
	BodyCreationSettings settings(new PlaneShape(plane, nullptr, half_extent),
		RVec3(0, 0, 0), Quat::sIdentity(), EMotionType::Static, object_layer);
	BodyID body_id = body_interface.CreateAndAddBody(settings, EActivation::DontActivate);
	return body_id.GetIndexAndSequenceNumber();
}

// Constraints: ids are indices into a per-world registry. Returns 0 on
// failure (body ids invalid or settings rejected); valid ids start at 1.
uint32_t bjolt_create_fixed_constraint(BJoltWorld *world, uint32_t body1_raw, uint32_t body2_raw, uint8_t joint_space)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	FixedConstraintSettings settings;
	settings.mSpace = bjolt_decode_joint_space(joint_space);
	settings.mAutoDetectPoint = true;
	TwoBodyConstraint *constraint = body_interface.CreateConstraint(&settings, BodyID(body1_raw), BodyID(body2_raw));
	if (constraint == nullptr)
		return 0;
	world->physics_system->AddConstraint(constraint);
	world->constraint_registry.push_back(constraint);
	return (uint32_t)world->constraint_registry.size();
}

uint32_t bjolt_create_distance_constraint(BJoltWorld *world, uint32_t body1_raw, uint32_t body2_raw,
	float point1_x, float point1_y, float point1_z,
	float point2_x, float point2_y, float point2_z,
	float min_distance, float max_distance, uint8_t joint_space)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	DistanceConstraintSettings settings;
	settings.mSpace = bjolt_decode_joint_space(joint_space);
	settings.mPoint1 = RVec3(point1_x, point1_y, point1_z);
	settings.mPoint2 = RVec3(point2_x, point2_y, point2_z);
	settings.mMinDistance = min_distance;
	settings.mMaxDistance = max_distance;
	TwoBodyConstraint *constraint = body_interface.CreateConstraint(&settings, BodyID(body1_raw), BodyID(body2_raw));
	if (constraint == nullptr)
		return 0;
	world->physics_system->AddConstraint(constraint);
	world->constraint_registry.push_back(constraint);
	return (uint32_t)world->constraint_registry.size();
}

uint32_t bjolt_create_hinge_constraint(BJoltWorld *world, uint32_t body1_raw, uint32_t body2_raw,
	float point_x, float point_y, float point_z,
	float hinge_axis1_x, float hinge_axis1_y, float hinge_axis1_z,
	float normal_axis1_x, float normal_axis1_y, float normal_axis1_z,
	float hinge_axis2_x, float hinge_axis2_y, float hinge_axis2_z,
	float normal_axis2_x, float normal_axis2_y, float normal_axis2_z,
	float limits_min, float limits_max,
	float motor_frequency, float motor_damping, float motor_force_limit,
	uint8_t joint_space)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	HingeConstraintSettings settings;
	settings.mSpace = bjolt_decode_joint_space(joint_space);
	settings.mPoint1 = RVec3(point_x, point_y, point_z);
	settings.mPoint2 = RVec3(point_x, point_y, point_z);
	settings.mHingeAxis1 = Vec3(hinge_axis1_x, hinge_axis1_y, hinge_axis1_z);
	settings.mNormalAxis1 = Vec3(normal_axis1_x, normal_axis1_y, normal_axis1_z);
	settings.mHingeAxis2 = Vec3(hinge_axis2_x, hinge_axis2_y, hinge_axis2_z);
	settings.mNormalAxis2 = Vec3(normal_axis2_x, normal_axis2_y, normal_axis2_z);
	settings.mLimitsMin = limits_min;
	settings.mLimitsMax = limits_max;
	settings.mMotorSettings = MotorSettings(ESpringMode::FrequencyAndDamping, motor_frequency, motor_damping);
	settings.mMotorSettings.SetTorqueLimit(motor_force_limit);
	TwoBodyConstraint *constraint = body_interface.CreateConstraint(&settings, BodyID(body1_raw), BodyID(body2_raw));
	if (constraint == nullptr)
		return 0;
	world->physics_system->AddConstraint(constraint);
	world->constraint_registry.push_back(constraint);
	return (uint32_t)world->constraint_registry.size();
}

uint32_t bjolt_create_point_constraint(BJoltWorld *world, uint32_t body1_raw, uint32_t body2_raw,
	float point_x, float point_y, float point_z, uint8_t joint_space)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	PointConstraintSettings settings;
	settings.mSpace = bjolt_decode_joint_space(joint_space);
	settings.mPoint1 = RVec3(point_x, point_y, point_z);
	settings.mPoint2 = RVec3(point_x, point_y, point_z);
	TwoBodyConstraint *constraint = body_interface.CreateConstraint(&settings, BodyID(body1_raw), BodyID(body2_raw));
	if (constraint == nullptr)
		return 0;
	world->physics_system->AddConstraint(constraint);
	world->constraint_registry.push_back(constraint);
	return (uint32_t)world->constraint_registry.size();
}

uint32_t bjolt_create_slider_constraint(BJoltWorld *world, uint32_t body1_raw, uint32_t body2_raw,
	float slider_axis1_x, float slider_axis1_y, float slider_axis1_z,
	float normal_axis1_x, float normal_axis1_y, float normal_axis1_z,
	float slider_axis2_x, float slider_axis2_y, float slider_axis2_z,
	float normal_axis2_x, float normal_axis2_y, float normal_axis2_z,
	float limits_min, float limits_max,
	float motor_frequency, float motor_damping, float motor_force_limit,
	uint8_t joint_space)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	SliderConstraintSettings settings;
	settings.mSpace = bjolt_decode_joint_space(joint_space);
	settings.mAutoDetectPoint = true;
	settings.mSliderAxis1 = Vec3(slider_axis1_x, slider_axis1_y, slider_axis1_z);
	settings.mNormalAxis1 = Vec3(normal_axis1_x, normal_axis1_y, normal_axis1_z);
	settings.mSliderAxis2 = Vec3(slider_axis2_x, slider_axis2_y, slider_axis2_z);
	settings.mNormalAxis2 = Vec3(normal_axis2_x, normal_axis2_y, normal_axis2_z);
	settings.mLimitsMin = limits_min;
	settings.mLimitsMax = limits_max;
	settings.mMotorSettings = MotorSettings(ESpringMode::FrequencyAndDamping, motor_frequency, motor_damping);
	settings.mMotorSettings.SetForceLimit(motor_force_limit);
	TwoBodyConstraint *constraint = body_interface.CreateConstraint(&settings, BodyID(body1_raw), BodyID(body2_raw));
	if (constraint == nullptr)
		return 0;
	world->physics_system->AddConstraint(constraint);
	world->constraint_registry.push_back(constraint);
	return (uint32_t)world->constraint_registry.size();
}

uint32_t bjolt_create_cone_constraint(BJoltWorld *world, uint32_t body1_raw, uint32_t body2_raw,
	float point_x, float point_y, float point_z,
	float twist_axis1_x, float twist_axis1_y, float twist_axis1_z,
	float twist_axis2_x, float twist_axis2_y, float twist_axis2_z,
	float half_cone_angle, uint8_t joint_space)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	ConeConstraintSettings settings;
	settings.mSpace = bjolt_decode_joint_space(joint_space);
	settings.mPoint1 = RVec3(point_x, point_y, point_z);
	settings.mPoint2 = RVec3(point_x, point_y, point_z);
	settings.mTwistAxis1 = Vec3(twist_axis1_x, twist_axis1_y, twist_axis1_z);
	settings.mTwistAxis2 = Vec3(twist_axis2_x, twist_axis2_y, twist_axis2_z);
	settings.mHalfConeAngle = half_cone_angle;
	TwoBodyConstraint *constraint = body_interface.CreateConstraint(&settings, BodyID(body1_raw), BodyID(body2_raw));
	if (constraint == nullptr)
		return 0;
	world->physics_system->AddConstraint(constraint);
	world->constraint_registry.push_back(constraint);
	return (uint32_t)world->constraint_registry.size();
}

uint32_t bjolt_create_swing_twist_constraint(BJoltWorld *world, uint32_t body1_raw, uint32_t body2_raw,
	float position_x, float position_y, float position_z,
	float twist_axis1_x, float twist_axis1_y, float twist_axis1_z,
	float plane_axis1_x, float plane_axis1_y, float plane_axis1_z,
	float twist_axis2_x, float twist_axis2_y, float twist_axis2_z,
	float plane_axis2_x, float plane_axis2_y, float plane_axis2_z,
	float normal_half_cone_angle, float plane_half_cone_angle,
	float twist_min_angle, float twist_max_angle, uint8_t joint_space)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	SwingTwistConstraintSettings settings;
	settings.mSpace = bjolt_decode_joint_space(joint_space);
	settings.mPosition1 = RVec3(position_x, position_y, position_z);
	settings.mPosition2 = RVec3(position_x, position_y, position_z);
	settings.mTwistAxis1 = Vec3(twist_axis1_x, twist_axis1_y, twist_axis1_z);
	settings.mPlaneAxis1 = Vec3(plane_axis1_x, plane_axis1_y, plane_axis1_z);
	settings.mTwistAxis2 = Vec3(twist_axis2_x, twist_axis2_y, twist_axis2_z);
	settings.mPlaneAxis2 = Vec3(plane_axis2_x, plane_axis2_y, plane_axis2_z);
	settings.mNormalHalfConeAngle = normal_half_cone_angle;
	settings.mPlaneHalfConeAngle = plane_half_cone_angle;
	settings.mTwistMinAngle = twist_min_angle;
	settings.mTwistMaxAngle = twist_max_angle;
	// Motors start off: the drive call arms swing or twist on demand.
	// Same arming as the ragdoll build path; without torque limits the
	// drive branch spins against zero torque.
	settings.mSwingMotorSettings = MotorSettings(ESpringMode::FrequencyAndDamping, 8.0f, 1.0f);
	settings.mSwingMotorSettings.SetTorqueLimit(1.0e6f);
	settings.mTwistMotorSettings = MotorSettings(ESpringMode::FrequencyAndDamping, 8.0f, 1.0f);
	settings.mTwistMotorSettings.SetTorqueLimit(1.0e6f);
	TwoBodyConstraint *constraint = body_interface.CreateConstraint(&settings, BodyID(body1_raw), BodyID(body2_raw));
	if (constraint == nullptr)
		return 0;
	world->physics_system->AddConstraint(constraint);
	world->constraint_registry.push_back(constraint);
	return (uint32_t)world->constraint_registry.size();
}

void bjolt_remove_constraint(BJoltWorld *world, uint32_t constraint_id)
{
	if (constraint_id == 0 || constraint_id > world->constraint_registry.size())
		return;
	Ref<Constraint> &entry = world->constraint_registry[constraint_id - 1];
	if (entry == nullptr)
		return;
	if (entry->GetSubType() == EConstraintSubType::Vehicle)
		world->physics_system->RemoveStepListener(static_cast<VehicleConstraint *>(entry.GetPtr()));
	world->physics_system->RemoveConstraint(entry);
	entry = nullptr;
}


// Velocity motor on a slider/hinge constraint. Spring + limits were stored
// at creation from the Rust-side JointMotor; this only retargets speed.
// Returns false on invalid ids or wrong joint types.
bool bjolt_constraint_drive_at(BJoltWorld *world, uint32_t constraint_id, float target_velocity)
{
	if (constraint_id == 0 || constraint_id > world->constraint_registry.size())
		return false;
	Constraint *base = world->constraint_registry[constraint_id - 1];
	if (base == nullptr)
		return false;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	switch (base->GetSubType())
	{
	case EConstraintSubType::Slider:
	{
		SliderConstraint *slider = static_cast<SliderConstraint *>(base);
		slider->SetMotorState(EMotorState::Velocity);
		slider->SetTargetVelocity(target_velocity);
		break;
	}
	case EConstraintSubType::Hinge:
	{
		HingeConstraint *hinge = static_cast<HingeConstraint *>(base);
		hinge->SetMotorState(EMotorState::Velocity);
		hinge->SetTargetAngularVelocity(target_velocity);
		break;
	}
	case EConstraintSubType::Path:
	{
		PathConstraint *path = static_cast<PathConstraint *>(base);
		path->SetPositionMotorState(EMotorState::Velocity);
		path->SetTargetVelocity(target_velocity);
		if (path->GetMaxFrictionForce() <= 0.0f)
			path->SetMaxFrictionForce(1.0e9f);
		break;
	}
	case EConstraintSubType::SixDOF:
	{
		SixDOFConstraint *six_dof = static_cast<SixDOFConstraint *>(base);
		// Which axes carry a live motor was decided at creation: every axis
		// in Velocity state gets the new speed, others stay untouched.
		// Targets are absolute, not accumulated.
		Vec3 linear_velocity = six_dof->GetTargetVelocityCS();
		Vec3 angular_velocity = six_dof->GetTargetAngularVelocityCS();
		for (int axis = 0; axis < 6; ++axis)
		{
			if (six_dof->GetMotorState((SixDOFConstraintSettings::EAxis)axis) != EMotorState::Velocity)
				continue;
			if (axis == 0)
				linear_velocity.SetX(target_velocity);
			else if (axis == 1)
				linear_velocity.SetY(target_velocity);
			else if (axis == 2)
				linear_velocity.SetZ(target_velocity);
			else if (axis == 3)
				angular_velocity.SetX(target_velocity);
			else if (axis == 4)
				angular_velocity.SetY(target_velocity);
			else
				angular_velocity.SetZ(target_velocity);
		}
		six_dof->SetTargetVelocityCS(linear_velocity);
		six_dof->SetTargetAngularVelocityCS(angular_velocity);
		break;
	}
	default:
		return false;
	}
	// Retargeting alone doesn't wake a body the solver put to sleep at its
	// limit; kick both ends awake so the new direction actually starts.
	TwoBodyConstraint *paired = static_cast<TwoBodyConstraint *>(base);
	body_interface.ActivateBody(paired->GetBody1()->GetID());
	body_interface.ActivateBody(paired->GetBody2()->GetID());
	return true;
}

// Velocity motor on a swing-twist constraint. `axis` 0 = twist (spin about
// the constraint X axis), 1 = swing (sweep about constraint Y/Z); anything
// else stops both motors. Same convention as `bjolt_ragdoll_drive`.
// Returns false on invalid ids or non-swing-twist joints.
// NOTE: Jolt's twist is about constraint X (see `Quat::GetSwingTwist` and
// the powered test's `sTargetVelocityCS = (90deg, 0, 0)`): an earlier
// version drove twist on Y, which spun the motor against the wrong axis
// and froze every swing-twist cut at zero velocity.
bool bjolt_constraint_drive_swing_twist(BJoltWorld *world, uint32_t constraint_id, uint8_t axis, float target_velocity)
{
	if (constraint_id == 0 || constraint_id > world->constraint_registry.size())
		return false;
	Constraint *base = world->constraint_registry[constraint_id - 1];
	if (base == nullptr || base->GetSubType() != EConstraintSubType::SwingTwist)
		return false;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	SwingTwistConstraint *swing_twist = static_cast<SwingTwistConstraint *>(base);
	// Twist spins about constraint X; swing sweeps about constraint Y/Z.
	// Constraint-space directions (body 2's constraint frame).
	Vec3 swing_target = Vec3::sZero();
	Vec3 twist_target = Vec3::sZero();
	if (axis == 0)
	{
		swing_twist->SetSwingMotorState(EMotorState::Off);
		swing_twist->SetTwistMotorState(EMotorState::Velocity);
		twist_target.SetX(target_velocity);
	}
	else if (axis == 1)
	{
		swing_twist->SetSwingMotorState(EMotorState::Velocity);
		swing_twist->SetTwistMotorState(EMotorState::Off);
		swing_target.SetY(target_velocity);
	}
	else
	{
		swing_twist->SetSwingMotorState(EMotorState::Off);
		swing_twist->SetTwistMotorState(EMotorState::Off);
		return true;
	}
	swing_twist->SetTargetAngularVelocityCS(swing_target + twist_target);
	TwoBodyConstraint *paired = static_cast<TwoBodyConstraint *>(base);
	body_interface.ActivateBody(paired->GetBody1()->GetID());
	body_interface.ActivateBody(paired->GetBody2()->GetID());
	return true;
}

// Debug: current path fraction of a path constraint. NaN on invalid ids
// or non-path joints.
float bjolt_constraint_path_fraction(BJoltWorld *world, uint32_t constraint_id)
{
	if (constraint_id == 0 || constraint_id > world->constraint_registry.size())
		return nanf("");
	Constraint *base = world->constraint_registry[constraint_id - 1];
	if (base == nullptr || base->GetSubType() != EConstraintSubType::Path)
		return nanf("");
	return static_cast<PathConstraint *>(base)->GetPathFraction();
}

// Debug: whether a path constraint's spline is looping. -1 on bad ids.
int bjolt_constraint_path_looping(BJoltWorld *world, uint32_t constraint_id)
{
	if (constraint_id == 0 || constraint_id > world->constraint_registry.size())
		return -1;
	Constraint *base = world->constraint_registry[constraint_id - 1];
	if (base == nullptr || base->GetSubType() != EConstraintSubType::Path)
		return -1;
	return static_cast<PathConstraint *>(base)->GetPath()->IsLooping() ? 1 : 0;
}
// Fully general six-DOF joint: per-axis limits (min/max per axis, Jolt
// conventions: min > max fixes the axis, +-FLT_MAX frees it) plus an
// Returns 0 on failure.
uint32_t bjolt_create_six_dof(BJoltWorld *world, uint32_t body1_raw, uint32_t body2_raw,
	float position1_x, float position1_y, float position1_z,
	float axis_x1_x, float axis_x1_y, float axis_x1_z,
	float axis_y1_x, float axis_y1_y, float axis_y1_z,
	float position2_x, float position2_y, float position2_z,
	float axis_x2_x, float axis_x2_y, float axis_x2_z,
	float axis_y2_x, float axis_y2_y, float axis_y2_z,
	float limit_min_tx, float limit_max_tx,
	float limit_min_ty, float limit_max_ty,
	float limit_min_tz, float limit_max_tz,
	float limit_min_rx, float limit_max_rx,
	float limit_min_ry, float limit_max_ry,
	float limit_min_rz, float limit_max_rz,
	uint8_t motor_axes,
	float motor_frequency, float motor_damping, float motor_force_limit,
	uint8_t joint_space)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	SixDOFConstraintSettings settings;
	settings.mSpace = bjolt_decode_joint_space(joint_space);
	settings.mPosition1 = RVec3(position1_x, position1_y, position1_z);
	settings.mAxisX1 = Vec3(axis_x1_x, axis_x1_y, axis_x1_z);
	settings.mAxisY1 = Vec3(axis_y1_x, axis_y1_y, axis_y1_z);
	settings.mPosition2 = RVec3(position2_x, position2_y, position2_z);
	settings.mAxisX2 = Vec3(axis_x2_x, axis_x2_y, axis_x2_z);
	settings.mAxisY2 = Vec3(axis_y2_x, axis_y2_y, axis_y2_z);
	float limit_mins[6] = { limit_min_tx, limit_min_ty, limit_min_tz, limit_min_rx, limit_min_ry, limit_min_rz };
	float limit_maxs[6] = { limit_max_tx, limit_max_ty, limit_max_tz, limit_max_rx, limit_max_ry, limit_max_rz };
	for (int axis = 0; axis < 6; ++axis)
	{
		settings.SetLimitedAxis((SixDOFConstraintSettings::EAxis)axis, limit_mins[axis], limit_maxs[axis]);
		if (motor_axes & (1 << axis))
		{
			settings.mMotorSettings[axis] = MotorSettings(ESpringMode::FrequencyAndDamping, motor_frequency, motor_damping);
			settings.mMotorSettings[axis].SetForceLimit(motor_force_limit);
		}
	}
	TwoBodyConstraint *constraint = body_interface.CreateConstraint(&settings, BodyID(body1_raw), BodyID(body2_raw));
	if (constraint == nullptr)
		return 0;
	// Settings carry no motor state (starts Off): arm each driven axis on
	// the live constraint so the drive retarget engages from tick one.
	SixDOFConstraint *six_dof = static_cast<SixDOFConstraint *>(constraint);
	for (int axis = 0; axis < 6; ++axis)
	{
		if (motor_axes & (1 << axis))
			six_dof->SetMotorState((SixDOFConstraintSettings::EAxis)axis, EMotorState::Velocity);
	}
	return bjolt_register_constraint(world, constraint);
}

uint32_t bjolt_create_pulley_constraint(BJoltWorld *world, uint32_t body1_raw, uint32_t body2_raw,
	float body_point1_x, float body_point1_y, float body_point1_z,
	float fixed_point1_x, float fixed_point1_y, float fixed_point1_z,
	float body_point2_x, float body_point2_y, float body_point2_z,
	float fixed_point2_x, float fixed_point2_y, float fixed_point2_z,
	float ratio, float min_length, float max_length, uint8_t joint_space)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	PulleyConstraintSettings settings;
	settings.mSpace = bjolt_decode_joint_space(joint_space);
	settings.mBodyPoint1 = RVec3(body_point1_x, body_point1_y, body_point1_z);
	settings.mFixedPoint1 = RVec3(fixed_point1_x, fixed_point1_y, fixed_point1_z);
	settings.mBodyPoint2 = RVec3(body_point2_x, body_point2_y, body_point2_z);
	settings.mFixedPoint2 = RVec3(fixed_point2_x, fixed_point2_y, fixed_point2_z);
	settings.mRatio = ratio;
	settings.mMinLength = min_length;
	settings.mMaxLength = max_length;
	TwoBodyConstraint *constraint = body_interface.CreateConstraint(&settings, BodyID(body1_raw), BodyID(body2_raw));
	if (constraint == nullptr)
		return 0;
	return bjolt_register_constraint(world, constraint);
}

// Couples two hinged bodies through a gear ratio. Both bodies must already be
// pinned by hinge constraints (id1/id2); the gear then links their rotations.
// Returns 0 on failure or when the hinge ids are invalid.
uint32_t bjolt_create_gear_constraint(BJoltWorld *world, uint32_t body1_raw, uint32_t body2_raw,
	float hinge_axis_x, float hinge_axis_y, float hinge_axis_z,
	float ratio, uint32_t hinge_id1, uint32_t hinge_id2, uint8_t joint_space)
{
	if (hinge_id1 == 0 || hinge_id1 > world->constraint_registry.size()
		|| hinge_id2 == 0 || hinge_id2 > world->constraint_registry.size())
		return 0;
	Constraint *hinge1 = world->constraint_registry[hinge_id1 - 1];
	Constraint *hinge2 = world->constraint_registry[hinge_id2 - 1];
	if (hinge1 == nullptr || hinge2 == nullptr)
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	GearConstraintSettings settings;
	settings.mSpace = bjolt_decode_joint_space(joint_space);
	settings.mHingeAxis1 = Vec3(hinge_axis_x, hinge_axis_y, hinge_axis_z);
	settings.mHingeAxis2 = Vec3(hinge_axis_x, hinge_axis_y, hinge_axis_z);
	settings.mRatio = ratio;
	GearConstraint *constraint = static_cast<GearConstraint *>(
		body_interface.CreateConstraint(&settings, BodyID(body1_raw), BodyID(body2_raw)));
	if (constraint == nullptr)
		return 0;
	constraint->SetConstraints(hinge1, hinge2);
	return bjolt_register_constraint(world, constraint);
}

// Links a spinning pinion to a sliding rack: pinion rotation drives rack
// travel. Pinion/rack hinge/slider constraints pass in as ids. Returns 0 on
// failure or invalid joint ids.
uint32_t bjolt_create_rack_pinion_constraint(BJoltWorld *world, uint32_t body1_raw, uint32_t body2_raw,
	float hinge_axis_x, float hinge_axis_y, float hinge_axis_z,
	float slider_axis_x, float slider_axis_y, float slider_axis_z,
	float ratio, uint32_t pinion_hinge_id, uint32_t rack_slider_id, uint8_t joint_space)
{
	if (pinion_hinge_id == 0 || pinion_hinge_id > world->constraint_registry.size()
		|| rack_slider_id == 0 || rack_slider_id > world->constraint_registry.size())
		return 0;
	Constraint *pinion = world->constraint_registry[pinion_hinge_id - 1];
	Constraint *rack = world->constraint_registry[rack_slider_id - 1];
	if (pinion == nullptr || rack == nullptr)
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	RackAndPinionConstraintSettings settings;
	settings.mSpace = bjolt_decode_joint_space(joint_space);
	settings.mHingeAxis = Vec3(hinge_axis_x, hinge_axis_y, hinge_axis_z);
	settings.mSliderAxis = Vec3(slider_axis_x, slider_axis_y, slider_axis_z);
	settings.mRatio = ratio;
	RackAndPinionConstraint *constraint = static_cast<RackAndPinionConstraint *>(
		body_interface.CreateConstraint(&settings, BodyID(body1_raw), BodyID(body2_raw)));
	if (constraint == nullptr)
		return 0;
	constraint->SetConstraints(pinion, rack);
	return bjolt_register_constraint(world, constraint);
}
// Caps for the vehicle/path FFI arrays: 8 wheels, 4 differentials, 2
// anti-roll bars, 8+8 gears, 8 knots per friction/torque curve, 16 Hermite
// points per path. Small stack arrays only.
static constexpr int BJOLT_MAX_VEHICLE_WHEELS = 8;
static constexpr int BJOLT_MAX_VEHICLE_DIFFS = 4;
static constexpr int BJOLT_MAX_VEHICLE_ROLLBARS = 2;
static constexpr int BJOLT_MAX_VEHICLE_GEARS = 8;
static constexpr int BJOLT_MAX_CURVE_POINTS = 8;
static constexpr int BJOLT_MAX_PATH_POINTS = 16;

// One Hermite spline knot: position plus tangent (arrival direction scaled
// by segment weight) and normal (track up). Jolt interpolates position,
// tangent, and normal between knots.
struct BJoltPathPoint {
	float pos_x, pos_y, pos_z;
	float tan_x, tan_y, tan_z;
	float nrm_x, nrm_y, nrm_z;
};
// Cart on a general Hermite spline: N knots through `points`, path-local
// (relative to the static body's frame, which starts at the world origin).
// `looping` closes the spline. `rotation_mode` maps to Jolt's
// `EPathRotationConstraintType` (0 free .. 5 fully constrained).
// Returns 0 on failure.
uint32_t bjolt_create_path_cart(BJoltWorld *world, uint32_t static_body_raw, uint32_t cart_body_raw,
	const BJoltPathPoint *points, uint8_t point_count, uint8_t looping,
	float motor_frequency, float motor_damping, float motor_force_limit, uint8_t rotation_mode)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	Vec3 anchor = body_interface.GetPosition(BodyID(static_body_raw));
	int knots = point_count < BJOLT_MAX_PATH_POINTS ? point_count : BJOLT_MAX_PATH_POINTS;
	if (knots < 2)
		return 0;
	Ref<PathConstraintPathHermite> path = new PathConstraintPathHermite();
	for (int i = 0; i < knots; ++i)
		path->AddPoint(
			Vec3(points[i].pos_x, points[i].pos_y, points[i].pos_z) - anchor,
			Vec3(points[i].tan_x, points[i].tan_y, points[i].tan_z),
			Vec3(points[i].nrm_x, points[i].nrm_y, points[i].nrm_z));
	if (looping)
		path->SetIsLooping(true);
	PathConstraintSettings settings;
	settings.mPath = path;
	settings.mPositionMotorSettings = MotorSettings(ESpringMode::FrequencyAndDamping, motor_frequency, motor_damping);
	settings.mPositionMotorSettings.SetForceLimit(motor_force_limit);
	settings.mMaxFrictionForce = motor_force_limit;
	settings.mRotationConstraintType = (EPathRotationConstraintType)(rotation_mode > 5 ? 0 : rotation_mode);
	settings.mPathPosition = anchor - body_interface.GetCenterOfMassTransform(BodyID(static_body_raw)).GetTranslation();
	// Start the cart at the fraction nearest its spawn pose. The default
	// fraction 0 yanks a cart spawned mid-track toward the start; the motor
	// then drags it off the spline instead of locking on. Path points are
	// anchor-relative, so the path-local cart position is the cart offset
	// from the anchor, unrotated by the anchor.
	{
		Vec3 cart_world = body_interface.GetCenterOfMassPosition(BodyID(cart_body_raw));
		Quat anchor_rotation = body_interface.GetRotation(BodyID(static_body_raw));
		Vec3 cart_local = anchor_rotation.Inversed() * (cart_world - anchor);
		settings.mPathFraction = path->GetClosestPoint(cart_local, 0.0f);
	}
	TwoBodyConstraint *constraint = body_interface.CreateConstraint(
		&settings, BodyID(static_body_raw), BodyID(cart_body_raw));
	if (constraint == nullptr)
		return 0;
	return bjolt_register_constraint(world, constraint);
}
// Full vehicle authoring: plain-data config structs cross the FFI (arrays
// would need lengths per call), one creator per controller kind.

// One (x, y) knot for a Jolt LinearCurve: Reserve + AddPoint per entry.
struct BJoltCurvePoint {
	float knot_x, knot_y;
};
// One wheel: base mount + suspension + tire. kind: 0 = wheeled (WV),
// 1 = tracked (TV). Friction curves cross as knot arrays; empty
// (count 0) keeps Jolt's tire-profile defaults.
struct BJoltWheelConfig {
	float pos_x, pos_y, pos_z;
	float susp_force_x, susp_force_y, susp_force_z;
	float susp_min_length, susp_max_length, susp_preload_length;
	float susp_frequency, susp_damping;
	float wheel_radius, wheel_width;
	float max_steer_angle_rad;
	float wheel_inertia, wheel_damping;
	float max_brake_torque, max_hand_brake_torque;
	// Tracked wheels only: plain friction pair instead of slip curves.
	float track_longitudinal_friction, track_lateral_friction;
	uint8_t kind;
	uint8_t long_curve_count, lat_curve_count;
	BJoltCurvePoint long_curve[BJOLT_MAX_CURVE_POINTS];
	BJoltCurvePoint lat_curve[BJOLT_MAX_CURVE_POINTS];
};

struct BJoltEngineConfig {
	float max_torque, min_rpm, max_rpm;
	float engine_inertia, engine_damping;
	// Normalized torque curve (RPM fraction -> torque ratio). Empty
	// keeps Jolt's 0.8 / 1.0 / 0.8 default.
	uint8_t torque_curve_count;
	BJoltCurvePoint torque_curve[BJOLT_MAX_CURVE_POINTS];
};

struct BJoltTransmissionConfig {
	uint8_t auto_mode; // 0 = manual, 1 = auto
	uint8_t gear_count; // forward gears used from gear_ratios
	uint8_t reverse_gear_count;
	float gear_ratios[BJOLT_MAX_VEHICLE_GEARS];
	float reverse_gear_ratios[BJOLT_MAX_VEHICLE_GEARS];
	float switch_time, clutch_release_time, switch_latency;
	float shift_up_rpm, shift_down_rpm, clutch_strength;
};

struct BJoltDifferentialConfig {
	int32_t left_wheel, right_wheel;
	float differential_ratio, left_right_split, limited_slip_ratio, engine_torque_ratio;
};

struct BJoltRollBarConfig {
	int32_t left_wheel, right_wheel;
	float stiffness;
};

// One track side (tank): which wheels belong, which one drives.
struct BJoltTrackConfig {
	uint8_t wheel_count;
	uint8_t wheel_indices[BJOLT_MAX_VEHICLE_WHEELS];
	uint8_t driven_wheel; // index into wheel_indices
	float track_inertia, track_damping, max_brake_torque, differential_ratio;
};

// Lean spring (motorcycle only): balance PID + smoothing.
struct BJoltLeanConfig {
	float max_lean_angle_rad;
	float lean_spring_constant, lean_spring_damping;
	float lean_integration_coefficient, lean_integration_decay, lean_smoothing;
};

// Shared chassis spawn: box body with center-of-mass offset, owned by the
static Body *bjolt_spawn_vehicle_chassis(BodyInterface &body_interface,
	uint16_t object_layer, float pos_x, float pos_y, float pos_z,
	float half_x, float half_y, float half_z,
	float com_off_x, float com_off_y, float com_off_z, float mass_kg)
{
	RefConst<Shape> chassis_shape = OffsetCenterOfMassShapeSettings(
		Vec3(com_off_x, com_off_y, com_off_z),
		new BoxShape(Vec3(half_x, half_y, half_z))).Create().Get();
	if (chassis_shape == nullptr)
		return nullptr;
	BodyCreationSettings chassis_settings(chassis_shape, RVec3(pos_x, pos_y, pos_z),
		Quat::sIdentity(), EMotionType::Dynamic, object_layer);
	chassis_settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
	chassis_settings.mMassPropertiesOverride.mMass = mass_kg;
	Body *chassis = body_interface.CreateBody(chassis_settings);
	if (chassis == nullptr)
		return nullptr;
	body_interface.AddBody(chassis->GetID(), EActivation::Activate);
	return chassis;
}

// Fill a LinearCurve from FFI knots, replacing whatever was there.
static void bjolt_fill_curve(LinearCurve &curve, const BJoltCurvePoint *knots, int knot_count)
{
	curve.Clear();
	curve.Reserve(knot_count);
	for (int i = 0; i < knot_count; ++i)
		curve.AddPoint(knots[i].knot_x, knots[i].knot_y);
}

// One wheel from FFI config: wheeled (slip-curve tires) or tracked (plain
// friction pair). Caller pushes the returned ref into vehicle.mWheels.
static Ref<WheelSettings> bjolt_make_wheel(const BJoltWheelConfig &cfg)
{
	if (cfg.kind == 1)
	{
		Ref<WheelSettingsTV> wheel = new WheelSettingsTV;
		wheel->mPosition = Vec3(cfg.pos_x, cfg.pos_y, cfg.pos_z);
		wheel->mSuspensionMinLength = cfg.susp_min_length;
		wheel->mSuspensionMaxLength = cfg.susp_max_length;
		wheel->mSuspensionPreloadLength = cfg.susp_preload_length;
		wheel->mSuspensionSpring = SpringSettings(ESpringMode::FrequencyAndDamping, cfg.susp_frequency, cfg.susp_damping);
		wheel->mRadius = cfg.wheel_radius;
		wheel->mWidth = cfg.wheel_width;
		wheel->mLongitudinalFriction = cfg.track_longitudinal_friction;
		wheel->mLateralFriction = cfg.track_lateral_friction;
		return wheel;
	}
	Ref<WheelSettingsWV> wheel = new WheelSettingsWV;
	wheel->mPosition = Vec3(cfg.pos_x, cfg.pos_y, cfg.pos_z);
	wheel->mSuspensionForcePoint = Vec3(cfg.susp_force_x, cfg.susp_force_y, cfg.susp_force_z);
	wheel->mSuspensionMinLength = cfg.susp_min_length;
	wheel->mSuspensionMaxLength = cfg.susp_max_length;
	wheel->mSuspensionPreloadLength = cfg.susp_preload_length;
	wheel->mSuspensionSpring = SpringSettings(ESpringMode::FrequencyAndDamping, cfg.susp_frequency, cfg.susp_damping);
	wheel->mRadius = cfg.wheel_radius;
	wheel->mWidth = cfg.wheel_width;
	wheel->mMaxSteerAngle = cfg.max_steer_angle_rad;
	wheel->mInertia = cfg.wheel_inertia;
	wheel->mAngularDamping = cfg.wheel_damping;
	wheel->mMaxBrakeTorque = cfg.max_brake_torque;
	wheel->mMaxHandBrakeTorque = cfg.max_hand_brake_torque;
	// Non-empty curves replace Jolt's tire-profile defaults; empty keeps them.
	if (cfg.long_curve_count > 0)
		bjolt_fill_curve(wheel->mLongitudinalFriction, cfg.long_curve, cfg.long_curve_count);
	if (cfg.lat_curve_count > 0)
		bjolt_fill_curve(wheel->mLateralFriction, cfg.lat_curve, cfg.lat_curve_count);
	return wheel;
}

static void bjolt_fill_engine(VehicleEngineSettings &engine, const BJoltEngineConfig &cfg)
{
	engine.mMaxTorque = cfg.max_torque;
	engine.mMinRPM = cfg.min_rpm;
	engine.mMaxRPM = cfg.max_rpm;
	engine.mInertia = cfg.engine_inertia;
	engine.mAngularDamping = cfg.engine_damping;
	if (cfg.torque_curve_count > 0)
		bjolt_fill_curve(engine.mNormalizedTorque, cfg.torque_curve, cfg.torque_curve_count);
}

static void bjolt_fill_transmission(VehicleTransmissionSettings &gearbox, const BJoltTransmissionConfig &cfg)
{
	gearbox.mMode = cfg.auto_mode ? ETransmissionMode::Auto : ETransmissionMode::Manual;
	gearbox.mGearRatios.clear();
	for (int i = 0; i < cfg.gear_count && i < BJOLT_MAX_VEHICLE_GEARS; ++i)
		gearbox.mGearRatios.push_back(cfg.gear_ratios[i]);
	gearbox.mReverseGearRatios.clear();
	for (int i = 0; i < cfg.reverse_gear_count && i < BJOLT_MAX_VEHICLE_GEARS; ++i)
		gearbox.mReverseGearRatios.push_back(cfg.reverse_gear_ratios[i]);
	gearbox.mSwitchTime = cfg.switch_time;
	gearbox.mClutchReleaseTime = cfg.clutch_release_time;
	gearbox.mSwitchLatency = cfg.switch_latency;
	gearbox.mShiftUpRPM = cfg.shift_up_rpm;
	gearbox.mShiftDownRPM = cfg.shift_down_rpm;
	gearbox.mClutchStrength = cfg.clutch_strength;
}

static void bjolt_fill_differentials(VehicleDifferentialSettings *diffs, int diff_count,
	const BJoltDifferentialConfig *configs)
{
	for (int i = 0; i < diff_count; ++i)
	{
		diffs[i].mLeftWheel = configs[i].left_wheel;
		diffs[i].mRightWheel = configs[i].right_wheel;
		diffs[i].mDifferentialRatio = configs[i].differential_ratio;
		diffs[i].mLeftRightSplit = configs[i].left_right_split;
		diffs[i].mLimitedSlipRatio = configs[i].limited_slip_ratio;
		diffs[i].mEngineTorqueRatio = configs[i].engine_torque_ratio;
	}
}

// Shared tail: collision tester, step listener, registry. Tester radius
// follows the first wheel's width; all wheels share one width in practice.
static uint32_t bjolt_finish_vehicle(BJoltWorld *world, Body *chassis,
	VehicleConstraintSettings &vehicle, uint16_t object_layer, float tester_radius,
	uint32_t *out_body_raw, uint32_t *out_constraint_id)
{
	VehicleConstraint *vehicle_constraint = new VehicleConstraint(*chassis, vehicle);
	vehicle_constraint->SetVehicleCollisionTester(
		new VehicleCollisionTesterCastSphere(object_layer, tester_radius));
	world->physics_system->AddConstraint(vehicle_constraint);
	world->physics_system->AddStepListener(vehicle_constraint);
	uint32_t constraint_id = bjolt_register_constraint(world, vehicle_constraint);
	*out_body_raw = chassis->GetID().GetIndexAndSequenceNumber();
	*out_constraint_id = constraint_id;
	return constraint_id;
}

// Fully specified wheeled vehicle: chassis box + N wheels + engine +
// transmission + differentials + anti-roll bars. Body frame X right, Y up,
// Z forward (matches Jolt's vehicle sample). Returns 0 on failure; the two
// out params stay untouched on failure.
uint32_t bjolt_create_wheeled_vehicle(BJoltWorld *world, uint16_t object_layer,
	float pos_x, float pos_y, float pos_z,
	float half_x, float half_y, float half_z,
	float com_off_x, float com_off_y, float com_off_z, float mass_kg,
	float max_pitch_roll_angle_rad, float tester_radius,
	const BJoltWheelConfig *wheels, uint8_t wheel_count,
	const BJoltEngineConfig *engine, const BJoltTransmissionConfig *gearbox,
	const BJoltDifferentialConfig *diffs, uint8_t diff_count,
	const BJoltRollBarConfig *roll_bars, uint8_t roll_bar_count,
	float limited_slip_ratio,
	uint32_t *out_body_raw, uint32_t *out_constraint_id)
{
	if (!bjolt_bodies_available(world))
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	Body *chassis = bjolt_spawn_vehicle_chassis(body_interface, object_layer,
		pos_x, pos_y, pos_z, half_x, half_y, half_z,
		com_off_x, com_off_y, com_off_z, mass_kg);
	if (chassis == nullptr)
		return 0;
	VehicleConstraintSettings vehicle;
	vehicle.mUp = Vec3::sAxisY();
	vehicle.mForward = Vec3::sAxisZ();
	vehicle.mMaxPitchRollAngle = max_pitch_roll_angle_rad;
	int wheels_to_add = wheel_count < BJOLT_MAX_VEHICLE_WHEELS ? wheel_count : BJOLT_MAX_VEHICLE_WHEELS;
	for (int i = 0; i < wheels_to_add; ++i)
		vehicle.mWheels.push_back(bjolt_make_wheel(wheels[i]));
	WheeledVehicleControllerSettings *controller = new WheeledVehicleControllerSettings;
	bjolt_fill_engine(controller->mEngine, *engine);
	bjolt_fill_transmission(controller->mTransmission, *gearbox);
	int diffs_to_add = diff_count < BJOLT_MAX_VEHICLE_DIFFS ? diff_count : BJOLT_MAX_VEHICLE_DIFFS;
	controller->mDifferentials.resize(diffs_to_add);
	bjolt_fill_differentials(controller->mDifferentials.data(), diffs_to_add, diffs);
	controller->mDifferentialLimitedSlipRatio = limited_slip_ratio;
	vehicle.mController = controller;
	int bars_to_add = roll_bar_count < BJOLT_MAX_VEHICLE_ROLLBARS ? roll_bar_count : BJOLT_MAX_VEHICLE_ROLLBARS;
	for (int i = 0; i < bars_to_add; ++i)
	{
		VehicleAntiRollBar bar;
		bar.mLeftWheel = roll_bars[i].left_wheel;
		bar.mRightWheel = roll_bars[i].right_wheel;
		bar.mStiffness = roll_bars[i].stiffness;
		vehicle.mAntiRollBars.push_back(bar);
	}
	return bjolt_finish_vehicle(world, chassis, vehicle, object_layer,
		tester_radius, out_body_raw, out_constraint_id);
}

// Tank: tracked wheels grouped into left/right sides, each with its driven
// wheel. Same chassis deal as wheeled. Returns 0 on failure.
uint32_t bjolt_create_tracked_vehicle(BJoltWorld *world, uint16_t object_layer,
	float pos_x, float pos_y, float pos_z,
	float half_x, float half_y, float half_z,
	float com_off_x, float com_off_y, float com_off_z, float mass_kg,
	float max_pitch_roll_angle_rad, float tester_radius,
	const BJoltWheelConfig *wheels, uint8_t wheel_count,
	const BJoltEngineConfig *engine, const BJoltTransmissionConfig *gearbox,
	const BJoltTrackConfig *tracks,
	uint32_t *out_body_raw, uint32_t *out_constraint_id)
{
	if (!bjolt_bodies_available(world))
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	Body *chassis = bjolt_spawn_vehicle_chassis(body_interface, object_layer,
		pos_x, pos_y, pos_z, half_x, half_y, half_z,
		com_off_x, com_off_y, com_off_z, mass_kg);
	if (chassis == nullptr)
		return 0;
	VehicleConstraintSettings vehicle;
	vehicle.mUp = Vec3::sAxisY();
	vehicle.mForward = Vec3::sAxisZ();
	vehicle.mMaxPitchRollAngle = max_pitch_roll_angle_rad;
	int wheels_to_add = wheel_count < BJOLT_MAX_VEHICLE_WHEELS ? wheel_count : BJOLT_MAX_VEHICLE_WHEELS;
	for (int i = 0; i < wheels_to_add; ++i)
		vehicle.mWheels.push_back(bjolt_make_wheel(wheels[i]));
	TrackedVehicleControllerSettings *controller = new TrackedVehicleControllerSettings;
	bjolt_fill_engine(controller->mEngine, *engine);
	bjolt_fill_transmission(controller->mTransmission, *gearbox);
	for (int side = 0; side < 2; ++side)
	{
		// Jolt wants the driven wheel as a global wheel index: resolve the
		// per-track offset through this side's member list.
		uint8_t driven_offset = tracks[side].driven_wheel < tracks[side].wheel_count
			? tracks[side].driven_wheel : 0;
		controller->mTracks[side].mDrivenWheel = tracks[side].wheel_indices[driven_offset];
		controller->mTracks[side].mWheels.clear();
		for (int i = 0; i < tracks[side].wheel_count && i < BJOLT_MAX_VEHICLE_WHEELS; ++i)
			controller->mTracks[side].mWheels.push_back(tracks[side].wheel_indices[i]);
		controller->mTracks[side].mInertia = tracks[side].track_inertia;
		controller->mTracks[side].mAngularDamping = tracks[side].track_damping;
		controller->mTracks[side].mMaxBrakeTorque = tracks[side].max_brake_torque;
		controller->mTracks[side].mDifferentialRatio = tracks[side].differential_ratio;
	}
	vehicle.mController = controller;
	return bjolt_finish_vehicle(world, chassis, vehicle, object_layer,
		tester_radius, out_body_raw, out_constraint_id);
}

// Motorcycle: wheeled pair plus the lean-spring balance controller. Same
// chassis deal. Returns 0 on failure.
uint32_t bjolt_create_motorcycle(BJoltWorld *world, uint16_t object_layer,
	float pos_x, float pos_y, float pos_z,
	float half_x, float half_y, float half_z,
	float com_off_x, float com_off_y, float com_off_z, float mass_kg,
	float max_pitch_roll_angle_rad, float tester_radius,
	const BJoltWheelConfig *wheels, uint8_t wheel_count,
	const BJoltEngineConfig *engine, const BJoltTransmissionConfig *gearbox,
	const BJoltDifferentialConfig *diffs, uint8_t diff_count,
	const BJoltLeanConfig *lean,
	uint32_t *out_body_raw, uint32_t *out_constraint_id)
{
	if (!bjolt_bodies_available(world))
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	Body *chassis = bjolt_spawn_vehicle_chassis(body_interface, object_layer,
		pos_x, pos_y, pos_z, half_x, half_y, half_z,
		com_off_x, com_off_y, com_off_z, mass_kg);
	if (chassis == nullptr)
		return 0;
	VehicleConstraintSettings vehicle;
	vehicle.mUp = Vec3::sAxisY();
	vehicle.mForward = Vec3::sAxisZ();
	vehicle.mMaxPitchRollAngle = max_pitch_roll_angle_rad;
	int wheels_to_add = wheel_count < BJOLT_MAX_VEHICLE_WHEELS ? wheel_count : BJOLT_MAX_VEHICLE_WHEELS;
	for (int i = 0; i < wheels_to_add; ++i)
		vehicle.mWheels.push_back(bjolt_make_wheel(wheels[i]));
	MotorcycleControllerSettings *controller = new MotorcycleControllerSettings;
	bjolt_fill_engine(controller->mEngine, *engine);
	bjolt_fill_transmission(controller->mTransmission, *gearbox);
	int diffs_to_add = diff_count < BJOLT_MAX_VEHICLE_DIFFS ? diff_count : BJOLT_MAX_VEHICLE_DIFFS;
	controller->mDifferentials.resize(diffs_to_add);
	bjolt_fill_differentials(controller->mDifferentials.data(), diffs_to_add, diffs);
	controller->mMaxLeanAngle = lean->max_lean_angle_rad;
	controller->mLeanSpringConstant = lean->lean_spring_constant;
	controller->mLeanSpringDamping = lean->lean_spring_damping;
	controller->mLeanSpringIntegrationCoefficient = lean->lean_integration_coefficient;
	controller->mLeanSpringIntegrationCoefficientDecay = lean->lean_integration_decay;
	controller->mLeanSmoothingFactor = lean->lean_smoothing;
	vehicle.mController = controller;
	return bjolt_finish_vehicle(world, chassis, vehicle, object_layer,
		tester_radius, out_body_raw, out_constraint_id);
}

static VehicleConstraint *bjolt_vehicle_at(BJoltWorld *world, uint32_t constraint_id)
{
	if (constraint_id == 0 || constraint_id > world->constraint_registry.size())
		return nullptr;
	Constraint *base = world->constraint_registry[constraint_id - 1];
	if (base == nullptr || base->GetSubType() != EConstraintSubType::Vehicle)
		return nullptr;
	return static_cast<VehicleConstraint *>(base);
}

// Controller kind via the first wheel's settings type: Jolt RTTI lives on
// the settings (SerializableObject), not on VehicleController. Tracked
// wheels (TV) mean a TrackedVehicleController; wheeled wheels (WV) mean a
// WheeledVehicleController, which also covers MotorcycleController.
static bool bjolt_vehicle_is_tracked(VehicleConstraint *vehicle_constraint)
{
	if (vehicle_constraint->GetWheels().empty())
		return false;
	return IsKindOf(vehicle_constraint->GetWheels()[0]->GetSettings(), JPH_RTTI(WheelSettingsTV));
}

// Wheeled + motorcycle (same controller interface): gas, steer, foot brake,
// hand brake. No-op on bad ids or wrong controller kind.
void bjolt_vehicle_drive(BJoltWorld *world, uint32_t constraint_id,
	float forward, float right, float brake, float hand_brake)
{
	VehicleConstraint *vehicle_constraint = bjolt_vehicle_at(world, constraint_id);
	if (vehicle_constraint == nullptr || bjolt_vehicle_is_tracked(vehicle_constraint))
		return;
	WheeledVehicleController *wheeled =
		static_cast<WheeledVehicleController *>(vehicle_constraint->GetController());
	wheeled->SetDriverInput(forward, right, brake, hand_brake);
	// `SetDriverInput` only stores floats: a parked chassis asleep on the
	// ground never wakes, so late input (key press after settling) is
	// silently ignored. Wake on any live input; zero input may sleep.
	if (forward != 0.0f || right != 0.0f || brake != 0.0f || hand_brake != 0.0f)
		world->physics_system->GetBodyInterface().ActivateBody(vehicle_constraint->GetVehicleBody()->GetID());
}
// Tank: gas plus per-track multipliers (1 = full, -1 = reversed). Brake is
// the foot brake on both tracks. No-op on bad ids or wrong kind.
void bjolt_tracked_drive(BJoltWorld *world, uint32_t constraint_id,
	float forward, float left_ratio, float right_ratio, float brake)
{
	VehicleConstraint *vehicle_constraint = bjolt_vehicle_at(world, constraint_id);
	if (vehicle_constraint == nullptr || !bjolt_vehicle_is_tracked(vehicle_constraint))
		return;
	TrackedVehicleController *tracked =
		static_cast<TrackedVehicleController *>(vehicle_constraint->GetController());
	tracked->SetDriverInput(forward, left_ratio, right_ratio, brake);
	// Same wake-on-input as wheeled: a sleeping hull ignores stored input.
	if (forward != 0.0f || left_ratio != 1.0f || right_ratio != 1.0f || brake != 0.0f)
		world->physics_system->GetBodyInterface().ActivateBody(vehicle_constraint->GetVehicleBody()->GetID());
}

// Manual gearbox: gear (-1 reverse, 0 neutral, 1+ forward) plus clutch
// friction 0..1. Auto boxes ignore this. Wheeled covers motorcycle (same
// controller base). No-op on bad ids.
void bjolt_vehicle_shift(BJoltWorld *world, uint32_t constraint_id,
	int32_t gear, float clutch_friction)
{
	VehicleConstraint *vehicle_constraint = bjolt_vehicle_at(world, constraint_id);
	if (vehicle_constraint == nullptr)
		return;
	VehicleController *controller = vehicle_constraint->GetController();
	if (bjolt_vehicle_is_tracked(vehicle_constraint))
		static_cast<TrackedVehicleController *>(controller)->GetTransmission().Set(gear, clutch_friction);
	else
		static_cast<WheeledVehicleController *>(controller)->GetTransmission().Set(gear, clutch_friction);
}

// One-shot linear + angular impulse at center of mass. Zero halves are
// skipped so callers pass one Vec3 pair for combined kicks. No-op on bad ids.
void bjolt_apply_impulse(BJoltWorld *world, uint32_t body_raw,
	float lin_x, float lin_y, float lin_z,
	float ang_x, float ang_y, float ang_z)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	Vec3 linear_impulse(lin_x, lin_y, lin_z);
	Vec3 angular_impulse(ang_x, ang_y, ang_z);
	if (!linear_impulse.IsNearZero())
		body_interface.AddImpulse(body_id, linear_impulse);
	if (!angular_impulse.IsNearZero())
		body_interface.AddAngularImpulse(body_id, angular_impulse);
}

// Persistent force + torque, applied every tick while the component is
// present. Jolt clears accumulated forces each step, so this re-adds them.
// Zero halves skipped. No-op on bad ids.
void bjolt_apply_force(BJoltWorld *world, uint32_t body_raw,
	float force_x, float force_y, float force_z,
	float torque_x, float torque_y, float torque_z)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	Vec3 push_force(force_x, force_y, force_z);
	Vec3 push_torque(torque_x, torque_y, torque_z);
	if (!push_force.IsNearZero())
		body_interface.AddForce(body_id, push_force);
	if (!push_torque.IsNearZero())
		body_interface.AddTorque(body_id, push_torque);
}

// Direct velocity set: overwrites rather than accumulates, so a zero half
// means "stop on this axis", not "leave it alone". No-op on bad ids.
void bjolt_set_velocity(BJoltWorld *world, uint32_t body_raw,
	float lin_x, float lin_y, float lin_z,
	float ang_x, float ang_y, float ang_z)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	body_interface.SetLinearVelocity(body_id, Vec3(lin_x, lin_y, lin_z));
	body_interface.SetAngularVelocity(body_id, Vec3(ang_x, ang_y, ang_z));
}

// Per-axis velocity overwrite: linear-only and angular-only variants so one
// component can drive movement without wiping out spin, and vice versa.
// No-op on bad ids.
void bjolt_set_linear_velocity(BJoltWorld *world, uint32_t body_raw,
	float lin_x, float lin_y, float lin_z)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	body_interface.SetLinearVelocity(body_id, Vec3(lin_x, lin_y, lin_z));
}

void bjolt_set_angular_velocity(BJoltWorld *world, uint32_t body_raw,
	float ang_x, float ang_y, float ang_z)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	body_interface.SetAngularVelocity(body_id, Vec3(ang_x, ang_y, ang_z));
}

// Applies the spawn rotation the wrapper drops at creation: creation bakes
// identity, so this rotates the live body into place before the first step.
// Activates the body so the new pose takes effect. No-op on invalid ids.
void bjolt_set_rotation(BJoltWorld *world, uint32_t body_raw,
	float rot_x, float rot_y, float rot_z, float rot_w)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	body_interface.SetRotation(body_id, Quat(rot_x, rot_y, rot_z, rot_w), EActivation::Activate);
}

// Surface grip + bounciness on a live body. Applied right after bake (same
// as the spawn-rotation fix-up) and changeable at runtime. No-op on bad ids.
void bjolt_set_friction(BJoltWorld *world, uint32_t body_raw, float friction)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	body_interface.SetFriction(BodyID(body_raw), friction);
}

void bjolt_set_restitution(BJoltWorld *world, uint32_t body_raw, float restitution)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	body_interface.SetRestitution(BodyID(body_raw), restitution);
}

// Continuous collision detection: LinearCast sweeps the shape from start
// to destination so fast bodies stop at the first hit instead of
// tunneling. Applied at bake for flagged bodies, toggleable at runtime.
// No-op on bad ids.
void bjolt_set_ccd(BJoltWorld *world, uint32_t body_raw, bool use_ccd)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	body_interface.SetMotionQuality(BodyID(body_raw),
		use_ccd ? EMotionQuality::LinearCast : EMotionQuality::Discrete);
}

// Buoyancy push for one tick: Jolt computes the submerged volume under a
// flat surface and shoves the body up, with water drag on the wet part.
// Call every Fixed tick before the step for each floating body. No-op on
// bad ids.
void bjolt_apply_buoyancy(BJoltWorld *world, uint32_t body_raw,
	float surface_y, float buoyancy, float linear_drag, float angular_drag,
	float fluid_vel_x, float fluid_vel_y, float fluid_vel_z,
	float gravity_x, float gravity_y, float gravity_z, float delta_time)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	RVec3 surface_position = body_interface.GetCenterOfMassPosition(body_id);
	surface_position.SetY(surface_y);
	body_interface.ApplyBuoyancyImpulse(body_id, surface_position, Vec3(0, 1, 0),
		buoyancy, linear_drag, angular_drag,
		Vec3(fluid_vel_x, fluid_vel_y, fluid_vel_z),
		Vec3(gravity_x, gravity_y, gravity_z), delta_time);
}


// Vertex count for mesh sizing. 0 on bad ids or non-soft bodies.
uint32_t bjolt_soft_vertex_count(BJoltWorld *world, uint32_t body_raw)
{
	BodyLockRead vertex_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	if (!vertex_lock.Succeeded() || !vertex_lock.GetBody().IsSoftBody())
		return 0;
	const SoftBodyMotionProperties *soft_props = static_cast<const SoftBodyMotionProperties *>(
		vertex_lock.GetBody().GetMotionProperties());
	return (uint32_t)soft_props->GetVertices().size();
}

// World-space vertex positions for mesh rebuild. Returns vertices written.
uint32_t bjolt_soft_vertices(BJoltWorld *world, uint32_t body_raw, float *out_positions, uint32_t capacity)
{
	BodyLockRead vertex_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	if (!vertex_lock.Succeeded() || !vertex_lock.GetBody().IsSoftBody())
		return 0;
	const Body &soft_body = vertex_lock.GetBody();
	const SoftBodyMotionProperties *soft_props = static_cast<const SoftBodyMotionProperties *>(
		soft_body.GetMotionProperties());
	RVec3 body_position = soft_body.GetCenterOfMassPosition();
	uint32_t written = 0;
	for (const SoftBodyMotionProperties::Vertex &soft_vertex : soft_props->GetVertices())
	{
		if (written >= capacity)
			break;
		RVec3 world_vertex = body_position + soft_vertex.mPosition;
		out_positions[written * 3] = world_vertex.GetX();
		out_positions[written * 3 + 1] = world_vertex.GetY();
		out_positions[written * 3 + 2] = world_vertex.GetZ();
		++written;
	}
	return written;
}

// Destroys a soft body. Never touches rigid bodies: order-independent.
void bjolt_soft_destroy(BJoltWorld *world, uint32_t body_raw)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	BodyLockRead destroy_lock(world->physics_system->GetBodyLockInterface(), body_id);
	bool is_soft = destroy_lock.Succeeded() && destroy_lock.GetBody().IsSoftBody();
	destroy_lock.ReleaseLock();
	if (is_soft)
		body_interface.DestroyBody(body_id);
}

// Wind for one tick: uniform breeze over the whole sheet, pins hold the
// top so folds ripple through the constraints. Call every Fixed tick
// before the step. No-op on bad ids or rigid bodies.
void bjolt_soft_push(BJoltWorld *world, uint32_t body_raw,
	float force_x, float force_y, float force_z)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	BodyLockRead push_lock(world->physics_system->GetBodyLockInterface(), body_id);
	bool is_soft = push_lock.Succeeded() && push_lock.GetBody().IsSoftBody();
	push_lock.ReleaseLock();
	if (is_soft) {
		// Pushing means awake: a sleeping sheet would eat the breeze and
		// hang frozen. Activation is cheap; the push is the intent.
		body_interface.ActivateBody(body_id);
		body_interface.AddForce(body_id, Vec3(force_x, force_y, force_z));
	}
}

// Shared soft-body settings registry: Rust builds plain shape data once,
// C++ assembles and optimizes it. Handles are heap ids, never raw
// pointers, so Rust can hold them in a u64 without lifetime risk.
static std::vector<Ref<SoftBodySharedSettings>> s_soft_shared_registry;

static uint64_t soft_shared_handle(Ref<SoftBodySharedSettings> shared_settings)
{
	s_soft_shared_registry.push_back(shared_settings);
	return (uint64_t)s_soft_shared_registry.size();
}

static SoftBodySharedSettings *soft_shared_lookup(uint64_t shared_handle)
{
	if (shared_handle == 0 || shared_handle > s_soft_shared_registry.size())
		return nullptr;
	Ref<SoftBodySharedSettings> shared_settings = s_soft_shared_registry[(size_t)shared_handle - 1];
	return shared_settings.GetPtr();
}

static SoftBodySharedSettings::EBendType soft_bend_type(uint8_t bend_type)
{
	switch (bend_type)
	{
	case 1: return SoftBodySharedSettings::EBendType::Distance;
	case 2: return SoftBodySharedSettings::EBendType::Dihedral;
	default: return SoftBodySharedSettings::EBendType::None;
	}
}

// Assembles shared settings from Rust-side arrays: flat xyz positions,
// velocities, inverse masses, triangle triples, edge pairs + compliances,
// volume quads + compliances. Null arrays (with zero counts) skip that
// constraint group; CreateConstraints auto-builds edges from faces when
// no hand-placed edges arrive.
uint64_t bjolt_create_shared_settings(
	const float *vertex_positions, const float *vertex_velocities, const float *vertex_inv_masses,
	uint32_t vertex_count,
	const uint32_t *face_indices, uint32_t face_count,
	const uint32_t *edge_pairs, const float *edge_compliances, uint32_t edge_count,
	const uint32_t *volume_quads, const float *volume_compliances, uint32_t volume_count,
	float edge_compliance, float shear_compliance, float bend_compliance, uint8_t bend_type)
{
	if (vertex_count == 0 || vertex_positions == nullptr || vertex_inv_masses == nullptr)
		return 0;
	Ref<SoftBodySharedSettings> shared_settings = new SoftBodySharedSettings;
	for (uint32_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
	{
		SoftBodySharedSettings::Vertex soft_vertex;
		soft_vertex.mPosition = Float3(
			vertex_positions[vertex_index * 3],
			vertex_positions[vertex_index * 3 + 1],
			vertex_positions[vertex_index * 3 + 2]);
		if (vertex_velocities != nullptr)
			soft_vertex.mVelocity = Float3(
				vertex_velocities[vertex_index * 3],
				vertex_velocities[vertex_index * 3 + 1],
				vertex_velocities[vertex_index * 3 + 2]);
		soft_vertex.mInvMass = vertex_inv_masses[vertex_index];
		shared_settings->mVertices.push_back(soft_vertex);
	}
	if (face_indices != nullptr)
		for (uint32_t face_index = 0; face_index < face_count; ++face_index)
		{
			SoftBodySharedSettings::Face soft_face(
				face_indices[face_index * 3],
				face_indices[face_index * 3 + 1],
				face_indices[face_index * 3 + 2]);
			if (!soft_face.IsDegenerate()
				&& soft_face.mVertex[0] < vertex_count
				&& soft_face.mVertex[1] < vertex_count
				&& soft_face.mVertex[2] < vertex_count)
				shared_settings->AddFace(soft_face);
		}
	if (edge_pairs != nullptr)
		for (uint32_t edge_index = 0; edge_index < edge_count; ++edge_index)
		{
			uint32_t edge_start = edge_pairs[edge_index * 2];
			uint32_t edge_end = edge_pairs[edge_index * 2 + 1];
			if (edge_start >= vertex_count || edge_end >= vertex_count || edge_start == edge_end)
				continue;
			float edge_stiffness = (edge_compliances != nullptr) ? edge_compliances[edge_index] : edge_compliance;
			shared_settings->mEdgeConstraints.emplace_back(edge_start, edge_end, edge_stiffness);
		}
	if (volume_quads != nullptr)
		for (uint32_t volume_index = 0; volume_index < volume_count; ++volume_index)
		{
			bool volume_valid = true;
			for (uint32_t corner = 0; corner < 4; ++corner)
				if (volume_quads[volume_index * 4 + corner] >= vertex_count)
					volume_valid = false;
			if (!volume_valid)
				continue;
			float volume_stiffness = (volume_compliances != nullptr) ? volume_compliances[volume_index] : 0.0f;
			shared_settings->mVolumeConstraints.emplace_back(
				volume_quads[volume_index * 4],
				volume_quads[volume_index * 4 + 1],
				volume_quads[volume_index * 4 + 2],
				volume_quads[volume_index * 4 + 3],
				volume_stiffness);
		}
	if (edge_pairs == nullptr && !shared_settings->mFaces.empty())
	{
		SoftBodySharedSettings::VertexAttributes auto_constraints(edge_compliance, shear_compliance, bend_compliance);
		shared_settings->CreateConstraints(&auto_constraints, 1, soft_bend_type(bend_type));
	}
	else
	{
		shared_settings->CalculateEdgeLengths();
		shared_settings->CalculateVolumeConstraintVolumes();
	}
	shared_settings->Optimize();
	return soft_shared_handle(shared_settings);
}

void bjolt_destroy_shared_settings(uint64_t shared_handle)
{
	if (shared_handle == 0 || shared_handle > s_soft_shared_registry.size())
		return;
	s_soft_shared_registry[(size_t)shared_handle - 1] = nullptr;
}

uint64_t bjolt_create_cube_settings(uint32_t grid_size, float grid_spacing)
{
	if (grid_size < 2 || grid_spacing <= 0.0f)
		return 0;
	return soft_shared_handle(SoftBodySharedSettings::sCreateCube(grid_size, grid_spacing));
}

uint64_t bjolt_create_cloth_settings(
	uint32_t grid_nx, uint32_t grid_nz, float grid_spacing, uint32_t pinned_rows, uint8_t bend_type)
{
	if (grid_nx < 2 || grid_nz < 2 || grid_spacing <= 0.0f)
		return 0;
	Ref<SoftBodySharedSettings> shared_settings = new SoftBodySharedSettings;
	float offset_x = -0.5f * grid_spacing * (grid_nx - 1);
	for (uint32_t row = 0; row < grid_nz; ++row)
		for (uint32_t x = 0; x < grid_nx; ++x)
		{
			SoftBodySharedSettings::Vertex soft_vertex;
			soft_vertex.mPosition = Float3(offset_x + x * grid_spacing, -(float)row * grid_spacing, 0.0f);
			soft_vertex.mInvMass = (row < pinned_rows) ? 0.0f : 1.0f;
			shared_settings->mVertices.push_back(soft_vertex);
		}
	auto cloth_index = [grid_nx](uint32_t x, uint32_t row) { return x + row * grid_nx; };
	for (uint32_t row = 0; row < grid_nz - 1; ++row)
		for (uint32_t x = 0; x < grid_nx - 1; ++x)
		{
			SoftBodySharedSettings::Face cloth_face;
			cloth_face.mVertex[0] = cloth_index(x, row);
			cloth_face.mVertex[1] = cloth_index(x, row + 1);
			cloth_face.mVertex[2] = cloth_index(x + 1, row + 1);
			shared_settings->AddFace(cloth_face);
			cloth_face.mVertex[1] = cloth_index(x + 1, row + 1);
			cloth_face.mVertex[2] = cloth_index(x + 1, row);
			shared_settings->AddFace(cloth_face);
		}
	SoftBodySharedSettings::VertexAttributes cloth_attributes(1.0e-5f, 1.0e-5f, 1.0e-5f);
	shared_settings->CreateConstraints(&cloth_attributes, 1, soft_bend_type(bend_type));
	shared_settings->Optimize();
	return soft_shared_handle(shared_settings);
}

uint64_t bjolt_create_sphere_settings(
	float sphere_radius, uint32_t theta_segments, uint32_t phi_segments, uint8_t bend_type)
{
	if (sphere_radius <= 0.0f || theta_segments < 3 || phi_segments < 3)
		return 0;
	Ref<SoftBodySharedSettings> shared_settings = new SoftBodySharedSettings;
	SoftBodySharedSettings::Vertex pole_vertex;
	(sphere_radius * Vec3::sUnitSpherical(0, 0)).StoreFloat3(&pole_vertex.mPosition);
	shared_settings->mVertices.push_back(pole_vertex);
	(sphere_radius * Vec3::sUnitSpherical(JPH_PI, 0)).StoreFloat3(&pole_vertex.mPosition);
	shared_settings->mVertices.push_back(pole_vertex);
	for (uint32_t theta = 1; theta < theta_segments - 1; ++theta)
		for (uint32_t phi = 0; phi < phi_segments; ++phi)
		{
			SoftBodySharedSettings::Vertex ring_vertex;
			(sphere_radius * Vec3::sUnitSpherical(
				JPH_PI * theta / (theta_segments - 1),
				2.0f * JPH_PI * phi / phi_segments)).StoreFloat3(&ring_vertex.mPosition);
			shared_settings->mVertices.push_back(ring_vertex);
		}
	auto sphere_index = [theta_segments, phi_segments](uint32_t theta, uint32_t phi) -> uint32_t
	{
		if (theta == 0)
			return 0;
		else if (theta == theta_segments - 1)
			return 1;
		else
			return 2 + (theta - 1) * phi_segments + phi % phi_segments;
	};
	for (uint32_t phi = 0; phi < phi_segments; ++phi)
		for (uint32_t theta = 0; theta < theta_segments - 2; ++theta)
		{
			SoftBodySharedSettings::Face sphere_face;
			sphere_face.mVertex[0] = sphere_index(theta, phi);
			sphere_face.mVertex[1] = sphere_index(theta + 1, phi);
			sphere_face.mVertex[2] = sphere_index(theta + 1, phi + 1);
			shared_settings->AddFace(sphere_face);
			if (theta > 0)
			{
				sphere_face.mVertex[1] = sphere_index(theta + 1, phi + 1);
				sphere_face.mVertex[2] = sphere_index(theta, phi + 1);
				shared_settings->AddFace(sphere_face);
			}
		}
	SoftBodySharedSettings::VertexAttributes sphere_attributes(1.0e-4f, 1.0e-4f, 1.0e-3f);
	shared_settings->CreateConstraints(&sphere_attributes, 1, soft_bend_type(bend_type));
	shared_settings->Optimize();
	return soft_shared_handle(shared_settings);
}

uint32_t bjolt_shared_vertex_count(uint64_t shared_handle)
{
	SoftBodySharedSettings *shared_settings = soft_shared_lookup(shared_handle);
	return (shared_settings != nullptr) ? (uint32_t)shared_settings->mVertices.size() : 0;
}

uint32_t bjolt_shared_face_count(uint64_t shared_handle)
{
	SoftBodySharedSettings *shared_settings = soft_shared_lookup(shared_handle);
	return (shared_settings != nullptr) ? (uint32_t)shared_settings->mFaces.size() : 0;
}

uint32_t bjolt_shared_faces(uint64_t shared_handle, uint32_t *out_triangles, uint32_t capacity)
{
	SoftBodySharedSettings *shared_settings = soft_shared_lookup(shared_handle);
	if (shared_settings == nullptr || out_triangles == nullptr)
		return 0;
	uint32_t written = 0;
	for (const SoftBodySharedSettings::Face &shared_face : shared_settings->mFaces)
	{
		if (written >= capacity)
			break;
		out_triangles[written * 3] = shared_face.mVertex[0];
		out_triangles[written * 3 + 1] = shared_face.mVertex[1];
		out_triangles[written * 3 + 2] = shared_face.mVertex[2];
		++written;
	}
	return written;
}

// Full creation-settings mirror: every Jolt per-body knob passes through,
// nothing hides in C++ defaults.
uint32_t bjolt_create_soft_body(
	BJoltWorld *world, uint64_t shared_handle,
	float pos_x, float pos_y, float pos_z,
	float rot_x, float rot_y, float rot_z, float rot_w,
	uint16_t object_layer,
	uint32_t num_iterations, float linear_damping, float max_linear_velocity,
	float restitution, float friction, float pressure, float gravity_factor,
	float vertex_radius, bool update_position, bool make_rotation_identity,
	bool allow_sleeping, bool faces_double_sided, uint64_t user_data)
{
	if (world == nullptr || !bjolt_bodies_available(world))
		return 0;
	SoftBodySharedSettings *shared_settings = soft_shared_lookup(shared_handle);
	if (shared_settings == nullptr)
		return 0;
	SoftBodyCreationSettings body_settings(shared_settings, RVec3(pos_x, pos_y, pos_z),
		Quat(rot_x, rot_y, rot_z, rot_w), (ObjectLayer)object_layer);
	body_settings.mNumIterations = num_iterations;
	body_settings.mLinearDamping = linear_damping;
	body_settings.mMaxLinearVelocity = max_linear_velocity;
	body_settings.mRestitution = restitution;
	body_settings.mFriction = friction;
	body_settings.mPressure = pressure;
	body_settings.mGravityFactor = gravity_factor;
	body_settings.mVertexRadius = vertex_radius;
	body_settings.mUpdatePosition = update_position;
	body_settings.mMakeRotationIdentity = make_rotation_identity;
	body_settings.mAllowSleeping = allow_sleeping;
	body_settings.mFacesDoubleSided = faces_double_sided;
	body_settings.mUserData = user_data;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID soft_id = body_interface.CreateAndAddSoftBody(body_settings, EActivation::Activate);
	return soft_id.GetIndexAndSequenceNumber();
}

// Reads one soft body under a read lock. Null body or non-soft reads zero.
static const SoftBodyMotionProperties *soft_motion_read(BJoltWorld *world, uint32_t body_raw, BodyLockRead &soft_lock)
{
	if (!soft_lock.Succeeded() || !soft_lock.GetBody().IsSoftBody())
		return nullptr;
	(void)world;
	return static_cast<const SoftBodyMotionProperties *>(soft_lock.GetBody().GetMotionProperties());
}

uint32_t bjolt_soft_velocities(BJoltWorld *world, uint32_t body_raw, float *out_velocities, uint32_t capacity)
{
	BodyLockRead soft_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	const SoftBodyMotionProperties *soft_props = soft_motion_read(world, body_raw, soft_lock);
	if (soft_props == nullptr || out_velocities == nullptr)
		return 0;
	uint32_t written = 0;
	for (const SoftBodyMotionProperties::Vertex &soft_vertex : soft_props->GetVertices())
	{
		if (written >= capacity)
			break;
		out_velocities[written * 3] = soft_vertex.mVelocity.GetX();
		out_velocities[written * 3 + 1] = soft_vertex.mVelocity.GetY();
		out_velocities[written * 3 + 2] = soft_vertex.mVelocity.GetZ();
		++written;
	}
	return written;
}

uint32_t bjolt_soft_inv_masses(BJoltWorld *world, uint32_t body_raw, float *out_inv_masses, uint32_t capacity)
{
	BodyLockRead soft_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	const SoftBodyMotionProperties *soft_props = soft_motion_read(world, body_raw, soft_lock);
	if (soft_props == nullptr || out_inv_masses == nullptr)
		return 0;
	uint32_t written = 0;
	for (const SoftBodyMotionProperties::Vertex &soft_vertex : soft_props->GetVertices())
	{
		if (written >= capacity)
			break;
		out_inv_masses[written] = soft_vertex.mInvMass;
		++written;
	}
	return written;
}

// Pin/unpin live: zero mass nails a vertex, positive mass frees it.
// Jolt says only mass and velocity are safe to touch at runtime.
void bjolt_soft_set_inv_masses(BJoltWorld *world, uint32_t body_raw, const float *inv_masses, uint32_t count)
{
	BodyLockWrite soft_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	if (!soft_lock.Succeeded() || !soft_lock.GetBody().IsSoftBody() || inv_masses == nullptr)
		return;
	SoftBodyMotionProperties *soft_props = static_cast<SoftBodyMotionProperties *>(
		soft_lock.GetBody().GetMotionProperties());
	uint32_t vertex_total = (uint32_t)soft_props->GetVertices().size();
	for (uint32_t vertex_index = 0; vertex_index < count && vertex_index < vertex_total; ++vertex_index)
		soft_props->GetVertex(vertex_index).mInvMass = inv_masses[vertex_index];
}

uint32_t bjolt_soft_contacts(BJoltWorld *world, uint32_t body_raw, uint8_t *out_contacted, uint32_t capacity)
{
	BodyLockRead soft_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	const SoftBodyMotionProperties *soft_props = soft_motion_read(world, body_raw, soft_lock);
	if (soft_props == nullptr || out_contacted == nullptr)
		return 0;
	uint32_t written = 0;
	for (const SoftBodyMotionProperties::Vertex &soft_vertex : soft_props->GetVertices())
	{
		if (written >= capacity)
			break;
		out_contacted[written] = soft_vertex.mHasContact ? 1 : 0;
		++written;
	}
	return written;
}

float bjolt_soft_pressure(BJoltWorld *world, uint32_t body_raw)
{
	BodyLockRead soft_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	const SoftBodyMotionProperties *soft_props = soft_motion_read(world, body_raw, soft_lock);
	return (soft_props != nullptr) ? soft_props->GetPressure() : 0.0f;
}

void bjolt_soft_set_pressure(BJoltWorld *world, uint32_t body_raw, float pressure)
{
	BodyLockWrite soft_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	if (!soft_lock.Succeeded() || !soft_lock.GetBody().IsSoftBody())
		return;
	static_cast<SoftBodyMotionProperties *>(
		soft_lock.GetBody().GetMotionProperties())->SetPressure(pressure);
}

uint32_t bjolt_soft_iterations(BJoltWorld *world, uint32_t body_raw)
{
	BodyLockRead soft_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	const SoftBodyMotionProperties *soft_props = soft_motion_read(world, body_raw, soft_lock);
	return (soft_props != nullptr) ? soft_props->GetNumIterations() : 0;
}

void bjolt_soft_set_iterations(BJoltWorld *world, uint32_t body_raw, uint32_t num_iterations)
{
	BodyLockWrite soft_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	if (!soft_lock.Succeeded() || !soft_lock.GetBody().IsSoftBody())
		return;
	static_cast<SoftBodyMotionProperties *>(
		soft_lock.GetBody().GetMotionProperties())->SetNumIterations(num_iterations);
}

float bjolt_soft_vertex_radius(BJoltWorld *world, uint32_t body_raw)
{
	BodyLockRead soft_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	const SoftBodyMotionProperties *soft_props = soft_motion_read(world, body_raw, soft_lock);
	return (soft_props != nullptr) ? soft_props->GetVertexRadius() : 0.0f;
}

void bjolt_soft_set_vertex_radius(BJoltWorld *world, uint32_t body_raw, float vertex_radius)
{
	if (vertex_radius < 0.0f)
		return;
	BodyLockWrite soft_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	if (!soft_lock.Succeeded() || !soft_lock.GetBody().IsSoftBody())
		return;
	static_cast<SoftBodyMotionProperties *>(
		soft_lock.GetBody().GetMotionProperties())->SetVertexRadius(vertex_radius);
}

float bjolt_soft_volume(BJoltWorld *world, uint32_t body_raw)
{
	BodyLockRead soft_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	const SoftBodyMotionProperties *soft_props = soft_motion_read(world, body_raw, soft_lock);
	return (soft_props != nullptr) ? soft_props->GetVolume() : 0.0f;
}

void bjolt_set_position(BJoltWorld *world, uint32_t body_raw,
	float pos_x, float pos_y, float pos_z)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	body_interface.SetPosition(body_id, RVec3(pos_x, pos_y, pos_z), EActivation::Activate);
}
// Full pose teleport: position + rotation in one call so the body never
// observes a half-moved frame. Activates the body. No-op on invalid ids.
void bjolt_set_position_rotation(BJoltWorld *world, uint32_t body_raw,
	float pos_x, float pos_y, float pos_z,
	float rot_x, float rot_y, float rot_z, float rot_w)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_id(body_raw);
	body_interface.SetPositionAndRotation(body_id, RVec3(pos_x, pos_y, pos_z),
		Quat(rot_x, rot_y, rot_z, rot_w), EActivation::Activate);
}

// Per-body gravity multiplier: scales world gravity for one body (0 = float,
// 1 = normal, 2 = double). Stored on the body's motion properties; the
// creation path bakes it in, this changes it on a live body. No-op on
// invalid ids.
void bjolt_set_gravity_factor(BJoltWorld *world, uint32_t body_raw, float gravity_factor)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	body_interface.SetGravityFactor(BodyID(body_raw), gravity_factor);
}

float bjolt_gravity_factor(BJoltWorld *world, uint32_t body_raw)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	return body_interface.GetGravityFactor(BodyID(body_raw));
}

// Global gravity all bodies feel, scaled per body by its gravity factor.
// Jolt defaults to (0, -9.81, 0); this reads or replaces it at runtime.
void bjolt_set_gravity(BJoltWorld *world, float gravity_x, float gravity_y, float gravity_z)
{
	world->physics_system->SetGravity(Vec3(gravity_x, gravity_y, gravity_z));
}

void bjolt_world_gravity(BJoltWorld *world, float *out_gravity)
{
	Vec3 gravity = world->physics_system->GetGravity();
	out_gravity[0] = gravity.GetX();
	out_gravity[1] = gravity.GetY();
	out_gravity[2] = gravity.GetZ();
}

// Virtual characters: a kinematic capsule the game steers by velocity, Jolt
// resolves collisions (stairs, slopes, platforms). 1-based ids like the
// constraint registry; destroyed slots null out and never recycle.
static CharacterVirtual *bjolt_character_at(BJoltWorld *world, uint32_t character_id)
{
	if (character_id == 0 || character_id > world->character_registry.size())
		return nullptr;
	Ref<CharacterVirtual> &slot = world->character_registry[character_id - 1];
	return slot.GetPtr();
}

uint32_t bjolt_character_create(BJoltWorld *world,
	float pos_x, float pos_y, float pos_z,
	float capsule_half_height, float capsule_radius,
	uint16_t object_layer,
	float mass_kg, float max_strength, float max_slope_degrees,
	float character_padding, float penetration_recovery)
{
	RefConst<Shape> capsule = new CapsuleShape(0.5f * (2.0f * capsule_half_height + 2.0f * capsule_radius) - capsule_radius, capsule_radius);
	CharacterVirtualSettings character_settings;
	character_settings.mShape = capsule;
	character_settings.mSupportingVolume = Plane(Vec3::sAxisY(), -1.0e10f);
	character_settings.mMaxSlopeAngle = DegreesToRadians(max_slope_degrees);
	character_settings.mMass = mass_kg;
	character_settings.mMaxStrength = max_strength;
	character_settings.mCharacterPadding = character_padding;
	character_settings.mPenetrationRecoverySpeed = penetration_recovery;
	Ref<CharacterVirtual> character = new CharacterVirtual(&character_settings,
		RVec3(pos_x, pos_y, pos_z), Quat::sIdentity(), world->physics_system);
	world->character_registry.push_back(character);
	// Remember the layer per character: filters below need it, and Jolt's
	// character itself carries no object layer.
	world->character_layers.push_back((ObjectLayer)object_layer);
	return (uint32_t)world->character_registry.size();
}

void bjolt_character_destroy(BJoltWorld *world, uint32_t character_id)
{
	if (character_id == 0 || character_id > world->character_registry.size())
		return;
	world->character_registry[character_id - 1] = nullptr;
}

// Filter shorthands: the character collides with everything its object layer
// is allowed to meet (same table the rigid bodies use).
static DefaultBroadPhaseLayerFilter bjolt_character_broad_filter(BJoltWorld *world, uint32_t character_id)
{
	return DefaultBroadPhaseLayerFilter(world->physics_system->GetObjectVsBroadPhaseLayerFilter(),
		world->character_layers[character_id - 1]);
}

static DefaultObjectLayerFilter bjolt_character_object_filter(BJoltWorld *world, uint32_t character_id)
{
	return DefaultObjectLayerFilter(world->physics_system->GetObjectLayerPairFilter(),
		world->character_layers[character_id - 1]);
}
void bjolt_character_move(BJoltWorld *world, uint32_t character_id,
	float delta_time,
	float velocity_x, float velocity_y, float velocity_z,
	float gravity_x, float gravity_y, float gravity_z,
	float step_up_height, float stick_to_floor_distance,
	float *out_position, float *out_velocity, float *out_ground_normal,
	uint32_t *out_ground_state, uint32_t *out_is_supported)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->SetLinearVelocity(Vec3(velocity_x, velocity_y, velocity_z));
	CharacterVirtual::ExtendedUpdateSettings move_settings;
	move_settings.mWalkStairsStepUp = Vec3(0.0f, step_up_height, 0.0f);
	move_settings.mStickToFloorStepDown = Vec3(0.0f, -stick_to_floor_distance, 0.0f);
	character->ExtendedUpdate(delta_time,
		Vec3(gravity_x, gravity_y, gravity_z), move_settings,
		bjolt_character_broad_filter(world, character_id),
		bjolt_character_object_filter(world, character_id),
		BodyFilter(), ShapeFilter(),
		*world->temp_allocator);
	RVec3 character_position = character->GetCenterOfMassPosition();
	Vec3 character_velocity = character->GetLinearVelocity();
	out_position[0] = (float)character_position.GetX();
	out_position[1] = (float)character_position.GetY();
	out_position[2] = (float)character_position.GetZ();
	out_velocity[0] = character_velocity.GetX();
	out_velocity[1] = character_velocity.GetY();
	Vec3 ground_normal = character->GetGroundNormal();
	out_ground_normal[0] = ground_normal.GetX();
	out_ground_normal[1] = ground_normal.GetY();
	out_ground_normal[2] = ground_normal.GetZ();
	*out_ground_state = (uint32_t)character->GetGroundState();
	*out_is_supported = character->IsSupported() ? 1u : 0u;
}

void bjolt_character_teleport(BJoltWorld *world, uint32_t character_id,
	float pos_x, float pos_y, float pos_z,
	float velocity_x, float velocity_y, float velocity_z)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->SetPosition(RVec3(pos_x, pos_y, pos_z));
	character->SetLinearVelocity(Vec3(velocity_x, velocity_y, velocity_z));
	character->RefreshContacts(
		bjolt_character_broad_filter(world, character_id),
		bjolt_character_object_filter(world, character_id),
		BodyFilter(), ShapeFilter(),
		*world->temp_allocator);
}

bool bjolt_character_stance(BJoltWorld *world, uint32_t character_id,
	float capsule_half_height, float capsule_radius)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return false;
	RefConst<Shape> capsule = new CapsuleShape(0.5f * (2.0f * capsule_half_height + 2.0f * capsule_radius) - capsule_radius, capsule_radius);
	return character->SetShape(capsule, 0.02f,
		bjolt_character_broad_filter(world, character_id),
		bjolt_character_object_filter(world, character_id),
		BodyFilter(), ShapeFilter(),
		*world->temp_allocator);
}

// Virtual character extras: thin mirrors over the CharacterVirtual API.
// Rotation, mass, and tuning setters; plain Update without stairs;
// standalone stair/floor helpers; full ground readings; contact refresh.
void bjolt_character_set_rotation(BJoltWorld *world, uint32_t character_id,
	float rot_x, float rot_y, float rot_z, float rot_w)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->SetRotation(Quat(rot_x, rot_y, rot_z, rot_w));
}

void bjolt_character_rotation(BJoltWorld *world, uint32_t character_id, float *out_rotation)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr || out_rotation == nullptr)
		return;
	Quat character_rotation = character->GetRotation();
	out_rotation[0] = character_rotation.GetX();
	out_rotation[1] = character_rotation.GetY();
	out_rotation[2] = character_rotation.GetZ();
	out_rotation[3] = character_rotation.GetW();
}

void bjolt_character_set_mass(BJoltWorld *world, uint32_t character_id, float mass_kg, float max_strength)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->SetMass(mass_kg);
	character->SetMaxStrength(max_strength);
}

void bjolt_character_set_padding(BJoltWorld *world, uint32_t character_id,
	float character_padding, float penetration_recovery)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return;
	// Padding is construction-only in Jolt (no setter); penetration
	// recovery is the live half. Padding stays in the create call.
	(void)character_padding;
	character->SetPenetrationRecoverySpeed(penetration_recovery);
}

void bjolt_character_set_up(BJoltWorld *world, uint32_t character_id,
	float up_x, float up_y, float up_z, float max_slope_degrees)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->SetUp(Vec3(up_x, up_y, up_z));
	character->SetMaxSlopeAngle(DegreesToRadians(max_slope_degrees));
}

void bjolt_character_set_shape_offset(BJoltWorld *world, uint32_t character_id,
	float offset_x, float offset_y, float offset_z)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->SetShapeOffset(Vec3(offset_x, offset_y, offset_z));
}

void bjolt_character_set_user_data(BJoltWorld *world, uint32_t character_id, uint64_t user_data)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->SetUserData(user_data);
}

// Plain Update: collide + settle without stairs or floor stick. Same out
// block as the full move so Rust reuses one reader.
void bjolt_character_update(BJoltWorld *world, uint32_t character_id,
	float delta_time,
	float velocity_x, float velocity_y, float velocity_z,
	float gravity_x, float gravity_y, float gravity_z,
	float *out_position, float *out_velocity, float *out_ground_normal,
	uint32_t *out_ground_state, uint32_t *out_is_supported)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->SetLinearVelocity(Vec3(velocity_x, velocity_y, velocity_z));
	character->Update(delta_time,
		Vec3(gravity_x, gravity_y, gravity_z),
		bjolt_character_broad_filter(world, character_id),
		bjolt_character_object_filter(world, character_id),
		BodyFilter(), ShapeFilter(),
		*world->temp_allocator);
	RVec3 updated_position = character->GetCenterOfMassPosition();
	Vec3 updated_velocity = character->GetLinearVelocity();
	Vec3 updated_normal = character->GetGroundNormal();
	if (out_position != nullptr) {
		out_position[0] = (float)updated_position.GetX();
		out_position[1] = (float)updated_position.GetY();
		out_position[2] = (float)updated_position.GetZ();
	}
	if (out_velocity != nullptr) {
		out_velocity[0] = updated_velocity.GetX();
		out_velocity[1] = updated_velocity.GetY();
		out_velocity[2] = updated_velocity.GetZ();
	}
	if (out_ground_normal != nullptr) {
		out_ground_normal[0] = updated_normal.GetX();
		out_ground_normal[1] = updated_normal.GetY();
		out_ground_normal[2] = updated_normal.GetZ();
	}
	if (out_ground_state != nullptr)
		*out_ground_state = (uint32_t)character->GetGroundState();
	if (out_is_supported != nullptr)
		*out_is_supported = character->IsSupported() ? 1u : 0u;
}

bool bjolt_character_can_walk_stairs(BJoltWorld *world, uint32_t character_id,
	float velocity_x, float velocity_y, float velocity_z)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return false;
	return character->CanWalkStairs(Vec3(velocity_x, velocity_y, velocity_z));
}

bool bjolt_character_walk_stairs(BJoltWorld *world, uint32_t character_id,
	float delta_time,
	float step_up_height, float step_forward, float step_forward_test, float step_down_extra)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return false;
	Vec3 step_forward_vec = character->GetLinearVelocity();
	float forward_len = step_forward_vec.Length();
	if (forward_len > 1.0e-6f)
		step_forward_vec = step_forward_vec * (step_forward / forward_len);
	else
		step_forward_vec = Vec3(step_forward, 0.0f, 0.0f);
	return character->WalkStairs(delta_time,
		Vec3(0.0f, step_up_height, 0.0f), step_forward_vec,
		Vec3(0.0f, 0.0f, 0.0f) + Vec3(step_forward_test, 0.0f, 0.0f),
		Vec3(0.0f, -step_down_extra, 0.0f),
		bjolt_character_broad_filter(world, character_id),
		bjolt_character_object_filter(world, character_id),
		BodyFilter(), ShapeFilter(),
		*world->temp_allocator);
}

bool bjolt_character_stick_to_floor(BJoltWorld *world, uint32_t character_id,
	float stick_down_distance)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return false;
	return character->StickToFloor(Vec3(0.0f, -stick_down_distance, 0.0f),
		bjolt_character_broad_filter(world, character_id),
		bjolt_character_object_filter(world, character_id),
		BodyFilter(), ShapeFilter(),
		*world->temp_allocator);
}

void bjolt_character_refresh_contacts(BJoltWorld *world, uint32_t character_id)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->RefreshContacts(
		bjolt_character_broad_filter(world, character_id),
		bjolt_character_object_filter(world, character_id),
		BodyFilter(), ShapeFilter(),
		*world->temp_allocator);
}

// Full ground reading: position, normal, surface velocity, supporting body
// id (0 when airborne), and user data off that body.
void bjolt_character_ground(BJoltWorld *world, uint32_t character_id,
	float *out_ground_position, float *out_ground_normal, float *out_ground_velocity,
	uint32_t *out_ground_body, uint64_t *out_ground_user_data)
{
	CharacterVirtual *character = bjolt_character_at(world, character_id);
	if (character == nullptr)
		return;
	RVec3 ground_position = character->GetGroundPosition();
	Vec3 ground_normal = character->GetGroundNormal();
	Vec3 ground_velocity = character->GetGroundVelocity();
	if (out_ground_position != nullptr) {
		out_ground_position[0] = (float)ground_position.GetX();
		out_ground_position[1] = (float)ground_position.GetY();
		out_ground_position[2] = (float)ground_position.GetZ();
	}
	if (out_ground_normal != nullptr) {
		out_ground_normal[0] = ground_normal.GetX();
		out_ground_normal[1] = ground_normal.GetY();
		out_ground_normal[2] = ground_normal.GetZ();
	}
	if (out_ground_velocity != nullptr) {
		out_ground_velocity[0] = ground_velocity.GetX();
		out_ground_velocity[1] = ground_velocity.GetY();
		out_ground_velocity[2] = ground_velocity.GetZ();
	}
	if (out_ground_body != nullptr)
		*out_ground_body = character->GetGroundBodyID().GetIndexAndSequenceNumber();
	if (out_ground_user_data != nullptr)
		*out_ground_user_data = character->GetGroundUserData();
}


// Rigid characters: a real capsule body Jolt simulates. The game drives it
// by velocity; PostSimulation refreshes ground after each physics step.
static Character *bjolt_rigid_character_at(BJoltWorld *world, uint32_t character_id)
{
	if (character_id == 0 || character_id > world->rigid_character_registry.size())
		return nullptr;
	Ref<Character> &slot = world->rigid_character_registry[character_id - 1];
	return slot.GetPtr();
}

uint32_t bjolt_rigid_character_create(BJoltWorld *world,
	float pos_x, float pos_y, float pos_z,
	float rot_x, float rot_y, float rot_z, float rot_w,
	float capsule_half_height, float capsule_radius,
	uint16_t object_layer, float mass_kg, float friction, float gravity_factor,
	uint8_t allowed_dofs, uint64_t user_data)
{
	if (!bjolt_bodies_available(world))
		return 0;
	RefConst<Shape> capsule = new CapsuleShape(
		0.5f * (2.0f * capsule_half_height + 2.0f * capsule_radius) - capsule_radius, capsule_radius);
	CharacterSettings character_settings;
	character_settings.mLayer = (ObjectLayer)object_layer;
	character_settings.mMass = mass_kg;
	character_settings.mFriction = friction;
	character_settings.mGravityFactor = gravity_factor;
	character_settings.mAllowedDOFs = (EAllowedDOFs)allowed_dofs;
	character_settings.mShape = capsule;
	Ref<Character> character = new Character(&character_settings,
		RVec3(pos_x, pos_y, pos_z), Quat(rot_x, rot_y, rot_z, rot_w),
		user_data, world->physics_system);
	character->AddToPhysicsSystem(EActivation::Activate);
	world->rigid_character_registry.push_back(character);
	return (uint32_t)world->rigid_character_registry.size();
}

void bjolt_rigid_character_destroy(BJoltWorld *world, uint32_t character_id)
{
	Character *character = bjolt_rigid_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->RemoveFromPhysicsSystem();
	world->rigid_character_registry[character_id - 1] = nullptr;
}


void bjolt_rigid_character_post(BJoltWorld *world, uint32_t character_id, float max_separation)
{
	Character *character = bjolt_rigid_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->PostSimulation(max_separation);
}

void bjolt_rigid_character_set_velocity(BJoltWorld *world, uint32_t character_id,
	float lin_x, float lin_y, float lin_z,
	float ang_x, float ang_y, float ang_z)
{
	Character *character = bjolt_rigid_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->SetLinearAndAngularVelocity(Vec3(lin_x, lin_y, lin_z), Vec3(ang_x, ang_y, ang_z));
}

void bjolt_rigid_character_add_velocity(BJoltWorld *world, uint32_t character_id,
	float lin_x, float lin_y, float lin_z)
{
	Character *character = bjolt_rigid_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->AddLinearVelocity(Vec3(lin_x, lin_y, lin_z));
}

void bjolt_rigid_character_add_impulse(BJoltWorld *world, uint32_t character_id,
	float imp_x, float imp_y, float imp_z)
{
	Character *character = bjolt_rigid_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->AddImpulse(Vec3(imp_x, imp_y, imp_z));
}

void bjolt_rigid_character_pose(BJoltWorld *world, uint32_t character_id,
	float *out_position, float *out_rotation, float *out_velocity,
	float *out_ground_normal, uint32_t *out_ground_state, uint32_t *out_is_supported)
{
	Character *character = bjolt_rigid_character_at(world, character_id);
	if (character == nullptr)
		return;
	RVec3 character_position = character->GetPosition();
	Quat character_rotation = character->GetRotation();
	Vec3 character_velocity = character->GetLinearVelocity();
	if (out_position != nullptr) {
		out_position[0] = (float)character_position.GetX();
		out_position[1] = (float)character_position.GetY();
		out_position[2] = (float)character_position.GetZ();
	}
	if (out_rotation != nullptr) {
		out_rotation[0] = character_rotation.GetX();
		out_rotation[1] = character_rotation.GetY();
		out_rotation[2] = character_rotation.GetZ();
		out_rotation[3] = character_rotation.GetW();
	}
	if (out_velocity != nullptr) {
		out_velocity[0] = character_velocity.GetX();
		out_velocity[1] = character_velocity.GetY();
		out_velocity[2] = character_velocity.GetZ();
	}
	Vec3 ground_normal = character->GetGroundNormal();
	if (out_ground_normal != nullptr) {
		out_ground_normal[0] = ground_normal.GetX();
		out_ground_normal[1] = ground_normal.GetY();
		out_ground_normal[2] = ground_normal.GetZ();
	}
	if (out_ground_state != nullptr)
		*out_ground_state = (uint32_t)character->GetGroundState();
	if (out_is_supported != nullptr)
		*out_is_supported = character->IsSupported() ? 1u : 0u;
}

void bjolt_rigid_character_set_pose(BJoltWorld *world, uint32_t character_id,
	float pos_x, float pos_y, float pos_z,
	float rot_x, float rot_y, float rot_z, float rot_w)
{
	Character *character = bjolt_rigid_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->SetPositionAndRotation(RVec3(pos_x, pos_y, pos_z), Quat(rot_x, rot_y, rot_z, rot_w));
}

uint32_t bjolt_rigid_character_body(BJoltWorld *world, uint32_t character_id)
{
	Character *character = bjolt_rigid_character_at(world, character_id);
	if (character == nullptr)
		return 0;
	return character->GetBodyID().GetIndexAndSequenceNumber();
}

void bjolt_rigid_character_set_layer(BJoltWorld *world, uint32_t character_id, uint16_t object_layer)
{
	Character *character = bjolt_rigid_character_at(world, character_id);
	if (character == nullptr)
		return;
	character->SetLayer((ObjectLayer)object_layer);
}

bool bjolt_rigid_character_stance(BJoltWorld *world, uint32_t character_id,
	float capsule_half_height, float capsule_radius, float max_penetration)
{
	Character *character = bjolt_rigid_character_at(world, character_id);
	if (character == nullptr)
		return false;
	RefConst<Shape> capsule = new CapsuleShape(0.5f * (2.0f * capsule_half_height + 2.0f * capsule_radius) - capsule_radius, capsule_radius);
	return character->SetShape(capsule, max_penetration);
}

void bjolt_rigid_character_ground(BJoltWorld *world, uint32_t character_id,
	float *out_ground_position, float *out_ground_normal, float *out_ground_velocity,
	uint32_t *out_ground_body, uint64_t *out_ground_user_data)
{
	Character *character = bjolt_rigid_character_at(world, character_id);
	if (character == nullptr)
		return;
	RVec3 ground_position = character->GetGroundPosition();
	Vec3 ground_normal = character->GetGroundNormal();
	Vec3 ground_velocity = character->GetGroundVelocity();
	if (out_ground_position != nullptr) {
		out_ground_position[0] = (float)ground_position.GetX();
		out_ground_position[1] = (float)ground_position.GetY();
		out_ground_position[2] = (float)ground_position.GetZ();
	}
	if (out_ground_normal != nullptr) {
		out_ground_normal[0] = ground_normal.GetX();
		out_ground_normal[1] = ground_normal.GetY();
		out_ground_normal[2] = ground_normal.GetZ();
	}
	if (out_ground_velocity != nullptr) {
		out_ground_velocity[0] = ground_velocity.GetX();
		out_ground_velocity[1] = ground_velocity.GetY();
		out_ground_velocity[2] = ground_velocity.GetZ();
	}
	if (out_ground_body != nullptr)
		*out_ground_body = character->GetGroundBodyID().GetIndexAndSequenceNumber();
	if (out_ground_user_data != nullptr)
		*out_ground_user_data = character->GetGroundUserData();
}

// One rigid body from box/sphere/capsule parts: offsets + quaternions pose
// each part relative to the body origin. Layout matches CompoundPartFfi.
struct BJoltCompoundPart {
	uint32_t part_kind;
	float part_half_x, part_half_y, part_half_z;
	float offset_x, offset_y, offset_z;
	float rot_x, rot_y, rot_z, rot_w;
};

uint32_t bjolt_create_compound(BJoltWorld *world,
	const BJoltCompoundPart *parts, uint32_t part_count,
	float pos_x, float pos_y, float pos_z, uint16_t object_layer, uint8_t motion,
	float density_kg_per_m3, float gravity_factor)
{
	if (!bjolt_bodies_available(world))
		return 0;
	if (parts == nullptr || part_count == 0 || part_count > 16)
		return 0;
	StaticCompoundShapeSettings compound_settings;
	for (uint32_t part_index = 0; part_index < part_count; ++part_index)
	{
		const BJoltCompoundPart &part = parts[part_index];
		Ref<ConvexShapeSettings> part_settings;
		if (part.part_kind == 1)
			part_settings = new SphereShapeSettings(part.part_half_x);
		else if (part.part_kind == 2)
			part_settings = new CapsuleShapeSettings(part.part_half_x, part.part_half_y);
		else
			part_settings = new BoxShapeSettings(Vec3(part.part_half_x, part.part_half_y, part.part_half_z));
		part_settings->mDensity = density_kg_per_m3;
		part_settings->SetEmbedded();
		compound_settings.AddShape(
			Vec3(part.offset_x, part.offset_y, part.offset_z),
			Quat(part.rot_x, part.rot_y, part.rot_z, part.rot_w),
			part_settings);
	}
	Shape::ShapeResult shape_result = compound_settings.Create();
	if (shape_result.HasError())
		return 0;
	Ref<Shape> compound_shape = shape_result.Get();
	EMotionType motion_type = motion == 0 ? EMotionType::Static :
		(motion == 1 ? EMotionType::Kinematic : EMotionType::Dynamic);
	BodyCreationSettings settings(compound_shape, RVec3(pos_x, pos_y, pos_z), Quat::sIdentity(), motion_type, object_layer);
	settings.mGravityFactor = gravity_factor;
	settings.mAllowSleeping = (motion_type != EMotionType::Kinematic);
	BodyID body_id = world->physics_system->GetBodyInterface().CreateAndAddBody(settings,
		motion_type == EMotionType::Static ? EActivation::DontActivate : EActivation::Activate);
	return body_id.GetIndexAndSequenceNumber();
}

// Shrink-wrapped lump from a point soup: interior points fine, dents filled.
// Convex only, so dynamics can tumble it. 0 on empty input or cook failure.
uint32_t bjolt_create_hull(BJoltWorld *world,
	const float *points_xyz, uint32_t point_count,
	float pos_x, float pos_y, float pos_z, uint16_t object_layer, uint8_t motion,
	float density_kg_per_m3, float gravity_factor)
{
	if (!bjolt_bodies_available(world))
		return 0;
	if (points_xyz == nullptr || point_count == 0)
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	Array<Vec3> hull_points;
	hull_points.reserve(point_count);
	for (uint32_t point_index = 0; point_index < point_count; ++point_index)
		hull_points.push_back(Vec3(
			points_xyz[point_index * 3], points_xyz[point_index * 3 + 1], points_xyz[point_index * 3 + 2]));
	ConvexHullShapeSettings hull_settings(hull_points);
	hull_settings.mDensity = density_kg_per_m3;
	hull_settings.SetEmbedded();
	Shape::ShapeResult shape_result = hull_settings.Create();
	if (shape_result.HasError())
		return 0;
	Ref<Shape> hull_shape = shape_result.Get();
	EMotionType motion_type = motion == 0 ? EMotionType::Static :
		(motion == 1 ? EMotionType::Kinematic : EMotionType::Dynamic);
	BodyCreationSettings settings(hull_shape, RVec3(pos_x, pos_y, pos_z), Quat::sIdentity(), motion_type, object_layer);
	settings.mGravityFactor = gravity_factor;
	settings.mAllowSleeping = (motion_type != EMotionType::Kinematic);
	BodyID body_id = body_interface.CreateAndAddBody(settings,
		motion_type == EMotionType::Static ? EActivation::DontActivate : EActivation::Activate);
	return body_id.GetIndexAndSequenceNumber();
}

// Exact-triangle static scenery: keeps every dent and hole, so dynamics
// only. Statics never integrate, so motion/density/gravity are ignored.
// 0 on empty input or cook failure.
uint32_t bjolt_create_mesh(BJoltWorld *world,
	const float *vertices_xyz, uint32_t vertex_count,
	const uint32_t *triangle_indices, uint32_t triangle_count,
	float pos_x, float pos_y, float pos_z, uint16_t object_layer)
{
	if (!bjolt_bodies_available(world))
		return 0;
	if (vertices_xyz == nullptr || vertex_count == 0
		|| triangle_indices == nullptr || triangle_count == 0)
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	VertexList mesh_vertices;
	mesh_vertices.reserve(vertex_count);
	for (uint32_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index)
		mesh_vertices.push_back(Float3(
			vertices_xyz[vertex_index * 3], vertices_xyz[vertex_index * 3 + 1], vertices_xyz[vertex_index * 3 + 2]));
	IndexedTriangleList mesh_triangles;
	mesh_triangles.reserve(triangle_count);
	for (uint32_t triangle_index = 0; triangle_index < triangle_count; ++triangle_index)
		mesh_triangles.push_back(IndexedTriangle(
			triangle_indices[triangle_index * 3], triangle_indices[triangle_index * 3 + 1], triangle_indices[triangle_index * 3 + 2], 0));
	MeshShapeSettings mesh_settings(mesh_vertices, mesh_triangles);
	mesh_settings.SetEmbedded();
	Shape::ShapeResult shape_result = mesh_settings.Create();
	if (shape_result.HasError())
		return 0;
	Ref<Shape> mesh_shape = shape_result.Get();
	BodyCreationSettings settings(mesh_shape, RVec3(pos_x, pos_y, pos_z), Quat::sIdentity(), EMotionType::Static, object_layer);
	BodyID body_id = body_interface.CreateAndAddBody(settings, EActivation::DontActivate);
	return body_id.GetIndexAndSequenceNumber();
}

// Terrain grid: height per cell, one ground level per (x, z). Grid lookup
// instead of a tree walk, so big open ground stays cheap. Static only.
// Offset centers the grid under the spawn; scale maps cells to meters.
// 0 on empty input, non-square counts, or cook failure.
uint32_t bjolt_create_heightfield(BJoltWorld *world,
	const float *sample_heights, uint32_t sample_count, uint32_t grid_width,
	float cell_size, float pos_x, float pos_y, float pos_z, uint16_t object_layer)
{
	if (!bjolt_bodies_available(world))
		return 0;
	if (sample_heights == nullptr || sample_count == 0 || grid_width < 2
		|| sample_count != grid_width * grid_width || cell_size <= 0.0f)
		return 0;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	float grid_extent = (float)(grid_width - 1) * cell_size;
	Vec3 field_offset(pos_x - grid_extent * 0.5f, pos_y, pos_z - grid_extent * 0.5f);
	Vec3 field_scale(cell_size, 1.0f, cell_size);
	HeightFieldShapeSettings field_settings(sample_heights, field_offset, field_scale, grid_width);
	field_settings.SetEmbedded();
	Shape::ShapeResult shape_result = field_settings.Create();
	if (shape_result.HasError())
		return 0;
	Ref<Shape> field_shape = shape_result.Get();
	BodyCreationSettings settings(field_shape, RVec3(0, 0, 0), Quat::sIdentity(), EMotionType::Static, object_layer);
	BodyID body_id = body_interface.CreateAndAddBody(settings, EActivation::DontActivate);
	return body_id.GetIndexAndSequenceNumber();
}
// Stops two bodies colliding: one shared GroupFilterTable with both bodies
// in subgroups 0/1 and that pair disabled. Simple per-pair tables, not a
// global ragdoll group: enough for a limb chain.
void bjolt_bodies_no_collide(BJoltWorld *world, uint32_t body_a_raw, uint32_t body_b_raw)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	BodyID body_a(body_a_raw);
	BodyID body_b(body_b_raw);
	Ref<GroupFilterTable> pair_filter = new GroupFilterTable(2);
	pair_filter->DisableCollision(0, 1);
	static uint32_t next_group_id = 1;
	CollisionGroup::GroupID pair_group = next_group_id++;
	body_interface.SetCollisionGroup(body_a, CollisionGroup(pair_filter, pair_group, 0));
	body_interface.SetCollisionGroup(body_b, CollisionGroup(pair_filter, pair_group, 1));
}

// Drains one contact queue into flat body-id pairs and resets the count.
static uint32_t bjolt_drain_pairs(uint32_t *stored_a, uint32_t *stored_b, uint32_t &stored_count,
	uint32_t *out_a, uint32_t *out_b, uint32_t pair_capacity)
{
	uint32_t kept = stored_count < pair_capacity ? stored_count : pair_capacity;
	for (uint32_t pair_index = 0; pair_index < kept; ++pair_index)
	{
		out_a[pair_index] = stored_a[pair_index];
		out_b[pair_index] = stored_b[pair_index];
	}
	stored_count = 0;
	return kept;
}

uint32_t bjolt_drain_contact_added(BJoltWorld *world, uint32_t *out_a, uint32_t *out_b, uint32_t pair_capacity)
{
	if (world->contact_listener == nullptr || out_a == nullptr || out_b == nullptr || pair_capacity == 0)
		return 0;
	return bjolt_drain_pairs(world->contact_listener->added_a, world->contact_listener->added_b,
		world->contact_listener->added_count, out_a, out_b, pair_capacity);
}

uint32_t bjolt_drain_contact_removed(BJoltWorld *world, uint32_t *out_a, uint32_t *out_b, uint32_t pair_capacity)
{
	if (world->contact_listener == nullptr || out_a == nullptr || out_b == nullptr || pair_capacity == 0)
		return 0;
	return bjolt_drain_pairs(world->contact_listener->removed_a, world->contact_listener->removed_b,
		world->contact_listener->removed_count, out_a, out_b, pair_capacity);
}

// Drains slept/woke body ids and resets the counts. Single ids, not pairs.
uint32_t bjolt_drain_slept(BJoltWorld *world, uint32_t *out_ids, uint32_t id_capacity)
{
	if (world->activation_listener == nullptr || out_ids == nullptr || id_capacity == 0)
		return 0;
	uint32_t kept = world->activation_listener->slept_count < id_capacity ?
		world->activation_listener->slept_count : id_capacity;
	for (uint32_t id_index = 0; id_index < kept; ++id_index)
		out_ids[id_index] = world->activation_listener->slept_ids[id_index];
	world->activation_listener->slept_count = 0;
	return kept;
}

uint32_t bjolt_drain_woke(BJoltWorld *world, uint32_t *out_ids, uint32_t id_capacity)
{
	if (world->activation_listener == nullptr || out_ids == nullptr || id_capacity == 0)
		return 0;
	uint32_t kept = world->activation_listener->woke_count < id_capacity ?
		world->activation_listener->woke_count : id_capacity;
	for (uint32_t id_index = 0; id_index < kept; ++id_index)
		out_ids[id_index] = world->activation_listener->woke_ids[id_index];
	world->activation_listener->woke_count = 0;
	return kept;
}

// Marks a body as a sensor: overlaps report through the contact listener
// but apply no collision response (trigger volumes, pickups, checkpoints).
void bjolt_body_set_sensor(BJoltWorld *world, uint32_t body_raw, bool is_sensor)
{
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	body_interface.SetIsSensor(BodyID(body_raw), is_sensor);
}

// Linear + angular damping on a live body (0 = Jolt default glide, higher
// bleeds velocity faster). Ragdolls settle visibly instead of crawling.
void bjolt_body_set_damping(BJoltWorld *world, uint32_t body_raw, float linear_damping, float angular_damping)
{
	BodyLockWrite body_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	if (body_lock.Succeeded())
	{
		Body &body = body_lock.GetBody();
		if (!body.IsStatic())
		{
			body.GetMotionProperties()->SetLinearDamping(linear_damping);
			body.GetMotionProperties()->SetAngularDamping(angular_damping);
		}
	}
}

// Live density: rescale the body's mass properties to density × shape
// volume (inertia scales with the mass). No-op on bad or static ids.
void bjolt_body_set_density(BJoltWorld *world, uint32_t body_raw, float density_kg_per_m3)
{
	BodyLockWrite body_lock(world->physics_system->GetBodyLockInterface(), BodyID(body_raw));
	if (body_lock.Succeeded())
	{
		Body &body = body_lock.GetBody();
		if (!body.IsStatic())
		{
			MassProperties mass_properties = body.GetShape()->GetMassProperties();
			mass_properties.ScaleToMass(density_kg_per_m3 * body.GetShape()->GetVolume());
			body.GetMotionProperties()->SetMassProperties(
				body.GetMotionProperties()->GetAllowedDOFs(), mass_properties);
		}
	}
}

// Ragdoll creation through Jolt's RagdollSettings: skeleton + per-part
// bodies with mass/inertia stabilization, constraint priorities, and
// parent-child no-collide handled inside Jolt instead of by hand.
// Parts are added parents-first (index < child); the joint index, part
// index, and collision subgroup are all the same number.
struct BJoltRagdollBuild {
	Ref<RagdollSettings> settings;
	Ref<Skeleton> skeleton;
};

BJoltRagdollBuild *bjolt_ragdoll_build_create()
{
	BJoltRagdollBuild *build = new BJoltRagdollBuild();
	build->skeleton = new Skeleton();
	build->settings = new RagdollSettings();
	build->settings->mSkeleton = build->skeleton;
	return build;
}

// Adds one ragdoll part. `shape_kind`: 0 = capsule (dims = cylinder
// half height, radius), 1 = box (dims = half extents), 2 = sphere
// (dims.x = radius). Position + quaternion pose the body origin.
// Returns the part index, or -1 when the shape is invalid.
int bjolt_ragdoll_build_add_part(BJoltRagdollBuild *build, int parent_index,
 uint8_t shape_kind, float dim_x, float dim_y, float dim_z,
 float pos_x, float pos_y, float pos_z,
 float rot_x, float rot_y, float rot_z, float rot_w,
 uint16_t object_layer, float density_kg_per_m3, uint8_t motion_type)
{
 if (build == nullptr)
 return -1;
 Shape::ShapeResult shape_result;
 switch (shape_kind)
 {
 case 1:
 {
 BoxShapeSettings box_settings(Vec3(dim_x, dim_y, dim_z));
 box_settings.mDensity = density_kg_per_m3;
 box_settings.SetEmbedded();
 shape_result = box_settings.Create();
 break;
 }
 case 2:
 {
 SphereShapeSettings sphere_settings(dim_x);
 sphere_settings.mDensity = density_kg_per_m3;
 sphere_settings.SetEmbedded();
 shape_result = sphere_settings.Create();
 break;
 }
 default:
 {
 CapsuleShapeSettings capsule_settings(dim_x, dim_y);
 capsule_settings.mDensity = density_kg_per_m3;
 capsule_settings.SetEmbedded();
 shape_result = capsule_settings.Create();
 break;
 }
 }
 if (shape_result.HasError())
 return -1;
 char joint_name[32];
 snprintf(joint_name, sizeof(joint_name), "part_%d", build->skeleton->GetJointCount());
 build->skeleton->AddJoint(joint_name, parent_index);
 RagdollSettings::Part part;
 part.SetShape(shape_result.Get());
 part.mPosition = RVec3(pos_x, pos_y, pos_z);
 part.mRotation = Quat(rot_x, rot_y, rot_z, rot_w);
 // 0 = static, 1 = kinematic (hitboxes), anything else = dynamic.
 part.mMotionType = motion_type == 0 ? EMotionType::Static :
 (motion_type == 1 ? EMotionType::Kinematic : EMotionType::Dynamic);
 part.mObjectLayer = object_layer;
 build->settings->mParts.push_back(part);
 return build->skeleton->GetJointCount() - 1;
}

// Hinge limit between a part and its parent: rotation about the hinge axis
// within [limits_min, limits_max] about `anchor` (world space). Per-side
// frames (`1` from the parent's seated rotation, `2` from the child's) so
// the seated pose reads zero — identical axes would pre-bend every joint
// whose bodies differ. Same conventions as bjolt_create_hinge_constraint.
bool bjolt_ragdoll_build_set_hinge(BJoltRagdollBuild *build, int part_index,
 float anchor_x, float anchor_y, float anchor_z,
 float hinge_axis1_x, float hinge_axis1_y, float hinge_axis1_z,
 float normal_axis1_x, float normal_axis1_y, float normal_axis1_z,
 float hinge_axis2_x, float hinge_axis2_y, float hinge_axis2_z,
 float normal_axis2_x, float normal_axis2_y, float normal_axis2_z,
 float limits_min, float limits_max)
{
 if (build == nullptr || part_index < 0 || part_index >= (int)build->settings->mParts.size())
 return false;
 Ref<HingeConstraintSettings> hinge = new HingeConstraintSettings();
 hinge->mSpace = EConstraintSpace::WorldSpace;
 hinge->mPoint1 = RVec3(anchor_x, anchor_y, anchor_z);
 hinge->mPoint2 = RVec3(anchor_x, anchor_y, anchor_z);
 hinge->mHingeAxis1 = Vec3(hinge_axis1_x, hinge_axis1_y, hinge_axis1_z);
 hinge->mNormalAxis1 = Vec3(normal_axis1_x, normal_axis1_y, normal_axis1_z);
 hinge->mHingeAxis2 = Vec3(hinge_axis2_x, hinge_axis2_y, hinge_axis2_z);
 hinge->mNormalAxis2 = Vec3(normal_axis2_x, normal_axis2_y, normal_axis2_z);
	hinge->mLimitsMin = limits_min;
	hinge->mLimitsMax = limits_max;
	// Motors start off: the drive call arms one axis at a time on demand.
	hinge->mMotorSettings = MotorSettings(ESpringMode::FrequencyAndDamping, 8.0f, 1.0f);
	hinge->mMotorSettings.SetTorqueLimit(1.0e6f);
	build->settings->mParts[part_index].mToParent = hinge;
	return true;
}

// Swing-twist limit between a part and its parent: cone swing about the
// twist axis plus bounded twist, about `anchor` (world space). Per-side
// frames like the hinge above. Same conventions as
// bjolt_create_swing_twist_constraint.
bool bjolt_ragdoll_build_set_swing_twist(BJoltRagdollBuild *build, int part_index,
 float anchor_x, float anchor_y, float anchor_z,
 float twist_axis1_x, float twist_axis1_y, float twist_axis1_z,
 float plane_axis1_x, float plane_axis1_y, float plane_axis1_z,
 float twist_axis2_x, float twist_axis2_y, float twist_axis2_z,
 float plane_axis2_x, float plane_axis2_y, float plane_axis2_z,
 float normal_half_cone_angle, float plane_half_cone_angle,
 float twist_min_angle, float twist_max_angle)
{
 if (build == nullptr || part_index < 0 || part_index >= (int)build->settings->mParts.size())
 return false;
 Ref<SwingTwistConstraintSettings> swing_twist = new SwingTwistConstraintSettings();
 swing_twist->mSpace = EConstraintSpace::WorldSpace;
 swing_twist->mPosition1 = RVec3(anchor_x, anchor_y, anchor_z);
 swing_twist->mPosition2 = RVec3(anchor_x, anchor_y, anchor_z);
 swing_twist->mTwistAxis1 = Vec3(twist_axis1_x, twist_axis1_y, twist_axis1_z);
 swing_twist->mPlaneAxis1 = Vec3(plane_axis1_x, plane_axis1_y, plane_axis1_z);
 swing_twist->mTwistAxis2 = Vec3(twist_axis2_x, twist_axis2_y, twist_axis2_z);
 swing_twist->mPlaneAxis2 = Vec3(plane_axis2_x, plane_axis2_y, plane_axis2_z);
	swing_twist->mNormalHalfConeAngle = normal_half_cone_angle;
	swing_twist->mPlaneHalfConeAngle = plane_half_cone_angle;
	swing_twist->mTwistMinAngle = twist_min_angle;
	swing_twist->mTwistMaxAngle = twist_max_angle;
	// Motors start off: the drive call arms swing or twist on demand.
	swing_twist->mSwingMotorSettings = MotorSettings(ESpringMode::FrequencyAndDamping, 8.0f, 1.0f);
	swing_twist->mSwingMotorSettings.SetTorqueLimit(1.0e6f);
	swing_twist->mTwistMotorSettings = MotorSettings(ESpringMode::FrequencyAndDamping, 8.0f, 1.0f);
	swing_twist->mTwistMotorSettings.SetTorqueLimit(1.0e6f);
	build->settings->mParts[part_index].mToParent = swing_twist;
	return true;
}

// Mass-ratio clamp + parent-inertia boost, in place. Must run after all
// parts are added, before create. False only on inertia-decomposition
// failure.
bool bjolt_ragdoll_build_stabilize(BJoltRagdollBuild *build)
{
	if (build == nullptr)
		return false;
	return build->settings->Stabilize();
}

// Root-biased constraint priorities (leaves solve first) + one shared
// parent-child no-collide filter. Both require correctly ordered joints,
// which parents-first insertion guarantees.
void bjolt_ragdoll_build_finalize(BJoltRagdollBuild *build)
{
	if (build == nullptr)
		return;
	build->settings->CalculateConstraintPriorities();
	build->settings->DisableParentChildCollisions();
}

// Creates bodies + constraints from stabilized settings and adds them to
// the system in one shot. `group_id` must be unique per ragdoll in the
// system. Returns the 1-based registry id, or 0 on failure. The `Ref` in the
// registry owns the ragdoll; Rust never sees a pointer.
uint32_t bjolt_ragdoll_create(BJoltWorld *world, BJoltRagdollBuild *build,
	uint32_t group_id, uint64_t user_data)
{
	if (world == nullptr || build == nullptr)
		return 0;
	uint32_t part_count = (uint32_t)build->settings->mParts.size();
	uint32_t live_bodies = world->physics_system->GetNumBodies();
	if (live_bodies + part_count > world->max_bodies)
	{
		fprintf(stderr,
			"[bevy_jolt] out of Jolt bodies creating ragdoll (%u parts): %u live of %u max. "
			"Raise the world budget (max bodies) and retry.\n",
			part_count, live_bodies, world->max_bodies);
		return 0;
	}
	Ragdoll *ragdoll = build->settings->CreateRagdoll(group_id, user_data,
		world->physics_system);
	if (ragdoll == nullptr)
		return 0;
	ragdoll->AddToPhysicsSystem(EActivation::Activate);
	world->ragdoll_registry.push_back(ragdoll);
	return (uint32_t)world->ragdoll_registry.size();
}

static Ragdoll *bjolt_ragdoll_lookup(BJoltWorld *world, uint32_t ragdoll_id)
{
	if (world == nullptr || ragdoll_id == 0 || ragdoll_id > world->ragdoll_registry.size())
		return nullptr;
	Ref<Ragdoll> &slot = world->ragdoll_registry[ragdoll_id - 1];
	return slot.GetPtr();
}

uint32_t bjolt_ragdoll_body_count(BJoltWorld *world, uint32_t ragdoll_id)
{
	Ragdoll *ragdoll = bjolt_ragdoll_lookup(world, ragdoll_id);
	if (ragdoll == nullptr)
		return 0;
	return (uint32_t)ragdoll->GetBodyCount();
}

// Writes body ids (Jolt index+sequence scheme, same as every other create)
// in part order into `out_ids`. Returns ids written.
uint32_t bjolt_ragdoll_body_ids(BJoltWorld *world, uint32_t ragdoll_id, uint32_t *out_ids, uint32_t id_capacity)
{
	Ragdoll *ragdoll = bjolt_ragdoll_lookup(world, ragdoll_id);
	if (ragdoll == nullptr || out_ids == nullptr || id_capacity == 0)
		return 0;
	uint32_t body_count = (uint32_t)ragdoll->GetBodyCount();
	uint32_t kept = body_count < id_capacity ? body_count : id_capacity;
	for (uint32_t body_index = 0; body_index < kept; ++body_index)
		out_ids[body_index] = ragdoll->GetBodyID(body_index).GetIndexAndSequenceNumber();
	return kept;
}

// Flips every body in the ragdoll to one motion: 0 static, 1 kinematic,
// 2 dynamic. Same wake rules as bjolt_set_motion_type. Kinematic bodies
// follow bones (hitbox mode); dynamic bodies simulate (ragdoll mode).
void bjolt_ragdoll_set_motion(BJoltWorld *world, uint32_t ragdoll_id, uint8_t motion_type)
{
	Ragdoll *ragdoll = bjolt_ragdoll_lookup(world, ragdoll_id);
	if (ragdoll == nullptr)
		return;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	EMotionType jolt_motion = motion_type == 0 ? EMotionType::Static :
		(motion_type == 1 ? EMotionType::Kinematic : EMotionType::Dynamic);
	EActivation wake = jolt_motion == EMotionType::Static ?
		EActivation::DontActivate : EActivation::Activate;
	for (const BodyID &body_id : ragdoll->GetBodyIDs())
		body_interface.SetMotionType(body_id, jolt_motion, wake);
}
// Moves every part body to another object layer (re-inserts in the
// broadphase, no velocity change). Lets hitbox-follow parts ride a quiet
// team and simulated parts join one that meets the world.
void bjolt_ragdoll_set_layer(BJoltWorld *world, uint32_t ragdoll_id, uint16_t object_layer)
{
	Ragdoll *ragdoll = bjolt_ragdoll_lookup(world, ragdoll_id);
	if (ragdoll == nullptr)
		return;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	for (const BodyID &body_id : ragdoll->GetBodyIDs())
		body_interface.SetObjectLayer(body_id, object_layer);
}

// Removes bodies + constraints from the system and releases the registry
// slot (nulled, never reused). Never mix with per-body
// bjolt_body_remove_destroy on these ids.
void bjolt_ragdoll_destroy(BJoltWorld *world, uint32_t ragdoll_id)
{
	Ragdoll *ragdoll = bjolt_ragdoll_lookup(world, ragdoll_id);
	if (ragdoll == nullptr)
		return;
	ragdoll->RemoveFromPhysicsSystem();
	world->ragdoll_registry[ragdoll_id - 1] = nullptr;
}

// Velocity motor on the joint feeding `part_index` (0 = root, no joint).
// Motors were armed at build with a fixed spring + torque cap; this only
// picks the axis and retargets speed. `axis`: hinge = 0; swing-twist: 0 =
// twist, 1 = swing; anything else stops both motors. Speed 0 stops motion
// but holds the motor on (brake); use `bjolt_ragdoll_motor_off` to release.
// Returns false on a bad id, root part, out-of-range part, or wrong type.
bool bjolt_ragdoll_drive(BJoltWorld *world, uint32_t ragdoll_id, uint32_t part_index, uint8_t axis, float target_velocity)
{
	Ragdoll *ragdoll = bjolt_ragdoll_lookup(world, ragdoll_id);
	if (ragdoll == nullptr || part_index == 0 || (size_t)part_index >= ragdoll->GetBodyIDs().size())
		return false;
	if ((size_t)(part_index - 1) >= ragdoll->GetConstraintCount())
		return false;
	TwoBodyConstraint *constraint = ragdoll->GetConstraint((int)(part_index - 1));
	if (constraint == nullptr)
		return false;
	BodyInterface &body_interface = world->physics_system->GetBodyInterface();
	switch (constraint->GetSubType())
	{
	case EConstraintSubType::Hinge:
	{
		HingeConstraint *hinge = static_cast<HingeConstraint *>(constraint);
		hinge->SetMotorState(EMotorState::Velocity);
		hinge->SetTargetAngularVelocity(target_velocity);
		break;
	}
	case EConstraintSubType::SwingTwist:
	{
		SwingTwistConstraint *swing_twist = static_cast<SwingTwistConstraint *>(constraint);
		// Twist spins about constraint X; swing sweeps about constraint
		// Y/Z (see `bjolt_constraint_drive_swing_twist`).
		Vec3 swing_target = Vec3::sZero();
		Vec3 twist_target = Vec3::sZero();
		if (axis == 0)
		{
			swing_twist->SetSwingMotorState(EMotorState::Off);
			swing_twist->SetTwistMotorState(EMotorState::Velocity);
			twist_target.SetX(target_velocity);
		}
		else if (axis == 1)
		{
			swing_twist->SetSwingMotorState(EMotorState::Velocity);
			swing_twist->SetTwistMotorState(EMotorState::Off);
			swing_target.SetY(target_velocity);
		}
		else
		{
			swing_twist->SetSwingMotorState(EMotorState::Off);
			swing_twist->SetTwistMotorState(EMotorState::Off);
			break;
		}
		swing_twist->SetTargetAngularVelocityCS(swing_target + twist_target);
		break;
	}
	default:
		return false;
	}
	body_interface.ActivateBody(ragdoll->GetBodyID(part_index));
	return true;
}

// Releases both motors on the joint feeding `part_index`, so the limb hangs
// on limits alone. Returns false on a bad id, root part, or wrong type.
bool bjolt_ragdoll_motor_off(BJoltWorld *world, uint32_t ragdoll_id, uint32_t part_index)
{
	Ragdoll *ragdoll = bjolt_ragdoll_lookup(world, ragdoll_id);
	if (ragdoll == nullptr || part_index == 0 || (size_t)part_index >= ragdoll->GetBodyIDs().size())
		return false;
	if ((size_t)(part_index - 1) >= ragdoll->GetConstraintCount())
		return false;
	TwoBodyConstraint *constraint = ragdoll->GetConstraint((int)(part_index - 1));
	if (constraint == nullptr)
		return false;
	switch (constraint->GetSubType())
	{
	case EConstraintSubType::Hinge:
		static_cast<HingeConstraint *>(constraint)->SetMotorState(EMotorState::Off);
		break;
	case EConstraintSubType::SwingTwist:
	{
		SwingTwistConstraint *swing_twist = static_cast<SwingTwistConstraint *>(constraint);
		swing_twist->SetSwingMotorState(EMotorState::Off);
		swing_twist->SetTwistMotorState(EMotorState::Off);
		break;
	}
	default:
		return false;
	}
	return true;
}

void bjolt_ragdoll_build_destroy(BJoltRagdollBuild *build)
{
	delete build;
}

} // extern "C"
