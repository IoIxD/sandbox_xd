use std::time::SystemTime;

use nalgebra::Vector2;
use parking_lot::Mutex;
use rapier2d::prelude::*;

pub struct State {
    rigid_body_set: Mutex<RigidBodySet>,
    collider_set: Mutex<ColliderSet>,
    gravity: Vector2<Real>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: Mutex<PhysicsPipeline>,
    island_manager: Mutex<IslandManager>,
    broad_phase: Mutex<DefaultBroadPhase>,
    narrow_phase: Mutex<NarrowPhase>,
    impulse_joint_set: Mutex<ImpulseJointSet>,
    multibody_joint_set: Mutex<MultibodyJointSet>,
    ccd_solver: Mutex<CCDSolver>,
    query_pipeline: Mutex<QueryPipeline>,

    box_left: Mutex<ColliderHandle>,
    box_right: Mutex<ColliderHandle>,
    box_top: Mutex<ColliderHandle>,
    box_bottom: Mutex<ColliderHandle>,
}

impl State {
    pub fn new() -> Self {
        let rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        /* Create the ground. */
        let box_bottom = ColliderBuilder::cuboid(30020.0, 10000.0)
            .translation(vector![0.0, 139.0])
            .build();
        let box_bottom = collider_set.insert(box_bottom);
        let box_left = ColliderBuilder::cuboid(10000.0, 20040.0)
            .translation(vector![-99.0, 0.0])
            .build();
        let box_left = collider_set.insert(box_left);
        let box_top = ColliderBuilder::cuboid(30020.0, 10000.0)
            .translation(vector![0.0, 0.0])
            .build();
        let box_top = collider_set.insert(box_top);
        let box_right = ColliderBuilder::cuboid(10000.0, 20040.0)
            .translation(vector![319.0, 0.0])
            .build();
        let box_right = collider_set.insert(box_right);

        /* Create other structures necessary for the simulation. */
        let gravity = vector![0.0, 9.81];
        let integration_parameters = IntegrationParameters::default();
        let physics_pipeline = PhysicsPipeline::new();
        let island_manager = IslandManager::new();
        let broad_phase = DefaultBroadPhase::new();
        let narrow_phase = NarrowPhase::new();
        let impulse_joint_set = ImpulseJointSet::new();
        let multibody_joint_set = MultibodyJointSet::new();
        let ccd_solver = CCDSolver::new();
        let query_pipeline = QueryPipeline::new();
        let physics_hooks = ();
        let event_handler = ();

        Self {
            rigid_body_set: Mutex::new(rigid_body_set),
            collider_set: Mutex::new(collider_set),
            gravity: gravity,
            integration_parameters: integration_parameters,
            physics_pipeline: Mutex::new(physics_pipeline),
            island_manager: Mutex::new(island_manager),
            broad_phase: Mutex::new(broad_phase),
            narrow_phase: Mutex::new(narrow_phase),
            impulse_joint_set: Mutex::new(impulse_joint_set),
            multibody_joint_set: Mutex::new(multibody_joint_set),
            ccd_solver: Mutex::new(ccd_solver),
            query_pipeline: Mutex::new(query_pipeline),
            box_left: Mutex::new(box_left),
            box_right: Mutex::new(box_right),
            box_top: Mutex::new(box_top),
            box_bottom: Mutex::new(box_bottom),
        }
    }

    pub fn step(&self) {
        let mut physics_pipeline = self.physics_pipeline.lock();
        let gravity = self.gravity;
        let integration_parameters = self.integration_parameters;
        let mut island_manager = self.island_manager.lock();
        let mut broad_phase = self.broad_phase.lock();
        let mut narrow_phase = self.narrow_phase.lock();
        let mut rigid_body_set = self.rigid_body_set.lock();
        let mut collider_set = self.collider_set.lock();
        let mut impulse_joint_set = self.impulse_joint_set.lock();
        let mut multibody_joint_set = self.multibody_joint_set.lock();
        let mut ccd_solver = self.ccd_solver.lock();
        let mut query_pipeline = self.query_pipeline.lock();

        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut *broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            Some(&mut query_pipeline),
            &(),
            &(),
        );
    }

    pub fn insert_particle(&self, x: f32, y: f32) {
        let mut rigid_body_set = self.rigid_body_set.lock();
        let mut collider_set = self.collider_set.lock();

        let mut rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![x, y])
            .build();

        rigid_body.wake_up(true);
        let mut collider = ColliderBuilder::cuboid(1.0, 1.0).restitution(-1.0).build();

        collider.user_data = SystemTime::UNIX_EPOCH.elapsed().unwrap().as_nanos();

        let ball_body_handle = rigid_body_set.insert(rigid_body);
        collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);
    }

    pub fn resize(&self, x: f32, y: f32, width: f32, height: f32) {
        let mut collider_set = self.collider_set.lock();
        {
            let mut box_top = collider_set.get_mut(*self.box_top.lock()).unwrap();
            box_top.set_translation(vector![0.0 + x, -9999.0 + y]);
        }
        {
            let mut box_left = collider_set.get_mut(*self.box_left.lock()).unwrap();
            box_left.set_translation(vector![-9999.0 + x, 0.0 + y]);
        }
        {
            let mut box_bottom = collider_set.get_mut(*self.box_bottom.lock()).unwrap();
            box_bottom.set_translation(vector![0.0 + x, height + 9999.0 + y]);
        };
        {
            let mut box_right = collider_set.get_mut(*self.box_right.lock()).unwrap();
            box_right.set_translation(vector![width + 9999.0 + x, 0.0 + y]);
        }
    }

    pub fn for_each_cube(&self, mut func: impl FnMut(f32, f32, f32, f32, u128)) {
        let collider_set = self.collider_set.lock().clone();

        for (handle, body) in collider_set.iter() {
            let pos = body.translation();
            match body.shape().as_typed_shape() {
                TypedShape::Ball(b) => todo!(),
                TypedShape::Cuboid(c) => {
                    let half_extents = c.half_extents;
                    func(
                        pos.x,
                        pos.y,
                        half_extents.x * 2.0,
                        half_extents.y * 2.0,
                        body.user_data,
                    );
                }
                TypedShape::Capsule(_) => todo!(),
                TypedShape::Segment(_) => todo!(),
                TypedShape::Triangle(_) => todo!(),
                TypedShape::TriMesh(_) => todo!(),
                TypedShape::Polyline(_) => todo!(),
                TypedShape::HalfSpace(_) => todo!(),
                TypedShape::HeightField(_) => todo!(),
                TypedShape::Compound(_) => todo!(),
                TypedShape::ConvexPolygon(_) => todo!(),
                TypedShape::RoundCuboid(_) => todo!(),
                TypedShape::RoundTriangle(_) => todo!(),
                TypedShape::RoundConvexPolygon(_) => todo!(),
                TypedShape::Custom(_) => todo!(),
            }
        }
    }
}
