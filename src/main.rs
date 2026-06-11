use macroquad::prelude::*;
use macroquad::rand::gen_range;
use rapier2d::prelude::*;

const SCALE: f32 = 40.0;
const BALL_RADIUS: f32 = 0.4;
const WALL_THICKNESS: f32 = 0.4;
const GRAVITY: f32 = 9.81;

struct PhysicsWorld {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    integration_parameters: IntegrationParameters,
    gravity: nalgebra::Vector2<f32>,
}

impl PhysicsWorld {
    fn new() -> Self {
        Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            gravity: vector![0.0, GRAVITY],
            integration_parameters: IntegrationParameters::default(),
        }
    }

    fn step(&mut self) {
        self.pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &(),
            &(),
        );
    }

    fn pixel_to_physics(x: f32, y: f32) -> nalgebra::Vector2<f32> {
        vector![x / SCALE, (screen_height() - y) / SCALE]
    }

    fn spawn_ball(&mut self, pixel_x: f32, pixel_y: f32) -> RigidBodyHandle {
        let pos = Self::pixel_to_physics(pixel_x, pixel_y);
        let vel = vector![gen_range(-4.0, 4.0), gen_range(-2.0, 2.0)];

        let body = RigidBodyBuilder::dynamic()
            .translation(pos)
            .linvel(vel)
            .build();
        let handle = self.rigid_body_set.insert(body);

        let collider = ColliderBuilder::ball(BALL_RADIUS)
            .restitution(0.8)
            .friction(0.2)
            .build();
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        handle
    }

    fn add_wall(&mut self, center_x: f32, center_y: f32, half_width: f32, half_height: f32) {
        let collider = ColliderBuilder::cuboid(half_width, half_height)
            .translation(vector![center_x, center_y])
            .restitution(0.6)
            .build();
        self.collider_set.insert(collider);
    }

    fn init_boundaries(&mut self, world_width: f32, world_height: f32) {
        let half_thick = WALL_THICKNESS / 2.0;
        let half_w = world_width / 2.0;
        let half_h = world_height / 2.0;

        self.add_wall(half_w, half_thick, half_w, half_thick);
        self.add_wall(half_w, world_height - half_thick, half_w, half_thick);
        self.add_wall(half_thick, half_h, half_thick, half_h);
        self.add_wall(world_width - half_thick, half_h, half_thick, half_h);
    }

    fn draw(&self, ball_handles: &[RigidBodyHandle]) {
        let t = WALL_THICKNESS * SCALE;
        let h = screen_height();

        draw_rectangle(0.0, h - t, screen_width(), t, GRAY);
        draw_rectangle(0.0, 0.0, screen_width(), t, GRAY);
        draw_rectangle(0.0, 0.0, t, h, GRAY);
        draw_rectangle(screen_width() - t, 0.0, t, h, GRAY);

        for &handle in ball_handles {
            if let Some(body) = self.rigid_body_set.get(handle) {
                let pos = body.position().translation;
                let px = pos.x * SCALE;
                let py = h - pos.y * SCALE;
                draw_circle(px, py, BALL_RADIUS * SCALE, RED);
            }
        }
    }
}

#[macroquad::main("Macroquad + Rapier2D Simulation")]
async fn main() {
    let mut physics = PhysicsWorld::new();
    let mut ball_handles = Vec::new();
    let mut level_ready = false;

    loop {
        if !level_ready && screen_width() > 0.0 {
            let world_w = screen_width() / SCALE;
            let world_h = screen_height() / SCALE;
            physics.init_boundaries(world_w, world_h);
            level_ready = true;
        }

        if is_mouse_button_pressed(MouseButton::Left) {
            let (mx, my) = mouse_position();
            ball_handles.push(physics.spawn_ball(mx, my));
        }

        physics.step();

        clear_background(DARKGRAY);
        physics.draw(&ball_handles);

        draw_text("Click mouse to create balls!", 20.0, 30.0, 22.0, WHITE);
        draw_text(
            &format!("Number of balls: {}", ball_handles.len()),
            20.0,
            60.0,
            20.0,
            LIGHTGRAY,
        );

        next_frame().await;
    }
}