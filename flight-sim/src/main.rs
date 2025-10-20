use macroquad::prelude::*;

const GRAVITY: Vec3 = Vec3::from_array([0.0, -9.81, 0.0]);
const DRAG_COEFF: f32 = 0.08;
const LIFT_COEFF: f32 = 11.5;
const THROTTLE_STEP: f32 = 0.5;
const MAX_SPEED: f32 = 130.0;
const MIN_SPEED: f32 = 12.0;
const ROLL_RATE: f32 = 1.4;
const PITCH_RATE: f32 = 0.9;
const YAW_RATE: f32 = 0.4;

struct Plane {
    position: Vec3,
    velocity: Vec3,
    orientation: Quat,
    throttle: f32,
}

impl Plane {
    fn new() -> Self {
        Self {
            position: Vec3::new(0.0, 90.0, 0.0),
            velocity: Vec3::new(0.0, 0.0, -50.0),
            orientation: Quat::IDENTITY,
            throttle: 0.7,
        }
    }

    fn forward(&self) -> Vec3 {
        self.orientation * Vec3::new(0.0, 0.0, -1.0)
    }

    fn right(&self) -> Vec3 {
        self.orientation * Vec3::new(1.0, 0.0, 0.0)
    }

    fn up(&self) -> Vec3 {
        self.orientation * Vec3::new(0.0, 1.0, 0.0)
    }

    fn update(&mut self, dt: f32, input: &InputState) {
        let yaw_input = input.yaw_left as i8 as f32 - input.yaw_right as i8 as f32;
        let pitch_input = input.pitch_up as i8 as f32 - input.pitch_down as i8 as f32;
        let roll_input = input.roll_right as i8 as f32 - input.roll_left as i8 as f32;

        self.throttle = (self.throttle + input.throttle_delta * THROTTLE_STEP * dt).clamp(0.1, 1.4);

        let target_speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * self.throttle;
        let forward = self.forward();
        let speed_along_forward = Vec3::dot(self.velocity, forward);
        let thrust = forward * (target_speed - speed_along_forward) * 14.0;

        let speed = self.velocity.length().max(1.0);
        let lift_dir = self.up();
        let lift = lift_dir * speed * speed * LIFT_COEFF * forward.y.abs().max(0.08);
        let drag = -self.velocity * speed * DRAG_COEFF;
        let gravity = GRAVITY;
        let net_force = thrust + lift + drag + gravity;

        self.velocity += net_force * dt;
        self.position += self.velocity * dt;

        let rotation_delta = Quat::from_euler(
            EulerRot::XYZ,
            pitch_input * PITCH_RATE * dt,
            yaw_input * YAW_RATE * dt,
            roll_input * ROLL_RATE * dt,
        );
        self.orientation = (self.orientation * rotation_delta).normalize();

        if self.position.y < 2.5 {
            self.position.y = 2.5;
            self.velocity.y = self.velocity.y.max(0.0);
        }
    }
}

struct InputState {
    roll_left: bool,
    roll_right: bool,
    pitch_up: bool,
    pitch_down: bool,
    yaw_left: bool,
    yaw_right: bool,
    throttle_delta: f32,
    brake: bool,
    cockpit: bool,
}

impl InputState {
    fn gather() -> Self {
        let throttle_raise = is_key_down(KeyCode::Equal) || is_key_down(KeyCode::KpAdd);
        let throttle_lower = is_key_down(KeyCode::Minus) || is_key_down(KeyCode::KpSubtract);
        Self {
            roll_left: is_key_down(KeyCode::A) || is_key_down(KeyCode::Left),
            roll_right: is_key_down(KeyCode::D) || is_key_down(KeyCode::Right),
            pitch_up: is_key_down(KeyCode::W) || is_key_down(KeyCode::Up),
            pitch_down: is_key_down(KeyCode::S) || is_key_down(KeyCode::Down),
            yaw_left: is_key_down(KeyCode::Q),
            yaw_right: is_key_down(KeyCode::E),
            throttle_delta: if throttle_raise { 1.0 } else { 0.0 }
                - if throttle_lower { 1.0 } else { 0.0 },
            brake: is_key_down(KeyCode::Space),
            cockpit: is_key_down(KeyCode::C),
        }
    }
}

fn update_camera(camera: &mut Camera3D, plane: &Plane, input: &InputState) {
    let forward = plane.forward();
    let up = plane.up();
    if input.cockpit {
        camera.position = plane.position + forward * 1.6 + up * 0.4;
        camera.target = plane.position + forward * 14.0 + up * 0.3;
        camera.up = up;
    } else {
        let chase_offset = -forward * 32.0 + up * 14.0 + plane.right() * 3.0;
        camera.position = plane.position + chase_offset;
        camera.target = plane.position + forward * 18.0;
        camera.up = up;
    }
}

fn draw_box(center: Vec3, axes: (Vec3, Vec3, Vec3), size: Vec3, color: Color) {
    let (right, up, forward) = axes;
    let offset = center - right * (size.x * 0.5) - up * (size.y * 0.5) - forward * (size.z * 0.5);
    draw_affine_parallelepiped(
        offset,
        right * size.x,
        up * size.y,
        forward * size.z,
        None,
        color,
    );
}

fn draw_plane_model(plane: &Plane) {
    let forward = plane.forward();
    let right = plane.right();
    let up = plane.up();

    draw_box(
        plane.position + forward * 1.5,
        (right, up, forward),
        Vec3::new(2.2, 0.8, 9.0),
        Color::new(0.86, 0.89, 0.93, 1.0),
    );

    draw_box(
        plane.position,
        (right, forward, up),
        Vec3::new(14.0, 0.6, 0.25),
        Color::new(0.8, 0.82, 0.88, 1.0),
    );

    draw_box(
        plane.position + forward * -3.0 + up * -0.2,
        (right, up, forward),
        Vec3::new(4.5, 0.4, 3.0),
        Color::new(0.75, 0.78, 0.82, 1.0),
    );

    draw_box(
        plane.position + forward * 4.0 + up * 0.6,
        (right, up, forward),
        Vec3::new(1.1, 0.9, 1.6),
        Color::new(0.75, 0.83, 0.95, 1.0),
    );
}

fn draw_environment(plane: &Plane) {
    clear_background(Color::from_rgba(36, 115, 195, 255));

    draw_plane(
        Vec3::new(0.0, 0.0, 0.0),
        Vec2::new(2500.0, 2500.0),
        None,
        Color::new(0.25, 0.47, 0.18, 1.0),
    );

    draw_grid(
        80,
        40.0,
        Color::new(0.3, 0.35, 0.3, 0.3),
        Color::new(0.2, 0.25, 0.2, 0.2),
    );

    for i in -4..=4 {
        let offset = i as f32 * 320.0;
        draw_cube(
            Vec3::new(offset, 300.0, 900.0),
            Vec3::splat(14.0),
            None,
            Color::new(0.9, 0.97, 1.0, 0.55),
        );
        draw_cube(
            Vec3::new(offset * 1.4, 240.0, -1100.0),
            Vec3::new(22.0, 16.0, 22.0),
            None,
            Color::new(0.92, 0.95, 1.0, 0.45),
        );
    }

    draw_plane_model(plane);
}

fn draw_hud(plane: &Plane, input: &InputState) {
    set_default_camera();
    let speed = plane.velocity.length();
    let altitude = plane.position.y.max(0.0);
    let throttle = (plane.throttle * 100.0).clamp(0.0, 140.0);
    let pitch = plane.forward().y.asin().to_degrees();
    let yaw = plane.forward().x.atan2(-plane.forward().z).to_degrees();
    let roll = plane.right().y.atan2(plane.up().y).to_degrees();
    let info = format!(
        "Hız: {:>6.1} km/h\nİrtifa: {:>6.1} m\nGaz: {:>5.1}%\nPitch: {:>5.1}°\nRoll: {:>5.1}°\nYaw: {:>5.1}°",
        speed, altitude, throttle, pitch, roll, yaw
    );
    draw_text(&info, 24.0, 32.0, 28.0, WHITE);

    let controls = "Kontroller: W/S Pitch | A/D Roll | Q/E Yaw | +/- Gaz | Space Fren | C Kokpit";
    let dims = measure_text(controls, None, 22, 1.0);
    draw_text(
        controls,
        screen_width() * 0.5 - dims.width * 0.5,
        screen_height() - 24.0,
        22.0,
        LIGHTGRAY,
    );

    if input.brake {
        draw_rectangle(
            screen_width() * 0.5 - 90.0,
            screen_height() * 0.5 - 28.0,
            180.0,
            56.0,
            Color::new(0.9, 0.15, 0.15, 0.8),
        );
        draw_text(
            "FRENLER",
            screen_width() * 0.5 - 62.0,
            screen_height() * 0.5 + 10.0,
            34.0,
            WHITE,
        );
    }
}

fn apply_brake(plane: &mut Plane, dt: f32) {
    let speed = plane.velocity.length();
    if speed > 1.0 {
        plane.velocity *= (1.0 - dt * 5.0).clamp(0.0, 0.95);
    }
}

#[macroquad::main("Rust Flight Simulator")]
async fn main() {
    let mut plane = Plane::new();
    let mut camera = Camera3D {
        position: Vec3::new(0.0, 120.0, 140.0),
        target: plane.position,
        up: Vec3::Y,
        fovy: 65.0,
        ..Default::default()
    };

    loop {
        let dt = get_frame_time().clamp(1.0 / 200.0, 1.0 / 30.0);
        let input = InputState::gather();

        plane.update(dt, &input);
        if input.brake {
            apply_brake(&mut plane, dt);
        }

        update_camera(&mut camera, &plane, &input);
        set_camera(&camera);
        draw_environment(&plane);
        draw_hud(&plane, &input);

        next_frame().await;
    }
}
