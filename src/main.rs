use raylib::prelude::*;

type Float = f32;
const PI: Float = std::f32::consts::PI;

type JointId = usize;

#[derive(Clone, PartialEq, Debug)]
struct Connection {
    from: JointId,
    to: JointId,
    distance: Float,
    weight: Float,
}

impl Connection {
    pub fn gravity(&self) -> Vector2 {
        Vector2::new(0.0, -self.weight)
    }
}

#[derive(Clone, PartialEq, Debug, Default)]
struct Joint {
    position: Vector2,
    velocity: Vector2,
    forces: Vector2,
}

impl Joint {
    pub fn new(position: Vector2) -> Self {
        Self {
            position,
            ..Self::default()
        }
    }
    pub fn reset_forces(&mut self) {
        self.forces = Vector2::zero()
    }
}

#[derive(Clone, PartialEq, Debug)]
struct AngularConstraint {
    pivot: JointId,
    a: JointId,
    b: JointId,
    target_angle: Float,
}

#[derive(Debug, Default)]
struct Armature {
    joints: Vec<Joint>,
    conns: Vec<Connection>,
    ang_conn: Vec<AngularConstraint>,
    corr_fac: Float,
    damp_fac: Float,
}

impl Armature {
    fn find_forces(&mut self) {
        self.joints.iter_mut().for_each(Joint::reset_forces);

        for con in self.conns.iter() {
            let a = con.from;
            let b = con.to;

            let rod = self.joints[b].position - self.joints[a].position;
            let speed = (self.joints[b].velocity - self.joints[a].velocity).dot(rod.normalized());

            let damp = rod.normalized() * self.damp_fac * speed;
            let spring = rod.normalized() * self.corr_fac * (rod.length() - con.distance);

            self.joints[a].forces += con.gravity() + spring + damp;
            self.joints[b].forces += con.gravity() - spring - damp;
        }

        for con in self.ang_conn.iter() {
            let dir_a =
                (self.joints[con.a].position - self.joints[con.pivot].position).normalized();
            let dir_b =
                (self.joints[con.b].position - self.joints[con.pivot].position).normalized();

            let det = dir_a.x * dir_b.y - dir_a.y * dir_b.x;
            let dot = dir_a.dot(dir_b);
            let angle = det.atan2(dot);

            let mut angle_dif = angle - con.target_angle;
            while angle_dif <= -PI {
                angle_dif += 2.0 * PI;
            }
            while angle_dif > PI {
                angle_dif -= 2.0 * PI;
            }

            let force = angle_dif * self.corr_fac * 50.0;

            let vec_a = dir_a.rotate(PI * 0.5);
            let vec_b = dir_b.rotate(-PI * 0.5);

            self.joints[con.a].forces += vec_a * force;
            self.joints[con.b].forces += vec_b * force;
            self.joints[con.pivot].forces -= (vec_a + vec_b) * force;
        }
    }

    fn apply_forces(&mut self, dt: Float) {
        for joint in self.joints.iter_mut() {
            joint.velocity += joint.forces * dt;
            joint.position += joint.velocity * dt;

            let min_y = 32.0;

            if joint.position.y < min_y {
                joint.position.y = min_y;
                joint.velocity.y = 0.0;
            }
        }
    }
}

fn main() {
    let (mut rl, thread) = raylib::init().size(640, 480).title("Hello, World").build();

    let mut arm = Armature::default();
    arm.corr_fac = 300.0;
    arm.damp_fac = 100.0;

    let circle_len = 20;

    for i in 0..circle_len {
        let angle = (i as f32) / (circle_len as f32) * 2.0 * PI;
        let dir = Vector2::new(0.0, 150.0).rotate(angle);
        let pos = Vector2::new(300.0, 200.0) + dir;
        arm.joints.push(Joint::new(pos));

        arm.conns.push(Connection {
            from: i,
            to: (i + 1) % circle_len,
            distance: 60.0,
            weight: 1.0,
        });

        arm.ang_conn.push(AngularConstraint {
            a: i,
            b: (i + 2) % circle_len,
            pivot: (i + 1) % circle_len,
            target_angle: 2.0 * PI,
        });
    }

    rl.set_target_fps(240);

    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);

        d.clear_background(Color::WHITE);

        fn tp(pos: Vector2) -> Vector2 {
            Vector2::new(pos.x, 480.0 - pos.y)
        }

        for con in arm.conns.iter() {
            let from = tp(arm.joints[con.from].position);
            let to = tp(arm.joints[con.to].position);
            d.draw_line_v(from, to, Color::BLACK);
        }

        for (i, joint) in arm.joints.iter().enumerate() {
            let pos = tp(joint.position);
            d.draw_circle_v(pos, 5.0, Color::RED);
            let i = format!("{}", i);
            d.draw_text(&i, pos.x as _, pos.y as _, 20, Color::BLUE);
        }

        d.draw_fps(5, 5);

        for _ in 0..1000 {
            arm.find_forces();
            arm.apply_forces(0.005 * d.get_frame_time());
        }
    }
}

trait Rotation {
    fn rotate(&self, angle: Float) -> Self;
}

impl Rotation for Vector2 {
    fn rotate(&self, angle: Float) -> Self {
        Self::new(
            angle.cos() * self.x - angle.sin() * self.y,
            angle.sin() * self.x + angle.cos() * self.y,
        )
    }
}
