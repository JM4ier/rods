use super::*;

/// Tries to keep joints a fixed distance from each other
#[derive(Clone, PartialEq, Debug)]
pub struct DistanceConstraint {
    pub from: JointId,
    pub to: JointId,
    pub distance: Float,
    pub weight: Float,
}

impl DistanceConstraint {
    pub fn gravity(&self) -> Vector2 {
        Vector2::new(0.0, self.weight)
    }
}

/// Tries to keep joints in a certain angle to each other
#[derive(Clone, PartialEq, Debug)]
pub struct AngularConstraint {
    pub pivot: JointId,
    pub a: JointId,
    pub b: JointId,
    pub target_angle: Float,
}

impl Constraint for DistanceConstraint {
    fn apply(&self, joints: &mut [Joint], config: &Config) {
        let a = self.from;
        let b = self.to;

        let rod = joints[b].position - joints[a].position;
        let speed = (joints[b].velocity - joints[a].velocity).dot(rod.normalized());

        let damp = rod.normalized() * config.damping * speed;
        let spring = rod.normalized() * config.stiffness * (rod.length() - self.distance);

        joints[a].forces += self.gravity() + spring + damp;
        joints[b].forces += self.gravity() - spring - damp;
    }
    fn visualize(&self, joints: &[Joint], draw: &mut RaylibDrawHandle) {
        draw.draw_line_v(
            joints[self.from].position,
            joints[self.to].position,
            Color::BLUE,
        )
    }
}

impl Constraint for AngularConstraint {
    fn apply(&self, joints: &mut [Joint], config: &Config) {
        let (a, b, pivot) = (self.a, self.b, self.pivot);

        let dir_a = (joints[a].position - joints[pivot].position).normalized();
        let dir_b = (joints[b].position - joints[pivot].position).normalized();

        let det = dir_a.x * dir_b.y - dir_a.y * dir_b.x;
        let dot = dir_a.dot(dir_b);
        let angle = det.atan2(dot);

        let mut angle_dif = angle - self.target_angle;
        while angle_dif <= -PI {
            angle_dif += 2.0 * PI;
        }
        while angle_dif > PI {
            angle_dif -= 2.0 * PI;
        }

        let force = angle_dif * config.angle_stiffness;

        let vec_a = dir_a.rotate(PI * 0.5);
        let vec_b = dir_b.rotate(-PI * 0.5);

        joints[a].forces += vec_a * force;
        joints[b].forces += vec_b * force;
        joints[pivot].forces -= (vec_a + vec_b) * force;
    }
    fn visualize(&self, joints: &[Joint], draw: &mut RaylibDrawHandle) {
        draw.draw_circle_v(joints[self.pivot].position, 1.0, Color::BLUE)
    }
}

/// Keeps all joints in a box
#[derive(Clone, Debug)]
pub struct BoxConstraint {
    pub min: Vector2,
    pub max: Vector2,
}

impl Constraint for BoxConstraint {
    fn apply(&self, joints: &mut [Joint], _: &Config) {
        let Self { min, max } = self;

        for joint in joints.iter_mut() {
            if joint.position.y < min.y {
                joint.position.y = min.y;
                joint.velocity.y = 0.0;
            } else if joint.position.y > max.y {
                joint.position.y = max.y;
                joint.velocity.y = 0.0;
            }

            if joint.position.x < min.x {
                joint.position.x = min.x;
                joint.velocity.x = 0.0;
            } else if joint.position.x > max.x {
                joint.position.x = max.x;
                joint.velocity.x = 0.0;
            }
        }
    }
    fn visualize(&self, _: &[Joint], draw: &mut RaylibDrawHandle) {
        let Self { min, max } = self;
        let one = Vector2::new(1.0, 1.0);
        let min = *min - one;
        let max = *max + one;

        let tl = Vector2::new(min.x, max.y);
        let br = Vector2::new(max.x, min.y);

        draw.draw_line_v(min, tl, Color::GRAY);
        draw.draw_line_v(min, br, Color::GRAY);
        draw.draw_line_v(max, tl, Color::GRAY);
        draw.draw_line_v(max, br, Color::GRAY);
    }
}

/// Keeps a joint at a specific position
#[derive(Clone, Debug)]
pub struct FixPoint {
    pub point: JointId,
    pub position: Vector2,
}

impl Constraint for FixPoint {
    fn apply(&self, joints: &mut [Joint], _: &Config) {
        joints[self.point] = Joint::new(self.position);
    }
    fn visualize(&self, joints: &[Joint], draw: &mut RaylibDrawHandle) {
        draw.draw_circle_v(joints[self.point].position, 1.0, Color::RED)
    }
}

/// General speed damping
#[derive(Clone, Debug)]
pub struct Damping;

impl Constraint for Damping {
    fn apply(&self, joints: &mut [Joint], config: &Config) {
        for joint in joints.iter_mut() {
            joint.forces -= joint.velocity * config.damping;
        }
    }
}
