use super::*;

pub struct RodDistance;
pub struct RodAngle;
pub struct Gravity;
pub struct Wind;
pub struct Damping;
pub struct FixPoint;
pub struct Bounding;

impl Force for RodDistance {
    fn apply(&self, joints: &mut [Joint], data: &InnerWorld) {
        for rod in data.rods.iter() {
            let [a, b] = rod.ends;

            let dir = joints[b].position - joints[a].position;
            let speed = (joints[b].velocity - joints[a].velocity).dot(dir.normalized());

            let spring = dir.normalized() * data.rod_stiffness * (dir.length() - rod.dist);
            let damping = dir.normalized() * data.rod_damping * speed;

            let force = spring + damping;
            joints[a].forces += force;
            joints[b].forces -= force;
        }
    }
}

impl Force for RodAngle {
    fn apply(&self, joints: &mut [Joint], data: &InnerWorld) {
        for angle in data.angles.iter() {
            let [a, pivot, b] = angle.joints;
            let target_angle = angle.angle;

            let dir_a = (joints[a].position - joints[pivot].position).normalized();
            let dir_b = (joints[b].position - joints[pivot].position).normalized();

            let angle = angle_between(dir_a, Vector2::zero(), dir_b);

            let mut angle_dif = angle - target_angle;
            while angle_dif <= -PI {
                angle_dif += 2.0 * PI;
            }
            while angle_dif > PI {
                angle_dif -= 2.0 * PI;
            }

            let force = angle_dif * data.angle_stiffness;

            let vec_a = dir_a.rotate(PI * 0.5);
            let vec_b = dir_b.rotate(-PI * 0.5);

            joints[a].forces += vec_a * force;
            joints[b].forces += vec_b * force;
            joints[pivot].forces -= (vec_a + vec_b) * force;
        }
    }
}

impl Force for Gravity {
    fn apply(&self, joints: &mut [Joint], data: &InnerWorld) {
        for rod in data.rods.iter() {
            let [a, b] = rod.ends;
            let gravity = Vector2::new(0.0, rod.weight);
            joints[a].forces += gravity;
            joints[b].forces += gravity;
        }
    }
}

impl Force for Damping {
    fn apply(&self, joints: &mut [Joint], data: &InnerWorld) {
        for joint in joints.iter_mut() {
            joint.forces -= joint.velocity * data.general_damping;
        }
    }
}

impl Force for Wind {
    fn apply(&self, joints: &mut [Joint], data: &InnerWorld) {
        let dir = Vector2::new(0.1, 0.0);
        for rod in data.rods.iter() {
            let [a, b] = rod.ends;
            let rod_dir = joints[b].position - joints[a].position;
            let force = dir.det(rod_dir.normalized());
            let normal = rod_dir.rotate(PI * 0.5);

            let force = normal * force;
            joints[a].forces += force;
            joints[b].forces += force;
        }
    }
}

impl Force for FixPoint {
    fn apply(&self, joints: &mut [Joint], _: &InnerWorld) {
        for joint in joints.iter_mut() {
            if joint.fix {
                joint.forces = Vector2::zero();
            }
        }
    }
}

impl Force for Bounding {
    fn apply(&self, joints: &mut [Joint], data: &InnerWorld) {
        for joint in joints.iter_mut() {
            for b in data.bounds.iter() {
                if joint.position.x < b.min.x {
                    joint.position.x = b.min.x;
                    joint.velocity.x = 0.0;
                } else if joint.position.x > b.max.x {
                    joint.position.x = b.max.x;
                    joint.velocity.x = 0.0;
                }
                if joint.position.y < b.min.y {
                    joint.position.y = b.min.y;
                    joint.velocity.y = 0.0;
                } else if joint.position.y > b.max.y {
                    joint.position.y = b.max.y;
                    joint.velocity.y = 0.0;
                }
            }
        }
    }
}
