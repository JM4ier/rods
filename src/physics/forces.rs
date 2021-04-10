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
            let gravity = Vector2::new(0.0, 40.0 * rod.weight);
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

impl Wind {
    #[inline]
    fn is_blowing_at(&self, gust: &WindConfig, data: &InnerWorld, position: Vector2) -> bool {
        let period = gust.low + gust.high;
        let distance = gust.dir.dot(position);
        let mut distance = (distance - gust.speed * data.time) % period;
        if distance < 0.0 {
            distance += period;
        }

        distance >= gust.low
    }
}

impl Force for Wind {
    fn apply(&self, joints: &mut [Joint], data: &InnerWorld) {
        for gust in data.wind.iter() {
            for rod in data.rods.iter() {
                let [a, b] = rod.ends;

                let pos = (joints[a].position + joints[b].position) * 0.5;

                if !self.is_blowing_at(gust, data, pos) {
                    continue;
                }

                let rod_dir = joints[b].position - joints[a].position;
                let normal = rod_dir.rotate(PI * 0.5);

                let wind_velocity = gust.dir * gust.speed;
                joints[a].forces -= normal
                    * (wind_velocity - joints[a].velocity).det(rod_dir.normalized())
                    * gust.viscosity;

                joints[b].forces -= normal
                    * (wind_velocity - joints[b].velocity).det(rod_dir.normalized())
                    * gust.viscosity;
            }
        }
    }

    fn visualize(&self, data: &World, draw: &mut RaylibDrawHandle) {
        for gust in data.wind.iter() {
            for x in (0..640).step_by(10) {
                for y in (0..480).step_by(10) {
                    if Wind.is_blowing_at(gust, data, Vector2::new(x as Float, y as Float)) {
                        draw.draw_pixel(x, y, Color::GREEN);
                    }
                }
            }
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
