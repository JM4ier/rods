use crate::prelude::*;
use std::ops::*;

mod forces;

pub type JointId = usize;
pub type RodId = usize;

#[derive(Clone, Debug, Default)]
pub struct Joint {
    position: Vector2,
    velocity: Vector2,
    forces: Vector2,
    weight: Float,
    fix: bool,
}

impl Joint {
    fn from_pos(position: Vector2) -> Self {
        Self {
            position,
            ..Self::default()
        }
    }
}

#[derive(Clone, Debug)]
pub struct Rod {
    ends: [JointId; 2],
    dist: Float,
    weight: Float,
}

#[derive(Clone, Debug)]
pub struct Angle {
    joints: [JointId; 3],
    angle: Float,
}

#[derive(Clone, Debug)]
pub struct Bounds {
    pub min: Vector2,
    pub max: Vector2,
}

#[derive(Clone, Debug, Default)]
pub struct World {
    joints: Vec<Joint>,
    inner: InnerWorld,
}

#[derive(Clone, Debug, Default)]
pub struct InnerWorld {
    rods: Vec<Rod>,
    angles: Vec<Angle>,
    bounds: Vec<Bounds>,
    dt: Float,
    time: Float,
}

impl Deref for World {
    type Target = InnerWorld;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl DerefMut for World {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

trait Force {
    fn apply(&self, _joints: &mut [Joint], _data: &InnerWorld) {}
    fn visualize(&self, _world: &World, _draw: &mut RaylibDrawHandle) {}
}

impl World {
    pub fn add_joint(&mut self, pos: Vector2) -> JointId {
        self.joints.push(Joint::from_pos(pos));
        self.joints.len() - 1
    }
    pub fn add_rod(&mut self, ends: [JointId; 2], weight: Float) -> RodId {
        let dist = (self.joints[ends[0]].position - self.joints[ends[1]].position).length();
        self.rods.push(Rod { ends, dist, weight });
        self.rods.len() - 1
    }
    pub fn keep_angle(&mut self, joints: [JointId; 3]) {
        let angle = self.angle_between(joints[0], joints[1], joints[2]);
        self.angles.push(Angle { joints, angle })
    }
    pub fn fix(&mut self, joint: JointId) {
        self.joints[joint].fix = true
    }

    pub fn update(&mut self, dt: Float) {
        self.dt = dt;
        self.time += dt;

        macro_rules! apply_forces {
            ($($force:expr),*) => {
                let sworld = &self.inner;
                let joints = &mut self.joints;
                $(
                    $force.apply(joints, sworld);
                )*
            }
        }

        use forces::*;
        apply_forces!(RodDistance, RodAngle);
    }

    fn angle_between(&self, a: JointId, b: JointId, c: JointId) -> Float {
        angle_between(
            self.joints[a].position,
            self.joints[b].position,
            self.joints[c].position,
        )
    }
}

fn angle_between(a: Vector2, b: Vector2, c: Vector2) -> Float {
    let dir_0 = (a - b).normalized();
    let dir_1 = (c - b).normalized();

    let det = dir_0.x * dir_1.y - dir_0.y * dir_1.x;
    let dot = dir_0.dot(dir_1);
    let angle = det.atan2(dot);

    angle
}
