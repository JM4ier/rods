use crate::prelude::*;
use std::ops::*;

pub mod forces;

pub type JointId = usize;
pub type RodId = usize;

#[derive(Clone, Debug, Default)]
pub struct Joint {
    pub position: Vector2,
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
    pub ends: [JointId; 2],
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
    pub joints: Vec<Joint>,
    inner: InnerWorld,
}

#[derive(Clone, Debug, Default)]
pub struct InnerWorld {
    pub rods: Vec<Rod>,
    angles: Vec<Angle>,
    bounds: Vec<Bounds>,
    dt: Float,
    time: Float,

    config: WorldConfig,
}

#[derive(Clone, Debug, Default)]
pub struct WorldConfig {
    pub rod_stiffness: Float,
    pub rod_damping: Float,
    pub angle_stiffness: Float,
    pub general_damping: Float,
    pub wind: Vec<WindConfig>,
    pub time_scale: Option<Float>,
}

#[derive(Clone, Debug, Default)]
pub struct WindConfig {
    /// direction of moving wind, may not be zero, normalized
    pub dir: Vector2,
    pub speed: Float,
    /// Viscosity of gas
    pub viscosity: Float,
    /// how far between individual wind gusts
    pub low: Float,
    /// how long one wind gust is
    pub high: Float,
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

impl Deref for InnerWorld {
    type Target = WorldConfig;
    fn deref(&self) -> &Self::Target {
        &self.config
    }
}

impl DerefMut for InnerWorld {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.config
    }
}

trait Force {
    fn apply(&self, _joints: &mut [Joint], _data: &InnerWorld) {}
    fn visualize(&self, _world: &World, _draw: &mut RaylibDrawHandle) {}
}

impl World {
    pub fn from_config(config: WorldConfig) -> Self {
        Self {
            joints: Vec::new(),
            inner: InnerWorld {
                config,
                ..Default::default()
            },
        }
    }
    pub fn add_joint(&mut self, pos: Vector2) -> JointId {
        self.joints.push(Joint::from_pos(pos));
        self.joints.len() - 1
    }
    pub fn add_rod(&mut self, ends: [JointId; 2], weight: Float) -> RodId {
        let dist = (self.joints[ends[0]].position - self.joints[ends[1]].position).length();

        self.joints[ends[0]].weight += weight / 2.0;
        self.joints[ends[1]].weight += weight / 2.0;

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
    pub fn add_bounds(&mut self, bound: Bounds) {
        self.bounds.push(bound)
    }

    pub fn update(&mut self, mut dt: Float) {
        if let Some(scale) = self.time_scale {
            dt *= scale;
        }

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
        apply_forces![
            RodDistance,
            RodAngle,
            Gravity,
            Wind,
            FixPoint,
            Bounding,
            Damping
        ];

        for joint in self.joints.iter_mut() {
            joint.velocity += joint.forces / joint.weight * dt;
            joint.position += joint.velocity * dt;
            joint.forces = Vector2::zero();
        }
    }
    pub fn visualize(&self, draw: &mut RaylibDrawHandle) {
        for rod in self.rods.iter() {
            draw.draw_line_v(
                self.joints[rod.ends[0]].position,
                self.joints[rod.ends[1]].position,
                Color::BLUE,
            )
        }
        forces::Wind.visualize(&self, draw);
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

    dir_0.det(dir_1).atan2(dir_0.dot(dir_1))
}
