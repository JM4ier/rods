use crate::prelude::*;

pub mod constraint;

pub type JointId = usize;

#[derive(Clone, PartialEq, Debug, Default)]
pub struct Joint {
    pub position: Vector2,
    pub velocity: Vector2,
    pub forces: Vector2,
}

impl Joint {
    pub fn new(position: Vector2) -> Self {
        Self {
            position,
            ..Self::default()
        }
    }
    fn reset_forces(&mut self) {
        self.forces = Vector2::zero()
    }
}

pub trait Constraint {
    fn apply(&self, joints: &mut [Joint], config: &Config);
}

#[derive(Default, Debug)]
pub struct Config {
    pub stiffness: Float,
    pub damping: Float,
    pub angle_stiffness: Float,
}

pub struct Armature {
    pub joints: Vec<Joint>,
    pub constraints: Vec<Box<dyn Constraint>>,
    pub config: Config,
}

impl Armature {
    pub fn from_config(config: Config) -> Self {
        Self {
            config,
            joints: Vec::new(),
            constraints: Vec::new(),
        }
    }
    pub fn add_joint(&mut self, joint: Vector2) -> JointId {
        self.joints.push(Joint::new(joint));
        self.joints.len() - 1
    }
    pub fn add_constraint<T: 'static + Constraint>(&mut self, constraint: T) {
        self.constraints.push(Box::new(constraint))
    }
    pub fn apply_forces(&mut self, dt: Float) {
        self.joints.iter_mut().for_each(Joint::reset_forces);

        for constraint in self.constraints.iter() {
            constraint.apply(&mut self.joints, &self.config);
        }

        for joint in self.joints.iter_mut() {
            joint.velocity += joint.forces * dt;
            joint.position += joint.velocity * dt;
        }
    }
}
