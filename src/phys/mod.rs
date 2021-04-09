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

pub trait Constraint: ConstraintClone {
    fn apply(&self, joints: &mut [Joint], config: &Config);
    fn visualize(&self, _joints: &[Joint], _draw: &mut RaylibDrawHandle) {}
}

#[derive(Default, Debug, Clone)]
pub struct Config {
    pub stiffness: Float,
    pub damping: Float,
    pub angle_stiffness: Float,
}

pub trait ConstraintClone {
    fn dyn_clone(&self) -> Box<dyn Constraint>;
}

impl<T> ConstraintClone for T
where
    T: Constraint + Clone + 'static,
{
    fn dyn_clone(&self) -> Box<dyn Constraint> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn Constraint> {
    fn clone(&self) -> Self {
        self.dyn_clone()
    }
}

#[derive(Clone)]
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
    pub fn fixate(&mut self, joint: JointId) {
        self.add_constraint(constraint::FixPoint {
            point: joint,
            position: self.joints[joint].position,
        })
    }
    pub fn connect(&mut self, from: JointId, to: JointId, weight: Float) {
        self.add_constraint(constraint::DistanceConstraint {
            from,
            to,
            weight,
            distance: (self.joints[from].position - self.joints[to].position).length(),
        });
    }
    pub fn connect_angle(&mut self, a: JointId, pivot: JointId, b: JointId) {
        let target_angle = angle_between(
            self.joints[a].position,
            self.joints[pivot].position,
            self.joints[b].position,
        );
        self.add_constraint(constraint::AngularConstraint {
            a,
            b,
            pivot,
            target_angle,
        })
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
    pub fn visualize(&self, draw: &mut RaylibDrawHandle) {
        for constraint in self.constraints.iter() {
            constraint.visualize(&self.joints, draw);
        }
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
