pub use raylib::prelude::*;

pub type Float = f32;
pub const PI: Float = std::f32::consts::PI;

pub trait Rotation {
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
