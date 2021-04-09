pub use raylib::prelude::*;

pub type Float = f32;
pub const PI: Float = std::f32::consts::PI;

pub trait VectorExtension {
    fn rotate(&self, angle: Float) -> Self;
    fn det(&self, other: Self) -> Float;
}

impl VectorExtension for Vector2 {
    fn rotate(&self, angle: Float) -> Self {
        Self::new(
            angle.cos() * self.x - angle.sin() * self.y,
            angle.sin() * self.x + angle.cos() * self.y,
        )
    }
    fn det(&self, other: Self) -> Float {
        self.x * other.y - self.y * other.x
    }
}
