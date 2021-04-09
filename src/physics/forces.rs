use super::*;

pub struct RodDistance;
pub struct RodAngle;
pub struct Gravity;
pub struct Wind;

impl Force for RodDistance {}
impl Force for RodAngle {}
impl Force for Gravity {}
impl Force for Wind {}
