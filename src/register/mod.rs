pub mod main;

pub trait SensorState {}

pub struct OnState;
impl SensorState for OnState {}
