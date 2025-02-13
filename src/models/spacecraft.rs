pub trait SpacecraftProperties {
    fn mass(&self) -> f64;
    fn drag_coefficient(&self) -> f64;
    fn reference_area(&self) -> f64;
}
