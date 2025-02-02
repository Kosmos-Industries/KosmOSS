use crate::physics::dynamics::EquationsOfMotion;

pub struct RK4<T: EquationsOfMotion> {
    eom: T,
}

impl<T: EquationsOfMotion> RK4<T> 
where 
    T::State: Clone + std::ops::Add<Output = T::State> + std::ops::Mul<f64, Output = T::State>
{
    pub fn new(eom: T) -> Self {
        RK4 { eom }
    }
    
    pub fn integrate(&self, state: &T::State, dt: f64) -> T::State {
        let k1 = self.eom.compute_derivative(state);
        
        let state2 = state.clone() + k1.clone() * (dt/2.0);
        let k2 = self.eom.compute_derivative(&state2);
        
        let state3 = state.clone() + k2.clone() * (dt/2.0);
        let k3 = self.eom.compute_derivative(&state3);
        
        let state4 = state.clone() + k3.clone() * dt;
        let k4 = self.eom.compute_derivative(&state4);
        
        state.clone() + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt/6.0)
    }
}