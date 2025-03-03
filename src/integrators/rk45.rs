use crate::physics::dynamics::EquationsOfMotion;

pub struct RK45<T: EquationsOfMotion> {
    eom: T,
}

impl<T: EquationsOfMotion> RK45<T>
where
    T::State: Clone + std::ops::Add<Output = T::State> + std::ops::Mul<f64, Output = T::State>,
{
    pub fn new(eom: T) -> Self {
        RK45 { eom }
    }

    pub fn integrate(&self, state: &T::State, dt: f64) -> T::State {
        let hmin = 1e-6;
        let hmax = 0.1;
        let tol = 1e-5;

        let k1 = self.eom.compute_derivative(state);

        let state2 = state.clone() + k1.clone() * (dt / 4.0);
        let k2 = self.eom.compute_derivative(&state2);

        let state3 = state.clone() + k1.clone() * (dt * 3.0 / 32.0) 
                                                                + k2.clone() * (dt * 9.0 / 32.0);
        let k3 = self.eom.compute_derivative(&state3);

        let state4 = state.clone() + k1.clone() * (dt * 1932.0 / 2197.0) 
                                                                + k2.clone() * (dt * 7200.0 / 2197.0) * (-1.0)
                                                                + k3.clone() * (dt * 7296.0 / 2197.0);
        let k4 = self.eom.compute_derivative(&state4);
        
        let state5 = state.clone() + k1.clone() * (dt * 439.0 / 216.0)
                                                                + k2.clone() * (dt * 8.0) * (-1.0)
                                                                + k3.clone() * (dt * 3680.0 / 513.0)
                                                                + k4.clone() * (dt * 845.0 / 4104.0) * (-1.0);
        let k5 = self.eom.compute_derivative(&state5);

        let state6 = state.clone() + k1.clone() * (dt * 8.0 / 27.0) * (-1.0)
                                                                + k2.clone() * (dt * 2.0)
                                                                + k3.clone() * (dt * 3544.0 / 2565.0) * (-1.0)
                                                                + k4.clone() * (dt * 1859.0 / 4104.0)
                                                                + k5.clone() * (dt * 11.0 / 40.0) * (-1.0); 
        let k6 = self.eom.compute_derivative(&state6);

        let delta_45 = k1.clone() * (dt * 1.0 / 360.0)
                                                                 + k3.clone() * (dt * 128.0 / 4275.0) * (-1.0)
                                                                 + k4.clone() * (dt * 2197.0 / 75240.0) * (-1.0)
                                                                 + k5.clone() * (dt * 1.0 / 50.0)
                                                                 + k6.clone() * (dt * 2.0 / 55.0);

        if delta_45. < &tol {
            state.clone() + k1.clone() * (25.0 / 216.0)
                            + k3.clone() * (1408.0 / 2565.0)
                            + k4.clone() * (2197.0 / 4104.0)
                            - k5.clone() * (1.0 / 5.0)
        } else {
            state.clone() + k1.clone() * (16.0 / 135.0)
                            + k3.clone() * (6656.0 / 12825.0)
                            + k4.clone() * (28561.0 / 56430.0)
                            - k5.clone() * (9.0 / 50.0)
                            + k6.clone() * (2.0 / 55.0)
            
        }

        
    }
}