use crate::physics::dynamics::EquationsOfMotion;

pub struct RK8<T: EquationsOfMotion> {
    eom: T,
}

impl<T: EquationsOfMotion> RK8<T>
where
    T::State: Clone + std::ops::Add<Output = T::State> + std::ops::Mul<f64, Output = T::State>,
{
    pub fn new(eom: T) -> Self {
        RK8 { eom }
    }

    pub fn integrate(&self, state: &T::State, dt: f64) -> T::State {
        let k1 = self.eom.compute_derivative(state);

        let state2 = state.clone() + k1.clone() * (dt * 4.0/27.0);
        let k2 = self.eom.compute_derivative(&state2);

        let state3 = state.clone() + ( k1.clone() 
                                                                  + k2.clone() * 3.0 ) * (dt / 18.0);
        let k3 = self.eom.compute_derivative(&state3);

        let state4 = state.clone() + ( k1.clone()
                                                                  + k3.clone() * 3.0 ) * (dt / 12.0);
        let k4 = self.eom.compute_derivative(&state4);

        let state5 = state.clone() + ( k1.clone()
                                                                  + k4.clone() * 3.0 ) * (dt / 8.0);
        let k5 = self.eom.compute_derivative(&state5);

        let state6 = state.clone() + ( k1.clone() * (13.0)
                                                                  + k3.clone() * (-27.0)
                                                                  + k4.clone() * (42.0)
                                                                  + k5.clone() * (8.0) ) * (dt / 54.0);
        let k6 = self.eom.compute_derivative(&state6);

        let state7 = state.clone() + ( k1.clone() * (389.0)
                                                                  + k3.clone() * (-54.0)
                                                                  + k4.clone() * (966.0)
                                                                  + k5.clone() * (-824.0)
                                                                  + k6.clone() * (243.0) ) * (dt / 4320.0);
        let k7 = self.eom.compute_derivative(&state7);

        let state8 = state.clone() + ( k1.clone() * (-234.0)
                                                                  + k3.clone() * (81.0)
                                                                  + k4.clone() * (-1164.0)
                                                                  + k5.clone() * (656.0)
                                                                  + k6.clone() * (-122.0)
                                                                  + k7.clone() * (800.0) ) * (dt / 20.0);
        let k8 = self.eom.compute_derivative(&state8);

        let state9 = state.clone() + ( k1.clone() * (-127.0)
                                                                  + k3.clone() * (18.0)
                                                                  + k4.clone() * (-678.0)
                                                                  + k5.clone() * (456.0)
                                                                  + k6.clone() * (-9.0)
                                                                  + k7.clone() * (576.0)
                                                                  + k8.clone() * (4.0) ) * (dt / 288.0);
        let k9 = self.eom.compute_derivative(&state9);

        let state10 = state.clone() + ( k1.clone() * (1481.0)
                                                                    + k3.clone() * (-81.0)
                                                                    + k4.clone() * (7104.0)
                                                                    + k5.clone() * (-3376.0)
                                                                    + k6.clone() * (72.0)
                                                                    + k7.clone() * (-5040.0)
                                                                    + k8.clone() * (-60.0)
                                                                    + k9.clone() * (720.0) ) * (dt / 820.0);
        let k10 = self.eom.compute_derivative(&state10);


        state.clone() + ( k1 * (41.0 )
                        + k4 * (27.0)
                        + k5 * (272.0)
                        + k6 * (27.0)
                        + k7 * (216.0)
                        + k9 * (216.0)
                        + k10 * (41.0) ) * (dt / 840.0)
    }
}