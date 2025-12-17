use crate::math::{Quaternion, Vec3};
use core::ops::{Add, Mul};


//state
#[derive(Debug, Copy, Clone)]
pub struct State {
    pub p: Vec3,
    pub v: Vec3,
    pub q: Quaternion,
    pub w: Vec3,
}

impl State {
    pub fn new() -> Self {
        Self {
            p: Vec3::zero(),
            v: Vec3::zero(),
            q: Quaternion::identity(),
            w: Vec3::zero(),
        }
    }
}

// //action
// #[derive(Debug, Copy, Clone)]
// pub struct Action {
//     pub omega_sq: [f32; 4],
// }

//dynamics
#[derive(Debug, Copy, Clone)]
pub struct Dynamics {
    pub mass: f32,
    pub g: Vec3,
    pub j: Vec3,
    pub kf: f32,
    pub kt: f32,
    pub a: f32,
}

#[derive(Debug, Copy, Clone)]
pub struct StateDerivative {
    pub v: Vec3, // p_dot
    pub a: Vec3, // v_dot
    pub q_dot: Quaternion,
    pub w_dot: Vec3,
}

impl Add<StateDerivative> for State {
    type Output = Self;
    fn add(self, deriv: StateDerivative) -> Self {
        Self {
            p: self.p + deriv.v,
            v: self.v + deriv.a,
            q: self.q + deriv.q_dot,
            w: self.w + deriv.w_dot,
        }
    }
}

// StateDerivative * f32
impl Mul<f32> for StateDerivative {
    type Output = Self;
    fn mul(self, scalar: f32) -> Self {
        Self {
            v: self.v * scalar,
            a: self.a * scalar,
            q_dot: self.q_dot * scalar,
            w_dot: self.w_dot * scalar,
        }
    }
}

// StateDerivative + StateDerivative
impl Add for StateDerivative {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            v: self.v + other.v,
            a: self.a + other.a,
            q_dot: self.q_dot + other.q_dot,
            w_dot: self.w_dot + other.w_dot,
        }
    }
}

pub fn derivatives(dyn_params: &Dynamics, state: State, omega_sq: [f32; 4]) -> StateDerivative {
    // 直接从数组中读取电机转速平方
    let w1_sq = omega_sq[0];
    let w2_sq = omega_sq[1];
    let w3_sq = omega_sq[2];
    let w4_sq = omega_sq[3];
    
    // calculate forces and torques
    let f = dyn_params.kf * (w1_sq + w2_sq + w3_sq + w4_sq);
    let tau_u = Vec3::new(
        dyn_params.kf * dyn_params.a * (-w1_sq - w2_sq + w3_sq + w4_sq),
        dyn_params.kf * dyn_params.a * (-w1_sq + w2_sq + w3_sq - w4_sq),
        dyn_params.kt * (-w1_sq + w2_sq - w3_sq + w4_sq),
    );

    // 2. calculate derivatives
    let p_dot = state.v;

    let thrust_world = state.q.rotate_vector(Vec3::new(0.0, 0.0, f));
    let v_dot = (thrust_world * (1.0 / dyn_params.mass)) + dyn_params.g;

    let q_dot = (state.q * state.w) * 0.5;

    // gyroscopic torque
    let j = dyn_params.j;
    let j_w_cross_w = Vec3::new(
        (j.z - j.y) * state.w.y * state.w.z,
        (j.x - j.z) * state.w.z * state.w.x,
        (j.y - j.x) * state.w.x * state.w.y,
    );
    let w_dot = Vec3::new(
        (tau_u.x - j_w_cross_w.x) / j.x,
        (tau_u.y - j_w_cross_w.y) / j.y,
        (tau_u.z - j_w_cross_w.z) / j.z,
    );

    StateDerivative {
        v: p_dot,
        a: v_dot,
        q_dot: q_dot,
        w_dot: w_dot,
    }
}