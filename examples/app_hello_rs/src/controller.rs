use crate::dynamics::{Dynamics, State};
use crate::math::{Mat3, Vec3, sin, cos};
// use libm::{sinf, cosf};


#[derive(Debug, Copy, Clone)]
pub struct Setpoint {
    pub p: Vec3,// desired position
    pub v: Vec3,// desired velocity
    pub a: Vec3,// desired acceleration
    pub j: Vec3,// desired jerk
    pub yaw: f32,// desired yaw
    pub yaw_dot: f32,// desired yaw rate
}

impl Setpoint {
    pub fn new() -> Self {
        Self {
            p: Vec3::zero(),
            v: Vec3::zero(),
            a: Vec3::zero(),
            j: Vec3::zero(),
            yaw: 0.0,
            yaw_dot: 0.0,
        }
    }
}

pub struct LeeController {
    pub kp: Vec3, // Position gain
    pub kv: Vec3, // Velocity gain
    pub kr: Vec3, // Attitude gain
    pub kw: Vec3, // Angular velocity gain
}

impl LeeController {
    pub fn new() -> Self {
        // Initialize some default gains (adjust according to assignment2 requirements)
        Self {
            kp: Vec3::new(7.0, 7.0, 7.0),      // Position gain
            kv: Vec3::new(4.0, 4.0, 4.0),      // Velocity gain
            kr: Vec3::new(0.007, 0.007, 0.008), // Attitude gain 
            kw: Vec3::new(0.00115, 0.00115, 0.002), // Angular velocity gain
        }
    }

    pub fn update(&self, dyn_params: &Dynamics, state: State, setpoint: Setpoint) -> (f32, Vec3) {

        let e_p = state.p - setpoint.p;
        let e_v = state.v - setpoint.v;
        
        let gravity_comp = Vec3::new(0.0, 0.0, 9.81);
        let a_des = setpoint.a 
                    + gravity_comp 
                    - Vec3::new(self.kp.x * e_p.x, self.kp.y * e_p.y, self.kp.z * e_p.z)
                    - Vec3::new(self.kv.x * e_v.x, self.kv.y * e_v.y, self.kv.z * e_v.z);
        
        let f_des_vector = a_des * dyn_params.mass;


        // 2. Attitude Control (Slide 24-25)
 

        let y_c_des = Vec3::new(-sin(setpoint.yaw), cos(setpoint.yaw), 0.0);
        let x_b_des_raw = y_c_des.cross(f_des_vector);
        let x_b_des = x_b_des_raw * (1.0 / x_b_des_raw.norm());
        let y_b_des_raw = f_des_vector.cross(x_b_des);
        let y_b_des = y_b_des_raw * (1.0 / y_b_des_raw.norm());
        let z_b_des = x_b_des.cross(y_b_des);
        let r_des = Mat3::new(
            x_b_des.x, y_b_des.x, z_b_des.x,
            x_b_des.y, y_b_des.y, z_b_des.y,
            x_b_des.z, y_b_des.z, z_b_des.z,
        );

        // Rotation matrix of current attitude
        let r_curr = state.q.to_rotation_matrix();

        // 2.2 Calculate attitude error e_R dlide 25
        let r_des_t = r_des.transpose();
        let r_curr_t = r_curr.transpose();
        
        let e_r_mat = (r_des_t * r_curr) - (r_curr_t * r_des);
        let e_r = e_r_mat.vee() * 0.5;

        // 2.3 e_w desired angular velocity is zero
        let w_des = Vec3::zero(); 
        let e_w =  state.w  - (r_curr_t * r_des).mul_vec(w_des);

        // 2.4 Calculate total thrust f ad tau slide 26
        let curr_z_axis = r_curr.col(2); // the third column of R is the body z-axis
        let thrust = f_des_vector.dot(curr_z_axis);

        let j = dyn_params.j;
        let j_w = Vec3::new(j.x * state.w.x, j.y * state.w.y , j.z * state.w.z );
        let gyro_comp = state.w.cross(j_w) ;

        let torque = Vec3::new(
            -self.kr.x * e_r.x - self.kw.x * e_w.x,
            -self.kr.y * e_r.y - self.kw.y * e_w.y,
            -self.kr.z * e_r.z - self.kw.z * e_w.z,
        ) + gyro_comp;

  
        // self.control_allocation(dyn_params, thrust, torque)
        return (thrust, torque);
    }
}
