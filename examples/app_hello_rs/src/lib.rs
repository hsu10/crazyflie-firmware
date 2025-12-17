#![no_std]

use panic_halt as _;

extern crate cty;
#[allow(unused_imports)]
#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(non_upper_case_globals)]
#[allow(non_snake_case)]

pub mod sys {
    include!("bindings.rs");
}

mod math;
mod dynamics;
mod controller;

use controller::{LeeController, Setpoint};
use dynamics::{Dynamics, State};
use math::{Quaternion, Vec3};


extern "C" {
    pub fn vTaskDelay(ticks: u32);
    pub fn consolePutchar(ch: i32) -> i32;
}

fn console_print(msg: &str) {
    for c in msg.as_bytes() {
        unsafe{ consolePutchar(*c as i32); }
    }
}

#[inline]
unsafe fn quat_from_sys(q: &sys::quaternion_t) -> Quaternion {
    let raw = (*q).__bindgen_anon_1.__bindgen_anon_2;
    Quaternion::new(raw.w, raw.x, raw.y, raw.z)
}

#[inline]
unsafe fn write_control_force_torque(control: *mut sys::control_t, thrust_n: f32, torque_nm: Vec3) {
    (*control).controlMode = sys::control_mode_e_controlModeForceTorque;
    (*control).__bindgen_anon_1.__bindgen_anon_2.thrustSi = thrust_n;
    (*control)
        .__bindgen_anon_1
        .__bindgen_anon_2
        .__bindgen_anon_1
        .__bindgen_anon_1
        .torqueX = torque_nm.x;
    (*control)
        .__bindgen_anon_1
        .__bindgen_anon_2
        .__bindgen_anon_1
        .__bindgen_anon_1
        .torqueY = torque_nm.y;
    (*control)
        .__bindgen_anon_1
        .__bindgen_anon_2
        .__bindgen_anon_1
        .__bindgen_anon_1
        .torqueZ = torque_nm.z;
}

#[inline]
unsafe fn omega_from_sensor(sensor_data: *const sys::sensorData_t) -> Vec3 {
    let gyro = (*sensor_data).gyro.__bindgen_anon_1 ;
    Vec3::new(gyro.x* (core::f32::consts::PI / 180.0) , gyro.y* (core::f32::consts::PI / 180.0) , gyro.z* (core::f32::consts::PI / 180.0))
}

// static mut CONTROLLER = controller::LeeController::new();
static mut CONTROLLER: Option<LeeController> = None;

static DYNAMICS: Dynamics = Dynamics {
    mass: 0.034,
    g: Vec3 { x: 0.0, y: 0.0, z: 9.81 },
    j: Vec3 { x: 16.57e-6, y: 16.66e-6, z: 29.26e-6 },
    kf: 2.22e-10,
    kt: 1e-12,
    a: 0.046,
};

#[no_mangle]
pub extern "C" fn appMain() {
    console_print("Hello from Rust!\n");

    loop {
        unsafe { vTaskDelay(1000); }
    }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTreeInit(){
    unsafe {
        CONTROLLER = Some(LeeController::new());
    }

}

#[no_mangle]
pub extern "C" fn controllerOutOfTreeTest()->bool { 
    true
 }

#[no_mangle]
pub extern "C" fn controllerOutOfTree(
    control: *mut crate::sys::control_t,
    setpoint: *mut crate::sys::setpoint_t,
    sensor_data: *mut crate::sys::sensorData_t,
    state: *mut crate::sys::state_t,
    tick: u32
){
    unsafe{


        let ctrl = match CONTROLLER.as_mut() {
            Some(c) => c,
            None => return,
        };

        let current_state = State {
            p: Vec3::new((*state).position.x, (*state).position.y, (*state).position.z),
            v: Vec3::new((*state).velocity.x, (*state).velocity.y, (*state).velocity.z),
            q: quat_from_sys(&(*state).attitudeQuaternion),
            w: omega_from_sensor(sensor_data),
        };

        let desired_setpoint = Setpoint {
            p: Vec3::new((*setpoint).position.x, (*setpoint).position.y, (*setpoint).position.z),
            v: Vec3::new((*setpoint).velocity.x, (*setpoint).velocity.y, (*setpoint).velocity.z),
            a: Vec3::new((*setpoint).acceleration.x, (*setpoint).acceleration.y, (*setpoint).acceleration.z),
            j: Vec3::zero(),
            yaw: (*setpoint).attitude.yaw * core::f32::consts::PI / 180.0,
            yaw_dot: (*setpoint).attitudeRate.yaw * core::f32::consts::PI / 180.0,
        };

        let (thrust_n, torque_nm) = ctrl.update(&DYNAMICS, current_state, desired_setpoint);

        write_control_force_torque(control, thrust_n, torque_nm);

    }

}