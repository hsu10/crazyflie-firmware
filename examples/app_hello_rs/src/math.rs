#![allow(non_snake_case)]

use core::ops::{Add, Mul, Sub};

extern crate cty;
use cty::c_float;

extern "C" {
    fn sinf(x: c_float) -> c_float;
    fn cosf(x: c_float) -> c_float;
    fn sqrtf(x: c_float) -> c_float;
}

pub fn sin(x: f32) -> f32 {
    unsafe { sinf(x) }
}

pub fn cos(x: f32) -> f32 {
    unsafe { cosf(x) }
}

pub fn sqrt(x: f32) -> f32 {
    unsafe { sqrtf(x) }
}



#[derive(Debug, Copy, Clone)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    // J*w x w (Cross product)
    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn norm(self) -> f32 {
        sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    // Hat map: 向量 -> 反对称矩阵 (Skew-symmetric matrix)
    // w x v = hat(w) * v
    pub fn hat(self) -> Mat3 {
        Mat3::new(
            0.0, -self.z, self.y,
            self.z, 0.0, -self.x,
            -self.y, self.x, 0.0
        )
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f32> for Vec3 {
    type Output = Self;
    fn mul(self, scalar: f32) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Mat3 {
    pub m: [[f32; 3]; 3],
}

impl Mat3 {
    #[allow(clippy::too_many_arguments)]
    pub fn new(m00: f32, m01: f32, m02: f32,
               m10: f32, m11: f32, m12: f32,
               m20: f32, m21: f32, m22: f32) -> Self {
        Self {
            m: [
                [m00, m01, m02],
                [m10, m11, m12],
                [m20, m21, m22]
            ]
        }
    }

    pub fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0)
    }

    // 从四元数转换为旋转矩阵
    pub fn from_quaternion(q: Quaternion) -> Self {
        let w = q.w;
        let x = q.x;
        let y = q.y;
        let z = q.z;

        let x2 = x + x; let y2 = y + y; let z2 = z + z;
        let xx = x * x2; let xy = x * y2; let xz = x * z2;
        let yy = y * y2; let yz = y * z2; let zz = z * z2;
        let wx = w * x2; let wy = w * y2; let wz = w * z2;

        Self::new(
            1.0 - (yy + zz), xy - wz,         xz + wy,
            xy + wz,         1.0 - (xx + zz), yz - wx,
            xz - wy,         yz + wx,         1.0 - (xx + yy)
        )
    }

    pub fn transpose(self) -> Self {
        Self::new(
            self.m[0][0], self.m[1][0], self.m[2][0],
            self.m[0][1], self.m[1][1], self.m[2][1],
            self.m[0][2], self.m[1][2], self.m[2][2]
        )
    }

    // Vee map: 反对称矩阵 -> 向量 (Hat map 的逆运算)
    // 用于计算姿态误差 e_R
    pub fn vee(self) -> Vec3 {
        Vec3::new(
            self.m[2][1], // x = M(2,1) (or -M(1,2) in skew symmetric)
            self.m[0][2], // y = M(0,2)
            self.m[1][0]  // z = M(1,0)
        )
    }

    // 矩阵乘向量
    pub fn mul_vec(self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }
    
    // 获取某一列 (辅助构建 Rd)
    pub fn col(self, idx: usize) -> Vec3 {
        Vec3::new(self.m[0][idx], self.m[1][idx], self.m[2][idx])
    }
}

// Matrix * Matrix
impl Mul for Mat3 {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        let mut result = Mat3::new(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
        for i in 0..3 {
            for j in 0..3 {
                let mut sum = 0.0;
                for k in 0..3 {
                    sum += self.m[i][k] * other.m[k][j];
                }
                result.m[i][j] = sum;
            }
        }
        result
    }
}

// Matrix - Matrix
impl Sub for Mat3 {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self::new(
            self.m[0][0] - other.m[0][0], self.m[0][1] - other.m[0][1], self.m[0][2] - other.m[0][2],
            self.m[1][0] - other.m[1][0], self.m[1][1] - other.m[1][1], self.m[1][2] - other.m[1][2],
            self.m[2][0] - other.m[2][0], self.m[2][1] - other.m[2][1], self.m[2][2] - other.m[2][2],
        )
    }
}

// Matrix * scalar
impl Mul<f32> for Mat3 {
    type Output = Self;
    fn mul(self, s: f32) -> Self {
         Self::new(
            self.m[0][0]*s, self.m[0][1]*s, self.m[0][2]*s,
            self.m[1][0]*s, self.m[1][1]*s, self.m[1][2]*s,
            self.m[2][0]*s, self.m[2][1]*s, self.m[2][2]*s,
        )
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn conjugate(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn norm(self) -> f32 {
        sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }

    pub fn normalize(self) -> Self {
        let n = self.norm();
        if n == 0.0 {
            return Self::identity();
        }
        Self {
            w: self.w / n,
            x: self.x / n,
            y: self.y / n,
            z: self.z / n,
        }
    }

    pub fn rotate_vector(self, v: Vec3) -> Vec3 {
        let v_quat = Quaternion::new(0.0, v.x, v.y, v.z);
        let result = self * v_quat * self.conjugate();
        Vec3::new(result.x, result.y, result.z)
    }
    
    // 转换为旋转矩阵的快捷方法
    pub fn to_rotation_matrix(self) -> Mat3 {
        Mat3::from_quaternion(self)
    }
}

impl Mul for Quaternion {
    type Output = Self;
    fn mul(self, p: Self) -> Self {
        Self {
            w: self.w * p.w - self.x * p.x - self.y * p.y - self.z * p.z,
            x: self.w * p.x + self.x * p.w + self.y * p.z - self.z * p.y,
            y: self.w * p.y - self.x * p.z + self.y * p.w + self.z * p.x,
            z: self.w * p.z + self.x * p.y - self.y * p.x + self.z * p.w,
        }
    }
}

impl Mul<Vec3> for Quaternion {
    type Output = Self;
    fn mul(self, v: Vec3) -> Self {
        let p = Quaternion::new(0.0, v.x, v.y, v.z);
        self * p
    }
}

impl Mul<f32> for Quaternion {
    type Output = Self;
    fn mul(self, scalar: f32) -> Self {
        Self {
            w: self.w * scalar,
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Add for Quaternion {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            w: self.w + other.w,
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}