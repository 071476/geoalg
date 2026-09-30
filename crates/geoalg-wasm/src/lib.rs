use wasm_bindgen::prelude::*;

#[wasm_bindgen]
#[derive(Clone, Debug)]
pub struct WasmMotor {
    data: [f64; 8],
}

#[wasm_bindgen]
impl WasmMotor {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self { data: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] }
    }

    pub fn from_translation(dx: f64, dy: f64, dz: f64) -> Self {
        Self { data: [1.0, 0.0, 0.0, 0.0, dx / 2.0, dy / 2.0, dz / 2.0, 0.0] }
    }

    pub fn from_rotation(ax: f64, ay: f64, az: f64, angle: f64) -> Self {
        let half = angle / 2.0;
        let s = half.sin();
        let c = half.cos();
        let len = (ax * ax + ay * ay + az * az).sqrt();
        if len < 1e-15 {
            return Self::new();
        }
        Self {
            data: [c, -s * ax / len, -s * ay / len, -s * az / len, 0.0, 0.0, 0.0, 0.0],
        }
    }

    pub fn compose(&self, other: &WasmMotor) -> WasmMotor {
        let a = &self.data;
        let b = &other.data;
        let r = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
        let e1 = a[0] * b[1] + a[1] * b[0] - a[2] * b[3] + a[3] * b[2];
        let e2 = a[0] * b[2] + a[1] * b[3] + a[2] * b[0] - a[3] * b[1];
        let e3 = a[0] * b[3] - a[1] * b[2] + a[2] * b[1] + a[3] * b[0];
        let t0 = a[4] + b[4];
        let t1 = a[5] + b[5];
        let t2 = a[6] + b[6];
        let t3 = a[7] + b[7];
        WasmMotor { data: [r, e1, e2, e3, t0, t1, t2, t3] }
    }

    pub fn apply_point(&self, x: f64, y: f64, z: f64) -> Vec<f64> {
        let m = &self.data;
        let (w, qx, qy, qz) = (m[0], m[1], m[2], m[3]);
        let px = x;
        let py = y;
        let pz = z;
        let t0 = -qx * px - qy * py - qz * pz;
        let t1 = w * px + qy * pz - qz * py;
        let t2 = w * py - qx * pz + qz * px;
        let t3 = w * pz + qx * py - qy * px;
        let rx = t1 * w - t0 * qx + t2 * qz - t3 * qy;
        let ry = t2 * w - t0 * qy + t3 * qx - t1 * qz;
        let rz = t3 * w - t0 * qz + t1 * qy - t2 * qx;
        vec![rx + 2.0 * m[4], ry + 2.0 * m[5], rz + 2.0 * m[6]]
    }

    pub fn to_mat4(&self) -> Vec<f64> {
        let m = &self.data;
        let (w, x, y, z) = (m[0], m[1], m[2], m[3]);
        let n = (w * w + x * x + y * y + z * z).sqrt();
        if n < 1e-15 {
            return vec![
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ];
        }
        let (w, x, y, z) = (w / n, x / n, y / n, z / n);
        let m00 = 1.0 - 2.0 * (y * y + z * z);
        let m01 = 2.0 * (x * y - w * z);
        let m02 = 2.0 * (x * z + w * y);
        let m10 = 2.0 * (x * y + w * z);
        let m11 = 1.0 - 2.0 * (x * x + z * z);
        let m12 = 2.0 * (y * z - w * x);
        let m20 = 2.0 * (x * z - w * y);
        let m21 = 2.0 * (y * z + w * x);
        let m22 = 1.0 - 2.0 * (x * x + y * y);
        vec![
            m00, m01, m02, 0.0,
            m10, m11, m12, 0.0,
            m20, m21, m22, 0.0,
            2.0 * m[4], 2.0 * m[5], 2.0 * m[6], 1.0,
        ]
    }

    pub fn to_position(&self) -> Vec<f64> {
        vec![2.0 * self.data[4], 2.0 * self.data[5], 2.0 * self.data[6]]
    }

    pub fn to_quat(&self) -> Vec<f64> {
        vec![self.data[0], self.data[1], self.data[2], self.data[3]]
    }

    pub fn from_pose(px: f64, py: f64, pz: f64, qw: f64, qx: f64, qy: f64, qz: f64) -> Self {
        Self { data: [qw, qx, qy, qz, px / 2.0, py / 2.0, pz / 2.0, 0.0] }
    }

    pub fn get(&self, index: usize) -> f64 {
        self.data[index.min(7)]
    }
}

#[wasm_bindgen]
pub fn version() -> String {
    format!("geoalg-wasm {}", env!("CARGO_PKG_VERSION"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity() {
        let m = WasmMotor::new();
        assert!((m.get(0) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_pose_roundtrip() {
        let m = WasmMotor::from_pose(1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0);
        let pos = m.to_position();
        assert!((pos[0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_mat4() {
        let m = WasmMotor::new();
        let mat = m.to_mat4();
        assert!((mat[0] - 1.0).abs() < 1e-10);
        assert!((mat[15] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_compose() {
        let a = WasmMotor::from_translation(1.0, 0.0, 0.0);
        let b = WasmMotor::from_translation(0.0, 2.0, 0.0);
        let c = a.compose(&b);
        let pos = c.to_position();
        assert!((pos[0] - 1.0).abs() < 1e-10);
        assert!((pos[1] - 2.0).abs() < 1e-10);
    }
}
