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
        assert!((pos[1] - 2.0).abs() < 1e-10);
        assert!((pos[2] - 3.0).abs() < 1e-10);
    }
}
