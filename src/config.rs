use nalgebra::{Matrix3, Vector3};

pub struct MagCalibration {
    a1: Matrix3<f32>,
    b: Vector3<f32>,
}

impl MagCalibration {
    pub fn new() -> Self {
        let a1 = Matrix3::new(
            0.01678127,
            0.000020900285,
            0.00000021276907,
            0.000020900285,
            0.017263092,
            -0.000022182905,
            0.00000021276907,
            -0.000022182905,
            0.017370177,
        );
        let b = Vector3::new(-2.2121878, -22.252243, -26.823246);
        Self { a1, b }
    }

    pub fn apply(&self, sample: Vector3<f32>) -> Vector3<f32> {
        self.a1 * (sample - self.b)
    }
}
