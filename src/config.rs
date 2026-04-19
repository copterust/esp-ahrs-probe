use nalgebra::{Matrix3, Vector3};

const STORAGE_MAGIC: [u8; 4] = *b"MCAL";
const STORAGE_VERSION: u8 = 1;
const STORAGE_RESERVED_LEN: usize = 3;
const MATRIX_ELEMENT_COUNT: usize = 9;
const BIAS_ELEMENT_COUNT: usize = 3;

pub const MAG_CALIBRATION_BLOB_LEN: usize = STORAGE_MAGIC.len()
    + 1
    + STORAGE_RESERVED_LEN
    + ((MATRIX_ELEMENT_COUNT + BIAS_ELEMENT_COUNT) * core::mem::size_of::<f32>());

#[derive(Clone, Copy, Debug)]
pub struct MagCalibration {
    a1: Matrix3<f32>,
    b: Vector3<f32>,
}

impl Default for MagCalibration {
    fn default() -> Self {
        Self::new()
    }
}

impl MagCalibration {
    pub fn new() -> Self {
        Self::from_parts(Matrix3::identity(), Vector3::zeros())
    }

    pub fn from_parts(a1: Matrix3<f32>, b: Vector3<f32>) -> Self {
        Self { a1, b }
    }

    pub fn apply(&self, sample: Vector3<f32>) -> Vector3<f32> {
        self.a1 * (sample - self.b)
    }

    pub fn matrix(&self) -> &Matrix3<f32> {
        &self.a1
    }

    pub fn bias(&self) -> &Vector3<f32> {
        &self.b
    }

    pub fn is_usable(&self) -> bool {
        self.a1.iter().all(|value| value.is_finite())
            && self.b.iter().all(|value| value.is_finite())
            && self.a1.determinant().is_finite()
            && self.a1.determinant().abs() > 1e-9
    }

    pub fn to_blob(&self) -> [u8; MAG_CALIBRATION_BLOB_LEN] {
        let mut blob = [0u8; MAG_CALIBRATION_BLOB_LEN];
        blob[..STORAGE_MAGIC.len()].copy_from_slice(&STORAGE_MAGIC);
        blob[STORAGE_MAGIC.len()] = STORAGE_VERSION;

        let mut offset = STORAGE_MAGIC.len() + 1 + STORAGE_RESERVED_LEN;
        for row in 0..3 {
            for col in 0..3 {
                let value = self.a1[(row, col)];
                blob[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
                offset += 4;
            }
        }
        for value in self.b.as_slice() {
            blob[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }

        blob
    }

    pub fn from_blob(blob: &[u8]) -> Option<Self> {
        if blob.len() != MAG_CALIBRATION_BLOB_LEN {
            return None;
        }
        if blob[..STORAGE_MAGIC.len()] != STORAGE_MAGIC {
            return None;
        }
        if blob[STORAGE_MAGIC.len()] != STORAGE_VERSION {
            return None;
        }

        let mut values = [0.0f32; MATRIX_ELEMENT_COUNT + BIAS_ELEMENT_COUNT];
        let mut offset = STORAGE_MAGIC.len() + 1 + STORAGE_RESERVED_LEN;
        for value in &mut values {
            let bytes = blob.get(offset..offset + 4)?;
            *value = f32::from_le_bytes(bytes.try_into().ok()?);
            offset += 4;
        }

        let calibration = Self::from_parts(
            Matrix3::from_row_slice(&values[..MATRIX_ELEMENT_COUNT]),
            Vector3::from_row_slice(&values[MATRIX_ELEMENT_COUNT..]),
        );

        calibration.is_usable().then_some(calibration)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix3, Vector3};

    #[test]
    fn default_is_do_nothing() {
        let c = MagCalibration::default();

        assert_eq!(c.matrix(), &Matrix3::identity());
        assert_eq!(c.bias(), &Vector3::zeros());
    }

    #[test]
    fn apply_with_default_returns_input_unchanged() {
        let c = MagCalibration::default();
        let sample = Vector3::new(1.25, -2.5, 3.75);

        assert_eq!(c.apply(sample), sample);
    }

    #[test]
    fn blob_round_trip_preserves_values() {
        let c = MagCalibration::from_parts(
            Matrix3::new(1.5, 0.2, -0.1, 0.0, 2.0, 0.3, -0.4, 0.1, 0.75),
            Vector3::new(-1.0, -2.0, -3.0),
        );

        let blob = c.to_blob();
        let decoded = MagCalibration::from_blob(&blob).expect("expected valid calibration blob");

        assert_eq!(decoded.matrix(), c.matrix());
        assert_eq!(decoded.bias(), c.bias());
    }

    #[test]
    fn from_blob_rejects_bad_magic() {
        let c = MagCalibration::default();
        let mut blob = c.to_blob();
        blob[0] ^= 0xFF;

        assert!(MagCalibration::from_blob(&blob).is_none());
    }

    #[test]
    fn from_blob_rejects_bad_version() {
        let c = MagCalibration::default();
        let mut blob = c.to_blob();
        blob[STORAGE_MAGIC.len()] = 99;

        assert!(MagCalibration::from_blob(&blob).is_none());
    }

    #[test]
    fn from_blob_rejects_non_finite_values() {
        let c = MagCalibration::default();
        let mut blob = c.to_blob();

        let offset = STORAGE_MAGIC.len() + 1 + STORAGE_RESERVED_LEN;
        let nan_bytes = f32::NAN.to_le_bytes();
        blob[offset..offset + 4].copy_from_slice(&nan_bytes);

        assert!(MagCalibration::from_blob(&blob).is_none());
    }

    #[test]
    fn is_usable_rejects_singular_matrix() {
        let c = MagCalibration::from_parts(Matrix3::zeros(), Vector3::zeros());

        assert!(!c.is_usable());
    }
}
