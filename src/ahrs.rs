use estima::manifold::{
    composite::CompositeManifold, euclidean::EuclideanManifold, quaternion::UnitQuaternionManifold,
    ManifoldMeasurement, ManifoldProcess,
};
use estima::sigma_points::MerweScaledSigmaPoints;
use estima::{UkfError, UnscentedKalmanFilter};
use nalgebra::{Matrix6, UnitQuaternion, Vector3, Vector6, U3, U6};

const DT_DEFAULT: f32 = 0.05;
const GYRO_NOISE: f32 = 0.01;
const ACCEL_NOISE: f32 = 0.1;
const MAG_NOISE: f32 = 0.1;
const MIN_SENSOR_NORM_SQ: f32 = 1e-12;

pub type AttitudeManifold = UnitQuaternionManifold<f32>;
pub type BiasManifold = EuclideanManifold<f32, U3>;
pub type AHRSState = CompositeManifold<f32, AttitudeManifold, BiasManifold, U3, U3>;

pub fn ahrs_state_new(attitude: UnitQuaternion<f32>, bias: Vector3<f32>) -> AHRSState {
    CompositeManifold::new(
        UnitQuaternionManifold::new(attitude),
        EuclideanManifold::new(bias),
    )
}

pub fn attitude_component(state: &AHRSState) -> &UnitQuaternion<f32> {
    state.first.as_quaternion()
}

pub fn bias_component(state: &AHRSState) -> &Vector3<f32> {
    state.second.as_vector()
}

#[derive(Clone)]
pub struct GyroscopeProcess;

impl ManifoldProcess<AHRSState, U3, f32> for GyroscopeProcess {
    fn predict(&self, state: &AHRSState, dt: f32, control: Option<&Vector3<f32>>) -> AHRSState {
        let bias = bias_component(state);
        let attitude = attitude_component(state);

        if let Some(gyro) = control {
            let corrected = gyro - bias;
            let delta = UnitQuaternion::from_scaled_axis(corrected * dt);
            ahrs_state_new(attitude * delta, *bias)
        } else {
            state.clone()
        }
    }
}

#[derive(Clone)]
pub struct AccelMagMeasurement {
    gravity_ref: Vector3<f32>,
    magnetic_ref: Vector3<f32>,
}

impl AccelMagMeasurement {
    pub fn new() -> Self {
        Self {
            gravity_ref: Vector3::new(0.0, 0.0, -9.81),
            // Don't forget to change for your location.
            // ENU frame with Z up: [E, N, U].
            magnetic_ref: Vector3::new(0.03166, 0.40230, -0.91496),
        }
    }

    fn normalize_or_zero(vector: Vector3<f32>) -> Vector3<f32> {
        vector.try_normalize(1e-12).unwrap_or_else(Vector3::zeros)
    }
}

impl Default for AccelMagMeasurement {
    fn default() -> Self {
        Self::new()
    }
}

impl ManifoldMeasurement<AHRSState, U6, U6, f32> for AccelMagMeasurement {
    fn measure(&self, state: &AHRSState) -> Vector6<f32> {
        let attitude = attitude_component(state);
        let accel = -attitude.inverse().transform_vector(&self.gravity_ref);
        let mag = attitude.inverse().transform_vector(&self.magnetic_ref);

        let accel_norm = Self::normalize_or_zero(accel);
        let mag_norm = Self::normalize_or_zero(mag);

        Vector6::new(
            accel_norm.x,
            accel_norm.y,
            accel_norm.z,
            mag_norm.x,
            mag_norm.y,
            mag_norm.z,
        )
    }

    fn residual(&self, predicted: &Vector6<f32>, measured: &Vector6<f32>) -> Vector6<f32> {
        let pred_accel =
            Self::normalize_or_zero(Vector3::new(predicted[0], predicted[1], predicted[2]));
        let pred_mag =
            Self::normalize_or_zero(Vector3::new(predicted[3], predicted[4], predicted[5]));

        let meas_accel =
            Self::normalize_or_zero(Vector3::new(measured[0], measured[1], measured[2]));
        let meas_mag = Self::normalize_or_zero(Vector3::new(measured[3], measured[4], measured[5]));

        let accel_residual = pred_accel.cross(&meas_accel);
        let mag_residual = pred_mag.cross(&meas_mag);

        Vector6::new(
            accel_residual.x,
            accel_residual.y,
            accel_residual.z,
            mag_residual.x,
            mag_residual.y,
            mag_residual.z,
        )
    }

    fn innovation(&self, measured: &Vector6<f32>, predicted_mean: &Vector6<f32>) -> Vector6<f32> {
        self.residual(predicted_mean, measured)
    }
}

pub struct AhrsUkf {
    ukf: UnscentedKalmanFilter<
        AHRSState,
        GyroscopeProcess,
        AccelMagMeasurement,
        U6,
        U3,
        U6,
        MerweScaledSigmaPoints<f32>,
        f32,
    >,
}

impl AhrsUkf {
    pub fn new() -> Self {
        let initial_state = ahrs_state_new(UnitQuaternion::identity(), Vector3::zeros());
        let initial_covariance = Matrix6::<f32>::identity() * 0.2;

        let mut process_noise = Matrix6::<f32>::zeros();
        process_noise
            .fixed_view_mut::<3, 3>(0, 0)
            .fill_diagonal(0.05f32 * 0.05f32);
        process_noise
            .fixed_view_mut::<3, 3>(3, 3)
            .fill_diagonal(GYRO_NOISE * GYRO_NOISE * DT_DEFAULT);

        let mut measurement_noise = Matrix6::<f32>::zeros();
        measurement_noise
            .fixed_view_mut::<3, 3>(0, 0)
            .fill_diagonal(ACCEL_NOISE * ACCEL_NOISE);
        measurement_noise
            .fixed_view_mut::<3, 3>(3, 3)
            .fill_diagonal(MAG_NOISE * MAG_NOISE);

        let sigma_gen = MerweScaledSigmaPoints::new(0.5, 2.0, 0.0);
        let weights = sigma_gen.weights::<U6>();

        let ukf = UnscentedKalmanFilter::new(
            initial_state,
            initial_covariance,
            GyroscopeProcess,
            process_noise,
            AccelMagMeasurement::new(),
            measurement_noise,
            sigma_gen,
            weights,
        )
        .with_regularization_factor(1e-3);

        Self { ukf }
    }

    pub fn predict(&mut self, dt: f32, gyro: Option<Vector3<f32>>) -> Result<(), UkfError> {
        self.ukf.predict(dt, gyro.as_ref())
    }

    pub fn update(&mut self, accel: Vector3<f32>, mag: Vector3<f32>) -> Result<bool, UkfError> {
        if accel.norm_squared() < MIN_SENSOR_NORM_SQ || mag.norm_squared() < MIN_SENSOR_NORM_SQ {
            return Ok(false);
        }

        let measurement = Vector6::new(accel.x, accel.y, accel.z, mag.x, mag.y, mag.z);
        self.ukf.update(&measurement)?;
        Ok(true)
    }

    pub fn attitude(&self) -> &UnitQuaternion<f32> {
        attitude_component(self.ukf.nominal_state())
    }

    pub fn bias(&self) -> &Vector3<f32> {
        bias_component(self.ukf.nominal_state())
    }
}
