float Kalman(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // Intialization
  //A = state transition matrix, based on the angular velocity measurements.
  //B = 0 (in this case) input matrix, relating the control input to the state change
  //Q = constant value (in this case) Process noise: Represents the uncertainty in the state transition due to factors like unmodeled dynamics.
  //R = constant value (in this case) Measurement noise: Represents the uncertainty in the gyroscope measurements.

  /*H = 1 (in this case)  measurement matrix: as the sensor reads the data directly adn we are not considerinf other parameters*/

loop{
            //Prediction equations

            //State Prediction
            KalmanState[k+1] = A * KalmanState[k] + B * ControlInput[k]

            //Covariance Prediction
            KalmanUncertainty[k+1] = A * KalmanUncertainty[k] * A^T + Q

            //Update Equations (when a new measurement arrives)

            //Kalman gain calculation
            G = kalmanUncertainty[k+1] * H^T / (H * kalmanUncertainty_k+1 * H^T + R)

            //State Update
            KalmanState_[k+1] = KalmanState[k+1] + update_factor

            //Update factor gives the gain multiplied with error between true measurement and predicted measurement
            update_factor = G * (Readings - H * KalmanState[k+1])

            //Covariance Update
            KalmanUncertainty_[k+1] = (I - G * H) * KalmanUncertainty[k+1]
      }
}



// Set parameters
Nsamples = 41500 // Number of data points
dt = 0.01 // Time step (seconds)
ignore_gravity = 0 // Set to 1 to ignore gravity (optional)

// Create data containers
gyro_data = array of size Nsamples x 3 (store gyro measurements)
accel_data = array of size Nsamples x 2 (store accelerometer measurements, if used)
euler_saved = array of size Nsamples x 3 (store calculated Euler angles)

// Get gyro data for all samples
for k = 1 to Nsamples:
    get_gyro(w1, w2, w3) // Replace with specific hardware implementation
    gyro_data[k, 0] = w1
    gyro_data[k, 1] = w2
    gyro_data[k, 2] = w3

// Get accelerometer data and calculate initial Euler angles (if gravity is used)
if not ignore_gravity:
    for k = 1 to Nsamples:
        get_accel(f1, f2) // Replace with specific hardware implementation
        euler_accel(f1, f2, theta, phi) /* Consider alternative calculation methods
        g = 9.8; // average of sqrt(ax^2 + ay^2 + az^2)
        theta = asin(  ax / g );
        phi   = asin( -ay / (g*cos(theta)) );*/

        euler_saved[k, 1] = theta
        euler_saved[k, 2] = phi

// Initialize filter state variables
psi = 0
theta = 0
phi = 0


    // Calculate state transition matrix A based on gyro data  
    A = Identity matrix(4) + dt*1/2*[ 0   -w1  -w2  -w3 ;
                                      w1   0    w3  -w2 ;
                                      w2  -w3   0    w1 ;
                                      w3   w2  -w1   0  ];
    //
    // x = Euler Parameter, beta (aka, quaternion) initially q0 = 1 , q1 = 0, q2 = 0, q3 = 0
    x = [ q0 ;
          q1 ;
          q2 ;
          q3 ];
    // P = Identity matrix(4); 
    // Q = 0.0001; 
    W = dt*1/2*[ -q1  -q2  -q3 ;
                  q0  -q3   q2 ;
                  q3   q0  -q1 ;
                 -q2   q1   q0 ];
    C = [q_hat_0^2 + q_hat_1^2 - q_hat_2^2 - q_hat_3^2  2(q_hat_1 * q_hat_2 - )];
    z = [a1 ;
         a2 ;
         a3 ;
         m1 ;
         m2 ;
         m3 ]; // data from sensors
    
    // omega
                        /*// Function prototype
                    void omega(float x[3], float result[4][4]);

                    // Implementation of the Omega operator
                    void omega(float x[3], float result[4][4]) {
                        // Initializing the result matrix to zero
                        for (int i = 0; i < 4; i++) {
                            for (int j = 0; j < 4; j++) {
                                result[i][j] = 0.0f;
                            }
                        }

                        // Filling in the non-zero values according to the Omega operator
                        result[0][1] = -x[0];
                        result[0][2] = -x[1];
                        result[0][3] = -x[2];
                        result[1][0] = x[0];
                        result[1][2] = x[2];
                        result[1][3] = -x[1];
                        result[2][0] = x[1];
                        result[2][1] = -x[2];
                        result[2][3] = x[0];
                        result[3][0] = x[2];
                        result[3][1] = x[1];
                        result[3][2] = -x[0];
                    }

                    // Example usage
                    int main() {
                        float x[3] = {1.0f, 2.0f, 3.0f}; // Example vector
                        float result[4][4]; // To hold the Omega matrix

                        omega(x, result);

                        // Printing the result for demonstration
                        for (int i = 0; i < 4; i++) {
                            for (int j = 0; j < 4; j++) {
                                printf("%.2f ", result[i][j]);
                            }
                            printf("\n");
                        }

                        return 0;
                    }
                    */

    // f
                    /*
                // Function to calculate the skew-symmetric Omega matrix for angular velocity
                void omega(float omega[3], float result[4][4]) {
                    for (int i = 0; i < 4; ++i)
                        for (int j = 0; j < 4; ++j)
                            result[i][j] = 0.0f;  // Initialize result with zeros

                    result[0][1] = -omega[0];
                    result[0][2] = -omega[1];
                    result[0][3] = -omega[2];
                    result[1][0] = omega[0];
                    result[1][2] = omega[2];
                    result[1][3] = -omega[1];
                    result[2][0] = omega[1];
                    result[2][1] = -omega[2];
                    result[2][3] = omega[0];
                    result[3][0] = omega[2];
                    result[3][1] = omega[1];
                    result[3][2] = -omega[0];
                }

                // Function to calculate the Jacobian of the linearized predicted state
                void dfdq(float omega[3], float dt, float F[4][4]) {
                    float Om[4][4];
                    omega(omega, Om); // Calculate Omega matrix

                    // Start with the identity matrix
                    for (int i = 0; i < 4; ++i)
                        for (int j = 0; j < 4; ++j)
                            F[i][j] = (i == j) ? 1.0f : 0.0f;

                    // Add (1/2) * Omega * dt to the identity matrix
                    for (int i = 0; i < 4; ++i)
                        for (int j = 0; j < 4; ++j)
                            F[i][j] += 0.5f * Om[i][j] * dt;
                }

                // Example usage
                int main() {
                    float omega[3] = {0.1f, 0.2f, 0.3f}; // Angular velocity vector
                    float dt = 0.01f; // Time step
                    float F[4][4]; // Jacobian matrix

                    dfdq(omega, dt, F);

                    // Printing the Jacobian matrix
                    for (int i = 0; i < 4; i++) {
                        for (int j = 0; j < 4; j++) {
                            printf("%f ", F[i][j]);
                        }
                        printf("\n");
                    }

                    return 0;
                }
                */

    // F
                                /*typedef struct {
                                double w, x, y, z;
                            } Quaternion;

                            // Function to multiply two quaternions
                            Quaternion quaternion_multiply(Quaternion q1, Quaternion q2) {
                                Quaternion result;
                                result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
                                result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
                                result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
                                result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
                                return result;
                            }

                            // Function to scale a quaternion
                            Quaternion quaternion_scale(Quaternion q, double scale) {
                                q.w *= scale;
                                q.x *= scale;                                                                                
                                q.y *= scale;
                                q.z *= scale;
                                return q;
                            }

                            // Function to add two quaternions
                            Quaternion quaternion_add(Quaternion q1, Quaternion q2) {
                                q1.w += q2.w;
                                q1.x += q2.x;
                                q1.y += q2.y;
                                q1.z += q2.z;
                                return q1;
                            }

                            // Function to update quaternion based on angular velocity
                            Quaternion update_quaternion(Quaternion q, double omega_x, double omega_y, double omega_z, double dt) {
                                Quaternion omega_prime = {0, omega_x, omega_y, omega_z};
                                Quaternion dq = quaternion_multiply(q, omega_prime);
                                dq = quaternion_scale(dq, 0.5 * dt);
                                return quaternion_add(q, dq);
                            }

                            int main() {
                                // Example usage
                                Quaternion q = {1, 0, 0, 0}; // Identity quaternion
                                double omega_x = 0.1, omega_y = 0.2, omega_z = 0.3; // Example angular velocities in rad/s
                                double dt = 0.01; // Time step in seconds

                                Quaternion q_new = update_quaternion(q, omega_x, omega_y, omega_z, dt);

                                printf("Updated Quaternion: (%f, %f, %f, %f)\n", q_new.w, q_new.x, q_new.y, q_new.z);

                                return 0;
                            }
                            */

    // h measurement model
                                    /*// Prototype for quaternion to expected measurement conversion
                    void h(const double q[4], double *measurement, int *measurement_size) {
                        // Assuming the gravity vector is aligned with the Z-axis in the sensor frame
                        const double g[3] = {0.0, 0.0, -9.81}; // Gravity vector
                        
                        // Convert quaternion to rotation matrix
                        double R[3][3] = {
                            {2*(q[0]*q[0] + q[1]*q[1]) - 1, 2*(q[1]*q[2] - q[0]*q[3]), 2*(q[1]*q[3] + q[0]*q[2])},
                            {2*(q[1]*q[2] + q[0]*q[3]), 2*(q[0]*q[0] + q[2]*q[2]) - 1, 2*(q[2]*q[3] - q[0]*q[1])},
                            {2*(q[1]*q[3] - q[0]*q[2]), 2*(q[2]*q[3] + q[0]*q[1]), 2*(q[0]*q[0] + q[3]*q[3]) - 1}
                        };
                        
                        // Compute the expected measurement due to gravity
                        for (int i = 0; i < 3; i++) {
                            measurement[i] = 0;
                            for (int j = 0; j < 3; j++) {
                                measurement[i] += R[i][j] * g[j];
                            }
                        }
                        
                        // Set measurement size (3 for gravity, 6 for gravity + geomagnetic field)
                        *measurement_size = 3; // Update this value to 6 if including geomagnetic field
                    }*/

    // H = Identity matrix(4);
                        /*void dhdq(const double q[4], double H[3][4], const char* mode) {
                        // Placeholder values for demonstration; actual computation depends on the measurement model
                        // For a gravity-based model, this might involve the partial derivatives of the
                        // rotation matrix elements with respect to the quaternion components.
                        
                        // For simplicity, assume a direct relation for demonstration
                        memset(H, 0, sizeof(double) * 3 * 4); // Initialize H to zeros
                        
                        if (strcmp(mode, "normal") == 0) {
                            // Compute Jacobian for "normal" mode
                            // This is a placeholder; actual derivative calculations should go here
                        } else if (strcmp(mode, "refactored") == 0) {
                            // Compute Jacobian for "refactored" mode with potentially different calculations
                            // This is a placeholder; actual derivative calculations should go here
                        }
                        
                        // Example placeholder logic; replace with actual computations
                        for (int i = 0; i < 3; ++i) {
                            for (int j = 0; j < 4; ++j) {
                                H[i][j] = (i + 1) * (j + 1); // Placeholder; not actual derivative calculation
                            }
                        }
                    }*/
    //update equation
                    /*void updateQuaternion(Quaternion *q, const float gyr[3], const float acc[3], const float mag[3], float dt);

                void updateQuaternion(Quaternion *q, const float gyr[3], const float acc[3], const float mag[3], float dt) {
                    // Check if the magnetometer data is provided
                    bool isMagAvailable = mag != NULL;

                    // Check if dt is provided (non-zero)
                    bool isDtAvailable = dt != 0.0f;

                    // Placeholder for the algorithm to update the quaternion based on the sensor readings.
                    // This part will depend on the specific algorithm you're using.
                    // For simplicity, let's just demonstrate modifying the quaternion slightly.
                    q->w += dt * gyr[0]; // This is not a real calculation, just a placeholder.
                    q->x += dt * gyr[1];
                    q->y += dt * gyr[2];
                    q->z += dt * (isMagAvailable ? mag[0] : 0.0f); // Example adjustment based on magnetometer

                    // Normally, here you would include the actual math to integrate the gyroscope readings,
                    // correct the drift with the accelerometer (and magnetometer, if available),
                    // and normalize the quaternion.
                }

                // Example usage
                int main() {
                    Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f}; // Example initial quaternion
                    float gyr[3] = {0.01f, 0.02f, 0.03f}; // Example gyroscope readings
                    float acc[3] = {9.81f, 0.0f, 0.0f}; // Example accelerometer readings
                    float mag[3] = {0.0f, 0.0f, 0.0f}; // Example magnetometer readings
                    float dt = 0.1f; // Time step

                    updateQuaternion(&q, gyr, acc, mag, dt);

                    // q now contains the updated orientation
                    return 0;
                }*/
    // R = 10*Identity matrix(4);


// Kalman filter loop for each sample
for k = 1 to Nsamples:
    
    //Prediction equations

            //State Prediction
            xp = A * x;

            //Covariance Prediction
            Pp = A * P * A^T + W * Q * W^T;

            //Measurement Residual
            V = (z - h*xp);

            //Maeasurement Prediction Covariance
            S = H * Pp * H^T + R;

            //Kalman gain calculation
            K = Pp * H^T / S;

            //State Update
            x = xp + K * v;

            //Covariance Update
            P = (I - K * H) * Pp

            // Kalman Filter Loop
for each sample k from 1 to Nsamples:
    // Prediction
    xp = A * x  // Predict next state
    Pp = A * P * A' + Q  // Predict error covariance
    
    // If using quaternions for orientation
    Update quaternion based on gyro data

    // Update (when a new measurement z is available)
    Z = Get measurement data // Could be a combination of gyro, accel, and magnetometer data
    V = Z - H * xp  // Measurement residual
    S = H * Pp * H' + R  // Measurement prediction covariance
    K = Pp * H' / S  // Kalman gain    
    x = xp + K * V  // Update state estimate with measurement
    P = (I - K * H) * Pp  // Update error covariance

    // Optional: Normalize quaternion to maintain unit length

// Repeat for each sample