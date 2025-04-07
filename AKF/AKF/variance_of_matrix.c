#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct {

    float q_x1[4][1];
   	float q_x[1][4];
	float q_xT[4][1];
	
	float P_x[4][4];
	float A[4][4];
    float P_i[4][4];
    float temp[4][4];
    float temp1[4][4];
	float AT[4][4];
    
    float H[2][4];
    float HT[4][2];
    float temp2[2][1];
	float Q[4][4];

	float z_t[2][1];
	float y_t[2][1];
    float temp3[2][4];
    float temp4[2][2];
    float temp5[4][2];

    float R[2][2];
    float S_t[2][2];
    float S_t_inv[2][2];
    float K[4][2];
	float temp6[4][1];
    float temp7[4][4];
    float temp8[4][4];

    float temp9[4][4];
    float Q_sf_I[4][4];
    float temp10[4][4];
    float y_tT[1][2];
    float temp11[2][2];
    float temp12[2][2];
    float I[4][4];
    float I1[4][4];
					
	// float a_sigma[3];
    // float g_sigma[3];
	// float P_x0[4][4];
	
	// float q_x1fin[4][1];
	// float finAngles[3];			
} AKF;

// Function to compute the mean and variance of a matrix
void matrix_mean_and_variance(int r, int c, float matrix[r][c], float *variance) {
    // Compute the mean of the matrix
    float sum = 0.0f;
    float mean = 0.0f;
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            sum += matrix[i][j];
        }
    }
    mean = sum / (r * c); // Store the mean value in the variable pointed to by 'mean'

    // Compute the sum of squared differences from the mean
    float sum_squared_diff = 0.0f;
    float diff = 0.0f;
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            diff = matrix[i][j] - mean;
            sum_squared_diff += diff * diff;
        }
    }

    // Compute the variance
    *variance = sum_squared_diff / (r * c);
}

int main() {
    // Example matrix
    int r = 2;
    int c = 3;
    float matrix[2][3] = {
        {1.2f, 2.3f, 3.4f},
        {4.5f, 5.6f, 6.7f}
    };
    float e=0;
    // Compute the mean and variance
    matrix_mean_and_variance(2, 3, matrix, &e);

    // Print the results
    // printf("Mean: %f\n", mean);
    printf("Variance: %f\n", e);

    return 0;
}

// #include <stdio.h>

// // static inline void matrix_mean(int r, int c, float matrix[r][c], float *mean) {
//     float sum = 0.0f;
//     for (int i = 0; i < r; i++) {
//         for (int j = 0; j < c; j++) {
//             sum += matrix[i][j];
//         }
//     }
//     *mean = sum / (r * c); // Store the mean value in the variable pointed to by 'mean'
// }

// void matrix_variance(int r, int c, float matrix[r][c], float *variance) {

//     // Compute the mean of the matrix
//     float mean = matrix_mean(r, c, matrix);

//     // Compute the sum of squared differences from the mean
//     float sum_squared_diff = 0.0f;
//     for (int i = 0; i < r * c; i++) {
//         float diff = matrix[i] - mean;
//         sum_squared_diff += diff * diff;
//     }

//     // Compute the variance
//     *variance = sum_squared_diff / (rows * cols);
// }
// int main() {
//     // Example matrix
//     int rows = 3;
//     int cols = 3;
//     float matrix[3][3] = {
//         {1.0f, 2.0f, 3.0f},
//         {4.0f, 5.0f, 6.0f},
//         {7.0f, 8.0f, 9.0f}
//     };

//     // Variable to store the mean
//     float mean;

//     // Compute the mean
//     matrix_mean(rows, cols, matrix, &mean); // Pass the address of the 'mean' variable

//     // Print the mean
//     printf("Mean: %f\n", mean);

//     return 0;
// }
