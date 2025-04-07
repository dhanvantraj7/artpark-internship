#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include <math.h>

#define MAX 8
#define NUM_SAMPLES 50

// Define structure for accelerometer and gyroscope data
typedef struct {
    float accx, accy, accz;
    float gyrx, gyry, gyrz; 
    float roll;
    float pitch;

} DATA;

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

const float deltaT = 6.75000000000001; //MILLISECONDS

static inline void readCsvData(const char* filename, float data[NUM_SAMPLES], int column) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        printf("Could not open file %s for reading.\n", filename);
        exit(1);
    }

    char buffer[1024];
    int i = 0;
    while (fgets(buffer, 1024, file) && i < NUM_SAMPLES) {
        char* token = strtok(buffer, ",");
        for (int j = 0; j <= column; ++j) {
            if (j == column) {
                data[i++] = atof(token);
            }
            token = strtok(NULL, ",");
        }
    }
    fclose(file);
}

static inline void initializeSensorData(DATA data[], int size, const char* filePath) {
    float accxValues[NUM_SAMPLES];
    float accyValues[NUM_SAMPLES];
    float acczValues[NUM_SAMPLES];
    float gyrxValues[NUM_SAMPLES];
    float gyryValues[NUM_SAMPLES];
    float gyrzValues[NUM_SAMPLES];


    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\AKF\\Roll_data.csv", accxValues, 0);
    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\AKF\\Roll_data.csv", accyValues, 1);
    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\AKF\\Roll_data.csv", acczValues, 2);
    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\AKF\\Roll_data.csv", gyrxValues, 3);
    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\AKF\\Roll_data.csv", gyryValues, 4);
    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\AKF\\Roll_data.csv", gyrzValues, 5);

    for (int i = 0; i < size; ++i) {
        data[i].accx = accxValues[i];
        data[i].accy = accyValues[i];
        data[i].accz = acczValues[i];
        data[i].gyrx = gyrxValues[i];
        data[i].gyry = gyryValues[i];
        data[i].gyrz = gyrzValues[i]; 
    }
}

void writeCsvData(const char* filename, DATA data[], int size) {
    FILE *file = fopen(filename, "w");
    if (!file) {
        // printf("Could not open file %s for writing.\n", filename);
        exit(1);
    }

    fprintf(file, "Roll,Pitch\n");

    for (int i = 0; i < size; ++i) {
        fprintf(file, "%.8f,%.8f\n",
                data[i].roll, data[i].pitch);
    }

    fclose(file);
    printf("Data written to %s successfully.\n", filename);
}


//Matrix operations to be done
static inline void InitIdentityMatrix(int r, float result[r][r]){
	for(int i = 0; i < r; i++){
		result[i][i] = 1.0;
	}
}

static inline void TransposeMatrix(int rows, int cols, float inputMatrix[rows][cols], float outputMatrix[cols][rows]) {
	for (int i = 0; i < rows; i++) {
	    for (int j = 0; j < cols; j++) {
	        outputMatrix[j][i] = inputMatrix[i][j];
	    }
	}
}

static inline void AddMatrices(int r, int c, float matrix1[r][c], float matrix2[r][c], float result[r][c]) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[i][j] = matrix1[i][j] + matrix2[i][j];
        }
    }
}

static inline void SubtractMatrices(int r, int c, float matrix1[r][c], float matrix2[r][c], float result[r][c]) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[i][j] = matrix1[i][j] - matrix2[i][j];
        }
    }
}

static inline void MultiplyMatrices(int r1, int c1, int r2, int c2, float matrix1[r1][c1], float matrix2[r2][c2], float result[r1][c2])
{
	if (c1 != r2) {
        return;
    }
    for (int i = 0; i < r1; i++) {
        for (int j = 0; j < c2; j++) {
            result[i][j] = 0;
        }
    }
	    // Perform matrix multiplication
    for (int i = 0; i < r1; i++) {
        for (int j = 0; j < c2; j++) {
            for (int k = 0; k < c1; k++) {
                result[i][j] += matrix1[i][k] * matrix2[k][j];
            }
        }
    }
}
float determinant(float matrix[MAX][MAX], int size) {
    float det = 0;
    float submatrix[MAX][MAX];
    if (size == 1) return matrix[0][0];

    for (int x = 0; x < size; x++) {
        int subi = 0;  // Submatrix's i index
        for (int i = 1; i < size; i++) {
            int subj = 0;  // Submatrix's j index
            for (int j = 0; j < size; j++) {
                if (j == x) continue;  // Skip the current column
                submatrix[subi][subj] = matrix[i][j];
                subj++;
            }
            subi++;
        }
        det += (x % 2 == 0 ? 1 : -1) * matrix[0][x] * determinant(submatrix, size - 1);
    }
    return det;
}
// Function to get the cofactor of a matrix
void cofactor(float matrix[MAX][MAX], float temp[MAX][MAX], int p, int q, int size) {
    int i = 0, j = 0;

    // Looping for each element of the matrix
    for (int row = 0; row < size; row++) {
        for (int col = 0; col < size; col++) {
            // Copying into temporary matrix only those elements
            // which are not in given row and column
            if (row != p && col != q) {
                temp[i][j++] = matrix[row][col];
                // Row is filled, so increase row index and reset col index
                if (j == size - 1) {
                    j = 0;
                    i++;
                }
            }
        }
    }
}
static inline void InvertMatrix(int r, float inputMatrix[r][r], float outputMatrix[r][r]){
	float det = determinant(inputMatrix, r);
	    if (det == 0) {
	        //printf("Singular matrix, can't find its inverse\n");
	        return;
	    }

	    // Find cofactor matrix
	    float adj[MAX][MAX];
	    for (int i=0; i<r; i++) {
	        for (int j=0; j<r; j++) {
	            float temp[MAX][MAX];
	            cofactor(inputMatrix, temp, i, j, r);
	            // sign of adj[j][i] positive if sum of row and column indexes is even.
	            float sign = ((i+j)%2==0)? 1: -1;
	            // Interchanging rows and columns to get the transpose of the cofactor matrix
	            adj[j][i] = (sign)*(determinant(temp, r-1));
	        }
	    }

	    // Divide adjoint by determinant to get the inverse
	    for (int i=0; i<r; i++)
	        for (int j=0; j<r; j++)
	            outputMatrix[i][j] = adj[i][j] / det;
}

static inline void scalarMatrixMultiplication(float scalar, int r, int c, float matrix[r][c], float result[r][c]) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[i][j] = scalar * matrix[i][j];
        }
    }
}

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
/***************************************************STEP 1 KALMAN FILTER PREDICTION***************************************************/
 
/*****************EQUATION 1 ; STATE PREDICTION ; (x_pred = A * x_est;) *****************/

static inline void QuaternionPred(DATA *data, AKF *Algo){

    float pi = 3.14159265359;

    //calculate roll and pitch
    data->roll = (atan((data->accy)/(sqrt((data->accx * data->accx) + (data->accz * data->accz)))))*180/pi;
	data->pitch = (atan((-data->accx)/(sqrt((data->accy * data->accy) + (data->accz * data->accz)))))*180/pi;
    printf("roll: %f ", data->roll);
	//getting A matrix
	Algo->A[0][0] = 0;
	Algo->A[0][1] = (-data->gyrx * deltaT)/2;
	Algo->A[0][2] = (-data->gyry * deltaT)/2;
	Algo->A[0][3] = (-data->gyrz * deltaT)/2;
	
   	Algo->A[1][0] = (data->gyrx * deltaT)/2;
    Algo->A[1][1] = 0;
	Algo->A[1][2] = (data->gyrz * deltaT)/2;
	Algo->A[1][3] = (-data->gyry * deltaT)/2;

	Algo->A[2][0] = (data->gyry * deltaT)/2;
	Algo->A[2][1] = (-data->gyrz * deltaT)/2;
    Algo->A[2][2] = 0;
	Algo->A[2][3] = (data->gyrx * deltaT)/2;

   	Algo->A[3][0] = (data->gyrz * deltaT)/2;
	Algo->A[3][1] = (data->gyry * deltaT)/2;
	Algo->A[3][2] = (-data->gyrx * deltaT)/2;
    Algo->A[3][3] = 0;

	TransposeMatrix(4, 4, Algo->A, Algo->AT);

	// quaternions 
	Algo->q_x[0][0] = 1;
	Algo->q_x[0][1] = 0;
	Algo->q_x[0][2] = 0;
	Algo->q_x[0][3] = 0;

    //Print A matrix
    // printf("A matrix:\n");
    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 4; j++) {
    //         printf("%f ", Algo->A[i][j]);
    //     }
    //     printf("\n");
    // }

	TransposeMatrix(1, 4, Algo->q_x, Algo->q_xT);
   
    // // Print q_x matrix
    // printf("q_x matrix:\n");
    // for (int i = 0; i < 1; i++) {
    //     for (int j = 0; j < 4; j++) {
    //         printf("%f ", Algo->q_x[i][j]);
    //     }
    //     printf("\n");
    // }

    
	// q_x1 = A * q_xT;       
	MultiplyMatrices(4, 4, 4, 1, Algo->A, Algo->q_xT, Algo->q_x1);
    
    // // Print q_x1 matrix
	// printf("q_x1 matrix:\n");
    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 1; j++) {
    //         printf("%f ", Algo->q_x1[i][j]);
    //     }
    //     printf("\n");
    // }


/*****************EQUATION 2 ; COVARIANCE PREDICTION ; (P_pred = A * P_est * A' + Q;) *****************/

	//temp = A * P_i
	MultiplyMatrices(4, 4, 4, 4, Algo->A, Algo->P_i, Algo->temp);
    
    printf("temp matrix:\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f ", Algo->temp[i][j]);
        }
        printf("\n");
    }

	//temp1 = temp*AT
	MultiplyMatrices(4, 4, 4, 4, Algo->temp, Algo->AT, Algo->temp1);
	
    printf("temp1 matrix:\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f ", Algo->temp1[i][j]);
        }
        printf("\n");
    }

    //P_x = temp1 + Q
	AddMatrices(4, 4, Algo->temp1, Algo->Q, Algo->P_x);

    printf("P_x:\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f ", Algo->P_x[i][j]);
        }
        printf("\n");
    }


/***************************************************STEP 2 KALMAN FILTER CORRECTION***************************************************/

/*****************EQUATION 3 ; INNOVATION (y = z - H * x_pred;) *****************/

    //creating H
	Algo->H[0][0] = 0;
	Algo->H[0][1] = 1;
	Algo->H[0][2] = 0;
	Algo->H[0][3] = 0;

	Algo->H[1][0] = 0;
	Algo->H[1][1] = 0;
	Algo->H[1][2] = 1;
	Algo->H[1][3] = 0;

	// Algo->H[2][0] = 2 * ((Algo->x[0][0] * Algo->q_xT[2][0]) - (Algo->x[1][0] * Algo->q_xT[1][0]) + (Algo->x[2][0] * Algo->q_xT[0][0]));
	// Algo->H[2][1] = 2 * ((Algo->x[0][0] * Algo->q_xT[3][0]) - (Algo->x[1][0] * Algo->q_xT[0][0]) - (Algo->x[2][0] * Algo->q_xT[1][0]));
	// Algo->H[2][2] = 2 * ((Algo->x[0][0] * Algo->q_xT[0][0]) + (Algo->x[1][0] * Algo->q_xT[3][0]) - (Algo->x[2][0] * Algo->q_xT[2][0]));
	// Algo->H[2][3] = 2 * ((Algo->x[0][0] * Algo->q_xT[1][0]) + (Algo->x[1][0] * Algo->q_xT[2][0]) + (Algo->x[2][0] * Algo->q_xT[3][0]));

    // temp2 = H * q_x1;     
    MultiplyMatrices(2, 4, 4, 1, Algo->H, Algo->q_x1, Algo->temp2);

    //getting accelerometer sensor data z_t
    Algo->z_t[0][0] = data->roll;
	Algo->z_t[1][0] = data->pitch;

    // y_t = z_t - temp2;
    SubtractMatrices(2, 1, Algo->z_t, Algo->temp2, Algo->y_t);

    printf("y_t:\n");
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 1; j++) {
            printf("%f ", Algo->y_t[i][j]);
        }
        printf("\n");
    }

/*****************EQUATION 4 ; INNOVATION COVARIANCE (S = H * P_pred * H' + R;) *****************/

    //temp3 = H * P_x
	MultiplyMatrices(2, 4, 4, 4, Algo->H, Algo->P_x, Algo->temp3);
    
    //HT
    TransposeMatrix(2, 4, Algo->H, Algo->HT);

	//temp4 = temp3 * HT
	MultiplyMatrices(2, 4, 4, 2, Algo->temp3, Algo->HT, Algo->temp4);

    //S_t = temp4 + R
    AddMatrices(2, 2, Algo->temp4, Algo->R, Algo->S_t);

    printf("S_t:\n");
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            printf("%f ", Algo->S_t[i][j]);
        }
        printf("\n");
    }

/*****************EQUATION 5 ; KALMAN GAIN (K = P_pred * H' / S;) *****************/
    //temp_5 = P_x * HT
   	MultiplyMatrices(4, 4, 4, 2, Algo->P_x, Algo->HT, Algo->temp5);
  
    //inverse the S matrix
    InvertMatrix(2, Algo->S_t, Algo->S_t_inv);

    //K = temp5 * S_t_inv 
   	MultiplyMatrices(4, 2, 2, 2, Algo->temp5, Algo->S_t_inv, Algo->K);

    printf("K:\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            printf("%f ", Algo->K[i][j]);
        }
        printf("\n");
    }

/*****************EQUATION 6 ; STATE UPDATION (x_est = x_pred + K * y;) *****************/

    //temp6 = K * y_t
 	MultiplyMatrices(4, 2, 2, 1, Algo->K, Algo->y_t, Algo->temp6);

    //q_xT = temp6 + q_x1
    AddMatrices(4, 1, Algo->temp6, Algo->q_x1, Algo->q_xT);
    
    printf("q_xT:\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 1; j++) {
            printf("%f ", Algo->q_xT[i][j]);
        }
        printf("\n");
    }

/*****************EQUATION 7 ; COVARIANCE UPDATION (P_est = (eye(size(K,1)) - K * H) * P_pred;) *****************/

    //temp7 = K * H
 	MultiplyMatrices(4, 2, 2, 4, Algo->K, Algo->H, Algo->temp7);

    InitIdentityMatrix(4, Algo->I);
    //temp8 = I - temp7
    SubtractMatrices(4, 4, Algo->I, Algo->temp7, Algo->temp8);

    //P_i = temp8 * P_x 
    MultiplyMatrices(4, 4, 4, 4, Algo->temp8, Algo->P_x, Algo->P_i);
   
    printf("P_i:\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f ", Algo->P_i[i][j]);
        }
        printf("\n");
    }


/***************************************************STEP 3 KALMAN FILTER ADAPTATION***************************************************/

/*****************EQUATION 8 ; (Q_scale_factor = alpha_Q * innovation_variance;) *****************/
    float a_Q = 0.1187309612;
    float innovation_variance;
    // float variance(const float *matrix, int rows, int cols) {
    //     if (rows * cols <= 1) {
    //         // Variance is undefined for a single value or empty matrix
    //         return 0.0f; // Return 0 as a convention
    //     }

    //     // Compute the mean of the values
    //     float sum = 0.0f;
    //     for (int i = 0; i < rows; i++) {
    //         for (int j = 0; j < cols; j++) {
    //             sum += matrix[i * cols + j];
    //         }
    //     }
    //     float mean = sum / (rows * cols);

    //     // Compute the sum of squared differences from the mean
    //     float sum_squared_diff = 0.0f;
    //     for (int i = 0; i < rows; i++) {
    //         for (int j = 0; j < cols; j++) {
    //             float diff = matrix[i * cols + j] - mean;
    //             sum_squared_diff += diff * diff;
    //         }
    //     }

    //     // Compute the variance
    //     float variance = sum_squared_diff / (rows * cols - 1); // Use (rows * cols - 1) for unbiased estimation

    //     return variance;
    // }
    
    // float innovation_variance = variance(Algo->y_t, 2, 1);
    matrix_mean_and_variance(2, 1, Algo->y_t, &innovation_variance);
    printf("Variance: %f\n", innovation_variance);

    float Q_sf = a_Q * innovation_variance;
    printf("Q_sf: %f\n", Q_sf);

/*****************EQUATION 9 ; (Q = (1 - alpha_Q) * Q + Q_scale_factor * eye(size(Q));) *****************/

    //temp9 = (1 - a_Q) * Q 
    scalarMatrixMultiplication((1 - a_Q), 4, 4, Algo->Q, Algo->temp9);
    Algo->temp9[4][4] = (1 - a_Q) * Algo->Q[4][4];

    //Q_sf_I = Q_sf * I1
    InitIdentityMatrix(4, Algo->I1);
   
    //Q = temp9 + Q_sf_I
    AddMatrices(4, 4, Algo->temp9, Algo->Q_sf_I, Algo->Q);

    printf("Q:\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f ", Algo->Q[i][j]);
        }
        printf("\n");
    }
/*****************EQUATION 10 ; adaptation based on latest measurement(R = (1 - alpha_R) * R + alpha_R * (y * y');) *****************/
    float a_R = 0.1187309612;

    //temp10 = (1 - a_R) * R
    scalarMatrixMultiplication((1 - a_R), 2, 2, Algo->R, Algo->temp10);

    //temp11 = y_t * y_tT 
    TransposeMatrix(2, 1, Algo->y_t, Algo->y_tT);
    MultiplyMatrices(2, 1, 1, 2, Algo->y_t, Algo->y_tT, Algo->temp11);

    //temp12 = a_R * temp11
    scalarMatrixMultiplication(a_R, 2, 2, Algo->temp11, Algo->temp12);

    //R = temp10 + temp12
    AddMatrices(4, 4, Algo->temp10, Algo->temp12, Algo->R);
    printf("R:\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%f ", Algo->R[i][j]);
        }
        printf("\n");
    }
}

int main() {

    DATA data;
    AKF Algo;

    const char* filePath = "C:\\Users\\Camras\\Documents\\VS code\\AKF\\Roll_data.csv";
    initializeSensorData(&data, NUM_SAMPLES, filePath);

    const char* outputFilePath = "C:\\Users\\Camras\\Documents\\VS code\\AKF\\AKF_Roll_data.csv";
    writeCsvData(outputFilePath, &data, NUM_SAMPLES);

    // Call QuaternionPred function
    QuaternionPred(&data, &Algo);
    return 0;
}

