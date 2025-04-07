#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include <math.h>

#define MAX 3 //for inverse matrix
#define NUM_SAMPLES 5560
#define R_TO_D 180/3.14159265359


// Define structure for accelerometer and gyroscope data
typedef struct {
    float accx, accy, accz;
    float gyrx, gyry, gyrz; 
    float roll;
    float pitch;
    float estimatedroll;
    float estimatedpitch;
    float estimatedyaw;
} DATA;

typedef struct {
    float q[4][NUM_SAMPLES];
    float q_x1[4][1];
    float qwxyz[4][1];
    float q_x1wxyz[4][1];

    float q_x2[4][1];
	float q_x3[4][1];

	float P_x[4][4];
	float A[4][4];
    float P_i[4][4];
    float temp[4][4];
    float temp1[4][4];
	float AT[4][4];
    float W[4][3];
    float WT[3][4];
    float temp2[4][3];
    float var[3][3];

    float H[3][4];
    float temp3[3][4];

    float HT[4][3];
	float Q[4][4];

    float temp4[3][3];
    float S_t[3][3];
    float S_t_inv[3][3];
    float temp5[3][3];
    float R[3][3];

    float C[3][3];
    float x[3][1];
    float h[3][1];

	float z_t[3][1];
	float y_t[3][1];

    float g[3][1];

    
    float K[4][3];
	float temp6[4][1];
    float temp7[4][1];

    float temp8[4][4];

    float temp9[4][4];
    float I[4][4];

    float Theetha_final[NUM_SAMPLES][1];
    float Pitch_final[NUM_SAMPLES][1];
    float Yaw_final[NUM_SAMPLES][1]; 
    float q_x1_history[NUM_SAMPLES][4][1]; // History of q_x1 values for each sample	
} EKF;

const float deltaT = 0.00675; 
const float asigmax = 0.003452073181473;
const float asigmay = 0.002956490260864;
const float asigmaz = 0.003215714364151;
const float gsigmax = 0.092837817557786;
const float gsigmay = 0.103655999126007;
const float gsigmaz = 0.110589671061154;

const float rollavg = 0.034007793773762;
const float pitchavg = -0.012051372646153;
const float yaw = 0;

float qw = 0;
float qx = 0;
float qy = 0;
float qz = 0;

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


    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\HP_EKF\\data\\data.csv", accxValues, 0);
    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\HP_EKF\\data\\data.csv", accyValues, 1);
    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\HP_EKF\\data\\data.csv", acczValues, 2);
    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\HP_EKF\\data\\data.csv", gyrxValues, 3);
    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\HP_EKF\\data\\data.csv", gyryValues, 4);
    readCsvData("C:\\Users\\Camras\\Documents\\VS code\\HP_EKF\\data\\data.csv", gyrzValues, 5);

    for (int i = 0; i < size; ++i) {
        data[i].accx = accxValues[i];
        data[i].accy = accyValues[i];
        data[i].accz = acczValues[i];
        data[i].gyrx = gyrxValues[i];
        data[i].gyry = gyryValues[i];
        data[i].gyrz = gyrzValues[i]; 
    }
}

static inline void writeCsvData(const char* filename, DATA data[], int size) {
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


static inline void writeQx1CsvData(const char* filename, EKF *Algo, int size) {
    FILE *file = fopen(filename, "w");
    if (!file) {
        printf("Could not open file %s for writing.\n", filename);
        exit(1);
    }

    // Header for q_x1 matrix values
    fprintf(file, "q_x1_0,q_x1_1,q_x1_2,q_x1_3\n");


    for (int i = 0; i < size; ++i) {
        fprintf(file, "%.8f,%.8f,%.8f,%.8f\n",
                Algo->q_x1_history[i][0][0], Algo->q_x1_history[i][1][0],
                Algo->q_x1_history[i][2][0], Algo->q_x1_history[i][3][0]);
    }

    fclose(file);
    printf("data written to %s successfully.\n", filename);
}
/***************************************************MATRIX OPERATION FUNCTIONS***************************************************/

    static inline void InitIdentityMatrix(int r, float result[r][r]){
        for(int i = 0; i < r; i++){
            result[i][i] = 1.0;
        }
    }

    static inline void TransposeMatrix(int r, int c, float inputMatrix[r][c], float outputMatrix[c][r]) {
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
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
    static inline float determinant(float matrix[MAX][MAX], int size) {
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
    static inline void cofactor(float matrix[MAX][MAX], float temp0[MAX][MAX], int p, int q, int size) {
        int i = 0, j = 0;

        // Looping for each element of the matrix
        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                // Copying into temporary matrix only those elements
                // which are not in given row and column
                if (row != p && col != q) {
                    temp0[i][j++] = matrix[row][col];
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
    
    static inline void MatrixDataTransfer(int r, int c, float inputMatrix[r][c], float outputMatrix[r][c]){
	for(int i=0;i<r;i++){
		for(int j=0;j<c;j++){
			outputMatrix[i][j] = inputMatrix[i][j];
		}
	}
    }

    // Function to compute the mean and variance of a matrix
    static inline void matrix_mean_and_variance(int r, int c, float matrix[r][c], float *variance) {
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

/***************************************************MATRIX OPERATION FUNCTIONS***************************************************/


/***************************************************STEP 1 KALMAN FILTER PREDICTION***************************************************/
 
/*****************EQUATION 1 ; STATE PREDICTION ; (x_pred = A * x_est;) *****************/

static inline void QuaternionPred(DATA *data, EKF *Algo){

    //calculate roll and pitch
    data->roll = (atan((data->accy)/(sqrt((data->accx * data->accx) + (data->accz * data->accz))))) * R_TO_D;
	data->pitch = (atan((-data->accx)/(sqrt((data->accy * data->accy) + (data->accz * data->accz))))) * R_TO_D;
            
    qw = (cos(rollavg/2) * cos(pitchavg/2) * cos(yaw/2)) + (sin(rollavg/2) * sin(pitchavg/2) * sin(yaw/2));
    qx = (sin(rollavg/2) * cos(pitchavg/2) * cos(yaw/2)) - (cos(rollavg/2) * sin(pitchavg/2) * sin(yaw/2));
    qy = (cos(rollavg/2) * sin(pitchavg/2) * cos(yaw/2)) + (sin(rollavg/2) * cos(pitchavg/2) * sin(yaw/2));
    qz = (cos(rollavg/2) * cos(pitchavg/2) * sin(yaw/2)) - (sin(rollavg/2) * sin(pitchavg/2) * cos(yaw/2));

    Algo->q[0][0] = qw;
    Algo->q[1][0]  = qx;
    Algo->q[2][0]  = qy;
    Algo->q[3][0]  = qz;
    
    // printf("q:\n");
    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 1; j++) {
    //         printf("%f ", Algo->q[i][j]);
    //     }
    //     printf("\n");
    // }

	//getting A matrix
	Algo->A[0][0] = 1;
	Algo->A[0][1] = (-data->gyrx * deltaT)/2;
	Algo->A[0][2] = (-data->gyry * deltaT)/2;
	Algo->A[0][3] = (-data->gyrz * deltaT)/2;
	
   	Algo->A[1][0] = (data->gyrx * deltaT)/2;
    Algo->A[1][1] = 1;
	Algo->A[1][2] = (data->gyrz * deltaT)/2;
	Algo->A[1][3] = (-data->gyry * deltaT)/2;

	Algo->A[2][0] = (data->gyry * deltaT)/2;
	Algo->A[2][1] = (-data->gyrz * deltaT)/2;
    Algo->A[2][2] = 1;
	Algo->A[2][3] = (data->gyrx * deltaT)/2;

   	Algo->A[3][0] = (data->gyrz * deltaT)/2;
	Algo->A[3][1] = (data->gyry * deltaT)/2;
	Algo->A[3][2] = (-data->gyrx * deltaT)/2;
    Algo->A[3][3] = 1;
 
    // printf("A:\n");
    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 4; j++) {
    //         printf("%f ", Algo->A[i][j]);
    //     }
    //     printf("\n");
    // }
	TransposeMatrix(4, 4, Algo->A, Algo->AT);

	// q_x1 = A * q;       
	MultiplyMatrices(4, 4, 4, 1, Algo->A, Algo->q, Algo->q_x1);

    MatrixDataTransfer(4, 1, Algo->q, Algo->qwxyz);

    MatrixDataTransfer(4, 1, Algo->q_x1, Algo->q_x1wxyz);
    // printf("q_x1:\n");
    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 1; j++) {
    //         printf("%f ", Algo->q_x1[i][j]);
    //     }
    //     printf("\n");
    // }
    
/*****************EQUATION 2 ; COVARIANCE PREDICTION ; (P_pred = A * P_est * A' + Q;) *****************/
    
    // Initialize P_i matrix
    Algo->P_i[0][0] = 1;
	Algo->P_i[0][1] = 0;
	Algo->P_i[0][2] = 0;
	Algo->P_i[0][3] = 0;

	Algo->P_i[1][0] = 0;
	Algo->P_i[1][1] = 1;
	Algo->P_i[1][2] = 0;
	Algo->P_i[1][3] = 0;

    Algo->P_i[2][0] = 0;
	Algo->P_i[2][1] = 0;
	Algo->P_i[2][2] = 1;
	Algo->P_i[2][3] = 0;

   	Algo->P_i[3][0] = 0;
	Algo->P_i[3][1] = 0;
	Algo->P_i[3][2] = 0;
	Algo->P_i[3][3] = 1;

    Algo->W[0][0] = (-Algo->qwxyz[1][0] * deltaT)/2;
	Algo->W[0][1] = (-Algo->qwxyz[2][0] * deltaT)/2;
	Algo->W[0][2] = (-Algo->qwxyz[3][0] * deltaT)/2;

	Algo->W[1][0] = (Algo->qwxyz[0][0] * deltaT)/2;
	Algo->W[1][1] = (-Algo->qwxyz[3][0] * deltaT)/2;
	Algo->W[1][2] = (Algo->qwxyz[1][0] * deltaT)/2;

    Algo->W[2][0] = (Algo->qwxyz[3][0] * deltaT)/2;
	Algo->W[2][1] = (Algo->qwxyz[0][0] * deltaT)/2;
	Algo->W[2][2] = (-Algo->qwxyz[1][0] * deltaT)/2;

    Algo->W[3][0] = (-Algo->qwxyz[2][0] * deltaT)/2;
	Algo->W[3][1] = (Algo->qwxyz[1][0] * deltaT)/2;
	Algo->W[3][2] = (Algo->qwxyz[0][0] * deltaT)/2;

    // printf("W:\n");
    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 3; j++) {
    //         printf("%f ", Algo->W[i][j]);
    //     }
    //     printf("\n");
    // }

    TransposeMatrix(4, 3, Algo->W, Algo->WT);

	//temp = A * P_i
	MultiplyMatrices(4, 4, 4, 4, Algo->A, Algo->P_i, Algo->temp);
    
	//temp1 = temp * AT
	MultiplyMatrices(4, 4, 4, 4, Algo->temp, Algo->AT, Algo->temp1);
    
	Algo->var[0][0] = (gsigmax * gsigmax);
	Algo->var[0][1] = 0;
	Algo->var[0][2] = 0;

    Algo->var[1][0] = 0;
	Algo->var[1][1] = (gsigmay * gsigmay);
	Algo->var[1][2] = 0;

    Algo->var[2][0] = 0;
	Algo->var[2][1] = 0;
	Algo->var[2][2] = (gsigmaz * gsigmaz);

    // printf("var:\n");
    // for (int i = 0; i < 3; i++) {
    //     for (int j = 0; j < 3; j++) {
    //         printf("%f ", Algo->var[i][j]);
    //     }
    //     printf("\n");
    // }
 
    //temp2 = W * var
    MultiplyMatrices(4, 3, 3, 3, Algo->W, Algo->var, Algo->temp2);

    //Q = temp2 * WT
    MultiplyMatrices(4, 3, 3, 4, Algo->temp2, Algo->WT, Algo->Q);

	//P_x = temp1 + Q
	AddMatrices(4, 4, Algo->temp1, Algo->Q, Algo->P_x);
    // printf("P_x:\n");
    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 4; j++) {
    //         printf("%f ", Algo->P_x[i][j]);
    //     }
    //     printf("\n");
    // }
/***************************************************STEP 2 KALMAN FILTER CORRECTION***************************************************/

/*****************EQUATION 3 ; INNOVATION COVARIANCE (S = H * P_pred * H' + R;*****************/
   
    //Intialize the R matrix 3x3
    Algo->R[0][0] = 0.0000000001;
	Algo->R[0][1] = 0;
    Algo->R[0][2] = 0;

    Algo->R[1][0] = 0;
    Algo->R[1][1] = 0.0000000001;
    Algo->R[1][2] = 0;
    
    Algo->R[2][0] = 0;
    Algo->R[2][1] = 0;
    Algo->R[2][2] = 0.0000000001;


    //Intitalizing g matrix 3x1
    Algo->g[0][0] = 0;
	Algo->g[1][0] = 0;
	Algo->g[2][0] = 1;

    //Intitalizing H matrix 3x4

    Algo->H[0][0] = ((2 * Algo->g[0][0] * Algo->q_x1wxyz[0][0]) + (2 * Algo->g[1][0] * Algo->q_x1wxyz[3][0]) - (2 * Algo->g[2][0] * Algo->q_x1wxyz[2][0]));
    Algo->H[0][1] = ((2 * Algo->g[0][0] * Algo->q_x1wxyz[1][0]) + (2 * Algo->g[1][0]* Algo->q_x1wxyz[2][0]) + (2 * Algo->g[2][0] * Algo->q_x1wxyz[3][0]));
    Algo->H[0][2] = ((-2 * Algo->g[0][0] * Algo->q_x1wxyz[2][0]) + (2 * Algo->g[1][0] * Algo->q_x1wxyz[1][0]) - (2 * Algo->g[2][0] * Algo->q_x1wxyz[0][0]));
    Algo->H[0][3] = ((-2 * Algo->g[0][0] * Algo->q_x1wxyz[3][0]) + (2 * Algo->g[1][0] * Algo->q_x1wxyz[0][0]) + (2 * Algo->g[2][0] * Algo->q_x1wxyz[1][0]));

    Algo->H[1][0] = ((-2 * Algo->g[0][0] * Algo->q_x1wxyz[3][0]) + (2 * Algo->g[1][0] * Algo->q_x1wxyz[0][0]) + (2 * Algo->g[2][0] * Algo->q_x1wxyz[1][0]));
    Algo->H[1][1] = ((2 * Algo->g[0][0] * Algo->q_x1wxyz[2][0]) - (2 * Algo->g[1][0] * Algo->q_x1wxyz[1][0]) + (2 * Algo->g[2][0] * Algo->q_x1wxyz[0][0]));
    Algo->H[1][2] = ((2 * Algo->g[0][0] * Algo->q_x1wxyz[1][0]) + (2 * Algo->g[1][0] * Algo->q_x1wxyz[2][0]) + (2 * Algo->g[2][0] * Algo->q_x1wxyz[3][0]));
    Algo->H[1][3] = ((-2 * Algo->g[0][0] * Algo->q_x1wxyz[0][0]) - (2 * Algo->g[1][0] * Algo->q_x1wxyz[3][0]) + (2 * Algo->g[2][0] * Algo->q_x1wxyz[2][0]));

    Algo->H[2][0] = ((2 * Algo->g[0][0] * Algo->q_x1wxyz[2][0]) - (2 * Algo->g[1][0] * Algo->q_x1wxyz[1][0]) + (2 * Algo->g[2][0] * Algo->q_x1wxyz[0][0]));
    Algo->H[2][1] = ((2 * Algo->g[0][0] * Algo->q_x1wxyz[3][0]) - (2 * Algo->g[1][0] * Algo->q_x1wxyz[0][0]) - (2 * Algo->g[2][0] * Algo->q_x1wxyz[1][0]));
    Algo->H[2][2] = ((2 * Algo->g[0][0] * Algo->q_x1wxyz[0][0]) + (2 * Algo->g[1][0] * Algo->q_x1wxyz[3][0]) - (2 * Algo->g[2][0] * Algo->q_x1wxyz[2][0]));
    Algo->H[2][3] = ((2 * Algo->g[0][0] * Algo->q_x1wxyz[1][0]) + (2 * Algo->g[1][0] * Algo->q_x1wxyz[2][0]) + (2 * Algo->g[2][0] * Algo->q_x1wxyz[3][0]));

    //HT
    TransposeMatrix(3, 4, Algo->H, Algo->HT);

    // temp3 = H * P_x;     
    MultiplyMatrices(3, 4, 4, 4, Algo->H, Algo->P_x, Algo->temp3);

    // temp4 = temp3 * HT
    MultiplyMatrices(3, 4, 4, 3, Algo->temp3, Algo->HT, Algo->temp4);

    // S_t = temp4 + R
    AddMatrices(3, 3, Algo->temp4, Algo->R, Algo->S_t);
    // printf("S_t:\n");
    // for (int i = 0; i < 3; i++) {
    //     for (int j = 0; j < 3; j++) {
    //         printf("%f ", Algo->S_t[i][j]);
    //     }
    //     printf("\n");
    // }
/*****************EQUATION 4 ; KALMAN GAIN CALCULATION  ) *****************/

    //temp_5 = P_x * HT
   	MultiplyMatrices(4, 4, 4, 3, Algo->P_x, Algo->HT, Algo->temp5);

    //inverse the S matrix
    InvertMatrix(3, Algo->S_t, Algo->S_t_inv);

    //K = temp5 * S_t_inv 
   	MultiplyMatrices(4, 3, 3, 3, Algo->temp5, Algo->S_t_inv, Algo->K);

/*****************EQUATION 5  *****************/

    Algo->C[0][0] = (pow(Algo->q_x1wxyz[0][0], 2) + pow(Algo->q_x1wxyz[1][0], 2) - pow(Algo->q_x1wxyz[2][0], 2) - pow(Algo->q_x1wxyz[3][0], 2));
    Algo->C[0][1] = 2 * ((Algo->q_x1wxyz[1][0] * Algo->q_x1wxyz[2][0]) + (Algo->q_x1wxyz[0][0] * Algo->q_x1wxyz[3][0]));
    Algo->C[0][2] = 2 * ((Algo->q_x1wxyz[1][0] * Algo->q_x1wxyz[3][0]) - (Algo->q_x1wxyz[0][0] * Algo->q_x1wxyz[2][0]));

    Algo->C[1][0] = 2 * ((Algo->q_x1wxyz[1][0] * Algo->q_x1wxyz[2][0]) - (Algo->q_x1wxyz[0][0] * Algo->q_x1wxyz[3][0]));
    Algo->C[1][1] = (pow(Algo->q_x1wxyz[0][0], 2) - pow(Algo->q_x1wxyz[1][0], 2) + pow(Algo->q_x1wxyz[2][0], 2) - pow(Algo->q_x1wxyz[3][0], 2));
    Algo->C[1][2] = 2 * ((Algo->q_x1wxyz[0][0] * Algo->q_x1wxyz[1][0]) + (Algo->q_x1wxyz[2][0] * Algo->q_x1wxyz[3][0]));

    Algo->C[2][0] = 2 * ((Algo->q_x1wxyz[1][0] * Algo->q_x1wxyz[3][0]) + (Algo->q_x1wxyz[0][0] * Algo->q_x1wxyz[2][0]));
    Algo->C[2][1] = 2 * ((Algo->q_x1wxyz[2][0] * Algo->q_x1wxyz[3][0]) - (Algo->q_x1wxyz[0][0] * Algo->q_x1wxyz[1][0]));
    Algo->C[2][2] = (pow(Algo->q_x1wxyz[0][0], 2) - pow(Algo->q_x1wxyz[1][0], 2) - pow(Algo->q_x1wxyz[2][0], 2) + pow(Algo->q_x1wxyz[3][0], 2));
    
    //Intialize x matrix
    Algo->x[0][0] = 0;
	Algo->x[1][0] = 0;
    Algo->x[2][0] = 1;

    // h = C * x; 
    MultiplyMatrices(3, 3, 3, 1, Algo->C, Algo->x, Algo->h);

    //getting accelerometer sensor data z_t
    Algo->z_t[0][0] = data->accx;
	Algo->z_t[1][0] = data->accy;
    Algo->z_t[2][0] = data->accz;

    // y_t = z_t - h;
    SubtractMatrices(3, 1, Algo->z_t, Algo->h, Algo->y_t);
       printf("y_t:\n");
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 1; j++) {
            printf("%f ", Algo->y_t[i][j]);
        }
        printf("\n");
    }

/*****************EQUATION 6 ; STATE UPDATION (x_est = x_pred + K * y;) *****************/

    //temp6 = K * y_t
 	MultiplyMatrices(4, 3, 3, 1, Algo->K, Algo->y_t, Algo->temp6);

    //temp7 = q_x1 + temp6
    AddMatrices(4, 1, Algo->q_x1, Algo->temp6, Algo->temp7);

    MatrixDataTransfer(4, 1, Algo->temp7, Algo->q);

/*****************EQUATION 7 ; COVARIANCE UPDATION (P_est = (eye(size(K,1)) - K * H) * P_pred;) *****************/

    //temp8 = K * H
 	MultiplyMatrices(4, 3, 3, 4, Algo->K, Algo->H, Algo->temp8);
    
    InitIdentityMatrix(4, Algo->I);

    //temp9 = I - temp8
    SubtractMatrices(4, 4, Algo->I, Algo->temp8, Algo->temp9);

    //P_i = temp9 * P_x 
    MultiplyMatrices(4, 4, 4, 4, Algo->temp9, Algo->P_x, Algo->P_i);

    // data->estimatedroll = (atan2((2.0 * (Algo->q_x1[1][0] * Algo->q_x1[2][0] + Algo->q_x1[3][0] * Algo->q_x1[4][0])), (1.0 - 2.0 * (Algo->q_x1[2][0] * Algo->q_x1[2][0] + Algo->q_x1[3][0] * Algo->q_x1[3][0])))) * R_TO_D;
   
    /*********************************************************************************************/

    // Calculation of Theetha
    data->estimatedroll = (atan2((2.0 * (Algo->q[1][0] * Algo->q[2][0] + Algo->q[3][0] * Algo->q[4][0])), (1.0 - 2.0 * (Algo->q[2][0] * Algo->q[2][0] + Algo->q[3][0]  *  Algo->q[3][0])))) * R_TO_D;

    // Calculation of Pitch
    data->estimatedpitch = asin(2.0 * (Algo->q[1][0] * Algo->q[3][0] - Algo->q[2][0] * Algo->q[4][0]));

    // Calculation of Yaw
    data->estimatedyaw = atan2((2.0 * (Algo->q[1][0] * Algo->q[4][0] + Algo->q[2][0] * Algo->q[3][0])), (1.0 - 2.0 * (Algo->q[3][0] * Algo->q[3][0] + Algo->q[4][0] * Algo->q[4][0])));
   
}

int main() {
    DATA data[NUM_SAMPLES];
    EKF Algo;

    // Initialize and process data as before
    const char* filePath = "C:\\Users\\Camras\\Documents\\VS code\\HP_EKF\\data\\data.csv";
    initializeSensorData(data, NUM_SAMPLES, filePath);

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        QuaternionPred(&data[i], &Algo);
        // Angleread(&data[i], &Algo);

        // Store the current q_x1 values in the history
        memcpy(Algo.q_x1_history[i], Algo.q_x1, sizeof(Algo.q_x1));
    }

    // Path for the q_x1 values CSV file
    const char* q_x1OutputFilePath = "C:\\Users\\Camras\\Documents\\VS code\\HP_EKF\\data\\filter_data.csv";
    // Write q_x1 data to CSV file after processing all data points
    writeQx1CsvData(q_x1OutputFilePath, &Algo, NUM_SAMPLES);

    // // Path for the S_t values CSV file
    // const char* q_x1OutputFilePath = "C:\\Users\\Camras\\Documents\\VS code\\AKF\\data\\K_data.csv";
    // // Write q_x1 data to CSV file after processing all data points
    // writeQx1CsvData(q_x1OutputFilePath, &Algo, NUM_SAMPLES);

    // Path for the output CSV file for roll and pitch values
    const char* outputFilePath = "C:\\Users\\Camras\\Documents\\VS code\\HP_EKF\\data\\Acc_Roll_and_pitch_data.csv";
    // Write roll and pitch data to CSV file after processing all data points
    writeCsvData(outputFilePath, data, NUM_SAMPLES);

    return 0;
}

