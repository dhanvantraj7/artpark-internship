#include <stdio.h>

#define MAX 6 // Adjusted for a 6x6 matrix

// Assume the determinant function is defined here
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
// Placeholder for the cofactor function - you need to implement this
void cofactor(float matrix[MAX][MAX], float temp[MAX][MAX], int p, int q, int n) {
    int i = 0, j = 0;

    // Looping for each element of the matrix
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            // Copying into temporary matrix only those elements
            // which are not in given row and column
            if (row != p && col != q) {
                temp[i][j++] = matrix[row][col];
                // Row is filled, so increase row index and reset col index
                if (j == n - 1) {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

// Function to invert a matrix
static inline void InvertMatrix(int r, float inputMatrix[r][r], float outputMatrix[r][r]){
    // Implementation as you provided
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

// Function to print matrices
void printMatrix(float matrix[MAX][MAX], int size) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            printf("%8.3f", matrix[i][j]);
        }
        printf("\n");
    }
}

int main() {
    float matrix[MAX][MAX] = {
        {1, 5, 6, 15, 5, 2},
        {4, 3, 2, 1, 5, 4},
        {8, 7, 6, 5, 4, 3},
        {2, 3, 4, 5, 6, 7},
        {9, 45, 7, 6, 5, 4},
        {1, 2, 3, 4, 6, 6}
    };
    float invertedMatrix[MAX][MAX];

    printf("Original Matrix:\n");
    printMatrix(matrix, MAX);

    InvertMatrix(MAX, matrix, invertedMatrix);

    printf("\nInverted Matrix:\n");
    printMatrix(invertedMatrix, MAX);

    return 0;
}
