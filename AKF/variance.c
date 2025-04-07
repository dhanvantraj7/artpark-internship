#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Function to compute the variance of a 2x1 matrix (column vector)
float variance(const float matrix[2][1]) {
    // Compute the mean of the values
    float mean = (matrix[0][0] + matrix[1][0]) / 2.0f;

    // Compute the sum of squared differences from the mean
    float sum_squared_diff = 0.0f;
    for (int i = 0; i < 2; i++) {
        float diff = matrix[i][0] - mean;
        sum_squared_diff += diff * diff;
    }

    // Compute the variance
    float variance = sum_squared_diff / 1; // Variance of a single value is itself

    return variance;
}

int main() {
    // Example 2x1 matrix (column vector)
    float y[2][1] = {{1.2f}, {2.3f}};

    // Compute the variance
    float innovation_variance = variance(y);

    // Print the result
    printf("Innovation Variance: %f\n", innovation_variance);

    return 0;
}
