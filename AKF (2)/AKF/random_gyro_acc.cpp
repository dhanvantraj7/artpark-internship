#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Define structure for accelerometer and gyroscope data
typedef struct {
    double accx, accy, accz;
    double gyrox, gyroy, gyroz;
} SensorData;

// Function to generate random double between min and max
double randomDouble(double min, double max) {
    return min + (rand() / (RAND_MAX / (max - min)));
}

// Function to generate random accelerometer and gyroscope data
void generateRandomSensorData(SensorData *data, int size) {
    for (int i = 0; i < size; i++) {
        // Generate random accelerometer values (-10 to 10 m/s^2)
        data[i].accx = randomDouble(-10.0, 10.0);
        data[i].accy = randomDouble(-10.0, 10.0);
        data[i].accz = randomDouble(-10.0, 10.0);

        // Generate random gyroscope values (-5 to 5 rad/s)
        data[i].gyrox = randomDouble(-5.0, 5.0);
        data[i].gyroy = randomDouble(-5.0, 5.0);
        data[i].gyroz = randomDouble(-5.0, 5.0);
    }
}

int main() {
    srand(time(NULL)); // Seed the random number generator with current time

    int size = 100; // Number of data points
    SensorData *data = (SensorData *)malloc(size * sizeof(SensorData));

    // Generate random accelerometer and gyroscope data
    generateRandomSensorData(data, size);

    // Print the generated data
    printf("Generated Sensor Data:\n");
    for (int i = 0; i < size; i++) {
        printf("Data Point %d:\n", i + 1);
        printf("Accelerometer: (%.2f, %.2f, %.2f)\n", data[i].accx, data[i].accy, data[i].accz);
        printf("Gyroscope: (%.2f, %.2f, %.2f)\n", data[i].gyrox, data[i].gyroy, data[i].gyroz);
        printf("\n");
    }

    // Free dynamically allocated memory
    free(data);

    return 0;
}