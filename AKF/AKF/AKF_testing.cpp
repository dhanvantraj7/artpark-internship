#include <stdio.h>
#include <stdlib.h>
#include <string.h> 

#define NUM_SAMPLES 10

typedef struct {
    float accx;
    float accy;
    float accz;
    float gyrx;
    float gyry;
    float gyrz;
} DATA;

DATA data[NUM_SAMPLES];

void readCsvData(const char* filename, float data[NUM_SAMPLES], int column) {
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

void initializeSensorData(DATA data[], int size, const char* filePath) {
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

/////////////////////////////////////////////enter the algo//////////////////////////////////////////////



/////////////////////////////////////////////end the algo////////////////////////////////////////////////


int main() {

    // Replace the path below with the absolute path to your Roll_data file
    const char* filePath = "C:\\Users\\Camras\\Documents\\VS code\\AKF\\Roll_data.csv";
    initializeSensorData(data, NUM_SAMPLES, filePath);

    // Print the loaded sensor data
    printf("Loaded Sensor Data:\n");
    for (int i = 0; i < NUM_SAMPLES; i++) {
        printf("Sample %d - AccX: %.8f, AccY: %.8f, AccZ: %.8f, GyroX: %.8f, GyroY: %.8f, GyroZ: %.8f\n", 
               i+1, 
               data[i].accx, data[i].accy, data[i].accz, 
               data[i].gyrx, data[i].gyry, data[i].gyrz);
    }

    // Proceed with using the sensor data
    return 0;
}