#include <NIDAQmx.h>
#include <stdio.h>
#include <stdbool.h>
#include <windows.h>
#include <process.h>
#include <stdint.h>
#include <string.h>

// Constants
#define NUM_TUBES 16
#define PORT0_LINE_COUNT 5
#define PORT1_LINE_COUNT 2

// Shared state structure
typedef struct {
    TaskHandle inputTask;
    TaskHandle outputTask;
    float timebase;
    volatile bool running;
    CRITICAL_SECTION mutex;
    CONDITION_VARIABLE resetCond;    // Signals when reset pulse occurs
    CONDITION_VARIABLE clockCond;    // Signals clock transitions
    bool resetActive;                // Indicates reset pulse is active
    bool clockHigh;                 // Indicates clock state
    int currentTube;               // Current tube being read
} SharedState;

// Tube reading structure
typedef struct {
    int value;
    bool isEating;
    CRITICAL_SECTION mutex;
} TubeReading;

// Global variables
TubeReading tubeReadings[NUM_TUBES];
SharedState state;

// Error checking macro
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

// Initialize the DAQ device
int initializeDevice(void) {
    int error = 0;
    
    // Configure digital input (P0.0-P0.4)
    DAQmxErrChk(DAQmxCreateTask("InputTask", &state.inputTask));
    DAQmxErrChk(DAQmxCreateDIChan(state.inputTask, "Dev2/port0/line0:4", "",
                                 DAQmx_Val_ChanForAllLines));
    
    // Configure digital output (P1.0-P1.1)
    DAQmxErrChk(DAQmxCreateTask("OutputTask", &state.outputTask));
    DAQmxErrChk(DAQmxCreateDOChan(state.outputTask, "Dev2/port1/line0:1", "",
                                 DAQmx_Val_ChanForAllLines));
    
    return 0;

Error:
    return error;
}

// Initialize shared state
void initializeState(void) {
    int i;
    state.timebase = 0.0002f;  // Default 0.2ms
    state.running = true;
    state.resetActive = false;
    state.clockHigh = false;
    state.currentTube = 0;

    InitializeCriticalSection(&state.mutex);
    InitializeConditionVariable(&state.resetCond);
    InitializeConditionVariable(&state.clockCond);
    
    for (i = 0; i < NUM_TUBES; i++) {
        tubeReadings[i].value = 0;
        tubeReadings[i].isEating = false;
        InitializeCriticalSection(&tubeReadings[i].mutex);
    }
}

// Process data read from a tube
void processData(uInt8 data[], int tubeNumber) {
    EnterCriticalSection(&tubeReadings[tubeNumber].mutex);
    //printf("%d  ", *data);
    printf("1. %d  ", data[0]);
    printf("2. %d  ", data[1]);
    printf("3. %d  ", data[2]);
    printf("4. %d  ", data[3]);
    printf("5. %d  ", data[4]);
    if (data[4] == 0) {  // DV is LOW - normal position reading
        tubeReadings[tubeNumber].value = 
            data[0] + (data[1] * 1) + (data[2] * 4) + (data[3] * 8);
        tubeReadings[tubeNumber].isEating = false;
    }
    else {  // DV is HIGH - check for eating condition
        if (data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 0 &&
            tubeReadings[tubeNumber].value == 1) {
            tubeReadings[tubeNumber].isEating = true;
        }
    }
    
    LeaveCriticalSection(&tubeReadings[tubeNumber].mutex);
}

// Output thread function - handles reset and clock signals
unsigned int __stdcall outputThread(void* arg) {
    uInt8 outputData[PORT1_LINE_COUNT];
    int i;
    
    while (state.running) {
        // Send reset pulse (P1.0 HIGH for 3Tb)
        EnterCriticalSection(&state.mutex);
        outputData[0] = 1;  // Reset high
        outputData[1] = 0;  // Clock low
        state.resetActive = true;
        state.currentTube = 0;
        WakeAllConditionVariable(&state.resetCond);
        LeaveCriticalSection(&state.mutex);
        
        DAQmxWriteDigitalLines(state.outputTask, 1, 1, state.timebase*3.0,
                              DAQmx_Val_GroupByChannel, outputData, NULL, NULL);
        
        // Reset low
        outputData[0] = 0;
        DAQmxWriteDigitalLines(state.outputTask, 1, 1, state.timebase,
                              DAQmx_Val_GroupByChannel, outputData, NULL, NULL);
        
        EnterCriticalSection(&state.mutex);
        state.resetActive = false;
        LeaveCriticalSection(&state.mutex);
        
        // Clock cycles for all tubes
        for (i = 0; i < NUM_TUBES; i++) {
            // Clock high
            EnterCriticalSection(&state.mutex);
            outputData[1] = 1;
            state.clockHigh = true;
            WakeAllConditionVariable(&state.clockCond);
            LeaveCriticalSection(&state.mutex);
            
            DAQmxWriteDigitalLines(state.outputTask, 1, 1, state.timebase*2.5,
                                  DAQmx_Val_GroupByChannel, outputData, NULL, NULL);
            
            // Clock low
            EnterCriticalSection(&state.mutex);
            outputData[1] = 0;
            state.clockHigh = false;
            WakeAllConditionVariable(&state.clockCond);
            LeaveCriticalSection(&state.mutex);
            
            DAQmxWriteDigitalLines(state.outputTask, 1, 1, state.timebase*2.5,
                                  DAQmx_Val_GroupByChannel, outputData, NULL, NULL);
        }
    }
    
    return 0;
}

// Input thread function - reads data from tubes
unsigned int __stdcall inputThread(void* arg) {
    uInt8 inputData[PORT0_LINE_COUNT];
    int i;
    
    while (state.running) {
        // Wait for reset pulse
        EnterCriticalSection(&state.mutex);
        while (!state.resetActive) {
            SleepConditionVariableCS(&state.resetCond, &state.mutex, INFINITE);
        }
        LeaveCriticalSection(&state.mutex);
        
        // Process all tubes
        for (i = 0; i < NUM_TUBES; i++) {
            // Wait for clock to go high
            EnterCriticalSection(&state.mutex);
            while (!state.clockHigh) {
                SleepConditionVariableCS(&state.clockCond, &state.mutex, INFINITE);
            }
            LeaveCriticalSection(&state.mutex);
            
            // Wait 1Tb then read data
            Sleep((DWORD)(state.timebase * 1000));
            DAQmxReadDigitalLines(state.inputTask, 1, state.timebase*2.0,
                                DAQmx_Val_GroupByChannel, inputData,
                                PORT0_LINE_COUNT, NULL, NULL, NULL);
            
            // Process the data
            processData(inputData, i);
            
            // Wait for clock to go low
            EnterCriticalSection(&state.mutex);
            while (state.clockHigh) {
                SleepConditionVariableCS(&state.clockCond, &state.mutex, INFINITE);
            }
            LeaveCriticalSection(&state.mutex);
        }
    }
    
    return 0;
}

// Display thread function - updates the console output
unsigned int __stdcall displayThread(void* arg) {
    int i;
    
    while (state.running) {
        printf("\033[2J\033[H");  // Clear screen and move cursor to top
        printf("Multibeam Activity Detector - Real-time Monitoring\n");
        printf("===============================================\n\n");
        printf("Tube | Position | Status | Activity\n");
        printf("-----|----------|---------|----------\n");
        
        for (i = 0; i < NUM_TUBES; i++) {
            EnterCriticalSection(&tubeReadings[i].mutex);
            
            printf("%4d | ", i + 1);
            
            if (tubeReadings[i].isEating) {
                printf("%8d | EATING  | Feeding at position 1\n", 1);
            } else if (tubeReadings[i].value > 0) {
                printf("%8d | ACTIVE  | Moving at position %d\n",
                       tubeReadings[i].value,
                       tubeReadings[i].value);
            } else {
                printf("%8s | IDLE    | No activity detected\n", "-");
            }
            
            LeaveCriticalSection(&tubeReadings[i].mutex);
        }
        
        Sleep(100);  // Update display every 100ms
    }
    
    return 0;
}

int main(void) {
    int error;
    HANDLE threads[3];
    char choice;
    
    error = initializeDevice();
    if (error) {
        printf("Failed to initialize device. Error: %d\n", error);
        return error;
    }
    
    initializeState();
    
    // Configure timebase
    printf("Select timebase (milliseconds):\n");
    printf("1. 0.01\n2. 0.1\n3. 1.0\n4. 10.0\n");
    printf("Choice: ");
    scanf_s(" %c", &choice, 1);
    
    switch(choice) {
        case '1': state.timebase = 0.00001f; break;
        case '2': state.timebase = 0.0001f; break;
        case '3': state.timebase = 0.001f; break;
        case '4': state.timebase = 0.01f; break;
        default:  printf("Using default timebase (0.2ms)\n");
    }
    
    // Create threads
    threads[0] = (HANDLE)_beginthreadex(NULL, 0, outputThread, NULL, 0, NULL);
    threads[1] = (HANDLE)_beginthreadex(NULL, 0, inputThread, NULL, 0, NULL);
    threads[2] = (HANDLE)_beginthreadex(NULL, 0, displayThread, NULL, 0, NULL);
    
    printf("\nPress Enter to stop acquisition...\n");
    getchar();
    getchar();
    
    // Stop acquisition and wait for threads
    state.running = false;
    WakeAllConditionVariable(&state.resetCond);
    WakeAllConditionVariable(&state.clockCond);
    WaitForMultipleObjects(3, threads, TRUE, INFINITE);
    
    // Close thread handles
    CloseHandle(threads[0]);
    CloseHandle(threads[1]);
    CloseHandle(threads[2]);
    
    // Delete critical sections
    DeleteCriticalSection(&state.mutex);

    for (int i = 0; i < NUM_TUBES; i++) {
        DeleteCriticalSection(&tubeReadings[i].mutex);
    }
    
    // Cleanup DAQ tasks
    if (state.inputTask != 0) {
        DAQmxStopTask(state.inputTask);
        DAQmxClearTask(state.inputTask);
    }
    if (state.outputTask != 0) {
        DAQmxStopTask(state.outputTask);
        DAQmxClearTask(state.outputTask);
    }
    
    return 0;
}