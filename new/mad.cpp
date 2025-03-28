// Timing parameters (in microseconds)
const int Tb = 200;                   // Base time unit: 200 µs (for 1 kHz clock, period = 1000 µs)
const int resetPulseDuration = 3 * Tb;  // 600 µs reset pulse
const int clockHighDuration = 500;      // 500 µs high (50% duty cycle)
const int clockLowDuration  = 500;      // 500 µs low
const int readDelay = Tb;               // Wait 200 µs after clock rising edge before reading
const int remainingHighDelay = clockHighDuration - readDelay; // 300 µs

// Create digital output task for P1.0 and P1.1 (reset and clock respectively)
DAQmxErrChk(DAQmxCreateTask("", &doTask));
// P1.0 and P1.1 are on port1; channel string "Dev1/port1/line0:1"
DAQmxErrChk(DAQmxCreateDOChan(doTask, "Dev1/port1/line0:1", "", DAQmx_Val_ChanForAllLines));
DAQmxErrChk(DAQmxStartTask(doTask));

// Create digital input task for P0.0 to P0.4 (data bits and DV)
DAQmxErrChk(DAQmxCreateTask("", &diTask));
// P0.0 to P0.4 are on port0; channel string "Dev1/port0/line0:4"
DAQmxErrChk(DAQmxCreateDIChan(diTask, "Dev1/port0/line0:4", "", DAQmx_Val_ChanForAllLines));
DAQmxErrChk(DAQmxStartTask(diTask));

std::cout << "Starting activity detector routine..." << std::endl;

while (true)
{
    // Step 1: Send reset pulse on P1.0 (reset high, clock low)
    // DO bit mapping (for port1): bit0 = reset, bit1 = clock.
    // Set reset high (bit0=1) and clock low (bit1=0) → value = 0x01.
    uInt8 doData = 0x01;
    DAQmxErrChk(DAQmxWriteDigitalLines(doTask, 1, 1, 10.0, 
        DAQmx_Val_GroupByChannel, &doData, NULL, NULL));
    std::this_thread::sleep_for(std::chrono::microseconds(resetPulseDuration));
    // Set reset low (value = 0x00)
    doData = 0x00;
    DAQmxErrChk(DAQmxWriteDigitalLines(doTask, 1, 1, 10.0, 
        DAQmx_Val_GroupByChannel, &doData, NULL, NULL));

    // For each tube (16 tubes)
    for (int tube = 0; tube < numTubes; tube++)
    {
        // Step 2: Generate clock pulse by setting clock high (P1.1 high, reset remains low)
        // Value for clock high is 0x02 (bit1=1)
        doData = 0x02;
        DAQmxErrChk(DAQmxWriteDigitalLines(doTask, 1, 1, 10.0, 
            DAQmx_Val_GroupByChannel, &doData, NULL, NULL));

        // Virtual counter increment (tube index corresponds to counter c)
        // Step 4: Wait for 1 Tb (200 µs) after clock pulse begins
        std::this_thread::sleep_for(std::chrono::microseconds(readDelay));

        // Step 5: Read DI channels (P0.0-P0.4)
        uInt8 diData = 0;
        int32 samplesRead = 0;
        DAQmxErrChk(DAQmxReadDigitalLines(diTask, 1, 10.0, 
            DAQmx_Val_GroupByChannel, &diData, 1, &samplesRead, NULL));

        // Extract the 4-bit data (P0.0–P0.3) and the DV bit (P0.4)
        int dataValue = diData & 0x0F;          // Lower 4 bits: binary data (LSB is P0.0)
        bool DV = (diData & 0x10) != 0;           // Bit 4: DV bit

        // Step 6: Process the reading
        if (!DV)
        {
            // If DV is low, convert the 4-bit data to a tube number (1 to 16)
            int tubeNumber = dataValue + 1; // Table mapping: 0→1, 15→16
            std::cout << "Tube " << (tube + 1) << " reading: " << tubeNumber << std::endl;
            lastValue[tube] = tubeNumber;
        }
        else
        {
            // DV is high: if all data bits are low and the last recorded value was 1, record "EATING"
            if (dataValue == 0 && lastValue[tube] == 1)
            {
                std::cout << "Tube " << (tube + 1) << " reading: EATING" << std::endl;
                lastValue[tube] = -1;  // Mark as "EATING" (or use another flag)
            }
            else
            {
                std::cout << "Tube " << (tube + 1) << " reading: (no change)" << std::endl;
            }
        }

        // Step 7: Wait the remaining high period (total high = 500 µs; we already waited 200 µs)
        std::this_thread::sleep_for(std::chrono::microseconds(remainingHighDelay));

        // End of high period: Set clock low (both reset and clock low → value = 0x00)
        doData = 0x00;
        DAQmxErrChk(DAQmxWriteDigitalLines(doTask, 1, 1, 10.0, 
            DAQmx_Val_GroupByChannel, &doData, NULL, NULL));

        // Step 8: Wait for the low period of the clock (500 µs) before next pulse
        std::this_thread::sleep_for(std::chrono::microseconds(clockLowDuration));
    }
    // The routine then repeats from Step 1 for the next cycle.
}

// Clean up tasks (this code is not reached in this infinite loop, but is provided for completeness)
DAQmxStopTask(doTask);
DAQmxClearTask(doTask);
DAQmxStopTask(diTask);
DAQmxClearTask(diTask);

return 0;
