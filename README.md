# **Wearable Parkinsonian Tremor Detector**

## **Objective**
The goal of this project is to build a wearable Parkinsonian tremor detector using an Adafruit Playground Classic board with its embedded accelerometer. The detector aims to capture time segments from the accelerometer, analyze the data, and provide a visual indication of the presence and intensity of resting tremor using board resources such as LEDs, speaker, and neopixels. No additional hardware may be used.

## **Background**
Over a million people in the USA and more than 10 million people worldwide suffer from Parkinson’s disease. More than 70% of patients experience a symptom called resting tremor, which occurs when a body part (usually the hand or wrist) is completely supported and minimal or absent during activity. The most common resting tremor, classical parkinsonian tremor, is 3 to 6 cycles/second (Hz).

## **Required Parts**
1. Adafruit Playground Classic with IMU
2. Power supply/USB power bank (or LIPO battery)

## **Restrictions**
As long as the device is connected to a power source, it will continuously measure the intensity of the tremors. Due to this, we recommend only using the device while at rest. Avoid using the device in situations that may cause vibrations, such as in a vehicle.

## **How It Works**
The device is used to detect the intensity of tremors for a patient with Parkinson’s disease. Every two seconds, the LEDs on the device will show the intensity of that tremor through 9 LED lights. Every 3 minutes, another LED will show the average intensity of the Parkinson’s tremor with the following color indicators:
- **Blue**: Intensity 1 - 3
- **Purple**: Intensity 3.1 - 14.9
- **Red**: Intensity 15 - 20

Average intensities below 1 are not considered Parkinson’s tremors. These values were determined by a range of intensities found in [this research article](https://jneuroengrehab.biomedcentral.com/articles/10.1186/s12984-019-0534-8) about Parkinson’s tremors.

## **Implementation**
The detector measures real-time acceleration from the built-in IMU, performs FFT analysis to determine tremor frequency and intensity, and provides visual feedback using the onboard LEDs.

### **Code Explanation**
1. **Libraries and Definitions**: The code uses the `Arduino.h`, `Adafruit_CircuitPlayground.h`, and `arduinoFFT.h` libraries. Various constants are defined, such as the number of samples, sampling frequency, and LED parameters.

2. **Global Variables**: Arrays for real and imaginary parts of the FFT, intensity calculation variables, flags for FFT and timer completion, and time tracking variables are declared.

3. **ISR Functions**: Two ISR functions handle the data collection from the accelerometer and a timer for periodic tasks.
   - `ISR(TIMER0_COMPA_vect)`: Collects accelerometer data and sets the FFT completion flag.
   - `ISR(TIMER1_COMPA_vect)`: Tracks the elapsed time for 3-minute intervals and sets a flag when the interval is complete.

4. **Intensity Calculation**: The `calculateIntensity` function computes the intensity of the tremor based on the FFT results.

5. **LED Display**: The `displayIntensityOnLEDs` function lights up the onboard LEDs based on the calculated intensity. The `determineTremorType` function uses a specific LED to indicate the type of tremor.

6. **Timer Setup**: The `timerSetUp` function configures the timers for data collection and periodic tasks.

7. **Main Loop**: The `setup` function initializes the serial communication, Circuit Playground, and timers. The `loop` function continuously checks for FFT completion, processes the data, calculates the intensity, and updates the LED display.

### **Testing the Device**
1. **Setup**:
   - Connect the Adafruit Playground Classic board to a power supply or USB power bank.
   - Ensure the board is running the provided code.

2. **Operation**:
   - The device will start collecting accelerometer data and performing FFT analysis.
   - Observe the onboard LEDs for visual feedback on tremor intensity.
   - Different LED colors indicate different types of tremors: green for visible intensity, red for big tremor, purple for medium tremor, and blue for small tremor.

3. **Analysis**:
   - The average tremor intensity is calculated every 3 minutes.
   - The results are printed to the serial monitor for further analysis and verification.

## **YouTube Link**
For a demonstration of the project, visit the [YouTube link](https://youtu.be/0vCS7bw00CE).
