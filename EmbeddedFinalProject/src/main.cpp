#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "arduinoFFT.h"


#define SAMPLES 128
#define SAMPLING_FREQUENCY 200
float vReal[SAMPLES];
float vImag[SAMPLES];
float totalIntensity = 0.0; // Add totalIntensity declaration
int intensityCount = 0;     // Add intensityCount declaration


bool displayTremorTypeOnly = false;


// Flag for FFT completion
volatile bool isFFTComplete = false;
volatile bool is3MinElapsed = false;


// Timer interval for 3 minutes (in milliseconds)
const unsigned long TIMER_INTERVAL = 180000;
volatile int overflowCount = 0;
// Time tracking for LED off delay
unsigned long tremorTypeLEDStartTime = 0;
const unsigned long LED_ON_DELAY = 2000; // 2 seconds


// Create an FFT object with template float.
ArduinoFFT<float> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);


volatile int i = 0;


void calculateIntensity();
void displayIntensityOnLEDs(float intensity);
void determineTremorType(float intensity);
void timerSetUp();


ISR(TIMER0_COMPA_vect) {
  // Collect data
  vImag[i] = 0;
  if (i < 128) {
    vReal[i++] = sqrt(pow(CircuitPlayground.motionX(), 2) + pow(CircuitPlayground.motionY(), 2) + pow(CircuitPlayground.motionZ(), 2));
  } else {
    i = 0;
    // Set flag indicating FFT analysis is complete
    isFFTComplete = true;
  }
}


ISR(TIMER1_COMPA_vect) {
  overflowCount++;
  if (overflowCount >= 45) {
    // Set flag indicating 3 minutes elapsed
    is3MinElapsed = true;
    overflowCount = 0; // Reset overflow count
  }
}


void calculateIntensity() {
  float sumSquared = 0.0;
  for (int j = 3; j < 6; j++) {  
    sumSquared += pow(vReal[j], 2);
  }


  // Calculate intensity
  float N = 4; // Number of frequencies in range
  float intensity = (sqrt(sumSquared / (N * N)) - 14.23); // adjust


  // Accumulate total intensity and count
  totalIntensity += intensity;
  intensityCount++;


  Serial.print("Intensity: ");
  Serial.println(intensity);
  Serial.println("======");


  // Display current intensity on LEDs
  displayIntensityOnLEDs(intensity);
}


void displayIntensityOnLEDs(float intensity) {
  const uint8_t maxIntensityLED = 9; // Maximum number of LEDs to use for intensity
  if (!displayTremorTypeOnly) {
    uint8_t ledsToLight = (uint8_t)(maxIntensityLED * intensity / 20); // Assuming intensity scale max is 20
    for (uint8_t i = 0; i < maxIntensityLED; i++) {
      if (i < ledsToLight) {
        CircuitPlayground.setPixelColor(i, 0, 255, 0); // Green color for visible intensity
      } else {
        CircuitPlayground.setPixelColor(i, 0, 0, 0); // Turn off the LED
      }
    }
  } else {
    // When displaying tremor type, ensure all intensity LEDs are off
    for (uint8_t i = 0; i < maxIntensityLED; i++) {
      CircuitPlayground.setPixelColor(i, 0, 0, 0); // Turn off the LED
    }
  }
}


void determineTremorType(float intensity) {
  const uint8_t tremorTypePixel = 9; // The index of the last NeoPixel reserved for tremor type
  CircuitPlayground.setPixelColor(tremorTypePixel, 0, 0, 0); // Ensure LED is off before setting new color
  if (intensity >= 15 && intensity <= 20) {
    CircuitPlayground.setPixelColor(tremorTypePixel, 255, 0, 0); // Big tremor - red light
    tremorTypeLEDStartTime = millis(); // Record start time for LED
  } else if (intensity >= 3.1 && intensity < 14.9) {
    CircuitPlayground.setPixelColor(tremorTypePixel, 128, 0, 128); // Medium Tremor - purple light
    tremorTypeLEDStartTime = millis(); // Record start time for LED
  } else if (intensity >= 1 && intensity <= 3) {
    CircuitPlayground.setPixelColor(tremorTypePixel, 0, 0, 255); // Small tremor - blue light
    tremorTypeLEDStartTime = millis(); // Record start time for LED
  }
}


void timerSetUp() {
  // Set up Timer0 for data collection at 64Hz
  TCCR0A = 0b00000010; // CTC mode
  TCCR0B = 0b00000101; // Prescaler 1024
  OCR0A = 124; // 128us x 125 = 16ms => 64Hz


  TIMSK0 |= (1 << OCIE0A); // Enable Timer0 compare match A interrupt


  // Set up Timer1 for 3 minutes interval
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0; // Initialize counter value to 0
  OCR1A = 31250; // Set compare match value for 3 minutes interval
  TCCR1B |= (1 << WGM12); // Configure timer for CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler = 1024
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare match A interrupt
}


void setup() {
  Serial.begin(115200);
  CircuitPlayground.begin();
  CircuitPlayground.setAccelRange(LIS3DH_RANGE_8_G);
  timerSetUp();
}


void loop() {
  if (isFFTComplete) {
    // Code to execute after FFT analysis is done
    // Reset isFFTComplete flag
    isFFTComplete = false;
   FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Apply a window function to minimize leakage effect
   FFT.compute(FFT_FORWARD);                         // Compute the FFT
   FFT.complexToMagnitude();                         // Compute magnitudes
   // Pass the max frequency and magnitude to calculate intensity function
   calculateIntensity();
  }


  if (is3MinElapsed) {
    // Calculate average intensity
    float averageIntensity = totalIntensity / intensityCount;
    Serial.print("Average Intensity over 3 minutes: ");
    Serial.println(averageIntensity);
    determineTremorType(averageIntensity);


    // Reset intensity count and total intensity
    intensityCount = 0;
    totalIntensity = 0;


    // Reset 3 minutes elapsed flag
    is3MinElapsed = false;
  }


  // Check if it's time to turn off the tremor type LED
  if (millis() - tremorTypeLEDStartTime >= LED_ON_DELAY) {
    CircuitPlayground.setPixelColor(9, 0, 0, 0); // Turn off the LED
  }
}

















// ORIGINAL CODE

// #include <Adafruit_CircuitPlayground.h>
// #include "arduinoFFT.h"

// const uint16_t SAMPLE_SIZE = 64;
// const float SAMPLING_FREQUENCY = 200; 

// float vReal[SAMPLE_SIZE];
// float vImag[SAMPLE_SIZE];

// ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLE_SIZE, SAMPLING_FREQUENCY);

// const unsigned long INTERVAL = 2000 / SAMPLING_FREQUENCY; // Sampling interval in milliseconds
// const unsigned long TOTAL_DURATION = 180000; // Sampling duration in milliseconds (3 minutes)
// unsigned long startTime = 0; // To store the start time of the sampling
// bool displayTremorTypeOnly = false;

// float totalIntensity = 0; // Sum of all intensity readings
// int intensityCount = 0; // Number of intensity readings taken

// // High-pass filter parameters
// const float alpha = 0.9;
// float filteredX = 0, filteredY = 0, filteredZ = 0;
// float lastAccX = 0, lastAccY = 0, lastAccZ = 0;

// void setup() {
//   Serial.begin(115200);
//   CircuitPlayground.begin();
//   while (!Serial); // Wait for the serial monitor to open
//   Serial.println("FFT Analysis Ready");
//   startTime = millis();
// }

// void collectAccelerometerData() {
//   for (uint16_t i = 0; i < SAMPLE_SIZE; i++) {
//     float accX = CircuitPlayground.motionX();
//     float accY = CircuitPlayground.motionY();
//     float accZ = CircuitPlayground.motionZ();

//     // Apply high-pass filter to make sure intensity starts low when no movement
//     filteredX = alpha * (filteredX + accX - lastAccX);
//     filteredY = alpha * (filteredY + accY - lastAccY);
//     filteredZ = alpha * (filteredZ + accZ - lastAccZ);

//     lastAccX = accX;
//     lastAccY = accY;
//     lastAccZ = accZ;

//     // Use filtered values for magnitude calculation
//     float magnitude = sqrt(filteredX * filteredX + filteredY * filteredY + filteredZ * filteredZ);
//     vReal[i] = magnitude; // Store magnitude
//     vImag[i] = 0; // Imaginary part is zero for real signals
//     delay(INTERVAL); // Delay for the sampling period
//   }
//   FFT.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD); // Apply Hanning window using built-in function
// }

// void displayIntensityOnLEDs(float intensity) {
//   const uint8_t maxIntensityLED = 9; // Maximum number of LEDs to use for intensity
//   if (!displayTremorTypeOnly) {
//     uint8_t ledsToLight = (uint8_t)(maxIntensityLED * intensity / 20); // Assuming intensity scale max is 20
//     for (uint8_t i = 0; i < maxIntensityLED; i++) {
//       if (i < ledsToLight) {
//         CircuitPlayground.setPixelColor(i, 0, 255, 0); // Green color for visible intensity
//       } else {
//         CircuitPlayground.setPixelColor(i, 0, 0, 0); // Turn off the LED
//       }
//     }
//   } else {
//     // When displaying tremor type, ensure all intensity LEDs are off
//     for (uint8_t i = 0; i < maxIntensityLED; i++) {
//       CircuitPlayground.setPixelColor(i, 0, 0, 0); // Turn off the LED
//     }
//   }
// }

// void determineTremorType(float intensity) {
//   const uint8_t tremorTypePixel = 9; // The index of the last NeoPixel reserved for tremor type
//   CircuitPlayground.setPixelColor(tremorTypePixel, 0, 0, 0); // Ensure LED is off before setting new color
//   if (intensity >= 15 && intensity <= 20) {
//     CircuitPlayground.setPixelColor(tremorTypePixel, 255, 0, 0); // Big tremor - red light
//   } else if (intensity >= 3.1 && intensity < 14.9) {
//     CircuitPlayground.setPixelColor(tremorTypePixel, 128, 0, 128); // Medium Tremor - purple light
//   } else if (intensity >= 1 && intensity <= 3) {
//     CircuitPlayground.setPixelColor(tremorTypePixel, 0, 0, 255); // Small tremor - blue light
//   }
// }

// float calculateIntensity(float *fftResults, uint16_t bufferSize, float peakFrequency) {
//   float intensity = 0.0;
  
//   // Check if the peak frequency is within the desired range (3 to 6 Hz)
//   if (peakFrequency >= 3 && peakFrequency <= 6) {
//     float sum = 0.0;
//     for (uint16_t i = 3; i <= 6; i++) {
//       uint16_t index = (uint16_t)((i / SAMPLING_FREQUENCY) * bufferSize);
//       if (index < bufferSize) {
//         sum += fftResults[index] * fftResults[index];
//       }
//     }
//     intensity = sqrt(sum) / SAMPLE_SIZE;
//     totalIntensity += intensity;
//     intensityCount++;
//   }
  
//   return intensity;
// }

// void performFFTAndIntensityCalculation() {
//   FFT.compute(vReal, vImag, SAMPLE_SIZE, FFT_FORWARD);
//   FFT.complexToMagnitude(vReal, vImag, SAMPLE_SIZE);
  
//   float peakFrequency = FFT.majorPeak();
//   Serial.print("Frequency: ");
//   Serial.print(peakFrequency);
//   Serial.println(" Hz");

//   // Check if the peak frequency is within the desired range (3 to 6 Hz)
//   if (peakFrequency >= 3 && peakFrequency <= 6) {
//     float intensity = calculateIntensity(vReal, SAMPLE_SIZE, peakFrequency);
//     Serial.print("Intensity: ");
//     Serial.println(intensity);
//     if (!displayTremorTypeOnly) {
//       displayIntensityOnLEDs(intensity);
//     } else {
//       determineTremorType(intensity);
//     }
//   } else {
//     Serial.println("Frequency not within desired range (3 to 6 Hz). Skipping intensity calculation.");
//   }
// }

// void loop() {
//   unsigned long currentTime = millis();

//   if (currentTime - startTime < TOTAL_DURATION) {
//     collectAccelerometerData();
//     performFFTAndIntensityCalculation();
//   } else {
//     if (!displayTremorTypeOnly) {
//       // Calculate and display average intensity
//       float averageIntensity = totalIntensity / intensityCount;
//       Serial.print("Average Intensity over 3 minutes: ");
//       Serial.println(averageIntensity);
//       displayTremorTypeOnly = true;  // Switch to displaying tremor type

//       // Determine tremor type and display
//       determineTremorType(averageIntensity);

//       // Reset all intensity LEDs
//       for (uint8_t i = 0; i < 9; i++) {  // Ensure to leave the last LED for tremor type
//         CircuitPlayground.setPixelColor(i, 0, 0, 0); // Turn off the LED
//       }

//       delay(10000); // Display for 10 seconds
//     } else {
//       // Turn off all LEDs after the tremor type display period
//       for (uint8_t i = 0; i < 10; i++) { // Now include the tremor type LED
//         CircuitPlayground.setPixelColor(i, 0, 0, 0); // Turn off the LED
//       }
//       displayTremorTypeOnly = false;
//       startTime = millis(); // Reset start time for another cycle
//       totalIntensity = 0; // Reset total intensity
//       intensityCount = 0; // Reset intensity count
//     }
//   }
// }













