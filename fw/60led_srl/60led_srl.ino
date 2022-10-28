// Copyright 2022 Josh Pieper, jjp@pobox.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


////////////////////////////////////
// Render a spectrum analyzer across a 60 LED neopixel display using the 
// onboard microphone of a Arduino Nano BLE Sense
// 
//  Circuit:
//  - Arduino Nano 33 BLE board
//  - Pin 3 connected to the neopixel data bus


#include <Adafruit_NeoPixel.h>
#include <Adafruit_ZeroFFT.h>
#include <PDM.h>

// Which pin on the Arduino is connected to the NeoPixels?
const int PIN = 3;

// How many NeoPixels are attached to the Arduino?
const int NUMPIXELS = 60;

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// The audio sampling rate.  Note, the Arduino Nano BLE Sense only
// supports 16000.
const int frequency = 16000;

// This must be a power of two.  It controls the maximum update rate 
// (frequency / fft_size).  Smaller values will also limit the amount 
// of low frequency information available.
static const int fft_size = 1024;

// In operation, audio data will be read into a buffer from an ISR, 
// when it is full it will be "handed off" to the main loop to run the
// FFT and update the LEDs.  Then the ISR will switch to the other
// buffer.
short audio_buffer1[fft_size];
int buffer1_size = 0;
short audio_buffer2[fft_size];
int buffer2_size = 0;

// The writer_buffer/write_size keep track of the buffer that is
// currently being written to by the ISR.
short* write_buffer = audio_buffer1;
int* volatile write_size = &buffer1_size;

// The read_buffer/read_size keep track of the buffer that is
// currently being read by the main loop.
short* read_buffer = audio_buffer1;
int* volatile read_size = &buffer1_size;


//////////////////////////////////////
// Constants that define the output display.

// How many FFT bins correspond to each pixel of output.
// This is hand fudged to look as good as it can with a 
// range of brass instruments.
const int frequency_bins_count[NUMPIXELS] = {
  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
  2,  2,  2,  2,  2,  2,  2,  2,  2,  3,
 25, 26, 27, 28, 29, 30, 32, 33, 34, 130,
};

const short raw_min_value = 6;

// Some bins have higher minimums to avoid noise causing
// them to constantly illuminate.
float bin_minimum[NUMPIXELS] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0.02, 0.05, 0.02, 0.02, 0.05, 0.02, 0, 0, 0.02, 0.05,  
};

const float min_auto_gain = 0.005f;
float auto_gain_level[NUMPIXELS];

uint32_t loop_count = 0;

void setup() {
  for (auto& v : auto_gain_level) { v = min_auto_gain; }
  
  Serial.begin(115200);
  while (!Serial);

  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and 24 on the Portenta Vision Shield
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  if (!PDM.begin(1, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  // Initialize the NeoPixel display.
  pixels.begin();
}

// Remove the DC bias from an fft_size block of data.
void RemoveDCBias(short* data) {
  int32_t avg = 0;
  for (int i = 0; i < fft_size; i++) {
    avg += data[i];
  }
  avg /= fft_size;
  for (int i = 0; i < fft_size; i++) {
    data[i] -= avg;
  }
}

void loop() {
  if (*read_size != fft_size) { 
    // We don't have a full buffer yet.  Just come back another time.
    return; 
  }

  // We have a full buffer of audio data, let's process it.

  // First, print out some debugging information.
  
//  Serial.print("loop ");
//  Serial.print(loop_count);
//  Serial.println();
  
  loop_count++;

  RemoveDCBias(read_buffer);

  ZeroFFT(read_buffer, fft_size);

//  for (int i = 0; i < fft_size / 2 ; i++) {
//    Serial.print(read_buffer[i]);
//    Serial.print(" ");
//  }

  // Calculate the total power in each pixel.
  float pixel_values[NUMPIXELS] = {};
  int fft_offset = 1;  // skip the first bin
  for (int i = 0; i < NUMPIXELS; i++) {
    float total = 0.0f;
    for (int j = 0; j < frequency_bins_count[i]; j++, fft_offset++) {
      total += std::max<short>(0, read_buffer[fft_offset] - raw_min_value);
    }

//    Serial.print(total);
//    Serial.print(" ");
    
    const auto level = total;
    if (level > auto_gain_level[i]) {
      auto_gain_level[i] = level;
    } else {
      auto_gain_level[i] = std::max(min_auto_gain, auto_gain_level[i] * 0.997f);
    }

    const float gain_level = level / auto_gain_level[i];
    
    pixel_values[i] = gain_level;

    const int r = 0;
    const int g = static_cast<int>(gain_level * 250.0f);
    const int b = 0;
  
    pixels.setPixelColor(i, pixels.Color(r, g, b));

//    if (i % 6 == 0) {
//      Serial.print(" ");
//      Serial.print(i);
//      Serial.print(",");
//      Serial.print(level, 4);
//      Serial.print(",");
//      Serial.print(gain_level, 3);
//      Serial.print(",");
//      Serial.print(auto_gain_level[i], 4);
//    }
  }

  pixels.show();

  Serial.println();
  
  // Mark this buffer as empty.
  *read_size = 0;

  // Then start looking at the next buffer.
  read_size = (read_size == &buffer1_size) ? &buffer2_size : &buffer1_size;
  read_buffer = (read_buffer == audio_buffer1) ? audio_buffer2 : audio_buffer1;  
}


// Process data from the PDM microphone.
//
// This is called from an ISR context, so we can't access peripherals,
// write to serial, or do much beyond manipulating memory.
void onPDMdata() {
  const int available_space_samples = fft_size - *write_size;
  
  // Query the number of available bytes
  const int samples_available = PDM.available() >> 1;
  const int samples_to_read = std::min(available_space_samples, samples_available);

  // Read into the sample buffer
  PDM.read(&write_buffer[*write_size], samples_to_read * 2);  // 2 bytes per sample
  *write_size += samples_to_read;
  
  if (*write_size == fft_size) {
    auto* maybe_next = (write_size == &buffer1_size) ? &buffer2_size : &buffer1_size;

    if (*maybe_next != 0) { 
      // no room. :(  Just discard what we have and start over.
      *write_size = 0;
    } else {
      // We need to switch to the next buffer.
      write_size = maybe_next;
      write_buffer = (write_buffer == audio_buffer1) ? audio_buffer2 : audio_buffer1;
    }
  }
}
