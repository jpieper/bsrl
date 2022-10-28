/*
  This example reads audio data from the on-board PDM microphones, and prints
  out the samples to the Serial console. The Serial Plotter built into the
  Arduino IDE can be used to plot the audio data (Tools -> Serial Plotter)

  Circuit:
  - Arduino Nano 33 BLE board, or
  - Arduino Nano RP2040 Connect, or
  - Arduino Portenta H7 board plus Portenta Vision Shield

  This example code is in the public domain.
*/

#include <Adafruit_NeoPixel.h>
#include <PDM.h>
#include <Adafruit_ZeroFFT.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        3 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 60 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

static const int fft_size = 1024;

// Buffer to read samples into, each sample is 16-bits
short audio_buffer1[fft_size];
int buffer1_size = 0;
short audio_buffer2[fft_size];
int buffer2_size = 0;

short* write_buffer = audio_buffer1;
int* volatile write_size = &buffer1_size;

short* read_buffer = audio_buffer1;
int* volatile read_size = &buffer1_size;

uint32_t loop_count = 0;

void setup() {
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
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shield
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

void loop() {
  if (*read_size != fft_size) { 
//    Serial.print("no ");
//    Serial.print(*read_size);
//    Serial.print(" ");
//    Serial.print(reinterpret_cast<uint32_t>(read_size));
//    Serial.print(" ");
//    Serial.print(buffer1_size);
//    Serial.print(" ");
//    Serial.print(buffer2_size);
//    Serial.println();
    return; 
  }

  // Do stuff with the read buffer.
  Serial.print("loop ");
  Serial.print(loop_count);
  Serial.print(" ");
  Serial.print(reinterpret_cast<uint32_t>(read_size));
  Serial.println();
  
  Serial.print(" d");
  for (int i = 0; i < 32; i++) {
    Serial.print(" ");
    Serial.print(read_buffer[i]);
  }
  Serial.println();

  loop_count++;


  // Remove the DC bias.
  int32_t avg = 0;
  for (int i = 0; i < fft_size; i++) {
    avg += read_buffer[i];
  }
  avg /= fft_size;
  for (int i = 0; i < fft_size; i++) {
    read_buffer[i] -= avg;
  }

  ZeroFFT(read_buffer, fft_size);

  Serial.print(" f");

  for (int i = 0; i < 32; i++) {
    Serial.print(" ");
    Serial.print(read_buffer[i]);
  }
  Serial.println();

  int pixel_value[NUMPIXELS];
  short max_val = 0;
  for (int i = 0; i < NUMPIXELS; i++) {
    const int start = (fft_size / 2) * i / NUMPIXELS;
    const int end = (fft_size / 2) * (i + 1) / NUMPIXELS;
    int32_t total = 0;
    for (int j = start; j < end; j++) {
      total += abs(read_buffer[j]);
    }
    total /= (end - start);
    pixel_value[i] = total;
    max_val = std::max<int>(pixel_value[i], max_val);
  }

  Serial.print(" p");

  for (int i = 0; i < 32; i++) {
    Serial.print(" ");
    Serial.print(pixel_value[i]);
  }
  Serial.println();


  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(
      0, 
      pixel_value[i] * 30 / max_val,
      0));
  }

  pixels.show();
  
  // Mark this buffer as empty.
  *read_size = 0;

  // Then start looking at the next buffer.
  read_size = (read_size == &buffer1_size) ? &buffer2_size : &buffer1_size;
  read_buffer = (read_buffer == audio_buffer1) ? audio_buffer2 : audio_buffer1;  
}

/**
 * Callback function to process the data from the PDM microphone.
 * NOTE: This callback is executed as part of an ISR.
 * Therefore using `Serial` to print messages inside this function isn't supported.
 * */
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
