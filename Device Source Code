#include <SenAi_Test_inferencing.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BluetoothSerial.h>


#define SAMPLING_FREQ_HZ      10
#define SAMPLING_PERIOD_MS    1000 / SAMPLING_FREQ_HZ
#define NUM_CHANNELS          EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME
#define NUM_READINGS          EI_CLASSIFIER_RAW_SAMPLE_COUNT
#define NUM_CLASSES           EI_CLASSIFIER_LABEL_COUNT      

Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT;

char com;
String trigger;
int state = 0;

void setup(void) {
  Serial.begin(115200);
  SerialBT.begin("Senyas Device");

  while (!Serial){
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Hardware Initiated");

  //setupt motion detection
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.println("MPU set");
  delay(1000);
}


void loop() {

    unsigned long timestamp;
    ei_impulse_result_t result;
    int err;
    float input_buf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
    signal_t signal;
    float maxProbability = 0.0;
    int maxIndex = 25;
  
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;

    while (SerialBT.available()){
    delay(10);
    com = SerialBT.read();
    trigger += com;
    }

    while(trigger == "a"){
    delay(10);
    for (int i = 0; i < NUM_READINGS; i++){
      timestamp = millis();

      int joy_1 = analogRead(25);
      int joy_2 = analogRead(33);
      int joy_3 = analogRead(32);
      int joy_4 = analogRead(35);
      int joy_5 = analogRead(34);

      acc_z = a.acceleration.x;
      acc_y = a.acceleration.y;
      acc_x = a.acceleration.z;
      gyro_x = g.gyro.x;
      gyro_y = g.gyro.y;
      gyro_z = g.gyro.z;
      //Perform Standardization
      input_buf[(NUM_CHANNELS * i) + 0] = joy_1;
      input_buf[(NUM_CHANNELS * i) + 1] = joy_2;
      input_buf[(NUM_CHANNELS * i) + 2] = joy_3;
      input_buf[(NUM_CHANNELS * i) + 3] = joy_4;
      input_buf[(NUM_CHANNELS * i) + 4] = joy_5;
      input_buf[(NUM_CHANNELS * i) + 5] = acc_x;
      input_buf[(NUM_CHANNELS * i) + 6] = acc_y;
      input_buf[(NUM_CHANNELS * i) + 7] = acc_z;
      input_buf[(NUM_CHANNELS * i) + 8] = gyro_x;
      input_buf[(NUM_CHANNELS * i) + 9] = gyro_y;
      input_buf[(NUM_CHANNELS * i) + 10] = gyro_z;
      while (millis() <  timestamp + SAMPLING_PERIOD_MS);
    }
 

    //Raw buffer to signal for inference
    err = numpy::signal_from_buffer(input_buf, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err!=0){
      Serial.print("ERROR: Failed to create signal from buffer");
      Serial.print(err);
      return;
    }
  
    //Run Inference
    err = run_classifier(&signal, &result, false);
    if (err != 0){
      Serial.print("ERROR: Failed to run classifier");
      Serial.print(err);
      return;
    }

  Serial.println("Predicted");
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      Serial.print(" ");
      Serial.print(result.classification[i].label);
      Serial.print(": ");
      Serial.println(result.classification[i].value);
      if (result.classification[i].value > maxProbability) {
          maxProbability = result.classification[i].value;
          maxIndex = i;
      }
  }

  if (maxProbability > 0.55){
    SerialBT.print(result.classification[maxIndex].label);
    SerialBT.print(" ");
    Serial.print("Result:");
    Serial.print(result.classification[maxIndex].label);
    Serial.print(" ");
  }else {Serial.println("");};

    trigger = "";
    break;
 }
  delay(100);
}

