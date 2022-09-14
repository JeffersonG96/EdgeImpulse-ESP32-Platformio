#include <Arduino.h>

#include <MPU_Acelerometro_esp_inferencing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define FREQUENCY_HZ 200 //160
#define INTERVAL_MS  (1000 / (FREQUENCY_HZ + 1))
static unsigned long last_interval_ms = 0;

Adafruit_MPU6050 mpu;


long tiempo_prev;
float dt;
float ang_x, ang_y, ang_z;
float ang_x_prev, ang_y_prev,ang_z_prev;
float G_R = 131;

//?variable para edge//
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;



void setup(void) {
  Serial.begin(115200);

  // Try to initialize!
  Serial.println(mpu.begin() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();
    // Serial.print("Acc X: ");
    // Serial.print(a.acceleration.x);
    // Serial.print(",");
    // Serial.print(a.acceleration.y);
    // Serial.print(",");
    // Serial.println(a.acceleration.z);
    // Serial.println(" m/s^2 - ");

    // float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    // size_t ix = 0;

        features[feature_ix++] = a.acceleration.x;
        features[feature_ix++] = a.acceleration.y;
        features[feature_ix++] = a.acceleration.z;

    if(feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE){
    Serial.println("Running the inference...");
    signal_t signal;
    ei_impulse_result_t result;
    int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);

    if(res != 0) return;

    // if (err != EI_IMPULSE_OK) {
    //     ei_printf("ERR: Failed to run classifier (%d)\n", err);
    //     return;
    // }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf(" %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        if(result.classification[ix].value > 0.7){
          if(result.classification[ix].label == "Horizontal"){
            Serial.println("****HORIZONTAL****");
          }
          if(result.classification[ix].label == "Vertical"){
            Serial.println("*****VERTICAL****");
          }
        }
    }


#if EI_CLASSIFIER_HAS_ANOMALY == 1
ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
    feature_ix = 0;

  }
  // Serial.println("variable ix");
  } 


}


