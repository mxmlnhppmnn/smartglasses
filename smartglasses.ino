#include <ESP32Servo.h>
#include <RingBuf.h>
#include "Wire.h"
#include "Adafruit_TCS34725.h"

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

#define DELAY_POTENTIOMETER_PIN 36
#define SENS_POTENTIOMETER_PIN 39

static const int servoPin = 13;


static const int MAX_BUF = 100; //Max history of measurments of blue light
static const int MIN_BUF = 3; //Min history

static const int DELAY_MINVAL = 0;
static const int DELAY_MAXVAL = 360;

static const int SENS_MINVAL = 0;
static const int SENS_MAXVAL = 360;

static const double HIGHEST_BLUE_VALUE = 0.9; //but realistically 0.9 since nothing contains 100% blue light

bool G_glasses_down = false;

Servo servo1;

RingBuf<double, MAX_BUF> dataHistory;

int loops_since_change = 0;
double blue_value_perc = 0.0;
double response_delay = 0.0;
double sensibility = 0.0;

double cumulated_blue_light_exporsure_avg = 0.0;

bool filter_mode_on = true;



void setup() {

  Serial.begin(115200);
  servo1.attach(servoPin);
  analogSetAttenuation(ADC_11db);
  //init_buffer();
  init_buffer2();

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

}

void loop() {

  delay(1000); //granularity

  loops_since_change++;

  //fetch and store blue light value
  blue_value_perc = get_blue_light_perc();
  dataHistory.pushOverwrite(blue_value_perc);

  
  if(filter_mode_on) {
    //Threshold ranges from HIGHEST VALUE when sensibility = 0
    //to 0.1 * HIGHTEST VALUE when sensibility = 1

    //normalize inputs to interval [0;1]
    int delay_measurement = read_delay();
    response_delay = normalize_measurement(delay_measurement, DELAY_MINVAL, DELAY_MAXVAL);

    int sens_measurement = read_sensitivity();
    sensibility = normalize_measurement(sens_measurement, SENS_MINVAL, SENS_MAXVAL);

    //calculate average blue light exposure (amt_summands = number of loops)
    //for response delay= 0.1 => 3 + 0.1 * 97 = aprox. 10 secs.
    //for response delay = 0 => 3 secs.
    int amt_summands = (double)MIN_BUF + response_delay * ((double)MAX_BUF - (double)MIN_BUF);


    //only calculate when minimum time of response delay has passed ()
    if(loops_since_change > amt_summands) {
      cumulated_blue_light_exporsure_avg = last_avg(amt_summands); 
    } 
    
    Serial.print("Response Delay: ");
    Serial.println(response_delay);
    Serial.print("Sensibility: ");
    Serial.println(sensibility);
    Serial.print("Amount summands: ");
    Serial.println(amt_summands);
    Serial.print("Loops since last change: ");
    Serial.println(loops_since_change);
    Serial.print("Cumulated Blue Light Exporsure Avg: ");
    Serial.println(cumulated_blue_light_exporsure_avg);

    print_buffer();
    //Serial.print(amt_summands);


    double THRESHOLD = HIGHEST_BLUE_VALUE * (1.0 - (0.9 * sensibility));
    Serial.print("THRESHOLD: ");
    Serial.println(THRESHOLD);

    if(!G_glasses_down && cumulated_blue_light_exporsure_avg > THRESHOLD) {
      activate_protection();
      loops_since_change = 0;
      G_glasses_down = true;
      Serial.println("Glasses are turned down! \n");
    }
    if(G_glasses_down && cumulated_blue_light_exporsure_avg < THRESHOLD) {
      deactivate_protection();
      loops_since_change = 0;
      G_glasses_down = false;
      Serial.println("Glasses are turned up! \n");
    }
  } 
  
}

void activate_protection() {
  for(int posDegrees = 0; posDegrees <= 90; posDegrees++) {
    servo1.write(posDegrees);
    delay(20);
  }
}

void deactivate_protection() {
  for(int posDegrees = 90; posDegrees >= 0; posDegrees--) {
    servo1.write(posDegrees);
    delay(20);
  }
}

int read_sensitivity() {
  int analogValue = analogRead(SENS_POTENTIOMETER_PIN);
  return map(analogValue, 0, 4095, 0, 360);
}

int read_delay() {
  int analogValue = analogRead(DELAY_POTENTIOMETER_PIN);
  return map(analogValue, 0, 4095, 0, 360);
}

double get_blue_light_perc() {
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC);
  Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
  //Blue light is not reliable, using light intensity (claridad c) for now...
  double result = (double)c/10000.0;
  Serial.print("Result: "); Serial.println(result);
  return result;
}

double last_avg(int amt_summands) {
  Serial.println("--------------");
  double data = 0.0;
  double sum = 0.0;
  for(int i=0; i<amt_summands; i++) {
    dataHistory.peek(data, dataHistory.size()-1-i); 
    sum += data;
    Serial.println(data);
  }
  Serial.println("--------------");
  return sum / amt_summands;
}

//helper functions

void init_buffer() {
  for(int i=0; i<dataHistory.size(); i++) {
    dataHistory.pushOverwrite(0.0);
  }
}

void init_buffer2() {
  double data = 0.0;
  while(dataHistory.push(data));
}

void print_buffer() {
  double data = 0.0;
  for(int i=0; i<dataHistory.size(); i++) {
    dataHistory.peek(data, i);
    Serial.print(data);
    Serial.print(" ");
  }
  Serial.println();
}

double normalize_measurement(int measurement, int MIN, int MAX) {
  return (measurement - (double)MIN) / ((double)MAX - (double)MIN);
}



