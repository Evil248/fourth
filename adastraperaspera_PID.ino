#include <Arduino.h>
#define pb push_back

enum State {
  LIFT,
  RETRACT,
  IDLE
};

#define controlPin A7
double setPoint = 30.0;

class Driver {
public:
  int in1, in2, ena, in3, in4, enb;
   int id;
  Driver(int in1, int in2, int ena, int in3, int in4, int enb, int id) {
    this->in1 = in1;
    this->in2 = in2;
    this->ena = ena;
    this->in3 = in3;
    this->in4 = in4;
    this->enb = enb;
    this->id = id;
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ena, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(enb, OUTPUT);
    // Serial.println("Driver initialized and pin modes set");
  }
};

const int maxPos = 73.8;
const int minPos = 0.0;
const int tolerance = 0.1;

class Actuator {
public:
  int IN1, IN2, ENA;
  Driver* driver;
  int potPin;
  State state;
  int outNum;

  Actuator(Driver* driver, int potPin, int outNum) {
    if (driver == NULL) {
      Serial.println("Driver cannot be null");
      return;
    }
    this->driver = driver;
    this->potPin = potPin;
    if (outNum == 1 || outNum == 2) {
      this->outNum = outNum;
    } else {
      Serial.println("Invalid OUT number. Must be 1 or 2");
      return;
    }
    this->IN1 = outNum == 1 ? driver->in1 : driver->in3;
    this->IN2 = outNum == 1 ? driver->in2 : driver->in4;
    this->ENA = outNum == 1 ? driver->ena : driver->enb;

    int xyi = analogRead(potPin);
    float distance = map(xyi, 0, 1023, 0, 7380) / 100.0;
    if (distance > 10) {
      state = RETRACT;
    }else{
      state = LIFT;
    }
    Serial.print("Setup done for actuator at pin ");
    Serial.println(potPin);
  }
};
// class Trajectory{
//   public:
//     vector<double> points;
//     void addPoint(double point){
//       points.pb(point);
//     }

// };


class PIDFController{
    public:
    double kp, ki, kd, kf;
    double setpoint;
    double error;
    double prevError;
    double prevTime;
    double totalError;
    double measuredValue;
    double period;

    PIDFController(double kp, double ki, double kd, double kf){
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->kf = kf;
        this->prevTime = 0;
        this->totalError = 0;
        this->prevError = 0;
        this->period = 0;
        this->measuredValue = 0;
    }

    PIDFController(double kp){
        this->kp = kp;
        this->ki = 0;
        this->kd = 0;
        this->kf = 0;
        this->prevTime = 0;
        this->totalError = 0;
        this->prevError = 0;
        this->period = 0;
        this->measuredValue = 0;
    }

    double setSetpoint(double setpoint){
        this->setpoint = setpoint;
    }

    double setPIDF(double kp, double ki, double kd, double kf){
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->kf = kf;
    }

    double calculate(double curDistance, double setDistance){
        if(setDistance > maxPos){
            setDistance = maxPos;
        }
        if(setDistance < minPos){
            setDistance = minPos;
        }
        prevError = error;
        double curTime = millis();
        if(this->prevTime == 0.0){
            this->prevTime = curTime;
        }
        this->period = curTime - this->prevTime;
        this->prevTime = curTime;
        this->measuredValue = curDistance;
        this->error = setDistance - curDistance;
        this->totalError += error;
        
        double p = this->kp * this->error;
        double i = this->ki * this->totalError;
        double d = this->kd * (this->error - this->prevError) / this->period;
        double f = this->kf * setDistance;

        return p + i + d + f;
    }
    
};

void showTelemetry(int driverId, int actuatorId, int sensorValue, float distance, bool upreached, bool minreached, State state, double power, double target) {
    Serial.print("Driver ID: ");
    Serial.print(driverId);
    Serial.print(". ");
  Serial.print("Actuator ID: ");
  Serial.print(actuatorId);
  Serial.print(". Sensor Value: ");
  Serial.print(sensorValue);
  Serial.print(" -> Distance: ");
  Serial.print(distance);
  Serial.print(" mm. ");
  Serial.print(" Uplim reached? ");
  Serial.print(upreached);
  Serial.print(" Minlim reached? ");
  Serial.print(minreached);
  Serial.print(" State? ");
  switch (state) {
    case LIFT:
      Serial.print("LIFT. ");
      break;
    case RETRACT:
      Serial.print("RETRACT. ");
      break;
    case IDLE:
      Serial.println("IDLE. ");
      break;
  }
    Serial.print("Power: ");
    Serial.print(power);
    Serial.print(" Target: ");
    Serial.print(target);
    Serial.print(" Error: ");
    Serial.print(target - distance);
    Serial.println(" mm.");
}



void updateActuator(Actuator* actuator, PIDFController* pidf, double setPoint){
  int in1 = actuator->IN1;
  int in2 = actuator->IN2;
  int ena = actuator->ENA;
  int potPin = actuator->potPin;
  int sensorValue = analogRead(actuator->potPin);
  float distance = map(sensorValue, 0, 1023, 0, 7380) / 100.0;
  //
  double dist = analogRead(actuator->potPin);
  dist = map(dist, 0, 1023, 0, 7380) / 100.0;
  double power = pidf->calculate(dist, setPoint);
  power = constrain(power, -255, 255);
  //
  bool upLimReached = (abs(maxPos - distance) <= tolerance);
  bool lowLimReached = (abs(minPos - distance) <= tolerance);
  bool targetReached = (abs(pidf->setpoint - distance) <= tolerance);
   if(!targetReached && actuator->state == IDLE){
     actuator->state = pidf->setpoint > distance ? LIFT : RETRACT;
   }
   if(power > 0){
        actuator->state = LIFT;
   }else if(power < 0){
        actuator->state = RETRACT;
   }else{
        actuator->state = IDLE;
   }
   showTelemetry(actuator->driver->id, actuator->outNum, sensorValue, distance, upLimReached, lowLimReached, actuator->state, power, setPoint);

  //  graphPID(setPoint, distance);

   switch(actuator->state){
    case LIFT:
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(ena, abs(power));
      if(targetReached){
        actuator->state = IDLE;
      }
      break;
    case RETRACT:
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(ena, abs(power));
      if(targetReached){
        actuator->state = IDLE;
      }
      break;
    case IDLE:
      // Implement any IDLE state behavior if needed
      // Serial.print("Actuator at pin ");
      //   Serial.print(potPin);
      //   Serial.println(" is now IDLE");
      break;
   }
}

// void updateActuatorByTrajectory(Actuator* actuator, Trajectory traj){
  
// }
// void moveActuatorToPoint(Actuator* actuator, double setPoint){
//   double dist = analogRead(actuator->potPin);
//   dist = map(dist, 0, 1023, 0, 7380) / 100.0;
//   double power = pidf->calculate(dist, setPoint);
//   power = constrain(power, -255, 255);
//   updateActuator(actuator, pidf, power);
// }

Driver* driver1;
Driver* driver2;
Driver* driver3;
Actuator* actuator1;
Actuator* actuator2;
Actuator* actuator3;
Actuator* actuator4;
Actuator* actuator5;
Actuator* actuator6;

PIDFController* pidf;

void setup() {
  Serial.begin(9600);
  // driver1 = new Driver(9, 10, 8, 40, 42, 44, 1);  // Pins for driver 1 // a0, a5
  driver1 = new Driver(43,41,2, 34, 32, 7, 1);
  driver2 = new Driver(42, 40, 3, 37, 39, 6, 2); // Pins for driver 2 // a4, a1
  driver3 = new Driver(38,36, 4,44,45,46, 3);
   
  actuator1 = new Actuator(driver1, A0, 1);
  actuator2 = new Actuator(driver1, A3, 2);
  actuator3 = new Actuator(driver2, A1, 1);
  actuator4 = new Actuator(driver2, A4, 2);
  actuator5 = new Actuator(driver3, A2, 1);
  actuator6 = new Actuator(driver3, A5, 2); // Added actuator3 for driver2

  // actuator3_pidf = new PIDFController(58, 0, 3.5, 0);
  // actuator3_pidf = new PIDFController(52, 0.00001, 5, 0.0);
  pidf = new PIDFController(100, 0, 0, 0);
}


double sp;
double pv;

void graphPID(double x, double y){
    sp = x;
    pv = y;
}


void readControlPin(){
    double val = analogRead(controlPin);
    setPoint = map(val, 0, 1023, 0, 7380) / 100.0;
}


void loop() {

//   updateActuator(actuator1);
//   updateActuator(actuator2);
//   updateActuator(actuator3);
//   updateActuator(actuator4);
//   updateActuator(actuator5);
//   updateActuator(actuator6);
    // readControlPin();
    
    // double dist = analogRead(actuator1->potPin);
    // dist = map(dist, 0, 1023, 0, 7380) / 100.0;
    // double power = pidf->calculate(dist, setPoint);
    // power = constrain(power, -255, 255);
    // updateActuator(actuator1, pidf, power);
  updateActuator(actuator1, pidf, setPoint);
  updateActuator(actuator3, pidf, setPoint);
  updateActuator(actuator5, pidf, setPoint);
  updateActuator(actuator4, pidf, setPoint);
  updateActuator(actuator2, pidf, setPoint);
  updateActuator(actuator6, pidf, setPoint);
  // returnActuatorToZero(actuator1);
    //  graphPID(dist, setPoint);
    // Serial.print("SP: ");
    // Serial.print(setPoint);
    // Serial.print("\t");
    // Serial.print("PV: ");
    // Serial.println(dist);
}

void returnActuatorToZero(Actuator* actuator){
    int in1 = actuator->IN1;
    int in2 = actuator->IN2;
    int ena = actuator->ENA;
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(ena, 255);
}

void justMoveItToSetTarget(Actuator* actuator, double target){
    int in1 = actuator->IN1;
    int in2 = actuator->IN2;
    int ena = actuator->ENA;
    double distance = analogRead(actuator->potPin);
    distance = map(distance, 0, 1023, 0, 7380) / 100.0;
    // showTelemetry(actuator->driver->id, actuator->outNum, analogRead(actuator->potPin), distance, false, false, actuator->state, 255, target);
    if(!(abs(distance - target) <= tolerance)){
        if(target > distance){
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            analogWrite(ena, 255);
        }else{
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            analogWrite(ena, 255);
        }
    }else{
        analogWrite(ena, 0);
    }
}
//Current workflow:

//worked witgh actuator 1, 2
//workes for 1,2,3 SOLO
//worekd FOR 1,2,3 MULTI !!!!!!!!!!!!!! lessgooooo
//worked for 4 SOLO
//worked for 1,2,3,4 MULTI !!!!!!!!!!!!
//worked for 5 SOLO
//worked for 6 solo
//worked for 1,2,3,4,5,6 MULTI!!!!!
//DONE

