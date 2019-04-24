#include <SimpleKalmanFilter.h>
#include <TimedAction.h>

const int NTRIG = 2;
const int NECHO = 3;
const int NETRIG = 4;
const int NEECHO = 5;
const int ETRIG = 6;
const int EECHO = 7;
const int SETRIG = 8;
const int SEECHO = 9;
const int STRIG = 10;
const int SECHO = 11;
const int SWTRIG = 12;
const int SWECHO = 13;
const int WTRIG = A0;
const int WECHO = A1;
const int NWTRIG = A2;
const int NWECHO = A3;

double northDistance = 800, northEastDistance = 800, eastDistance = 800, southEastDistance = 800, southDistance = 800, southWestDistance = 800, westDistance = 800, northWestDistance = 800;
double previousNorthDistance = 800, previousNorthEastDistance = 800, previousEastDistance = 800, previousSouthEastDistance = 800, previousSouthDistance = 800, previousSouthWestDistance = 800, previousWestDistance = 800, previousNorthWestDistance = 800;

const double E_MEA = 2;
const double E_EST = 2;
const double Q = 0.5;

SimpleKalmanFilter northFilter(E_MEA, E_EST, Q);
SimpleKalmanFilter northWestFilter(E_MEA, E_EST, Q);
SimpleKalmanFilter westFilter(E_MEA, E_EST, Q);
SimpleKalmanFilter southWestFilter(E_MEA, E_EST, Q);
SimpleKalmanFilter southFilter(E_MEA, E_EST, Q);
SimpleKalmanFilter southEastFilter(E_MEA, E_EST, Q);
SimpleKalmanFilter eastFilter(E_MEA, E_EST, Q);
SimpleKalmanFilter northEastFilter(E_MEA, E_EST, Q);

TimedAction northSensor = TimedAction(50, northFunc);
void northFunc() {sonar(NTRIG, NECHO, &northDistance, &previousNorthDistance);}

TimedAction northEastSensor = TimedAction(50, northEastFunc);
void northEastFunc() {sonar(NETRIG, NEECHO, &northEastDistance, &previousNorthEastDistance);}

TimedAction eastSensor = TimedAction(50, eastFunc);
void eastFunc() {sonar(ETRIG, EECHO, &eastDistance, &previousEastDistance);}

TimedAction southEastSensor = TimedAction(50, southEastFunc);
void southEastFunc() {sonar(SETRIG, SEECHO, &southEastDistance, &previousSouthEastDistance);}

TimedAction southSensor = TimedAction(50, southFunc);
void southFunc() {sonar(STRIG, SECHO, &southDistance, &previousSouthDistance);}

TimedAction southWestSensor = TimedAction(50, southWestFunc);
void southWestFunc() {sonar(SWTRIG, SWECHO, &southWestDistance, &previousSouthWestDistance);}

TimedAction westSensor = TimedAction(50, westFunc);
void westFunc() {sonar(WTRIG, WECHO, &westDistance, &previousWestDistance);}

TimedAction northWestSensor = TimedAction(50, northWestFunc);
void northWestFunc() {sonar(NWTRIG, NWECHO, &northWestDistance, &previousNorthWestDistance);}

TimedAction printThread = TimedAction(50, printSerial);

void printSerial() {
  Serial.print(northFilter.updateEstimate(northDistance) + 10.33); Serial.print(",");
  Serial.print(northWestFilter.updateEstimate(northWestDistance) + 10.33); Serial.print(",");
  Serial.print(westFilter.updateEstimate(westDistance) + 10.33); Serial.print(",");
  Serial.print(southWestFilter.updateEstimate(southWestDistance) + 10.33); Serial.print(",");
  Serial.print(southFilter.updateEstimate(southDistance) + 10.33); Serial.print(",");
  Serial.print(southEastFilter.updateEstimate(southEastDistance) + 10.33); Serial.print(",");
  Serial.print(eastFilter.updateEstimate(eastDistance) + 10.33); Serial.print(",");
  Serial.println(northEastFilter.updateEstimate(northEastDistance) + 10.33);
}

void sonar(int trigPin, int echoPin, double *distance, double *previousDistance) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  *previousDistance = *distance;
  if (fabs(*distance - *previousDistance) < 2) {
    *distance = (pulseIn(echoPin, HIGH, 125000)/2)/29.1;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(NTRIG, OUTPUT);
  pinMode(NECHO, INPUT);
  pinMode(NETRIG, OUTPUT);
  pinMode(NEECHO, INPUT);
  pinMode(ETRIG, OUTPUT);
  pinMode(EECHO, INPUT);
  pinMode(SETRIG, OUTPUT);
  pinMode(SEECHO, INPUT);
  pinMode(STRIG, OUTPUT);
  pinMode(SECHO, INPUT);
  pinMode(SWTRIG, OUTPUT);
  pinMode(SWECHO, INPUT);
  pinMode(WTRIG, OUTPUT);
  pinMode(WECHO, INPUT);
  pinMode(NWTRIG, OUTPUT);
  pinMode(NWECHO, INPUT);
}

void loop() {
  northFunc();
  northEastFunc();
  eastFunc();
  southEastFunc();
  southFunc();
  southWestFunc();
  westFunc();
  northWestFunc();
  /*
  northSensor.check();
  northEastSensor.check();
  eastSensor.check();
  southEastSensor.check();
  southSensor.check();
  southWestSensor.check();
  westSensor.check();
  northWestSensor.check();
  */
  printThread.check();
}
