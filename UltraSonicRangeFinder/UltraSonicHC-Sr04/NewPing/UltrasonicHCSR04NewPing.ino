#include <NewPing.h>
 
#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 1000
 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
 
void setup() {
   Serial.begin(9600);
}
 
void loop() {
   delay(50);
   unsigned int uS = sonar.ping_cm();
   //Between 3 meters - 10 meters, just assume it is 3 meters
   if(uS> 300){
    uS = 300;
   }
   Serial.print(uS);
   Serial.println("cm");
}
