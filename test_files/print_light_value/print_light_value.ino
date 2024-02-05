int lightSensor = A16;
int num = 0;
void setup(){
    Serial1.begin(57600);
    pinMode(lightSensor,INPUT);
}
void loop(){
    delay(100);
    Serial1.println(analogRead(lightSensor));
}