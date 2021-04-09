#include <ros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/UInt16.h>
#include <PID_v1.h>

#define pwm 9
#define hall 2
#define in1 6
#define in2 7

double setpoint, input, output, kp = 0.6, kd = 0, ki = 0.8; volatile long counter = 0;
unsigned long RPM;
float dt;
float ratio = 0.0024;
unsigned long prev_time = 0, cur_time = 0;


geometry_msgs::Twist sped_msg;

std_msgs::UInt16 PID_msg;
//geometry_msgs::Twist get_sped_msg;
std_msgs::UInt16 get_sped_msg;

int setpoints[] = {65, 70, 80, 90, 110, 130, 150, 180, 200, 220, 0};
int reading;
void subscriber_callback(const std_msgs::UInt16&get_sped_msg)
{


  reading = get_sped_msg.data;
  if (reading <= 10)

    setpoint = setpoints[reading];
  else
    setpoint = reading;
}


ros::Publisher PID_publisher("PID_output", &PID_msg );
ros::Publisher speed_publisher("Current_speed", &sped_msg);
ros:: Subscriber<std_msgs::UInt16> get_sped("Change_your_speed", &subscriber_callback);





ros:: NodeHandle nd;




PID sped_con(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void ISRA()
{
  counter++;
}
void setup() {
  // put your setup code here, to run once:
  pinMode(pwm, OUTPUT);
  pinMode(hall, INPUT_PULLUP);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(hall), ISRA, CHANGE);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  prev_time = millis();
  dt = 200.0;
  sped_con.SetMode(AUTOMATIC);
  setpoint = 80.0;
  //Serial.begin(9600);

  analogWrite(pwm, 150);
  //Serial.print("start....");
  delay(1000);
  sped_con.SetSampleTime(200);
  nd.initNode();
  nd.advertise(PID_publisher);
  nd.advertise(speed_publisher);
  nd.subscribe(get_sped);

}

void loop() {
  // put your main code here, to run repeatedly:

  cur_time = millis();
  if ((cur_time - prev_time) > dt)
  { RPM = (float) counter * ratio / (dt * 0.001) * 60.0;
    input = RPM;
    sped_con.Compute();
    analogWrite(pwm, output);



    sped_msg.angular.x = RPM;
    sped_msg.angular.y = setpoint;
    delay(3);
    PID_msg.data = output;


    delay(3);
    // Serial.print(counter);
    //Serial.print(" RPM is:   ");
    // Serial.print(RPM);
    //  Serial.print(" pid is:   ");
    //   Serial.println(output);
    prev_time = cur_time;
    counter = 0;



    // if (Serial.read()=='1')
    //
    //{
    //  setpoint=setpoint + 50;
    //  if (setpoint >= 350)
    //{setpoint = 70;}
    speed_publisher.publish(&sped_msg);
    PID_publisher.publish(&PID_msg );

    nd.spinOnce();

  }





}
