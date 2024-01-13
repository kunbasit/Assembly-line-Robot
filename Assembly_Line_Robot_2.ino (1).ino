/* ----------------------------------------------
 * ROBOT MECHANICAL SYSTEMS - ASSEMBLY LINE ROBOT
 * ----------------------------------------------
 * 
 *  AUTHOR: SHAUN HAMPSON
 *  DATE CREATED: 20/02/2022
 *  DESCRIPTION:
 *      CODE USED TO CALCULATE THE INVERSE KINEMATICS FOR A RRRR ROBOT
 *      USED TO LIFT A METAL PIN AND INSERT IT INTO A HOLE. THE ROBOT
 *      USES AN ELECTROMAGNET END EFFECTOR, THREE REVOLUTE JOINTS (3
 *      HORIZONTAL, 1 VERTICAL AXIS OF ROTATION).
 *      JOINT 4 IS GIVEN A SET ANGLE IN ORDER TO HAVE THE END-EFFECTOR
 *      SET TO A DESIRED DIRECTION.
 *      THE INVERSE KINEMATICS ARE SOLVED VIA ANALYTICAL 
 *  ISSUES:
 *      THE FIRST SERVO MOTORS USED DID NOT ROTATE IN BOTH DIRECTIONS
 *      CORRECTLY, A METHOD WAS CREATED TO TRY TO SOLVE THIS, HOWEVER
 *      THIS DID NOT SOLVE THE ISSUE. INSTEAD, NEW SERVO MOTORS WERE
 *      BOUGHT MEANNING THE CREATED METHOD WAS NOT IMPLEMENTED INTO THE
 *      FINAL WORKING CODE (THE METHOD HAS BEEN LEFT IN TO SHOW WORKING
 *      AND PROBLEM SOLVING, IT IS NOT CALLED ANYWHERE SO NEVER RUNS).
 */
 //LIBARIES
 #include <math.h>
 #include <Servo.h>

//SET PIN NAMES
 Servo joint1, joint2, joint3, joint4;
 int end_effector = 1;
 int start = 2;
 int run = 0;

//CREATE VARIABLES FOR LINKS AND JOINTS
double l1, l2, l3, l4;
double theta1, theta2, theta3, theta4;

//CREATE CONSTANTS FOR CONVERSTIONS
double radToDeg = 180/M_PI;

//SETUP PIN MODES AND FILL CONSTANT/INITIAL PARAMETERS
void setup() {
  //PIN MODES
  joint1.attach(11);
  joint2.attach(10);
  joint3.attach(9);
  joint4.attach(6);
  pinMode(end_effector, OUTPUT);
  pinMode(start, INPUT);

  //SET LINK LENGTHS
  l1 = 172.5;  //distance between joint1 and joint2
  l2 = 110;  //distance between joint2 and joint3
  l3 = 50;  //distance between joint3 and joint4
  l4 = 28;  //distance between joint4 and the tip of the end-effector

  //SET JOINT ANGLES FOR HOME
  theta1 = 0;  //angle for joint1
  theta2 = 0;  //angle for joint2
  theta3 = 0;  //angle for joint3
  theta4 = 0;  //angle for joint4
  jointsSet(theta1, theta2, theta3, theta4);
  delay(15);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(start) == HIGH){
    if(run == 0){
      run = 255;  
    } else {
      run = 0;
    }
  }
  if(run > 0){
    inverseKinematics(10, 10, 10, -45);
    pickupPin();
    inverseKinematics(10, 10, 10, 45);
    pickdownPin();
  }
}

void inverseKinematics(double x, double y, double z, double yaw){
  double yawRad = yaw*M_PI/180;
  
  //theta 1 calculations
  double cos_theta1 = x/sqrt(square(x) + square(y));
  double sin_theta1 = y/sqrt(square(x) + square(y));
  theta1 = atan2(sin_theta1, cos_theta1);

  //End effector joint position
  double x_prime_diff = cos(yawRad)*l4;
  double z_diff = sin(yawRad)*l4;
  double y_diff = sin(theta1)*x_prime_diff;
  double x_diff = cos(theta1)*x_prime_diff;
  double p_x = x - x_diff;
  double p_y = y - y_diff;
  double p_z = z - z_diff;

  //theta 3 calculations
  double x_prime = sqrt(square(p_x) + square(p_y));
  double z_prime = p_z - l1;
  double cos_theta3 = (square(x_prime) +  square(z_prime) - square(l2) - square(l3))/(2*l2*l3);
  double sin_theta3 = -sqrt(1-square(cos_theta3));
  theta3 = atan2(sin_theta3, cos_theta3);

  //theta 2 calculations
  double sin_beta = z_prime/sqrt(square(x_prime)+square(z_prime));
  double cos_beta = x_prime/sqrt(square(x_prime)+square(z_prime));
  double beta = atan2(sin_beta, cos_beta);
  double cos_phi = (square(x_prime)+square(z_prime)+square(l2)-square(l3))/(2*l2*sqrt(square(x_prime)+square(z_prime)));
  double sin_phi = (l3/sqrt(square(x_prime)+square(z_prime)))*sin_theta3;
  double phi = atan2(sin_phi, cos_phi);
  theta2 = beta - phi;

  //convert angles from radians to degrees (not necessarily required but is useful for debugging)
  theta1 = theta1 * radToDeg;
  theta2 = theta2 * radToDeg;
  theta3 = theta3 * radToDeg;

  //set joint angles
  jointsSet(theta1, theta2, theta3, yaw);
}

// SETS JOINT ANGLES, REMAPS ANGLES FROM THE WORKSPACE TO THE SERVOSPACE
void jointsSet(double theta1, double theta2, double theta3, double theta4){
  theta1 = map(theta1, -90, 90, 0, 180);
  theta2 = map(theta2, -90, 90, 0, 180);
  theta3 = map(theta3, -90, 90, 0, 180);
  theta4 = map(theta4, -90, 90, 0, 180);
  joint1.write(theta1);
  joint2.write(theta2);
  joint3.write(theta3);
  joint4.write(theta4);
  delay(1500);
}

// TURNS ON END EFFECTOR
void pickupPin(){
  digitalWrite(end_effector, HIGH);
}

// TURNS OFF END EFFECTOR
void pickdownPin(){
  digitalWrite(end_effector, LOW);
}

/*
 * OLD METHOD FOR FIXING THE FIRST SETV OF MOTORS. NO LONGER USED
 */
void jointsSetDown(double theta1, double theta2, int jointNo){
  theta1 = map(theta1, -90, 90, 0, 180);
  theta2 = map(theta2, -90, 90, 0, 180);
  for(double i = theta1; i >= theta2; i-=4){
    if((i-theta2 < 4)&&(i-theta2 != 0)){
      i-=(i-theta2);
    }
    switch(jointNo){
      case 1:
        joint1.write(i);
      case 2:
        joint2.write(i);
      case 3:
        joint3.write(i);
      case 4:
        joint4.write(i);
    }
    delay(150);
  }
}
