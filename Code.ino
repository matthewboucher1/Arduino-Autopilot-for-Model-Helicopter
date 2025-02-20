//Define Global Variables                                                                                                                 //global variables always defined first 

//Input/Output Pin Allocation                                                                                                             //names added to pins to make last-minute pin changes easy to implement
const int MotorPin = 9;                                                                                                                   //digital PWM pin 9 for controlling motor via H bridge controller
const int PotPin = A0;                                                                                                                    //analogue pin A0 for reading potentiometer
const int LDRPin = A5;                                                                                                                    //analogue pin A5 for reading light dependant resistor, activated on takeoff
const int HeightLSPin = 7;                                                                                                                //digital pin 7 for reading max height limit switch
const int TakeOffLEDPin = 2;                                                                                                              //digital pin 2 for controlling take off LED
const int TwoSecLEDPin = 3;                                                                                                               //digital pin 3 for controlling two sec passed LED
const int TargetHeightLEDPin = 4;                                                                                                         //digital pin 4 for controlling target height aquired LED
const int Bonus1LEDPin = 5;                                                                                                               //digital pin 5 for controlling bonus LED 
const int Bonus2LEDPin = 6;                                                                                                               //digital pin 6 for controlling bonus LED

//LDR Variables                                                                                                                           //variables responsible for reading and monitoring LDR measurements
int InitialLDR;                                                                                                                           //initial low LDR value measured before laser hits sensor
int CurrentLDR;                                                                                                                           //current LDR value monitored until threshold reached to trigger timer
const int TakeOffLDR = 512;                                                                                                               //take off high LDR threshold value when laser hits sensor

//Motor Variables                                                                                                                         //variables responsible for motor control
const float TakeOffDutyCycle = 100;                                                                                                       //motor duty cycle at take off (originally 50%, increased to 100% to allow helicopter to trigger height limit switch) 
const float HoverDutyCycle = 60;                                                                                                          //motor duty cycle at hover (originally 20%, increased to 60% based on experimental trials)
const float LandingDutyCycle = 20;                                                                                                        //motor duty cycle at landing (bonus feature, 20% should bring helicopter down gradually) 
float CurrentDutyCycle;                                                                                                                   //motor duty cycle during main loop
const float MotorBits = 255;                                                                                                              //total bits available for motor control
const float TakeOffMotorDemand = (TakeOffDutyCycle/100)*MotorBits;                                                                        //motor signal sent to H bridge controller at take off
const float HoverMotorDemand = (HoverDutyCycle/100)*MotorBits;                                                                            //motor signal sent to H bridge controller at hover
const float LandingMotorDemand = (LandingDutyCycle/100)*MotorBits;                                                                        //motor signal sent to H bridge controller at landing
float CurrentMotorDemand;                                                                                                                 //motor signal sent to H bridge controller during main loop

//Proportional Control Variables
float PropGain = 0.5;                                                                                                                     //proportional gain set to 0.5
float HeightError;                                                                                                                        //error between target and actual height, should be to +/- 0.01m

//Timer Variables
unsigned long StartTime;                                                                                                                  //start time in millisecs
unsigned long CurrentTime;                                                                                                                //current time in millisecs
unsigned long ElapsedTime;                                                                                                                //elapsed time in millisecs, difference between current and start time

//Height LS Variables
float HeightLS;

//Potentiometer/Altimeter Variables                                                                                                       //defining the variables involved in calibrating the altimeter/potentiometer and establishing helicopter height 
float InitialStep;                                                                                                                        //potentiometer reading at ground, arm not necessarily horizontal, useful for debugging and error checking
float TakeOffStep;                                                                                                                        //potentiometer reading at takeoff, instant when laser hits LDR, arm perfectly horizontal   
float CurrentStep;                                                                                                                        //current potentiometer reading 
float MaxHeightStep;                                                                                                                      //potentiometer reading at max height
const float MaxAngleDeg = 38;                                                                                                             //target angle from horizontal at max height
const float MaxAngleRad = MaxAngleDeg*PI/180.0;                                                                                           //convert angle to radians for use in calculations
float M;                                                                                                                                  //gradient of potentiometer step to angle altimeter calibration equation
float C;                                                                                                                                  //constnat of potentiometer step to angle altimeter calibration equation
float CurrentAngleRad;                                                                                                                    //current angle from calibrated potentiometer
const float SupportRodLength = 0.485;                                                                                                     //length of support rod in m
float CurrentHeight;                                                                                                                      //current height to be provided by height function  
const float TargetHeight = 0.2;                                                                                                           //target height, arbitarily chosen as 0.2m

void setup() {                                                                                                                            //setup code runs once
  //Start Serial Monitor
  Serial.begin(9600);                                                                                                                     //start serial monitor with standard baud rate of 9600 bits/sec
  delay(200);                                                                                                                             //delay to allow serial connection to be established
  
  //Pin Initialisation                                                                                                                    //set the appropiate pins to input or output
  pinMode(MotorPin,OUTPUT);                                                                                                               //motor is an 8 bit digital PWM output to H bridge controller
  pinMode(PotPin,INPUT);                                                                                                                  //potentiometer is a 10 bit analogue input
  pinMode(LDRPin,INPUT);                                                                                                                  //light dependant resistor is a 10 bit analogue input
  pinMode(HeightLSPin,INPUT);                                                                                                             //height limit switch is a binary digital input 
  pinMode(TakeOffLEDPin,OUTPUT);                                                                                                          //LED is a binary digital output
  pinMode(TwoSecLEDPin,OUTPUT);                                                                                                           //LED is a binary digital output
  pinMode(TargetHeightLEDPin,OUTPUT);                                                                                                     //LED is a binary digital output
  pinMode(Bonus1LEDPin,OUTPUT);                                                                                                           //LED is a binary digital output
  pinMode(Bonus2LEDPin,OUTPUT);                                                                                                           //LED is a binary digital output

  //Prepare for Takeoff                                                                                                                   //conduct initial sensor readings prior to take off
  InitialStep = analogRead(PotPin);                                                                                                       //read potentiometer value before takoff to obtain initial reading, useful for debugging purposes 
  InitialLDR = analogRead(LDRPin);                                                                                                        //read LDR value before takeoff to obtain initial reading before laser hits sensor
  
  //Initialise Takeoff                                                                                                                    //motor and LED activated to commence takeoff
  analogWrite(MotorPin,TakeOffMotorDemand);                                                                                               //set the motor to takeoff speed of 50% duty cycle
  digitalWrite(TakeOffLEDPin,HIGH);                                                                                                       //turn on the take off LED indicator
  Serial.println("1. Take-Off");                                                                                                          //print to serial monitor that take off has started
  Serial.flush(); 

  //Complete Takeoff                                                                                                                      //setup flight timer and takeoff potentiometer reading required for calibration
  CurrentLDR = analogRead(LDRPin);                                                                                                        //read the first current LDR value
  while(CurrentLDR < TakeOffLDR){                                                                                                         //while the laser hasn't hit the LDR continue to loop
  CurrentLDR = analogRead(LDRPin);                                                                                                        //update the LDR value
  }                                                                                                                                       //given the laser has hit the LDR and the loop can stop, takeoff potentiometer reading can be taken
  TakeOffStep = analogRead(PotPin);                                                                                                       //record potentiometer step when angle and h = 0
  Serial.println("2. Reached Take-Off Altitude");                                                                                         //print to serial monitor that take off altitude has been achieved
  Serial.flush(); 
  
  //Setup Calibration of Altimeter                                                                                                        //steps to obtain the max height potentiometer reading needed for altimeter calibration
  HeightLS = digitalRead(HeightLSPin);                                                                                                    //read the first current height LS value 
  while(HeightLS == LOW){                                                                                                                 //while the arm hasnt hit the height LS continue to loop
  HeightLS = digitalRead(HeightLSPin);                                                                                                    //update the current height LS value
  }                                                                                                                                       //given the arm has hit the LS and the loop can stop, maximum potentiometer reading can be taken
  MaxHeightStep = analogRead(PotPin);                                                                                                     //record potentiometer step at max angle (38deg)
                                     

  //Calibrate Altimeter
  CalibrateAltimeter();                                                                                                                   //call function to calibrate altimeter and display confirmation text 

  //Hover Helicopter
  StartTime = millis();                                                                                                                   //record start time now that calibration has finished
  analogWrite(MotorPin,HoverMotorDemand);                                                                                                 //set the helicopter to a hover, ideally so it falls to below the target height
  
  //Configure Serial Monitor Display
  Serial.println("4. Auto-Pilot Starting:");                                                                                              //confirm auto-pilot starting up in serial monitor
  Serial.println("LEDs ordered left to right");                                                                                           //bonus feature, added a key to show what each LED indicates
  Serial.println("Two Seconds Elapsed,Take-Off Initiated,Target Height Aquired,Four Seconds Elapsed,");                                   //LED names ordered left to right according to ReLoad camera view
  Serial.println("Time,Height Current,Height Error,Motor Control (%)");                                                                   //setup table headers for data to be recorded under
}

void loop() {                                                                                                                             //loop code runs cyclically 
  //Proportional Control                                                         
  CurrentTime = millis();                                                                                                                 //measure the current time at the start of the loop
  ElapsedTime = CurrentTime - StartTime;                                                                                                  //calculate elapsed time from current time and start time
  CurrentStep = analogRead(PotPin);                                                                                                       //find current potentiometer value 
  CurrentHeight = HelicopterHeight();                                                                                                     //find current height using helicopter height function
  HeightError = TargetHeight-CurrentHeight;                                                                                               //find difference between target height (set point) and current height (proceess variable)
  CurrentDutyCycle = (PropGain*HeightError)*100;                                                                                          //find correction duty cycle using gain value and height error 
  CurrentMotorDemand = (CurrentDutyCycle/100)*MotorBits;                                                                                  //apply correction to motor demand
  
  //Write Results to Serial Monitor                                               
  Serial.print(ElapsedTime);Serial.print(",");                                                                                            //print elapsed time to time collumn
  Serial.print(CurrentHeight);Serial.print(",");                                                                                          //print current height to height current collumn
  Serial.print(HeightError);Serial.print(",");                                                                                            //print height error in respective collumn
  Serial.println(CurrentDutyCycle);                                                                                                       //print %duty cycle in motor control collumn 
  
  //Target Height LED
  if(HeightError <= 0.01){                                                                                                                //As soon as target height +/-0.01m aquired LED will light 
  digitalWrite(TargetHeightLEDPin,HIGH);                                                                                                  //turn on LED
  }
  if(HeightError > 0.01){                                                                                                                 //if target height no longer accurate enough, LED will switch off
  digitalWrite(TargetHeightLEDPin,LOW);                                                                                                   //switch off LED
  }

  //Two Sec LED
  if(ElapsedTime >= 2000){                                                                                                                //if elapsed time is more than 2s
  digitalWrite(TwoSecLEDPin,HIGH);                                                                                                        //turn on LED
  }
  
  //Bonus Feature! Four Sec LED
  if(ElapsedTime >=4000){                                                                                                                 //if elapsed time is more than 4s 
  digitalWrite(Bonus1LEDPin,HIGH);                                                                                                        //turn on bonus1 LED
  delay(500);                                                                                                                             //leave on for 500ms
  digitalWrite(Bonus1LEDPin,LOW);                                                                                                         //switch off again, effect is to blink the light as an alert that more than double allocated time has passed
  delay(500);                                                                                                                             //leave off for 500ms
  }
  
  //Bonus Feature! Configure LEDs in Preparation for Landing
  while(ElapsedTime >= 5000){                                                                                                             //post 5s, the loop effectively must wait infinitely long, using while stops it in its tracks
  digitalWrite(TargetHeightLEDPin,LOW);                                                                                                   //Turn flight LEDs off
  digitalWrite(TwoSecLEDPin,LOW);                                                                                                         //Turn flight LEDs off
  digitalWrite(TakeOffLEDPin,LOW);                                                                                                        //Turn flight LEDs off
  digitalWrite(Bonus1LEDPin,LOW);                                                                                                         //Turn flight LEDs off
  digitalWrite(Bonus2LEDPin,HIGH);                                                                                                        //Turn bonus landing LED on

  //Bonus Feature! Decrease Power to Land Helicopter
  analogWrite(MotorPin,LandingMotorDemand);                                                                                               //land helicopter
  delay(500);                                                                                                                             //500ms delay to allow changes to take effect
  
  //Bonus Feature! Update Serial Monitor with Landing Stats
  CurrentTime = millis();                                                                                                                 //measure the current time
  ElapsedTime = CurrentTime - StartTime;                                                                                                  //calculate elapsed time from current time and start time
  CurrentStep = analogRead(PotPin);                                                                                                       //find current potentiometer value 
  CurrentHeight = HelicopterHeight();                                                                                                     //find current height using helicopter height function
  HeightError = TargetHeight-CurrentHeight;                                                                                               //find difference between target height (set point) and current height (proceess variable)
  CurrentDutyCycle = LandingDutyCycle;                                                                                                    //duty cycle landing update
  CurrentMotorDemand = (LandingDutyCycle/100)*MotorBits;                                                                                  //motor demand landing update
  
  //Bonus Feature! Write Landing Stats to Serial Monitor                                               
  Serial.print(ElapsedTime);Serial.print(",");                                                                                            //print elapsed time to time collumn
  Serial.print(CurrentHeight);Serial.print(",");                                                                                          //print current height to height current collumn
  Serial.print(HeightError);Serial.print(",");                                                                                            //print height error in respective collumn
  Serial.println(CurrentDutyCycle);                                                                                                       //print %duty cycle in motor control collumn 

  //Bonus Feature! Shut Down Helicopter
  analogWrite(MotorPin,0);                                                                                                                //stop the engines as landed, mission complete
  delay(2000);                                                                                                                            //2s delay to stop engines and let helicopter come to rest
  
  //Bonus Feature! Update Serial Monitor with Landing Stats
  CurrentTime = millis();                                                                                                                 //measure the current time
  ElapsedTime = CurrentTime - StartTime;                                                                                                  //calculate elapsed time from current time and start time
  CurrentStep = analogRead(PotPin);                                                                                                       //find current potentiometer value 
  CurrentHeight = HelicopterHeight();                                                                                                     //find current height using helicopter height function
  HeightError = TargetHeight-CurrentHeight;                                                                                               //find difference between target height (set point) and current height (proceess variable)
  CurrentDutyCycle = 0;                                                                                                                   //duty cycle landing update
  CurrentMotorDemand = 0;                                                                                                                 //motor demand landing update
  
  //Bonus Feature! Write Landing Stats to Serial Monitor                                               
  Serial.print(ElapsedTime);Serial.print(",");                                                                                            //print elapsed time to time collumn
  Serial.print(CurrentHeight);Serial.print(",");                                                                                          //print current height to height current collumn
  Serial.print(HeightError);Serial.print(",");                                                                                            //print height error in respective collumn
  Serial.println(CurrentDutyCycle);                                                                                                       //print %duty cycle in motor control collumn 

  //Bonus Feature! Final Mission Briefing
  Serial.println("Mission Impossible...");                                                                                                //print mission completion text
  Serial.println("... is Possible");                                                                                                      //print mission completion text
  Serial.println("THIS MESSAGE WILL SELF DESTRUCT in 1s");                                                                                //print mission completion text
  delay(1000);                                                                                                                            //1s delay before message 'self destructs'
  Serial.end();                                                                                                                           //message 'self destructs' when serial monitoring ends
  }
  
  delay(40);                                                                                                                              //delay to keep loop period 40ms to ensure 25Hz refresh operation
}
  
//Calibrate Altimeter Function
void CalibrateAltimeter(){                                                                                                                //function to calibrate altimeter, taking known inputs and returning current angle from potentiometer step readings 
  M = MaxAngleRad /(MaxHeightStep-TakeOffStep);                                                                                           //calculate M, gradient to linearly convert potentiometer step into angle
  C = MaxAngleRad - M*MaxHeightStep;                                                                                                      //calculate C, constant to linearly convert potentiometer step into angle                                                    
  Serial.print("3. Calibrated Altimeter: ");                                                                                              //print values of M and C to serial monitor (to appropiate number of decimal places)
  Serial.print("M = "); Serial.print(M,4);                                        
  Serial.print(", C = "); Serial.println(C,4);
  Serial.print("Target Height = "); Serial.println(TargetHeight);                                                                         //print target height to serial monitor
}

//Find Helicopter Height
float HelicopterHeight(){                                                                                                                 //function to calculate helicopter height based on values of M and C found in previous function
  CurrentAngleRad = M*CurrentStep + C;                                                                                                    //use y = mx+c relationship to obtain current angle based on potentiometer step
  CurrentHeight = 0.485*sin(CurrentAngleRad);                                                                                             //use basic trigonometry to calculate current height                                                         
  return CurrentHeight;                                                                                                                   //return the current height from the calibration equation                                                                     
}
