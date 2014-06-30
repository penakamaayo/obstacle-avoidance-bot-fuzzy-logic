/*--------------------------------------*
* OBSTACLE AVOIDANCE FOR INTELLIBRAIN-BOT
* USING FUZZY LOGIC CONTROLLER

* Group3
* Pena,James Kristopher
* Sabana,Stevenwil
* Lim, Harlie
* Samporna, Jason Dirk
* Naelga, Janine
* Acenas, Gemrie Mae
* Andrade, Cris Angelo
*--------------------------------------*/

import com.ridgesoft.robotics.SonarRangeFinder;
import com.ridgesoft.robotics.sensors.ParallaxPing;
import com.ridgesoft.intellibrain.IntelliBrain;
import com.ridgesoft.robotics.Motor;
import com.ridgesoft.robotics.Servo;

public class FinalRobot {
	public static final int LEFT = 20;
	public static final int RIGHT = 80;
	public static final int SPEED_LOW = 5;
	public static final int SPEED_MEDIUM = 10;
	public static final int SPEED_HIGH = 16;  
	public static final int X_FAR = 60;
	public static final int X_MEDIUM = 40;
	public static final int X_NEAR = 20;
	public static final int PERFECT_FUZZY= 1;

	public static SonarRangeFinder sonarSensor = new ParallaxPing(IntelliBrain.getDigitalIO(3));
	public static Motor left_motor = IntelliBrain.getMotor(1);
	public static Motor right_motor = IntelliBrain.getMotor(2);
	public static Servo servo = IntelliBrain.getServo(1);
	public static Speaker buzzer = IntelliBrain.getBuzzer();

	public static void main(String args[]) {
		try{
			int LeftWheel;
			int RightWheel;
	    
	    float rule[] = new float[9];
			float LRule[] = new float[9];
			float RRule[] = new float[9];
			
			while(true){
				//get sensor readings
				float left_reading_1 = getSensorReading(LEFT);
				float right_reading_1 = getSensorReading(RIGHT);

				//fuzzification
				float values_left[] = get_fuzzy(left_reading_1);
				float values_right[] = get_fuzzy(right_reading_1);
			
				//evaluate fuzzy rules
				//if left is FAR & right is FAR
				rule[0] = minimum(values_left[2],values_right[2]); 
				LRule[0] = rule[0]*(SPEED_HIGH); 
				RRule[0] = rule[0]*(SPEED_HIGH);

				//if left is NEAR & right is FAR
				rule[1] = minimum(values_left[0],values_right[2]);
				LRule[1] = rule[1]*(SPEED_HIGH); 
				RRule[1] = rule[1]*(SPEED_LOW); 

				//if left is FAR & right is near
				rule[2] = minimum(values_left[2],values_right[0]);
				LRule[2] = rule[2]*(SPEED_LOW); 
				RRule[2] = rule[2]*(SPEED_HIGH);

				//if left is NEAR and right is near
				rule[3] = minimum(values_left[0],values_right[0]);
				LRule[3] = rule[3]*(SPEED_LOW);
				RRule[3] = rule[3]*(SPEED_LOW);

				//if left is NEAR and right is MEDIUM
				rule[4] = minimum(values_left[0],values_right[1]);
				LRule[4] = rule[4]*(SPEED_MEDIUM);
				RRule[4] = rule[4]*(SPEED_LOW);

				//if left is MEDIUM and right is near
				rule[5] = minimum(values_left[1],values_right[0]);
				LRule[5] = rule[5]*(SPEED_LOW);
				RRule[5] = rule[5]*(SPEED_MEDIUM);

				//if left is MEDIUM and right is MEDIUM
				rule[6] = minimum(values_left[1],values_right[1]);
				LRule[6] = rule[6]*(SPEED_MEDIUM);
				RRule[6] = rule[6]*(SPEED_MEDIUM);
				
				//if left is MEDIUM and right is FAR
				rule[7] = get_minimum(values_left[MEDIUM],values_right[FAR]);
				LRule[7] = rule[7]*(SPEED_LOW);
				RRule[7] = rule[7]*(SPEED_MEDIUM);
				
				//if left is FAR and right is MEDIUM
				rule[8] = get_minimum(values_left[FAR],values_right[MEDIUM]);
				LRule[8] = rule[8]*(SPEED_MEDIUM);
				RRule[8] = rule[8]*(SPEED_HIGH);

				//defuzzification
				LeftWheel = Math.round((LRule[0]+LRule[1]+LRule[2]+LRule[3]+LRule[4]+LRule[5]+LRule[6]+LRule[7]+LRule[8])/(rule[0]+rule[1]+rule[2]+rule[3]+rule[4]+rule[5]+rule[6]+rule[7]+rule[8]));
				RightWheel = Math.round((RRule[0]+RRule[1]+RRule[2]+RRule[3]+RRule[4]+RRule[5]+RRule[6]+RRule[7]+RRule[8])/(rule[0]+rule[1]+rule[2]+rule[3]+rule[4]+rule[5]+rule[6]+rule[7]+rule[8]));

				left_motor.setPower(LeftWheel);
				right_motor.setPower(RightWheel);
			}	
		}
		catch(Throwable t) {
			t.printStackTrace();
		}
	}

	public static float getSensorReading(int pos) {
		try {
			servo.setPosition(pos);
			Thread.sleep(150);
			sonarSensor.ping();
			Thread.sleep(250);
		}
		catch(Throwable t){
			t.printStackTrace();
		}
		return sonarSensor.getDistanceCm();
	}

	public static float[] get_fuzzy(float x1){
		float values[] = new float[3];
		float near_y = 0;		//membership
		float medium_y = 0;
		float far_y = 0;

		if(x1<=X_NEAR) {
			near_y = PERFECT_FUZZY;
			medium_y = 0;
			far_y = 0;

		}
		else if(x1<=X_MEDIUM && x1>=X_NEAR){
			near_y = graph_of_near(x1);
			medium_y = graph_of_medium_near(x1);
			far_y = 0;
		}
		else if(x1<=X_FAR && x1>=X_MEDIUM) {
			near_y = 0;
			medium_y = graph_of_medium_far(x1);
			far_y = graph_of_far(x1);
		}

		else if(x1>=X_FAR){
			near_y = 0;
			medium_y = 0;
			far_y = PERFECT_FUZZY;
		}

		values[0] = near_y;
		values[1] = medium_y;
		values[2] = far_y;

		return values;
	}


	public static float graph_of_near(float x) {

		float y;

		y = (((0-1.0f)/(X_MEDIUM-X_NEAR))*(x-X_NEAR)) + 1;

		return y;
	}

	public static float graph_of_medium_near(float x) {

		float y;


		y = (((0-1.0f)/(X_NEAR-X_MEDIUM))*(x-X_MEDIUM)) + 1;

		return y;
	}


	public static float graph_of_medium_far(float x) {

		float y;


		y = (((0-1.0f)/(X_FAR-X_MEDIUM))*(x-X_MEDIUM)) + 1;

		return y;
	}


	public static float graph_of_far(float x) {

		float y;

		float a,b,c,d,e;

		a = 0-1.0f;
		b = X_MEDIUM-X_FAR;
		c = x-X_FAR;
		d = a/b;
		e=d*c;

		y = d + 1;

		return y;
	}

	public static float minimum(float a, float b){
		if(a<b)
			return a;
		else 
			return b;
	}
}
