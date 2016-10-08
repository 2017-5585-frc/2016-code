package org.usfirst.frc.team5585.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick xbox, stick;
	AnalogInput ultrasonic;
	Talon talon0, talon1, talon5, talon4;
	Compressor compressor;
	DoubleSolenoid grabber;
	int autoLoopCounter;
	int distance;
	int rotation;
	boolean fireMode;
	CameraServer server;
	Encoder armRotation;
	Victor victor2, victor3, victorClaw;
	
	public Robot() { //start camera
        server = CameraServer.getInstance(); 
        server.setQuality(30);
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");
        
	}
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	//initialize variables
    	rotation = 0;
    	fireMode = false;
    	//angle = 1;
    	//constructors
    	xbox = new Joystick(1); 
    	stick = new Joystick(0);
    	ultrasonic = new AnalogInput(0);
    	compressor = new Compressor(0);
    	grabber = new DoubleSolenoid(1,0); //valve for grabber cylinder
    	armRotation = new Encoder(0, 1, true, Encoder.EncodingType.k4X); //gearmotor encoder
    	victor2 = new Victor(2); //top shooter motor
    	victor3 = new Victor(3); //bottom shooter motor
    	victorClaw = new Victor(6); //pg71 gearmotor, claw
    	talon0 = new Talon(0); //front-right drive
    	talon1 = new Talon(1); //front-left drive
    	talon4 = new Talon(4); //rear-right drive
    	talon5 = new Talon(5); //rear-left drive
    	myRobot = new RobotDrive(talon1,talon5,talon0,talon4);
    	myRobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    	myRobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
    	//bootup code
    	compressor.start();
    	rotation = armRotation.get();
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	autoLoopCounter++;
    	autoMode(autoLoopCounter);
    }
    public void autoMode(int loopCount) { // odds of success 1:50
    	if (loopCount <= 150) {
    		myRobot.drive(-0.5, 0.0); // drive forward for 3 sec
    	}
    	if (loopCount > 150 && loopCount <= 225) { //curve left for 1.5 sec
    		myRobot.drive(-0.5, -0.5);
    	}
    	if (loopCount > 225 && loopCount <= 300) { // turn toward low goal, 1.5 sec
    		myRobot.drive(-0.5, 0.5);
    	}
    	if (loopCount > 300 && loopCount <= 500) { //drive toward low goal 4 sec
    		myRobot.drive(-0.5, 0.0);
    	}
    	if (loopCount > 500 ) { // shoot on low goal @ 12 sec, 3 sec left to score goal
    		victor2.set(1.0);
    		victor3.set(0.7);
    		Timer.delay(2);
    		fire();
    		victor2.set(0.0);
    		victor3.set(0.0); // finish @ 14 sec, wait for teleop 
    	}
    }
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        myRobot.arcadeDrive(stick);
        if (stick.getRawButton(3) == true) {
        	printRange();
        }
        //prints range on xbox start button press
        if (xbox.getRawButton(8) == true) {
        	printRange();
        }
        //grabs / releases ball
        grabberControl();
        //lowers arm on xbox L shoulder button press
        if (xbox.getRawButton(5) == true) {
        	armControl(true);
        }
        //raises arm on xbox R shoulder button press
        if (xbox.getRawButton(6) == true) {
        	armControl(false);
        }
        // sets fire mode
        fireControl();
        //starts firing sequence if fire mode = true
        if (fireMode = true) {
        	FireOps();
        }
        	
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    // prints ultrasonic distance to smart dashboard
    public void printRange() {
    	 distance = ultrasonic.getAverageValue() / 120; //distance in feet
         SmartDashboard.putNumber("range", distance);
    }
    
    public void grabberState(boolean state) {
    	if (state == true) {
    		//closes grabber
    		grabber.set(DoubleSolenoid.Value.kForward);
    		}
    		else {
    		// opens grabber
    		grabber.set(DoubleSolenoid.Value.kReverse);	
    		}
    }
    	
    public void grabberControl() {
    	//closes grabber
    	if (xbox.getTrigger(GenericHID.Hand.kLeft) == true) {
        	grabberState(true);
        }
    	//opens grabber
        if (xbox.getTrigger(GenericHID.Hand.kRight) == true) {
        	grabberState(false);
        }
    }

    public void armControl(boolean state) {
    	// TODO adjust encoder settings
    	if (state == true) {
    		rotation = armRotation.get(); // get encoder rotation
    		while (rotation < 120) { //lower arm
    			victorClaw.set(0.5); // TODO adjust gearmotor speed
    			rotation = armRotation.get();
    		}
    		while (rotation > 125) { //if arm goes to far, bring it back
    			victorClaw.set(-0.2);
    			rotation = armRotation.get();
    		}
    		rotation = armRotation.get();
    		if (rotation >= 120 && rotation <= 125) { //stop arm if in correct position
    			victorClaw.set(0.0);
    		}
    		
    	}
    	if (state == false) {
    		rotation = armRotation.get();
    		while (rotation > 5) { // raise arm
    			victorClaw.set(-0.5); // TODO set motor speed
    			rotation = armRotation.get();
    		}
    		while (rotation < 0) { // lower arm to correct position if it goes to far
    			victorClaw.set(0.2);
    			rotation = armRotation.get();
    		}
    		if (rotation >= 0 && rotation <= 5) { // stop arm in correct position
    			victorClaw.set(0);
    		}
    		
    	}
    }
    public void fireControl() {
    	if (xbox.getRawButton(1) == true  && xbox.getRawButton(2) == true) { //xbox A + B
    		fireMode = true;
    		xbox.setRumble(Joystick.RumbleType.kLeftRumble, 1);
    		xbox.setRumble(Joystick.RumbleType.kLeftRumble, 0);
    	}
    	if (stick.getRawButton(12) == true) { //joystick button 12
    		fireMode = true;
    		xbox.setRumble(Joystick.RumbleType.kLeftRumble, 1);
    		xbox.setRumble(Joystick.RumbleType.kLeftRumble, 0);
    	}
    }
    
    public void FireOps() {
    	double topSpeed = 1.0;
    	//myRobot.drive(0.0, 0.0); //stops robot if not already at stop
    	victor2.set(1.0); // top motor
    	victor3.set(-1.0);// bottom motor
    	while (fireMode == true) {
    		if (xbox.getTrigger(GenericHID.Hand.kRight) == true) {
        		grabberState(false); //includes 1 second wait
    		}
    		if (xbox.getRawButton(3) == true  && xbox.getRawButton(4) == true) { // xbox X + Y
        		fireMode = false;
        		xbox.setRumble(Joystick.RumbleType.kLeftRumble, 1);
        		xbox.setRumble(Joystick.RumbleType.kLeftRumble, 0);
        		victor2.set(0.0);
        		victor3.set(0.0); // stop motors
        	}
    		if (xbox.getPOV() > 90 && xbox.getPOV() < 270) { // angle shot up by slowing top motor speed
    			topSpeed = topSpeed-0.1;
    		}
    		if (xbox.getPOV() < 90 && xbox.getPOV() > 270 && xbox.getPOV() != -1) {//angle shot back to normal
    			topSpeed = topSpeed+0.1;
    		}
    		if (topSpeed > 1.0) { // keep speed within allowable limits
    			topSpeed = 1.0;
    		}
    		if (topSpeed < 0.7) { // TODO adjust
    			topSpeed = 0.7;
    		}
    		victor2.set(topSpeed);
    	}
    }
    public void fire() {
    	grabberState(false); // open grabber to release ball
		Timer.delay(1.0); // wait 1 second for ball to clear shooter TODO may need adjustment
    }
}
