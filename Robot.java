package org.usfirst.frc.team3826.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;

//import org.opencv.core.Rect;
//import org.opencv.imgproc.Imgproc;

//import com.ctre.CANTalon;
//import com.ctre.CANTalon.TalonControlMode;
//import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
//import edu.wpi.first.wpilibj.RobotDrive;//
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
//import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
//                                2018 Bot Code// v1.2.2
/**
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	SRF_Drive robot;
	Joystick xBox;
	
	//MotorType kFrontLeft;
	Talon frontLeft = new Talon(2);
	Talon rearLeft = new Talon(3); 
	Talon frontRight = new Talon(0); 
	Talon rearRight = new Talon(1);
	
	AHRS ahrs;
	double angleNow, scaledAngle; //the current angle and the angle to feed into the PID
	
	Timer t = new Timer();
	
	boolean start; //Whether or not we just started a case
	int autoID;
	String autoCase; //the name of the current autonomous step
	int autoStep;//the variable that tracks how far into autonomous the robot is
	
	String gameData; //variable for retrieving feild setup
	char switchSide; //side that is ours of the switch
	char scaleSide; //side that is ours of the scale
	char ourSide; //the side that our robot is on on the field
	
	String[][] auto = new String[13][8]; //first number is autoID
										//second number is place in list
	
	private SendableChooser placeCube; //Do we place a cube?
	private SendableChooser startingLocation; //Where do we start?
	private SendableChooser priorityType; //Decide based on side or scale/switch
	private SendableChooser priorityPlace; //Decide based on scale or switch
	private SendableChooser testMode; //Are we in testing mode
	private SendableChooser testCase; //What case are we testing
	
	SRF_PID positionPID;
	SRF_PID turningPID;
	
	Encoder rightSide;
	
	double countsPerInch = 54.3249;
	double output;
	double turnSetpoint;
	
    public void robotInit() {
    	robot = new SRF_Drive();
    	xBox = new Joystick(0);
    	
    	//initialize choosers
    	placeCube = new SendableChooser();
    	placeCube.addDefault("Null", 0);
    	placeCube.addObject("Place Cube", 1);
    	placeCube.addObject("Drive", 2);
    	
    	startingLocation = new SendableChooser();
    	startingLocation.addDefault("Null", 0);
    	startingLocation.addObject("Left", 1);
    	startingLocation.addObject("Middle", 2);
    	startingLocation.addObject("Right", 3);
    	
    	priorityType = new SendableChooser();
    	priorityType.addDefault("Null", 0);
    	priorityType.addObject("Closest Side", 1);
    	priorityType.addObject("Scale/Switch", 2);
    	
    	priorityPlace = new SendableChooser();
    	priorityPlace.addDefault("Null", 0);
    	priorityPlace.addObject("Switch", 1);
    	priorityPlace.addObject("Scale", 2);
    	
    	testMode = new SendableChooser();
    	testMode.addDefault("Null", 0);
    	testMode.addObject("Standard Mode", 1);
    	testMode.addObject("Testing Mode", 2);
    	
    	testCase = new SendableChooser();
    	testCase.addDefault("Null", 0);
    	testCase.addObject("Basic Drive", 5);
    	testCase.addObject("Initial Drive", 6);
    	testCase.addObject("Near Switch Place", 7);
    	testCase.addObject("Drive Farther", 8);
    	testCase.addObject("Near Scale Place", 9);
    	testCase.addObject("Cross Field", 10);
    	testCase.addObject("Switch Place", 11);
    	testCase.addObject("Scale Place", 12);
    	
    	
    	positionPID = new SRF_PID();
    	positionPID.setPID(0.002, 0.002, 0.002);
    	
    	turningPID = new SRF_PID();
    	turningPID.setPID(0.02, 0.02, 0.2);
    	
    	rightSide = new Encoder(0, 1);
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	//get switch/scale orientation
    	gameData = DriverStation.getInstance().getGameSpecificMessage();
    	switchSide = gameData.charAt(0);
    	scaleSide = gameData.charAt(1);
    	
    	start = true;//initialize variables
    	autoStep = -1;
    	
	robot.initDrive();
	    
    	//initialize potential autonomous steps
    	auto[0][0] = "Basic Drive";//drive forward and do nothing
    	auto[0][1] = "Done";
    	
    	auto[1][0] = "Initial Drive";//place cube on nearest switch platform
    	auto[1][1] = "Turn";
    	auto[1][2] = "Switch Approach";
    	auto[1][3] = "Switch Place";
    	auto[1][4] = "Done";
    	
    	auto[2][0] = "Drive Farther";//place cube on far switch platform
    	auto[2][1] = "Cross Field";
    	auto[2][2] = "Turn"; //adjust
    	auto[2][3] = "Turn";
    	auto[2][4] = "Switch Approach";
    	auto[2][5] = "Done";
    	
    	auto[3][0] = "Near Scale Initial";//place cube on near scale platform
    	auto[3][1] = "Turn";
    	auto[3][2] = "Scale Approach";
    	auto[3][3] = "Scale Place";
    	auto[3][4] = "Done";
    	
    	auto[4][0] = "Drive Farther"; //place cube on far scale platform
    	auto[4][1] = "Turn";
    	auto[4][2] = "Cross Field";
    	auto[4][3] = "Turn Opposite"; //adjust
    	auto[4][4] = "Turn Opposite";
    	auto[4][5] = "Scale Approach";
    	auto[4][6] = "Scale Place";
    	auto[4][7] = "Done";
    	
    	//testing sequences
    	auto[5][0] = "Basic Drive";
    	auto[5][1] = "Done";
    	
    	auto[6][0] = "Initial Drive";
    	auto[6][1] = "Done";
    	
    	auto[7][0] = "Turn";
    	auto[7][1] = "Switch Approach";
    	auto[7][2] = "Switch Place";
    	auto[7][3] = "Done";
    	
    	auto[8][0] = "Drive Farther";
    	auto[8][1] = "Done";
    	
    	auto[9][0] = "Near Scale Initial";
    	auto[9][1] = "Turn";
    	auto[9][2] = "Scale Approach";
    	auto[9][3] = "Scale Place";
    	auto[9][4] = "Done";
    	
    	auto[10][0] = "Cross Field";
    	auto[10][1] = "Done";
    	
    	auto[11][0] = "Switch Place";
    	auto[11][1] = "Done";
    	
    	auto[12][0] = "Scale Place";
    	auto[12][1] = "Done";
	    
    	
    	//initialize our Robot's location on the field
    	if((int) startingLocation.getSelected() == 1)
    		ourSide = 'L';
    	else if((int) startingLocation.getSelected() == 3)
    		ourSide = 'R';
    	else
    		ourSide = 'M';
    	
    	
    	//initiate autoID based on sendable chooser and field
    	if((int) testMode.getSelected() == 2)
    		autoID = (int) testCase.getSelected();
    	else if((int) placeCube.getSelected() == 2) //if we aren't planning on placing
    		autoID = 0;
    	else
    	{
    		//initialize turning direction multiplier
    		
    		if((int) priorityType.getSelected() == 1)//if looking for this side
    		{
    			if(switchSide != scaleSide)//if only one plate is on our side
    			{
    				if(switchSide == ourSide)//go for the one on our side
    					autoID = 1;
    				else
    					autoID = 3;
    			}
    			else //if the plates share a side
    			{
    				if((int) priorityPlace.getSelected() == 2)//if going to 
    				{										  //scale
    					if(scaleSide == ourSide)//if the scale is on our side
    						autoID = 3;
    					else
    						autoID = 4;
    				}
    				else//if gooing to the switch (default mode)
    				{
    					if(switchSide == ourSide)//if the switch is on our side
    						autoID = 1;
    					else
    						autoID = 2;
    				}
    			}
    		}
    		else//if looking based on switch or scale
    		{
    			if((int) priorityPlace.getSelected() == 2)//if going to scale
    			{
    				if(scaleSide == ourSide)//if the scale is on our side
    					autoID = 3;
    				else
    					autoID = 4;
    			}
    			else//if going to switch
    			{
    				if(switchSide == ourSide)//if the switch is on our side
    					autoID = 1;
    				else
    					autoID = 2;
    			}
    		}
    	}
    }
    
    /**
     * This function is called periodically during autonomous
     */
	public void autonomousPeriodic() {
		
		frontLeft.set(robot.getOutput(0));
		rearLeft.set(robot.getOutput(0));
		frontRight.set(robot.getOutput(1));
		rearRight.set(robot.getOutput(1));
		
		if(start)//if this is the first time in the match that this autostep
		{ 		 //is called
			//Reset sensors
			t.reset();
			//reset encoders
			
			//Determine next autonomous action
			autoStep++;
			autoCase = auto[autoID][autoStep];
			
			if(autoCase == "Turn")
				turnSetpoint+=(90/**turningMult*/);
			else if(autoCase == "Turn Opposite")
				turnSetpoint-=(90/**turningMult*/);
			
			//Disable initialization
			start = false;
		}
		
		
		switch(autoCase)
		{
			case "Basic Drive": //Just drive forward
				
				//UNTESTED
				robot.computeArcade(0.5, 0);
				
				if(t.get() > 1);
				{
					robot.computeArcade(0, 0);
					start = true;
				}
				
				break;
			case "Initial Drive": //Drive forward on the side and get ready to place
				
				//UNTESTED
				positionPID.setSetpoint(151.5*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(0/*encoder*/,t.get());
				//INCOMPLETE ^
				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				
				if(output < 0.05)
					start = true;
				
				break;
			case "Turn": //Turn 90 degrees
				//UNTESTED
				output = turningPID.computePID(ahrs.getAngle(),t.get());
				robot.computeArcade(0, output);
				
				if(output < 0.05)
					start = true;
				
				break;
			case "Turn Opposite": //Turn 90 degrees in the other direction
				//UNTESTED
				output = turningPID.computePID(ahrs.getAngle(),t.get());
				robot.computeArcade(0, output);
				
				if(output < 0.05)
					start = true;
				
				break;
			case "Switch Approach"://approach the switch
				//UNTESTED
				positionPID.setSetpoint(21.81*countsPerInch);
				output = positionPID.coputePID(0/*encoder stuff*/, t.get());
				
				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(),t.get()));
				
				if(output < 0.05)
					start = true;
				break;
			case "Switch Place": //Turn and place cube
				//UNTESTED
				robot.computeArcade(0,0);
				//place
				
				//when done
				break;
			case "Drive Farther": //Initial Drive plus some
				//UNTESTED
				positionPID.setSetpoint(226.72*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(0/*encoder*/,t.get());
				
				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				//INCOMPLETE need encoders ^
				if(output < 0.05)
					start = true;
				break;
			case "Near Scale Initial": //Travel Farther plus drive
				//UNTESTED
				positionPID.setSetpoint(304.25*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(0/*encoder*/,t.get());
				
				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				//INCOMPLETE need encoders ^
				if(output < 0.05)
					start = true;
				break;
			case "Scale Place"://place a cube on the scale
				break;
			case "Cross Field"://Travel across the field
				///UNTESTED
				positionPID.setSetpoint(295.68*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(0/*encoder*/,t.get());
				
				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				//INCOMPLETE need encoders ^
				if(output < 0.05)
					start = true;
				break;
			case "Far Switch Place": //Place the cube on the far switch
				//code to do that
				break;
			case "Scale Approach": //Adjust position relative to near scale plate
				//UNTESTED
				positionPID.setSetpoint(4.82*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(0/*encoder*/,t.get());
				
				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				//INCOMPLETE need encoders ^
				if(output < 0.05)
					start = true;
				
				break;
			case "Done": //terminate auto
				
				//set all motors to not move
				robot.computeArcade(0,0);
				
				break;
		}
	}
    
    /**
     * This function is called once each time the robot enters teleoperated mode
     */
    public void teleopInit(){
    	rightSide.reset();
    }

    /**
     * This function is called periodically during operator control
     */
    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj.IterativeRobot#teleopPeriodic()
     */
    public void teleopPeriodic() {
    	if(Math.abs(xBox.getRawAxis(1)) > 0.2 || Math.abs(xBox.getRawAxis(0)) > 0.2)
    	{
    		robot.computeArcade(-xBox.getRawAxis(1), -xBox.getRawAxis(0));
    	}
    	else
    	{
    		robot.computeArcade(0, 0);
    	}
    	
    	SmartDashboard.putNumber("rightSide", rightSide.get());
    }
    
    public void testInit()
    {
    	
    }
    
    /**
     * This function is called periodically during test mode
     */
    
    public void testPeriodic() {
    
   
    
    }
}
