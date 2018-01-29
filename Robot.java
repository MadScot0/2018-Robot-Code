package org.usfirst.frc.team3826.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;


import edu.wpi.first.wpilibj.Talon;
//import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Victor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//                                2018 Bot Code// v1.2.3a

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
	Talon frontLeft;
	Talon rearLeft; 
	Talon frontRight; 
	Talon rearRight;

	AHRS ahrs;

	double angleNow, scaledAngle; //the current angle and the angle to feed into the PID

	Timer t;

	boolean start; //Whether or not we just started a case

	int autoID;

// String autoCase; //the name of the current autonomous step

	public enum autoStates { 	Done,
						BasicDrive, DriveFarther,  							// Change "BasicDrive" to "DriveForward" or just "Drive"?
						Turn, TurnOpposite, CrossField,
						InitialDrive, SwitchApproach, SwitchPlace, FarSwitchPlace,	// FarSwitchInitial needed?
						NearScaleInitial, ScaleApproach, ScalePlace,		// FarScaleInitial and FarScalePlace needed?
																			// Add steps here needed to pick up a cube from the floor (locate, 
																			// acquire, and stow it for driving) to allow for Stage 3 auto tasks; 
																			// will probably also need an additional autoID since getting a cube
																			// is a separate multi step process. 
						NA };												// Also add NA, use to fill unused steps in your fixed auto array
																			// NA steps allow a check for whether autoStep gets out of step!
																			
																			// May also want to consider "auto" actions in teleop, especially 
																			// locating and acquiring a cube you cannot see. Also perhaps 
																			// placing a cube on a switch or scale plate automatically would 
																			// be faster then the driver could do it. Note: always allow for 
																			// abort on any teleop automatic actions!
																			
																			
	int autoStep;//the variable that tracks how far into autonomous the robot is

	String gameData; //variable for retrieving field setup

	char switchSide; //side that is ours of the switch
	char scaleSide;  //side that is ours of the scale
	char ourSide;    //the side that our robot is on on the field

										 

	autoStates	auto[][] = new autoStates[13][8];			// first index is autoID, second index is place in list (autoStep)
	autoStates autoCase;
										//second number is place in list, autoStep
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

    	frontLeft = new Talon(2);
    	rearLeft = new Talon(3); 
    	frontRight = new Talon(0); 
    	rearRight = new Talon(1);

    	t = new Timer();

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

    	try{
    		switchSide = gameData.charAt(0);
    	}
    	catch(Exception ex)
    	{
    		System.out.println("Exception initializing:" + ex);
    	}

    	scaleSide = gameData.charAt(1);

    	start = true;   //initialize variables
    	autoStep = -1;

    	robot.initDrive();

    	//initialize potential autonomous steps
    	auto[0][0] = autoStates.BasicDrive;	//drive forward and do nothing
    	auto[0][1] = autoStates.Done;
		auto[0][2] = autoStates.NA;
		auto[0][3] = autoStates.NA;
		auto[0][4] = autoStates.NA;
		auto[0][5] = autoStates.NA;
		auto[0][6] = autoStates.NA;
		auto[0][7] = autoStates.NA;
		
    	auto[1][0] = autoStates.InitialDrive;	//place cube on nearest switch platform
    	auto[1][1] = autoStates.Turn;
    	auto[1][2] = autoStates.SwitchApproach;
    	auto[1][3] = autoStates.SwitchPlace;
    	auto[1][4] = autoStates.Done;
		auto[1][5] = autoStates.NA;
		auto[1][6] = autoStates.NA;
		auto[1][7] = autoStates.NA;
		
    	auto[2][0] = autoStates.DriveFarther; //place cube on far switch platform
    	auto[2][1] = autoStates.CrossField;
    	auto[2][2] = autoStates.Turn; //adjust
    	auto[2][3] = autoStates.Turn;
    	auto[2][4] = autoStates.SwitchApproach;
    	auto[2][5] = autoStates.Done;
		auto[2][6] = autoStates.NA;
		auto[2][7] = autoStates.NA;
		
    	auto[3][0] = autoStates.NearScaleInitial;//place cube on near scale platform
    	auto[3][1] = autoStates.Turn;
    	auto[3][2] = autoStates.ScaleApproach;
    	auto[3][3] = autoStates.ScalePlace;
    	auto[3][4] = autoStates.Done;
		auto[3][5] = autoStates.NA;
		auto[3][6] = autoStates.NA;
		auto[3][7] = autoStates.NA;
		
    	auto[4][0] = autoStates.DriveFarther; //place cube on far scale platform
    	auto[4][1] = autoStates.Turn;
    	auto[4][2] = autoStates.CrossField;
    	auto[4][3] = autoStates.TurnOpposite; //adjust
    	auto[4][4] = autoStates.TurnOpposite;
    	auto[4][5] = autoStates.ScaleApproach;
    	auto[4][6] = autoStates.ScalePlace;
    	auto[4][7] = autoStates.Done;

    	//testing sequences
    	auto[5][0] = autoStates.BasicDrive;
    	auto[5][1] = autoStates.Done;
		auto[0][2] = autoStates.NA;
		auto[0][3] = autoStates.NA;
		auto[0][4] = autoStates.NA;
		auto[0][5] = autoStates.NA;
		auto[0][6] = autoStates.NA;
		auto[0][7] = autoStates.NA;
		
    	auto[6][0] = autoStates.InitialDrive;
    	auto[6][1] = autoStates.Done;
		auto[0][2] = autoStates.NA;
		auto[0][3] = autoStates.NA;
		auto[0][4] = autoStates.NA;
		auto[0][5] = autoStates.NA;
		auto[0][6] = autoStates.NA;
		auto[0][7] = autoStates.NA;
		
    	auto[7][0] = autoStates.Turn;
    	auto[7][1] = autoStates.SwitchApproach;
    	auto[7][2] = autoStates.SwitchPlace;
    	auto[7][3] = autoStates.Done;
		auto[7][2] = autoStates.NA;
		auto[7][3] = autoStates.NA;
		auto[7][4] = autoStates.NA;
		auto[7][5] = autoStates.NA;
		auto[7][6] = autoStates.NA;
		auto[7][7] = autoStates.NA;
		
    	auto[8][0] = autoStates.DriveFarther;
    	auto[8][1] = autoStates.Done;
		auto[8][2] = autoStates.NA;
		auto[8][3] = autoStates.NA;
		auto[8][4] = autoStates.NA;
		auto[8][5] = autoStates.NA;
		auto[8][6] = autoStates.NA;
		auto[8][7] = autoStates.NA;
		
    	auto[9][0] = autoStates.NearScaleInitial;
    	auto[9][1] = autoStates.Turn;
    	auto[9][2] = autoStates.ScaleApproach;
    	auto[9][3] = autoStates.ScalePlace;
    	auto[9][4] = autoStates.Done;
		auto[9][5] = autoStates.NA;
		auto[9][6] = autoStates.NA;
		auto[9][7] = autoStates.NA;
		
    	auto[10][0] = autoStates.CrossField;
    	auto[10][1] = autoStates.Done;
		auto[10][2] = autoStates.NA;
		auto[10][3] = autoStates.NA;
		auto[10][4] = autoStates.NA;
		auto[10][5] = autoStates.NA;
		auto[10][6] = autoStates.NA;
		auto[10][7] = autoStates.NA;
		
    	auto[11][0] = autoStates.SwitchPlace;
    	auto[11][1] = autoStates.Done;
		auto[11][2] = autoStates.NA;
		auto[11][3] = autoStates.NA;
		auto[11][4] = autoStates.NA;
		auto[11][5] = autoStates.NA;
		auto[11][6] = autoStates.NA;
		auto[11][7] = autoStates.NA;
		
    	auto[12][0] = autoStates.ScalePlace;
    	auto[12][1] = autoStates.Done;
		auto[12][2] = autoStates.NA;
		auto[12][3] = autoStates.NA;
		auto[12][4] = autoStates.NA;
		auto[12][5] = autoStates.NA;
		auto[12][6] = autoStates.NA;
		auto[12][7] = autoStates.NA;
	    
     	//initialize our Robot's location on the field
    	if((int) startingLocation.getSelected() == 1)
    		ourSide = 'L';
    	else if((int) startingLocation.getSelected() == 3)
    		ourSide = 'R';
    	else
    		ourSide = 'M';

    	//initiate autoID based on sendable chooser and field
    	if((int) testMode.getSelected() == 2)				//if "Testing mode"
    		autoID = (int) testCase.getSelected();
    	else if((int) placeCube.getSelected() == 2) //if we aren't planning on placing
    		autoID = 0;
    	else
    	{
    		//initialize turning direction multiplier
    		if((int) priorityType.getSelected() == 1) //if looking for this side
    		{
    			if(switchSide != scaleSide) //if only one plate is on our side
    			{
    				if(switchSide == ourSide) // go for the switch on our side
    					autoID = 1;
    				else					 // go for the scale on our side
    					autoID = 3;
    			}
    			else // the plates share a side
    			{
    				if((int) priorityPlace.getSelected() == 2) //if going to scale
    				{
    					if(scaleSide == ourSide) //if the scale is on our side
    						autoID = 3;
    					else
    						autoID = 4;
    				}
    				else // going to the switch (default mode - override to scale via sendable chooser for playoffs)
    				{
    					if(switchSide == ourSide)	//if the switch is on our side
    						autoID = 1;
    					else
    						autoID = 2;
    				}
    			}
    		}
    		else//if looking based on switch or scale
    		{
    			if((int) priorityPlace.getSelected() == 2) // if going to scale
    			{
    				if(scaleSide == ourSide)//if the scale is on our side
    					autoID = 3;
    				else
    					autoID = 4;
    			}
    			else // going to switch 
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

			if(autoCase == autoStates.Turn)
				turnSetpoint+=(90/**turningMult*/);
			else if(autoCase == autoStates.TurnOpposite)
				turnSetpoint-=(90/**turningMult*/);

			//Disable initialization
			start = false;
		}

		switch(autoCase)						// or (better) create a separate 
										// staging monitor or "executive", and when any autoID is "Done", don't quit before calling that
										// Executive so it can decide if a new autoID task should be started. That executive would 
										// consider various factors: time left in auto? Do we control our switch? Do we control our Scale? 
										// Is this a qual or playoff match? It might then set autoID appropriately, store whatever states or
										// flags that it needs to, and return. This switch/case then continues to serve, sequencing
										// though the steps of the specific autoID that the executive decided to try for.
		{
			case BasicDrive: //Just drive forward
				//UNTESTED
				robot.computeArcade(0.5, 0);

				if(t.get() > 1);
				{
					robot.computeArcade(0, 0);
					start = true;
				}
				break;

			case InitialDrive: //Drive forward on the side and get ready to place
				//UNTESTED
				positionPID.setSetpoint(151.5*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(0/*encoder*/,t.get());
				//INCOMPLETE ^
				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(), t.get()));

				if(output < 0.05)
					start = true;
				break;

			case Turn: //Turn 90 degrees
				//UNTESTED
				output = turningPID.computePID(ahrs.getAngle(),t.get());
				robot.computeArcade(0, output);

				if(output < 0.05)
					start = true;
				break;

			case TurnOpposite: //Turn 90 degrees in the other direction
				//UNTESTED
				output = turningPID.computePID(ahrs.getAngle(),t.get());
				robot.computeArcade(0, output);

				if(output < 0.05)
					start = true;
				break;

			case SwitchApproach: //approach the switch
				//UNTESTED
				positionPID.setSetpoint(21.81*countsPerInch);
				output = positionPID.computePID(0/*encoder stuff*/, t.get());
				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(),t.get()));

				if(output < 0.05)
					start = true;
				break;

			case SwitchPlace: //Turn and place cube
				//UNTESTED
				robot.computeArcade(0,0);
				//place
				//when done
				break;

			case DriveFarther: //Initial Drive plus some
				//UNTESTED
				positionPID.setSetpoint(226.72*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(0/*encoder*/,t.get());

				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				//INCOMPLETE need encoders ^
				
				if(output < 0.05)
					start = true;
				break;

			case NearScaleInitial: //Travel Farther plus drive
				//UNTESTED
				positionPID.setSetpoint(304.25*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(0/*encoder*/,t.get());

				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				//INCOMPLETE need encoders ^

				if(output < 0.05)
					start = true;
				break;

			case ScalePlace: //place a cube on the scale
				break;

			case CrossField: //Travel across the field
				///UNTESTED
				positionPID.setSetpoint(295.68*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(0/*encoder*/,t.get());

				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				//INCOMPLETE need encoders ^

				if(output < 0.05)
					start = true;
				break;

			case FarSwitchPlace: //Place the cube on the far switch 
				//code to do that
				break;

			case ScaleApproach: //Adjust position relative to near scale plate
				//UNTESTED
				positionPID.setSetpoint(4.82*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(0/*encoder*/,t.get());
	
				robot.computeArcade(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				//INCOMPLETE need encoders ^

				if(output < 0.05)
					start = true;
				break;

			case Done: //terminate auto
				robot.computeArcade(0,0);  //set all motors to not move
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
