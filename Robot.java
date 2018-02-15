package org.usfirst.frc.team3826.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;


import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//                                2018 Bot Code// v1.3.1a

/**
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

@SuppressWarnings("deprecation")

public class Robot extends IterativeRobot {

    RobotDrive robot;
	Joystick xBox;

	Talon frontLeft;
	Talon rearLeft; 
	Talon frontRight; 
	Talon rearRight;

	Victor intakeL;
	Victor intakeR;
	
	Victor hook;
	Victor winch;
	
	TalonSRX elevator;
	TalonSRX wrist;
	
	AHRS ahrs;

	double angleNow, scaledAngle; //the current angle and the angle to feed into the PID

	Timer t;

	boolean start; //Whether or not we just started a case

	int autoID;

// String autoCase; //the name of the current autonomous step


	public enum autoStates { 	Done,
						BasicDrive, DriveFarther,  							
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
	double turningMult;    //the variable used to mirror our operations depending on robot placement
	
	int targetWristPos; //variable for where the wrist will move to
	int targetElevatorPos; //variable for where the elevator will move to
	
	//the variables for the constants in each PID loop
	double upElevatorP = 0.0058, upElevatorI = 0, upElevatorD = 0.001;
	double downElevatorP = 0.0035, downElevatorI = 0, downElevatorD  = 0.003;
	double upWristP = 0.005, upWristI = 0.00000175, upWristD = 0.001;
	double downWristP = 0.003, downWristI = 0.000000015, downWristD = 0.001;
	
	public void robotInit() { 
		
		hook = new Victor(4);
		winch = new Victor(7);
		
		
		elevator = new TalonSRX(2);
		elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		elevator.config_kP(0, downElevatorP, 10);//first argument is slotIdx
		elevator.config_kI(0, downElevatorI, 10);
		elevator.config_kD(0, downElevatorD, 10);
		
		
		wrist = new TalonSRX(1);
		wrist.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		wrist.config_kP(0, upWristP, 10);//first argument is slotIdx
		wrist.config_kI(0, upWristI, 10);
		wrist.config_kD(0, upWristD, 10);
		
	
		intakeL = new Victor(5);
		intakeR = new Victor(6);
		
    	frontLeft = new Talon(2);
    	rearLeft = new Talon(3); 
    	frontRight = new Talon(0); 
    	rearRight = new Talon(1);
    	
    	robot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
    	xBox = new Joystick(0);

    	ahrs = new AHRS(SPI.Port.kMXP);
    	
    	t = new Timer();
    	t.start();

    	//initialize choosers
    	placeCube = new SendableChooser();
    	placeCube.addDefault("Place Cube", 1);
    	placeCube.addObject("Drive", 2);
    	SmartDashboard.putData("placeCube", placeCube);

    	startingLocation = new SendableChooser();
    	startingLocation.addDefault("Left", 1);
    	startingLocation.addObject("Middle", 2);
    	startingLocation.addObject("Right", 3);
    	SmartDashboard.putData("startingLocation", startingLocation);
    	
    	priorityType = new SendableChooser();
    	priorityType.addDefault("Closest Side", 1);
    	priorityType.addObject("Scale/Switch", 2);
    	SmartDashboard.putData("priorityType", priorityType);

    	priorityPlace = new SendableChooser();
    	priorityPlace.addDefault("Switch", 1);
    	priorityPlace.addObject("Scale", 2);
    	SmartDashboard.putData("priorityPlace", priorityPlace);

    	testMode = new SendableChooser();
    	testMode.addDefault("Standard Mode", 1);
    	testMode.addObject("Testing Mode", 2);
    	SmartDashboard.putData("testMode", testMode);

    	testCase = new SendableChooser();				
    	testCase.addDefault("Basic Drive", 5);
    	testCase.addObject("Initial Drive", 6);
    	testCase.addObject("Near Switch Place", 7);
    	testCase.addObject("Drive Farther", 8);
    	testCase.addObject("Near Scale Place", 9);
    	testCase.addObject("Cross Field", 10);
    	testCase.addObject("Switch Place", 11);
    	testCase.addObject("Scale Place", 12);
    	SmartDashboard.putData("testCase", testCase);
    	
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
    	System.out.println("Starting Init");
    	//get switch/scale orientation
    	gameData = DriverStation.getInstance().getGameSpecificMessage();

    	try
    	{
    		switchSide = gameData.charAt(0);
    	}
    	catch(Exception ex)
    	{
    		System.out.println("Exception initializing:" + ex);
    	}

    	scaleSide = gameData.charAt(1);

    	start = true;   //initialize variables
    	autoStep = -1;

    	//initialize potential autonomous steps
    	auto[0][0] = autoStates.BasicDrive; //drive forward over the auto line
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
		auto[5][2] = autoStates.NA;
		auto[5][3] = autoStates.NA;
		auto[5][4] = autoStates.NA;
		auto[5][5] = autoStates.NA;
		auto[5][6] = autoStates.NA;
		auto[5][7] = autoStates.NA;
		
    	auto[6][0] = autoStates.InitialDrive; //Does nothing + error no robot code running
    	auto[6][1] = autoStates.Done;
		auto[6][2] = autoStates.NA;
		auto[6][3] = autoStates.NA;
		auto[6][4] = autoStates.NA;
		auto[6][5] = autoStates.NA;
		auto[6][6] = autoStates.NA;
		auto[6][7] = autoStates.NA;
		
    	auto[7][0] = autoStates.Turn; //The state "NA" has been reached
    	auto[7][1] = autoStates.SwitchApproach;
    	auto[7][2] = autoStates.SwitchPlace;
    	auto[7][3] = autoStates.Done;
		auto[7][2] = autoStates.NA;
		auto[7][3] = autoStates.NA;
		auto[7][4] = autoStates.NA;
		auto[7][5] = autoStates.NA;
		auto[7][6] = autoStates.NA;
		auto[7][7] = autoStates.NA;
		
    	auto[8][0] = autoStates.DriveFarther; //Does nothing
    	auto[8][1] = autoStates.Done;
		auto[8][2] = autoStates.NA;
		auto[8][3] = autoStates.NA;
		auto[8][4] = autoStates.NA;
		auto[8][5] = autoStates.NA;
		auto[8][6] = autoStates.NA;
		auto[8][7] = autoStates.NA;
		
    	auto[9][0] = autoStates.NearScaleInitial;//does nothing until scale place
    	auto[9][1] = autoStates.Turn;
    	auto[9][2] = autoStates.ScaleApproach;
    	auto[9][3] = autoStates.ScalePlace; //RobotDrive Output not updated often enough
    	auto[9][4] = autoStates.Done;
		auto[9][5] = autoStates.NA;
		auto[9][6] = autoStates.NA;
		auto[9][7] = autoStates.NA;
		
    	auto[10][0] = autoStates.CrossField; //does nothing
    	auto[10][1] = autoStates.Done;
		auto[10][2] = autoStates.NA;
		auto[10][3] = autoStates.NA;
		auto[10][4] = autoStates.NA;
		auto[10][5] = autoStates.NA;
		auto[10][6] = autoStates.NA;
		auto[10][7] = autoStates.NA;
		
    	auto[11][0] = autoStates.SwitchPlace; //gets into SwitchPlace and stops
    	auto[11][1] = autoStates.Done; 		  //updating
		auto[11][2] = autoStates.NA;
		auto[11][3] = autoStates.NA;
		auto[11][4] = autoStates.NA;
		auto[11][5] = autoStates.NA;
		auto[11][6] = autoStates.NA;
		auto[11][7] = autoStates.NA;
		
    	auto[12][0] = autoStates.ScalePlace; //RD output not updated often enough
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
    	
    	if(ourSide == 'R')
    		turningMult = -1;
    	else
    		turningMult = 1;
    	
    	rightSide.reset();
    	
    	wrist.setSelectedSensorPosition(0, 0, 0);
    	elevator.setSelectedSensorPosition(0, 0, 0);
    	targetWristPos = 0;
    	targetElevatorPos = 0;
    	
    	System.out.println("init end");
    }

    /**
     * This function is called periodically during autonomous
     */

	public void autonomousPeriodic()
	{
		SmartDashboard.putNumber("rightSide", rightSide.get());
		
		if(start)//just starting into this loop
		{ 		 
			//Reset sensors
			System.out.println("Start begin");
			t.reset();
			rightSide.reset();
			ahrs.reset();
			
			//Determine next autonomous action
			autoStep++;
			autoCase = auto[autoID][autoStep];

			if(autoCase == autoStates.Turn)
				turnSetpoint+=(90*turningMult);
			else if(autoCase == autoStates.TurnOpposite)
				turnSetpoint-=(90*turningMult);

			//Disable initialization
			start = false;
			System.out.println(autoCase);
		}

		switch(autoCase)				//create a separate 
										// staging monitor or "executive", and when any autoID is "Done", don't quit before calling that
										// Executive so it can decide if a new autoID task should be started. That executive would 
										// consider various factors: time left in auto? Do we control our switch? Do we control our Scale? 
										// Is this a qual or playoff match? It might then set autoID appropriately, store whatever states or
										// flags that it needs to, and return. This switch/case then continues to serve, sequencing
										// though the steps of the specific autoID that the executive decided to try for.
		{
			case BasicDrive: //Just drive forward
				robot.arcadeDrive(-0.5,0);
				if(t.get() > 2)
				{
					robot.arcadeDrive(0,0);
					start = true;
					System.out.println("End BD");
				}
				break;
			case InitialDrive: //Drive forward on the side and get ready to place
				System.out.println("480:"+rightSide.get());//UNTESTED
				positionPID.setSetpoint(151.5*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(rightSide.get(),t.get());
				System.out.println("Output Computed:"+output);
				robot.arcadeDrive(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				
				if(output < 0.05 && t.get() > 0.5)//add turning output
					start = true;
				break;
			case Turn: //Turn 90 degrees
				//UNTESTED//
				System.out.println("Beginning");
				output = turningPID.computePID(ahrs.getAngle(),t.get());
				System.out.println("PID computed");
				robot.arcadeDrive(0, output);
				System.out.println("Arcade Called");
				if(output < 0.05)
					start = true;
				System.out.println("Done");
				break;
			case TurnOpposite: //Turn 90 degrees in the other direction
				//UNTESTED
				output = turningPID.computePID(ahrs.getAngle(),t.get());
				 robot.arcadeDrive(0, output);
				if(output < 0.05)
					start = true;
				break;
			case SwitchApproach: //approach the switch
				//UNTESTED
				positionPID.setSetpoint(21.81*countsPerInch);
				output = positionPID.computePID(rightSide.get(), t.get());
				robot.arcadeDrive(output, turningPID.computePID(ahrs.getAngle(),t.get()));
				if(output < 0.05)
					start = true;
				break;
			case SwitchPlace: //Turn and place cube
				//UNTESTED
				robot.arcadeDrive(0,0);
				//place
				//when done
				break;
			case DriveFarther: //Initial Drive plus some
				//UNTESTED
				positionPID.setSetpoint(226.72*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(rightSide.get(),t.get());

				robot.arcadeDrive(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				if(output < 0.05)
					start = true;
				break;
			case NearScaleInitial: //Travel Farther plus drive
				//UNTESTED
				positionPID.setSetpoint(304.25*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(rightSide.get(),t.get());

				robot.arcadeDrive(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				if(output < 0.05)
					start = true;
				
				break;
			case ScalePlace: //place a cube on the scale
				break;
			case CrossField: //Travel across the field
				///UNTESTED
				positionPID.setSetpoint(295.68*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(rightSide.get(),t.get());

				robot.arcadeDrive(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				if(output < 0.05)
					start = true;
				break;
			case FarSwitchPlace: //Place the cube on the far switch  (PV) this was not a defined case option! Added it to the new enum
				//code to do that
				break;
			case ScaleApproach: //Adjust position relative to near scale plate
				//UNTESTED
				positionPID.setSetpoint(4.82*countsPerInch);
				turningPID.setSetpoint(turnSetpoint);
				output = positionPID.computePID(rightSide.get(),t.get());
	
				robot.arcadeDrive(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				if(output < 0.05)
					start = true;
				break;
			case Done: //terminate auto
				robot.arcadeDrive(0,0);
				break;
			case NA: //the case after Done
				robot.arcadeDrive(0,0);
				System.out.println("The state \"NA\" has been reached");
				break;
		}
		
		
		SmartDashboard.putNumber("t", t.get());
	}

    
    /**
     * This function is called once each time the robot enters teleoperated mode
     */

    public void teleopInit()
    {
    	rightSide.reset();
    	wrist.setSelectedSensorPosition(0, 0, 0);
    	elevator.setSelectedSensorPosition(0, 0, 0);
    	targetWristPos = 0;
    	targetElevatorPos = 0;
		wrist.set(ControlMode.Position, 0);
		ahrs.reset();
    }


    /**
     * This function is called periodically during operator control
     */

    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj.IterativeRobot#teleopPeriodic()
     */

    public void teleopPeriodic()
    {
    	
    	if(Math.abs(xBox.getRawAxis(1)) > 0.2 || Math.abs(xBox.getRawAxis(0)) > 0.2)
    		 robot.arcadeDrive(xBox.getRawAxis(1), -xBox.getRawAxis(0));
    	else
    		 robot.arcadeDrive(0,0);
    	
    	
    	//cube intake
    	if(xBox.getRawAxis(2) > 0.2)//intake
    	{
    		intakeL.set(-.5);
    		intakeR.set(-.5);
    	}
    	else if(xBox.getRawAxis(3) > 0.2)//eject
    	{
    		intakeL.set(.5);
    		intakeR.set(.5);
    	}
   /* 	else if(Math.abs(targetWristPos - wrist.getSelectedSensorPosition(0)) > 2000)
    	{maybe
    		intakeL.set(0.2);
    		intakeR.set(0.2);
    	}
    */	else
    	{
    		intakeL.set(0);
    		intakeR.set(0);
    	}
    	
    	
    	//test wrist code//
    	if(xBox.getRawButton(8))
    	{
    		wrist.config_kP(0, upWristP, 10);//first argument is slotIdx
    		wrist.config_kI(0, upWristI, 10);
    		wrist.config_kD(0, upWristD, 10);
    		targetWristPos = 0;
    	}
    	else if(xBox.getRawButton(7))
    	{
    		wrist.config_kP(1, downWristP, 10);//first argument is slotIdx
    		wrist.config_kI(1, downWristI, 10);
    		wrist.config_kD(1, downWristD, 10);
    		targetWristPos = -106500;
    	}
    	
    	wrist.set(ControlMode.Position, targetWristPos);
    	
    	
    	//Elevator code
    	if(xBox.getRawButton(4))//Y
    	{
    		elevator.config_kP(0, upElevatorP, 10);//first argument is slotIdx
    		elevator.config_kI(0, upElevatorI, 10);
    		elevator.config_kD(0, upElevatorD, 10);
   			targetElevatorPos = -1000000;//go to top (this is a test value use:
    	}
   		else if(xBox.getRawButton(1))//A      -1500000)
   		{
   			elevator.config_kP(0, downElevatorP, 10);//first argument is slotIdx
   			elevator.config_kI(0, downElevatorI, 10);
   			elevator.config_kD(0, downElevatorD, 10);
   			targetElevatorPos = -20000; //go to bottom
   		}
   			
    	elevator.set(ControlMode.Position, targetElevatorPos);
    	
    	
    	if(xBox.getRawButton(2))//B
    		elevator.setSelectedSensorPosition(0, 0, 0);
    	
    	
    	
    	//hook code
    /*	if(Math.abs(xBox.getRawAxis(5)) > 0.2) //right stick y-axis
    		hook.set(xBox.getRawAxis(5));
    	else
    		hook.set(0);
    	*/
    	
    	   	//winch code
    	if(xBox.getRawButton(5)) //LB - wind out
    		winch.set(0.5);
    	else if(xBox.getRawButton(6)) //RB - wind in
    		winch.set(-0.5);
    	else
    		winch.set(0);
    		
    	SmartDashboard.putNumber("Elevator Position", elevator.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("Wrist Position", wrist.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("rightSide", rightSide.get());
    	SmartDashboard.putNumber("targetWristPos", targetWristPos);
    	SmartDashboard.putNumber("targetElevatorPos", targetElevatorPos);
    	SmartDashboard.putNumber("ahrs", ahrs.getAngle());
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

}
