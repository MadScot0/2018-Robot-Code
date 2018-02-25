package org.usfirst.frc.team3826.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Relay;


import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//                                2018 Bot Code// v1.3.2a //improved auto
									//remember to remove encoder reset from teleopInit before
									//competition
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
	
	int counter = 0;
	
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

	DigitalInput wristMin;
	
	double angleNow, scaledAngle; //the current angle and the angle to feed into the PID

	Timer t;

	boolean start; //Whether or not we just started a case

	int autoID;
	double turnSetpoint;
	double positionSetpoint;

	
	double timeNow;
	boolean firstSample;


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

	autoStates	auto[][] = new autoStates[17][8];			// first index is autoID, second index is place in list (autoStep)
	autoStates autoCase;
	
	private SendableChooser placeCube; //Do we place a cube?
	private SendableChooser startingLocation; //Where do we start?
	private SendableChooser priorityType; //Decide based on side or scale/switch
	private SendableChooser priorityPlace; //Decide based on scale or switch
	private SendableChooser testMode; //Are we in testing mode
	private SendableChooser testCase; //What case are we testing

	SRF_PID positionPID;
	SRF_PID turningPID;

	Encoder leftSide;
										
	double countsPerInch = 18.96;//54.3249;
	double output;
	double turningMult;    //the variable used to mirror our operations depending on robot placement
	
	
	int targetWristPos; //variable for where the wrist will move to
	int targetElevatorPos; //variable for where the elevator will move to
	
	//the variables for the constants in each PID loop
	double upElevatorP = 0.0058, upElevatorI = 0, upElevatorD = 0.001;
	double downElevatorP = 0.0035, downElevatorI = 0, downElevatorD  = 0.003;
	
	double upWristP = 0.0048, upWristI = 0.000002219, upWristD = 0.00207;
	double upFromMidP = 0.00466, upFromMidI = 0.0000019, upFromMidD = 0.0025;
	double downWristP = 0.0023, downWristI = 0.0000000175, downWristD = 0.0016;
	double upToMidP = 0.0069, upToMidI = 0.0000033, upToMidD = 0.0015;
	double downToMidP = 0.0037, downToMidI = 0.00000001775, downToMidD = 0.00305;
	
	int elevatorUp = -1600000, elevatorMid = -475000, elevatorDown = -20000;
	int wristUp = 0, wristMiddle = -53250, wristDown = -106500;
	int wristMode;
	
	boolean manualCon = false; //Used to change whether using manual controls for teleop
	boolean letUp10;
	
	int up = 0, mid = 1, down = 2;
	
	public void robotInit() { 
		
		wristMin = new DigitalInput(2);
		
		hook = new Victor(4);
		winch = new Victor(7);
		
		
		elevator = new TalonSRX(2);
		elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		elevator.config_kP(0, upElevatorP, 10); 
		elevator.config_kI(0, upElevatorI, 10);
		elevator.config_kD(0, upElevatorD, 10);
		
		
		wrist = new TalonSRX(1);
		wrist.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		wrist.config_kP(0, downWristP, 10); 
		wrist.config_kI(0, downWristI, 10);
		wrist.config_kD(0, downWristD, 10);
		
	
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

    	/*UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);
        camera.setExposureManual(45);
        camera.setFPS(30);*/
    	
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
    	testCase.addObject("Initial Drive", 6);//
    	testCase.addObject("Turn", 7);
    	testCase.addObject("Turn Opposite", 8);
    	testCase.addObject("Switch Approach", 9);
    	testCase.addObject("Switch Place", 10);
    	testCase.addObject("Drive Farther", 11);
    	testCase.addObject("Near Scale Initial", 12);
    	testCase.addObject("Scale Place", 13);
    	testCase.addObject("Cross Field", 14);
    	testCase.addObject("Far Switch Place", 15);
    	testCase.addObject("Scale Approach", 16);
    	SmartDashboard.putData("testCase", testCase);
    	
    	turningPID = new SRF_PID();
    	turningPID.setReverse(true);
    	turningPID.setPID(1.5, .75, .45);

    	positionPID = new SRF_PID();
    	positionPID.setLimits(0.8, -0.8);
    	positionPID.setReverse(true);
    	positionPID.setPID(0.1, 0, 0.005);

    	leftSide = new Encoder(0, 1);
    	System.out.println("End Robot Init");
    }

    /**
     * This function is run once each time the robot enters autonomous mode
     */

    public void autonomousInit() {
    	System.out.println("start auto init");
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
    	wristMode = up;
    	System.out.println(237);
    	//initialize potential autonomous steps
    	auto[0][0] = autoStates.BasicDrive; //drive forward over the auto line
    	auto[0][1] = autoStates.Done;
		auto[0][2] = autoStates.NA;//
		auto[0][3] = autoStates.NA;
		auto[0][4] = autoStates.NA;
		auto[0][5] = autoStates.NA;
		auto[0][6] = autoStates.NA;
		auto[0][7] = autoStates.NA;
		
    	auto[1][0] = autoStates.InitialDrive;//place cube on nearest switch platform
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
    	auto[2][5] = autoStates.SwitchPlace;
		auto[2][6] = autoStates.Done;
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
		
    	auto[6][0] = autoStates.InitialDrive;
    	auto[6][1] = autoStates.Done;
		auto[6][2] = autoStates.NA;
		auto[6][3] = autoStates.NA;
		auto[6][4] = autoStates.NA;
		auto[6][5] = autoStates.NA;
		auto[6][6] = autoStates.NA;
		auto[6][7] = autoStates.NA;
		
    	auto[7][0] = autoStates.Turn;
    	auto[7][1] = autoStates.Done;
    	auto[7][2] = autoStates.NA;
    	auto[7][3] = autoStates.NA;
		auto[7][4] = autoStates.NA;
		auto[7][5] = autoStates.NA;
		auto[7][6] = autoStates.NA;
		auto[7][7] = autoStates.NA;
		
    	auto[8][0] = autoStates.TurnOpposite;
    	auto[8][1] = autoStates.Done;
		auto[8][2] = autoStates.NA;
		auto[8][3] = autoStates.NA;
		auto[8][4] = autoStates.NA;
		auto[8][5] = autoStates.NA;
		auto[8][6] = autoStates.NA;
		auto[8][7] = autoStates.NA;
		
    	auto[9][0] = autoStates.SwitchApproach;
    	auto[9][1] = autoStates.Done;
    	auto[9][2] = autoStates.NA;
    	auto[9][3] = autoStates.NA; 
    	auto[9][4] = autoStates.NA;
		auto[9][5] = autoStates.NA;
		auto[9][6] = autoStates.NA;
		auto[9][7] = autoStates.NA;
		
    	auto[10][0] = autoStates.SwitchPlace;
    	auto[10][1] = autoStates.Done;
		auto[10][2] = autoStates.NA;
		auto[10][3] = autoStates.NA;
		auto[10][4] = autoStates.NA;
		auto[10][5] = autoStates.NA;
		auto[10][6] = autoStates.NA;
		auto[10][7] = autoStates.NA;
		
    	auto[11][0] = autoStates.DriveFarther; 
    	auto[11][1] = autoStates.Done; 		  
		auto[11][2] = autoStates.NA;
		auto[11][3] = autoStates.NA;
		auto[11][4] = autoStates.NA;
		auto[11][5] = autoStates.NA;
		auto[11][6] = autoStates.NA;
		auto[11][7] = autoStates.NA;
		
    	auto[12][0] = autoStates.NearScaleInitial; 
    	auto[12][1] = autoStates.Done;
		auto[12][2] = autoStates.NA;
		auto[12][3] = autoStates.NA;
		auto[12][4] = autoStates.NA;
		auto[12][5] = autoStates.NA;
		auto[12][6] = autoStates.NA;
		auto[12][7] = autoStates.NA;
	 
		auto[13][0] = autoStates.ScalePlace; 
    	auto[13][1] = autoStates.Done;
		auto[13][2] = autoStates.NA;
		auto[13][3] = autoStates.NA;
		auto[13][4] = autoStates.NA;
		auto[13][5] = autoStates.NA;
		auto[13][6] = autoStates.NA;
		auto[13][7] = autoStates.NA;
		
		auto[14][0] = autoStates.CrossField; 
    	auto[14][1] = autoStates.Done;
		auto[14][2] = autoStates.NA;
		auto[14][3] = autoStates.NA;
		auto[14][4] = autoStates.NA;
		auto[14][5] = autoStates.NA;
		auto[14][6] = autoStates.NA;
		auto[14][7] = autoStates.NA;
		
		auto[15][0] = autoStates.FarSwitchPlace; 
    	auto[15][1] = autoStates.Done;
		auto[15][2] = autoStates.NA;
		auto[15][3] = autoStates.NA;
		auto[15][4] = autoStates.NA;
		auto[15][5] = autoStates.NA;
		auto[15][6] = autoStates.NA;
		auto[15][7] = autoStates.NA;
		
		auto[16][0] = autoStates.ScaleApproach; 
    	auto[16][1] = autoStates.Done;
		auto[16][2] = autoStates.NA;
		auto[16][3] = autoStates.NA;
		auto[16][4] = autoStates.NA;
		auto[16][5] = autoStates.NA;
		auto[16][6] = autoStates.NA;
		auto[16][7] = autoStates.NA;
		
     	//initialize our Robot's location on the field
    	if((int) startingLocation.getSelected() == 1)
    		ourSide = 'L';
    	else if((int) startingLocation.getSelected() == 3)
    		ourSide = 'R';
    	else
    		ourSide = 'M';
    	
    	System.out.println(400);
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
    			}//
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
    	
    	ahrs.reset();
    	leftSide.reset();
    	System.out.println(461);
    	wrist.setSelectedSensorPosition(0, 0, 0);
    	elevator.setSelectedSensorPosition(0, 0, 0);
    	targetWristPos = 0;
    	targetElevatorPos = 0;
    	System.out.println("end autoInit");
    }

    /**
     * This function is called periodically during autonomous
     */

	public void autonomousPeriodic()
	{
		SmartDashboard.putNumber("leftSide", leftSide.get());
		
		//cycle counter
		counter++;
		
		if(start)//just starting into a new case
		{ 		 
			//Reset sensors
			System.out.println("Start begin");
			
			//Determine next autonomous action
			autoStep++;
			autoCase = auto[autoID][autoStep];

			if(autoCase == autoStates.Turn)
			{
				turningPID.setSetpoint(90);
				turnSetpoint = 90;
			}
			else if(autoCase == autoStates.TurnOpposite)
			{
				turningPID.setSetpoint(-90);
				turnSetpoint = -90;
			}
			else
			{
				turningPID.setSetpoint(0);
				turnSetpoint = 0;
			}

			firstSample = true;
			
			if(autoCase!=autoStates.Done)
			{
				t.reset();
				leftSide.reset();
				ahrs.reset();
			}
			
			//Disable initialization
			start = false;
			System.out.println(autoCase);
		}
		System.out.println(autoCase);
		switch(autoCase)				//create a separate 
										// staging monitor or "executive", and when any autoID is "Done", don't quit before calling that
										// Executive so it can decide if a new autoID task should be started. That executive would 
										// consider various factors: time left in auto? Do we control our switch? Do we control our Scale? 
										// Is this a qual or playoff match? It might then set autoID appropriately, store whatever states or
										// flags that it needs to, and return. This switch/case then continues to serve, sequencing
										// though the steps of the specific autoID that the executive decided to try for.
		{
			case BasicDrive: //Just drive forward//
				robot.arcadeDrive(-0.5,0);
				if(t.get() > 2)
				{
					robot.arcadeDrive(0,0);
					start = true;
					System.out.println("End BD");
				}
				break;
			case InitialDrive: //Drive forward on the side and get ready to place
				//UNTESTED
				positionSetpoint = 120*countsPerInch;
				positionPID.setSetpoint(positionSetpoint);//151.5
				output = positionPID.computePID(leftSide.get(),t.get());
			//	if(counter % 20 == 0)
				//	System.out.println(output+"::"+positionSetpoint+":"+leftSide.get()+"::"+t.get());
				robot.arcadeDrive(output, /*turningPID.computePID(ahrs.getAngle(), t.get())*/0);
				
				if(Math.abs(output) < 0.03 && Math.abs(leftSide.get()-positionSetpoint) < 57 && t.get() > 1)//add turning output?
				{
					System.out.println("Output Computed:"+output);
					start = true;
				}
				break;
			case Turn: //Turn 90 degrees
				//UNTESTED
				output = turningPID.computePID(ahrs.getAngle(),t.get());
				robot.arcadeDrive(0, output);
				System.out.println(output);
				System.out.println(ahrs.getAngle());
				if(Math.abs(ahrs.getAngle() - turnSetpoint) < 1 && output < 0.03)
					start = true;
				break;
			case TurnOpposite: //Turn 90 degrees in the other direction
				//UNTESTED
				output = turningPID.computePID(ahrs.getAngle(),t.get());
				robot.arcadeDrive(0, output);
				if(output < 0.03 && Math.abs(ahrs.getAngle() - turnSetpoint) < 1)
					start = true;
				break;
			case SwitchApproach: //approach the switch
				//UNTESTED
				positionPID.setSetpoint(21.81*countsPerInch);
				output = positionPID.computePID(leftSide.get(), t.get());
				robot.arcadeDrive(output, turningPID.computePID(ahrs.getAngle(),t.get()));
				if(output < 0.03 && t.get() > 1)
					start = true;
				break;
			case SwitchPlace: //Place cube
				//UNTESTED
				robot.arcadeDrive(0,0);
				
				//prepare to place
				elevator.set(ControlMode.Position, elevatorMid);
				wrist.set(ControlMode.Position, wristDown);
				
				if(firstSample)
					timeNow = 500;//useless number used as place holder
									//it must be big
				
				//place
				if(Math.abs(elevator.getSelectedSensorPosition(0)-elevatorMid) < 25000 && Math.abs(wrist.getSelectedSensorPosition(0)-wristDown) < 25000)
				{
					if(firstSample)
					{
						firstSample = false;
						timeNow = t.get();
					}
					
					intakeL.set(0.2);
					intakeR.set(0.2);
					
					System.out.println(t.get() + ":" + timeNow);
					
					if(t.get() - timeNow > 1)
						start = true;
				}
				break;
			case DriveFarther: //Initial Drive plus some
				//UNTESTED
				positionPID.setSetpoint(226.72*countsPerInch);
				output = positionPID.computePID(leftSide.get(),t.get());

				robot.arcadeDrive(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				if(output < 0.03 && t.get() > 1)
					start = true;
				break;
			case NearScaleInitial: //Travel Farther plus drive
				//UNTESTED
				positionPID.setSetpoint(304.25*countsPerInch);
				output = positionPID.computePID(leftSide.get(),t.get());

				robot.arcadeDrive(output, 0/*turningPID.computePID(ahrs.getAngle(), t.get())*/);
				if(output < 0.03 && t.get() > 1)
					start = true;
				break;
			case ScalePlace: //place a cube on the scale
					elevator.set(ControlMode.Position, elevatorUp);
					
					if(Math.abs(elevator.getSelectedSensorPosition(0)-elevatorMid) < 25000)
					{
						wrist.set(ControlMode.Position, wristDown);
						
						if(firstSample)//random number that needs to be big
							timeNow = 500;
						
						if(Math.abs(wrist.getSelectedSensorPosition(0)-wristDown) < 25000)
						{
							if(firstSample)
							{
								firstSample = false;
								timeNow = t.get();
							}
							
							intakeL.set(0.3);
							intakeR.set(0.3);
							
							if(t.get() - timeNow > 1)
								start = true;
						}
					}
				break;
			case CrossField: //Travel across the field
				///UNTESTED
				positionPID.setSetpoint(295.68*countsPerInch);
				output = positionPID.computePID(leftSide.get(),t.get());

				robot.arcadeDrive(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				if(output < 0.03 && t.get() > 1)
					start = true;
				break;
			case ScaleApproach: //Adjust position relative to near scale plate
				//UNTESTED
				positionPID.setSetpoint(4.82*countsPerInch);
				output = positionPID.computePID(leftSide.get(),t.get());
	
				robot.arcadeDrive(output, turningPID.computePID(ahrs.getAngle(), t.get()));
				if(output < 0.03 && t.get() > 1)
					start = true;
				break;
			case Done: //terminate auto
				robot.arcadeDrive(0,0);
				intakeL.set(0);
				intakeR.set(0);
				break;
			case NA: //the case after Done
				robot.arcadeDrive(0,0);
				intakeL.set(0);
				intakeR.set(0);
				System.out.println("The state \"NA\" has been reached");
				break;
		}
		
		SmartDashboard.putNumber("ahrs", ahrs.getAngle());
		SmartDashboard.putNumber("t", t.get());
	}

    
    /**
     * This function is called once each time the robot enters teleoperated mode
     */

    public void teleopInit()
    {
    	leftSide.reset();
    	wrist.setSelectedSensorPosition(0, 0, 0);
    	elevator.setSelectedSensorPosition(0, 0, 0);
    	targetWristPos = 0;
    	targetElevatorPos = 0;
		wrist.set(ControlMode.Position, 0);
		ahrs.reset();
		letUp10 = true;
		manualCon = false;
		wristMode = up;
    }


    /**
     * This function is called periodically during operator control
     */

    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj.IterativeRobot#teleopPeriodic()
     */

    public void teleopPeriodic()
    {
    	if(!manualCon)
    	{
	    	if(!xBox.getRawButton(9) && Math.abs(xBox.getRawAxis(1)) > 0.2 || Math.abs(xBox.getRawAxis(0)) > 0.2)
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
	    		intakeL.set(.65);
	    		intakeR.set(.65);
	    	}
	    	else
	    	{
	    		intakeL.set(0);
	    		intakeR.set(0);
	    	}
	    	
	    	
	    	//test wrist code//
	    	if(xBox.getRawButton(3))//X
	    	{
	    		if(wristMode == down)
	    		{
	    			wrist.config_kP(0, upWristP, 10); 
	    			wrist.config_kI(0, upWristI, 10);
	    			wrist.config_kD(0, upWristD, 10);
	    		}
	    		else
	    		{
	    			wrist.config_kP(0, upFromMidP, 10); 
	    			wrist.config_kI(0, upFromMidI, 10);
	    			wrist.config_kD(0, upFromMidD, 10);
	    		}
	    		targetWristPos = wristUp;
	    		wristMode = up;
	    	}
	    	else if(xBox.getRawButton(8))//Start
	    	{
	    		if(wristMode == down)
	    		{
	    			wrist.config_kP(0, upToMidP, 10); 
	    			wrist.config_kI(0, upToMidI, 10);
	    			wrist.config_kD(0, upToMidD, 10);
	    		}
	    		else
	    		{
	    			wrist.config_kP(0, downToMidP, 10); 
	    			wrist.config_kI(0, downToMidI, 10);
	    			wrist.config_kD(0, downToMidD, 10);
	    		}
	    		targetWristPos = wristMiddle;
	    		wristMode = mid;
	    		
	    	}
	    	else if(xBox.getRawButton(7))//Back
	    	{
	    		wrist.config_kP(0, downWristP, 10); 
	    		wrist.config_kI(0, downWristI, 10);
	    		wrist.config_kD(0, downWristD, 10);
	    		targetWristPos = wristDown;
	    		wristMode = down;
	    	}
	    	
	    	wrist.set(ControlMode.Position, targetWristPos);
	    	
	    	
	    /*	if(xBox.getRawButton(3))//X
	    		wristControl(up);
	    	else if(xBox.getRawButton(8))//Start
	    		wristControl(mid);
	    	else if(xBox.getRawButton(7))//Back
	    		wristControl(down);
	    	*/
	    	System.out.println("814:"+xBox.getRawButton(4));
	    	//Elevator code
	    	if(xBox.getRawButton(4))//Y
	    	{System.out.println("moving elevator up");
	    		elevator.config_kP(0, upElevatorP, 10); 
	    		elevator.config_kI(0, upElevatorI, 10);
	    		elevator.config_kD(0, upElevatorD, 10);
	   			targetElevatorPos = elevatorUp;//go to top (this is a test value use:
	    	}
	    	else if(xBox.getRawButton(2))//B
	    	{
	    		if(elevator.getSelectedSensorPosition(0) > -500000)
	    		{
	    			elevator.config_kP(0, downElevatorP, 10); 
	       			elevator.config_kI(0, downElevatorI, 10);
	       			elevator.config_kD(0, downElevatorD, 10);
	    		}
	    		else
	    		{
	    			elevator.config_kP(0, upElevatorP, 10); 
	        		elevator.config_kI(0, upElevatorI, 10);
	        		elevator.config_kD(0, upElevatorD, 10);
	    		}
	    		targetElevatorPos = elevatorMid;
	    	}
	   		else if(xBox.getRawButton(1))//A
	   		{
	   			elevator.config_kP(0, downElevatorP, 10); 
	   			elevator.config_kI(0, downElevatorI, 10);
	   			elevator.config_kD(0, downElevatorD, 10);
	   			targetElevatorPos = elevatorDown; //go to bottom
	   		}
	   			
	    	elevator.set(ControlMode.Position, targetElevatorPos);	    	
	    	
	    	   	//hook code
	    	if(xBox.getRawButton(5)) //LB - extend hook
	    		hook.set(0.5);
	    	else if(xBox.getRawButton(9))//Left joystick
	    		hook.set(-0.5);
	    	else
	    		hook.set(0);
	    		
	    		
	    		//winch code	    		
	    	if(xBox.getRawButton(6)) //RB - wind in
	    		winch.set(-0.5);
	    	else
	    		winch.set(0);
	    	
	    	
	    	//Change control layout
	    	if(xBox.getRawButton(10) && letUp10) //right stick - finicky detecting button press(press down hard)
	    	{
	    		letUp10 = false;
	    		manualCon = true;
	    	}   
    	}
	    else
	    {
	    	//manual elevator controls
	    	if(Math.abs(xBox.getRawAxis(5)) > 0.2 && letUp10) //right stick y-axis
	    		elevator.set(ControlMode.PercentOutput, xBox.getRawAxis(5));
	    	else
	    		elevator.set(ControlMode.PercentOutput, 0);
	    	
	    	if(xBox.getRawButton(2))//B
	    		elevator.setSelectedSensorPosition(0, 0, 0);
	    	
	    	
	    	
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
	    		intakeL.set(.65);
	    		intakeR.set(.65);
	    	}
	    	else
	    	{
	    		intakeL.set(0);
	    		intakeR.set(0);
	    	}	    	
	    	
	    	
	    	if(xBox.getRawButton(3))//X
	    		wrist.set(ControlMode.PercentOutput, 0.4);
	    	else if(xBox.getRawButton(4))//Y
	    		wrist.set(ControlMode.PercentOutput, -0.4);
	    	else
	    		wrist.set(ControlMode.PercentOutput, 0);
		    
	    	
		    //maual winch control - untested
		    if(Math.abs(xBox.getRawAxis(6)) > .2)
		    	hook.set(xBox.getRawAxis(6));
		    else
		    	hook.set(0);
		    	
		   
		    if(xBox.getRawButton(5))//LB
		    	winch.set(0.5);
			else if(xBox.getRawButton(6))//RB
				winch.set(-0.5);
			else
				winch.set(0);
		    	
		    	
		    //change control layout
		    if(xBox.getRawButton(10) && letUp10)
		    {
		    	winch.set(0);
		    	hook.set(0);
		    	intakeL.set(0);
		    	intakeR.set(0);
		    	targetElevatorPos = elevator.getSelectedSensorPosition(0);
		    	letUp10 = false;
		    	manualCon = false;
		    }
		   
	    }
    	System.out.println(manualCon);
    	
    	if(!xBox.getRawButton(10))
    		letUp10 = true;
	    	
    	SmartDashboard.putNumber("Elevator Position", elevator.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("Wrist Position", wrist.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("leftSide", leftSide.get());
    	SmartDashboard.putNumber("targetWristPos", targetWristPos);
    	SmartDashboard.putNumber("targetElevatorPos", targetElevatorPos);
    	SmartDashboard.putNumber("ahrs", ahrs.getAngle());
    }

    void wristControl(int pos)
    {
    	
    	if(pos == up)
    	{
    		if(wrist.getSelectedSensorPosition(0) < -55000)
    		{
    			wrist.config_kP(0, upWristP, 10); 
    			wrist.config_kI(0, upWristI, 10);
    			wrist.config_kD(0, upWristD, 10);
    		}
    		else
    		{
    			wrist.config_kP(0, upFromMidP, 10); 
    			wrist.config_kI(0, upFromMidI, 10);
    			wrist.config_kD(0, upFromMidD, 10);
    		}
    		targetWristPos = wristUp;
    	}
    	else if(pos == mid)
    	{
    		if(wrist.getSelectedSensorPosition(0) < -55000)
    		{
    			wrist.config_kP(0, upToMidP, 10); 
    			wrist.config_kI(0, upToMidI, 10);
    			wrist.config_kD(0, upToMidD, 10);
    		}
    		else
    		{
    			wrist.config_kP(0, downToMidP, 10); 
    			wrist.config_kI(0, downToMidI, 10);
    			wrist.config_kD(0, downToMidD, 10);
    		}
    		targetWristPos = wristMiddle;
    		
    	}
    	else if(pos == down)
    	{
    		wrist.config_kP(0, downWristP, 10); 
    		wrist.config_kI(0, downWristI, 10);
    		wrist.config_kD(0, downWristD, 10);
    		targetWristPos = wristDown;
    	}
    	
   /* 	if(wristMin.get())
    		wrist.setSelectedSensorPosition(0, 0, 0);
    	
    	if(wrist.getOutputCurrent() < 0 && wrist.getSelectedSensorPosition(0) < 10)
    		*/wrist.set(ControlMode.PercentOutput, 0);
    	/*else
    		wrist.set(ControlMode.Position, targetWristPos);
    	*/
    }
    
    
    public void testPeriodic(){
    	System.out.println(wristMin.get());
    }
}
