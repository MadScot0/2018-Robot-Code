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

//                                2018 Bot Code// v1.3.3 //cleaned up code
									//remember to set low scale encoder before
									//competition, ADD CASE, & remove encoder
									//reset from teleop
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
	
	int change = 1;
	
	double angleSetpoint = 0;
	double navXCorrect = 16;//4
//	double positionP=0.0045, positionI = 0.000556875, positionD = 0.0002475;
	double positionP=0.0045, positionI = 0.018225, positionD = 0.0017775;	
	double positionCloseP = 0.009, positionCloseI = 0.0273375, positionCloseD = 0.00263847656;
	
	boolean withinRange;//are we within an inch of the setpoint
	double timeReached;//the time we found we were within an inch of the setpoint
	
	final static boolean debug = false;//debug constant for turning multiple debug outputs on and off
													// Change before compiling.
	int counter = 0;
	
	boolean lastWristDetection;
	
	boolean letUp8;
	double manualWristPower;//the variable that controls the manual aiming of the wrist
	boolean manualWrist;//when this flag is triggered we gain manual control
						//of the wrist even tough we won't be in manualCon
	
	long lastCaseEndingEncoderCount;
	double lastCaseEndingTime;
	double lastCaseEndingHeading;
	
	boolean potentiallyArrived;
	double startTime;
	
	long currentPosition;
	double currentTime;
	double arrivalTime;
	double currentAngle;
	double currentAngleDifference;
	
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

	DigitalInput wristMax;
	
	double angleNow, scaledAngle; //the current angle and the angle to feed into the PID

	boolean goToHigh;
	boolean letUp4;
	
	Timer t;

	int autoID;
	double turnSetpoint;
	double positionSetpoint;
	
	double timeNow;
	boolean firstSample;

	public enum autoStates { 	Done,
						BasicDrive, DriveFarther,  							
						Turn, TurnOpposite, CrossField,
						InitialDrive, SwitchApproach, SwitchPlace, FarSwitchPlace,
						NearScaleInitial, ScaleApproach, ScalePlace, NineFoot,
																			// Add steps here needed to pick up a cube from the floor (locate, 
																			// acquire, and stow it for driving) to allow for Stage 3 auto tasks; 
																			// will probably also need an additional autoID since getting a cube
																			// is a separate multi step process. 
						NA };
																			// May also want to consider "auto" actions in teleop, especially 
																			// locating and acquiring a cube you cannot see. Also perhaps 
																			// placing a cube on a switch or scale plate automatically would 
																			// be faster then the driver could do it. Note: always allow for 
																			// abort on any teleop automatic actions!
																			
																			
	int autoStep;	 //the variable that tracks how far into autonomous the robot is

	String gameData; //variable for retrieving field setup

	char switchSide; //side that is ours of the switch
	char scaleSide;  //side that is ours of the scale
	char ourSide;    //the side that our robot is on on the field

	double targetDistance;//distance to move forwards
	double maxFeetPerSec = 12;//16;//edit
	double safetyTimeout;//max time allowed for autonomous step
	
	autoStates	auto[][] = new autoStates[18][8];			// first index is autoID, second index is place in list (autoStep)
	autoStates autoCase;
	
	private SendableChooser placeCube; 			//Do we place a cube?
	private SendableChooser startingLocation;	//Where do we start?
	private SendableChooser priorityType;		//Decide based on side or scale/switch
	private SendableChooser priorityPlace;		//Decide based on scale or switch
	private SendableChooser testMode;			//Are we in testing mode
	private SendableChooser testCase;			//What case are we testing

	SRF_PID positionPID;
	SRF_PID turningPID;

	Encoder leftSide;
										
	double countsPerInch = 18.96;				//was 54.3249
	double output;
	double turningMult;    		//the variable used to mirror our operations depending on robot placement
	
	int targetWristPos;			//variable for where the wrist will move to
	int targetElevatorPos; 		//variable for where the elevator will move to
	
	//the variables for the constants in each PID loop
	//double upElevatorP = 0.0058, upElevatorI = 0.0000000000001, upElevatorD = 0.008;
	//double downElevatorP = 0.0035, downElevatorI = 0.000002, downElevatorD  = 0.003;
	double upElevatorP = 0.0058, upElevatorI = 0, upElevatorD = 0.001;
	double downElevatorP = 0.0035, downElevatorI = 0, downElevatorD  = 0.003;
	
	double   upWristP = 0.00605,   upWristI = 0.0000176,     upWristD = 0.0016;
	double upFromMidP = 0.0047,		upFromMidI = 0.00005,     upFromMidD = 0.0025;
	double downWristP = 0.0075,  downWristI = 0.000016,  downWristD = 0.75;//practice values  0.0075,0.000008,0.5
	//double downWristP = 0.007,  downWristI = 0.0000252,  downWristD = 0.05;//old comp
	double   upToMidP = 0.5,    upToMidI = 0.05,       upToMidD = 0.00001;
	double downToMidP = 0.0043,  downToMidI = 0.00000001775, downToMidD = 0.00305;
	
	int elevatorUp = -1600000, elevatorMid = -475000, elevatorLowScale = -1370288, elevatorDown = -20000;
	int    wristUp = 0,        wristMiddle = -53250,     wristDown = -110000;
	int wristMode, elevatorMode;
	
	boolean manualCon = false; //Used to change whether using manual controls for teleop
	boolean letUp10;
	
	int up = 0, mid = 1, lowScale = 2, down = 3;
	
	public void robotInit() { 
		
		wristMax = new DigitalInput(2);
		
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
		wrist.configPeakOutputForward(0.45, 5);
		wrist.configPeakOutputReverse(-0.45, 5);
		
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
    	testCase.addObject("Nine Foot", 17);
    	SmartDashboard.putData("testCase", testCase);

    	positionPID = new SRF_PID();
    	positionPID.setLimits(0.9, -0.9);//
    	positionPID.setReverse(true);
    	positionPID.setPID(positionP, positionI, positionD);
    	leftSide = new Encoder(0, 1);
    	
    	turningPID = new SRF_PID();
    	turningPID.setReverse(true);
    	turningPID.setLimits(0.9, -0.9);
    	turningPID.setPID(0.05, 1.2, 0.01);
    	
    	if (debug) System.out.println("End Robot Init");
    }

	
	
    /**
     * This function is run once each time the robot enters autonomous mode
     */

    public void autonomousInit() {
    	if (debug) System.out.println("start auto init");
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

    	// start = true;   //initialize variables
    	autoStep = -1;
    	wristMode = up;

    	//initialize potential autonomous steps
    	auto[0][0] = autoStates.BasicDrive; //drive forward over the auto line
    	auto[0][1] = autoStates.Done;
		auto[0][2] = autoStates.NA;
		auto[0][3] = autoStates.NA;
		auto[0][4] = autoStates.NA;
		auto[0][5] = autoStates.NA;
		auto[0][6] = autoStates.NA;
		auto[0][7] = autoStates.NA;
														
    	auto[1][0] = autoStates.InitialDrive;
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
		
		auto[17][0] = autoStates.NineFoot; 
    	auto[17][1] = autoStates.Done;
		auto[17][2] = autoStates.NA;
		auto[17][3] = autoStates.NA;
		auto[17][4] = autoStates.NA;
		auto[17][5] = autoStates.NA;
		auto[17][6] = autoStates.NA;
		auto[17][7] = autoStates.NA;
		
     	//initialize our Robot's location on the field
    	if((int) startingLocation.getSelected() == 1)
    		ourSide = 'L';
    	else if((int) startingLocation.getSelected() == 3)
    		ourSide = 'R';
    	else
    		ourSide = 'M';
    	
    	if(debug)System.out.println(400);
    	//initiate autoID based on sendable chooser and field
    	if((int) testMode.getSelected() == 2)				//if "Testing mode"
    		autoID = (int) testCase.getSelected();
    	else if((int) placeCube.getSelected() == 2) //if we aren't planning on placing
    		autoID = 0;
    	else
    	{
    		//initialize turning direction multiplier
    		if((int) priorityType.getSelected() == 1) //if looking for closest side
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
    				if((int) priorityPlace.getSelected() == 2) // if going to scale
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
    		else //"else priority type 2 specifies we place cube (on either switch or scale, as Selected), regardless of side"
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

    	autoID=1;
    	
		ourSide='L';
		
		//initialize turning direction multiplier
		if(ourSide == 'R')
    		turningMult = -1;
    	else
    		turningMult = 1;
    	
    	ahrs.reset();
    	leftSide.reset();
    	if(debug)System.out.println(516);
    	wrist.setSelectedSensorPosition(0, 0, 0);
    	elevator.setSelectedSensorPosition(0, 0, 0);
    	targetWristPos = 0;
    	targetElevatorPos = 0;
    	advanceToNextAutoCase();
    	
    	
    	
    	if (debug) System.out.println("end autoInit");
    }

/*
 *	 	  support utility for all init needed to move  robot under PID control.
 *        this only works when the variables being set here are declared public. Only one movement 
 *		  at a time will be active, thus the variable values will apply only to whichever autoCase is currently active.
 */
	public void prepareToMove(double d)
	{
		targetDistance = d;
		positionSetpoint = targetDistance*countsPerInch;
		positionPID.setSetpoint(positionSetpoint);
		safetyTimeout = ((targetDistance/12)/(maxFeetPerSec*0.05)+.5); // Set drive to position safety timeouts by using .6 of max speed 															  
		// (.6 * 16 ft/sec? = ~ 9.6) divided into target distance to travel (in feet). 
																	  // To this add a half second fudge factor. 
																	  // Adjust speed factor if needed (i.e. to avoid spurious timeouts).
		positionPID.setPID(positionP, positionI, positionD);		// XXX
	}//
	
	
	
	public void advanceToNextAutoCase() //correct these values
	{
		
	int turnDirection = 1;
	
		if(debug) System.out.println("stopping robot");
		robot.arcadeDrive(0,0);							// cancel any residual motion of robot.
		
		withinRange = false;
		timeReached = 0;
// XXX		positionPID.setPID(positionP, positionI, positionD);
		
		if (debug) System.out.println("autoStepAdvance");
		
		autoStep++;
		if ((autoID < 0) || (autoID > 17) || (autoStep < 0) || (autoStep > 7))//error checks on array indices
			autoCase = autoStates.NA;
		else
			autoCase = auto[autoID][autoStep];
			
		safetyTimeout = 5;								// Default timeout.
		
		switch(autoCase)
		{
			case TurnOpposite:
				turnDirection = -1;
			case Turn:
				turnSetpoint = 90*turnDirection*turningMult;
				safetyTimeout = 10; //was 1.5
				turningPID.setSetpoint(turnSetpoint);
				break;

			case BasicDrive:
				safetyTimeout = 2.5;
				break;
			case InitialDrive: //Drive forward on the current side, in preparation to placing cube on switch
				prepareToMove(72);//xxx 174 
				break;		
			case DriveFarther: 	// Initial Drive distance plus more to prepare for scale placement
				prepareToMove(226.72);
				break;
			case NearScaleInitial: //Travel Farther plus drive
				prepareToMove(304.25);
				break;
			case CrossField: //Travel across the field
				prepareToMove(295.68);
				break;
			case ScaleApproach: //Adjust position relative to near scale plate
				prepareToMove(4.82);
				break;
			case NineFoot:
				//prepareToMove(24);
				prepareToMove(108);
				break;
			case SwitchPlace: //PMV - case reserved for placing cube being carried on switch
				//break?
			case ScalePlace:  //Place cube being carried on the scale
				// PMV- with Talon SRXs being used for elevator and wrist, PID control is obviously deferred to the Talons.
				// I don't understand the current elevator and wrist control architecture well enough to say if anything needs
				// changing here, but we still need notification of "Done" before proceeding with other cases,
				// and safety timeouts might also be useful. 
				// These components ideally should be able to do their standard operations under RIO control 
				// (always with the option to fully override with manual joystick control). This would allow limited automation to 
				// assist the drive team when picking up or placing a cube.
				// For now, at least until I know more, these two init cases are left blank.
				break;		
			case NA:
				System.out.println("Unexpected NA case in advanceToNextAutoCase");	
				intakeL.set(0);
				intakeR.set(0);
				elevator.set(ControlMode.PercentOutput,0);
				wrist.set(ControlMode.PercentOutput,0);
				break;
			case Done:
				intakeL.set(0);
				intakeR.set(0);
				elevator.set(ControlMode.Position,elevator.getSelectedSensorPosition(0));
				wrist.set(ControlMode.Position,wrist.getSelectedSensorPosition(0));
				break;
			default:
				System.out.println("Unexpected default case in advanceToNextAutoCase");	
				break;
		}
		
		firstSample = true;//used to sample the start time of basic actions

		lastCaseEndingEncoderCount = leftSide.get();
		lastCaseEndingTime = t.get();
		lastCaseEndingHeading = ahrs.getAngle();
		
		if(autoCase!=autoStates.Done) 
		{
			t.reset();
			leftSide.reset();
			ahrs.reset();
		}
		
		if (debug) System.out.println("637:"+autoID+":"+autoStep+":"+autoCase);

		potentiallyArrived = false;		//flag for use in ending PID controlled autoSteps.
		startTime = t.get();			//startTime allows for "safety" timeouts. 
	}

	
	
	
	
/**
 * This function is called periodically during autonomous
 */

	public void autonomousPeriodic()
	{
		SmartDashboard.putNumber("leftSide", leftSide.get());
		
		//cycle counter
		counter++;

		if (debug) System.out.println("658:"+autoID+":"+autoStep+":"+autoCase);

		switch(autoCase)				//create a separate 
										// staging monitor or "executive", and when any autoID is "Done", don't quit before calling that
										// Executive so it can decide if a new autoID task should be started. That executive would 
										// consider various factors: time left in auto? Do we control our switch? Do we control our Scale? 
										// Is this a qual or playoff match? It might then set autoID appropriately, store whatever states or
										// flags that it needs to, and return. This switch/case then continues to serve, sequencing
										// though the steps of the specific autoID that the executive decided to try for.
		{
			case BasicDrive: //Just drive forward
				robot.arcadeDrive(-0.65,ahrs.getAngle()/6);
				if(debug){
					System.out.println(t.get()+":"+startTime+":"+safetyTimeout);
				}
				if((t.get() - startTime) > safetyTimeout)				
				{
					if (debug) System.out.println("End BD");
					advanceToNextAutoCase();			
				}
				break;
			case NineFoot:
			case InitialDrive:
			case SwitchApproach:
			case DriveFarther:
			case NearScaleInitial:				
			case CrossField:
			case ScaleApproach:
				currentPosition = leftSide.get();
				currentTime = t.get();
				
				output = positionPID.computePID(currentPosition, currentTime);	// PMV - changed to temp variable arguments instead of method calls.
				
				robot.arcadeDrive(output, (ahrs.getAngle())/navXCorrect);
				
				if(debug && (counter % 20 == 0))
					System.out.println(output+"::"+positionSetpoint+":"+currentPosition+"::"+currentTime);
			
				// PMV - original test for termination (line just below, now commented out) needed fixing:			
				//   "if((Math.abs(output) < 0.03) && (Math.abs(currentPosition-positionSetpoint) < 57) && (currentTime > 1))"
				// 
				// To begin, I deferred current time check until later, added a test for encoder > setPoint (by any amount), 
				// in which case a new "potentiallyArrived" flag is set and a "settling time" timeout is started.
				// Allow the PID only a fixed amount of time to stabilize - 1 second should be more than enough if tuned well.
				// To cover the case where the setPoint is never actually crossed (because the robot stops asymptotically or otherwise short),
				// also test for being within a small threshold distance of the positionSetpoint (1" ?) AND calculated PID output is "small" (TBD)
				// I suspect .03 may be too small - try .1, since if I recall correctly, anything under .2 does not cause 
				// the robot to actually move...
				// If a timeout occurs without being close to the setpoint, something is wrong. Advance the state to "N/A" (assumed to always be
				// stored as the last autoStep index) to avoid further difficulties (and maybe arena fouls? ;-( ).
				
				if(positionSetpoint-currentPosition<48*countsPerInch)
					positionPID.setPID(positionCloseP, positionCloseI, positionCloseD);
				
				if (! potentiallyArrived)
				{
					if (currentPosition > positionSetpoint)
					{
						potentiallyArrived = true;
						arrivalTime = currentTime;
					} else if ((currentTime-startTime) > safetyTimeout)	// safety timeout (note: add Safety timeout declaration! 3 Sec?)
					{
						// Always print on timeout condition, DEBUG defined or not!
						System.out.println("Safety timeout "+autoID+":"+autoStep+" "+positionSetpoint+" : "+currentPosition+" :: "+(currentTime-startTime));			
						autoStep = 6;		//this assignment ensures that the "still to be incremented" AutoStep will end up 
						advanceToNextAutoCase();//
					}
				} else {								// "potentiallyArrived" flag is now true, so at the least we are close. 
														// Test for final arrival, and end case if so. Also end if "settling time" expires!
					if (! withinRange ) {
						if (Math.abs(currentPosition-positionSetpoint)<=countsPerInch) {
							withinRange=true;
							timeReached = currentTime;
						}
					} else {
						if (Math.abs(currentPosition-positionSetpoint)>countsPerInch) {
							withinRange=false;
						}
					}
					
					if ( withinRange && currentTime-timeReached>1)//xxx (Math.abs(output) < 0.01)
					{
						if (debug) System.out.println("End "+autoID+":"+autoStep+" "+output+" :: "+positionSetpoint+" : "+currentPosition+" :: "+currentTime+" : "+arrivalTime);
						advanceToNextAutoCase();
					}
					else if((currentTime - arrivalTime) > 2)
					{
						autoStep = 6;
						advanceToNextAutoCase();
					}
				}
				break;			
			case TurnOpposite:		// Turn -90 degrees 
			case Turn:			    // Turn 90 degrees
				currentAngle = ahrs.getAngle();
				currentAngleDifference = ahrs.getAngle()-turnSetpoint;
				currentTime = t.get();
				
				
				if(!withinRange && Math.abs(currentAngleDifference) < 2)
				{
					timeReached = currentTime;
					withinRange = true;
				}
				else if(Math.abs(currentAngleDifference) >= 2)
				{
					withinRange = false;
				}
				
				output = turningPID.computePID(currentAngle, currentTime);
				
				robot.arcadeDrive(0, output);

				if(debug && (counter % 20 == 0)) System.out.println(currentTime+" :: "+output+" :: "+turnSetpoint+" : "+currentAngle);

/*
 * 				if (! potentiallyArrived)
				{
					if ((turnSetpoint == 90) && (currentAngle > turnSetpoint)
					    ||
						(turnSetpoint == -90) && (currentAngle < turnSetpoint))
					{
						potentiallyArrived = true;
						arrivalTime = currentTime;
					} else if ((currentTime-startTime) > safetyTimeout)	// PMV - new safety timeout (set timeout value declaration in 
																		// advanceToNextAutoStep. 1 Sec should work for turns.
					{
						System.out.println("Turn safety timeout "+autoID+":"+autoStep+" "+turnSetpoint+" :: "+currentAngle+" :: "+(currentTime-startTime));			
						autoStep = 6;		//set autoStep to become 7 (N/A), which will shut down actuators for the remainder of autonomous.
						advanceToNextAutoCase();
					}
				} else {								// "potentiallyArrived" is true, so at the least we are close. 
														// This flag means safety timeout is effectively canceled, but remember setting 
														// timeout was started and is still active.
														// Test for final arrival based on threshold tests; end case and advance if so. 
														// Also end and advance if "settling time" expires!
 				}
*/
					
				if (withinRange &&  currentTime - timeReached > 1)
				{
					if (debug) System.out.println("End Turn: "+autoID+":"+autoStep+" "+turnSetpoint+" :: "+currentAngle+" :: "+(currentTime-startTime));
					advanceToNextAutoCase();
				}
				else if(currentTime - arrivalTime > 100)//1
				{
					autoStep = 6;
					advanceToNextAutoCase();
				}
				break;
				
			case SwitchPlace: //Place cube
				//UNTESTED
				robot.arcadeDrive(0,0);
				currentTime = t.get();
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
						timeNow = currentTime;
					}
					
					intakeL.set(0.2);
					intakeR.set(0.2);
					
					if (debug)System.out.println(t.get() + ":" + timeNow);
					
					if(currentTime - timeNow > 1)
						advanceToNextAutoCase();
				}
				
				if(currentTime - startTime > 3)
				{
					autoStep = 6;
					advanceToNextAutoCase();
				}
				break;
			case ScalePlace: //place a cube on the scale
				elevator.set(ControlMode.Position, elevatorUp);
				currentTime = t.get();
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
						
						if(currentTime - timeNow > 1)
							advanceToNextAutoCase();
					}
				}
				
				if(currentTime - startTime > 3)
				{
					autoStep = 6;
					advanceToNextAutoCase();
				}
				break;
			case NA: //the case after Done; nothing to do, but generally unexpected.
				if(debug && (counter % 20 == 0)) System.out.println("The state \"NA\" has been reached");
				// intentional fall through here
			case Done: //Do nothing
			default:
				break;
		}
		
		SmartDashboard.putNumber("ahrs", ahrs.getAngle());
		SmartDashboard.putNumber("t", t.get());
		SmartDashboard.putNumber("deltaTime", currentTime-startTime);
		SmartDashboard.putNumber("safetyTimeout", safetyTimeout);
		SmartDashboard.putNumber("autoStep", autoStep);
		SmartDashboard.putNumber("Turning Error", turnSetpoint-ahrs.getAngle());
		SmartDashboard.putNumber("Output", output);
	}

 
    /**
     * This function is called once each time the robot enters teleoperated mode
     */

    public void teleopInit()
    {
    	leftSide.reset();
    //	wrist.setSelectedSensorPosition(0, 0, 0);//take these out
    	elevator.setSelectedSensorPosition(0, 0, 0);//take these out
    	targetWristPos = wristUp;
    	targetElevatorPos = 0;
		wrist.set(ControlMode.Position, 0);
		ahrs.reset();
		letUp10 = true;
		manualCon = false;
		manualWrist = false;
		wristMode = up;
		lastWristDetection = wristMax.get();
		letUp4 = true;
		goToHigh= true;
		elevatorMode = down;
    }


    /**
     * This function is called periodically during operator control
     */

    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj.IterativeRobot#teleopPeriodic()
     */

    public void teleopPeriodic()
    {
    	if(!manualCon || !debug)
    	{	
	    	if(!xBox.getRawButton(9) && Math.abs(xBox.getRawAxis(1)) > 0.2 || Math.abs(xBox.getRawAxis(0)) > 0.2)
	    		 robot.arcadeDrive(xBox.getRawAxis(1), -xBox.getRawAxis(0));
	    	else
	    		 robot.arcadeDrive(0,0);
	    	
	    	
	    	//cube intake
	    	if(xBox.getRawAxis(2) > 0.2)//intake
	    	{
	    		intakeL.set(-.65);
	    		intakeR.set(-.65);
	    	}
	    	else if(xBox.getRawAxis(3) > 0.2)//eject
	    	{
	    		intakeL.set(.8);
	    		intakeR.set(.8);
	    	}
	    	else
	    	{
	    		intakeL.set(0);
	    		intakeR.set(0);
	    	}
	    	
	    	
	    	//test wrist code//
	    	if(xBox.getRawButton(3) && !manualWrist && wristMode!=up)//X
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
	    		if(!manualWrist && letUp8)
	    			manualWrist = true;
	    		else if(letUp8)
	    			manualWrist = false;
	    		
	    		letUp8 = false;
	    		/*if(wristMode == down)
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
	    		*/
	    	}
	    	else if(xBox.getRawButton(7) && !manualWrist &&wristMode!=down)//Back
	    	{
	    		wrist.config_kP(0, downWristP, 10); 
	    		wrist.config_kI(0, downWristI, 10);
	    		wrist.config_kD(0, downWristD, 10);
	    		targetWristPos = wristDown;
	    		wristMode = down;
	    	}
	    	
	    	if(!xBox.getRawButton(8))
	    		letUp8 = true;
	    	
	    	if(debug)System.out.println(":"+manualWrist);
	    	
	    	if(lastWristDetection != wristMax.get())
	    	{
	    		wrist.setSelectedSensorPosition(0,0,5);
	    		//System.out.println(wrist.getSelectedSensorPosition(0));
	    		//wrist.set(ControlMode.Position,0);
	    	}
	    		
	    	if(!manualWrist)
	    		wrist.set(ControlMode.Position, targetWristPos);
	    	else
	    	{
	    		manualWristPower = xBox.getRawAxis(5);
	    		
	    		if(Math.abs(manualWristPower) > 0.2)
	    		{
	    			if(manualWristPower > 0 && wrist.getSelectedSensorPosition(0) > -1500)
	    				wrist.set(ControlMode.PercentOutput, 0);
	    			else if(manualWristPower < 0 && wrist.getSelectedSensorPosition(0) < -160000)
	    				wrist.set(ControlMode.PercentOutput, 0);
	    			else
	    				wrist.set(ControlMode.PercentOutput, manualWristPower);
	    		}
	    		else
	    			wrist.set(ControlMode.PercentOutput, 0);
	    	}

	    	if(debug)System.out.println("814:"+xBox.getRawButton(4));
	    	//Elevator code
	    	if(xBox.getRawButton(4) && letUp4)//Y
	    	{
	    		/*letUp4 = false;
	    		if(elevatorMode != up)
	    		{*/
	    			elevatorMode = up;
	    			if(debug)System.out.println("moving elevator up");
	    			elevator.config_kP(0, upElevatorP, 10); 
	    			elevator.config_kI(0, upElevatorI, 10);
	    			elevator.config_kD(0, upElevatorD, 10);
	   				targetElevatorPos = elevatorUp;//go to top (this is a test value use:
	    		/*}
	    		else
	    		{
	    			elevatorMode = lowScale;
	    			elevator.config_kP(0, downElevatorP, 10); 
	    			elevator.config_kI(0, downElevatorI, 10);
	    			elevator.config_kD(0, downElevatorD, 10);
	   				targetElevatorPos = elevatorLowScale;
	    		}*/
	    	}
	    	else if(xBox.getRawButton(2))//B
	    	{
	    		if(elevatorMode == up || elevatorMode == lowScale)
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
	    		
	    		elevatorMode = mid;
	    		targetElevatorPos = elevatorMid;
	    	}
	   		else if(xBox.getRawButton(1))//A//
	   		{
	   			elevatorMode = down;
	   			elevator.config_kP(0, downElevatorP, 10); 
	   			elevator.config_kI(0, downElevatorI, 10);
	   			elevator.config_kD(0, downElevatorD, 10);
	   			targetElevatorPos = elevatorDown; //go to bottom
	   		}
	   			
	    	if(!xBox.getRawButton(4))
	    		letUp4 = true;
	    	
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
	    		winch.set(-1);
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
	    	if(xBox.getRawAxis(3) > 0.2)//intake
	    	{
	    		intakeL.set(-.5);
	    		intakeR.set(-.5);
	    	}
	    	else if(xBox.getRawAxis(2) > 0.2 && (wristMode != up || (wristMode == up && manualWrist)))//eject
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
    	if(debug)System.out.println(manualCon);
    	
    	if(!xBox.getRawButton(10))
    		letUp10 = true;
    	
    	lastWristDetection = wristMax.get();
    	
    	if(wristMax.get() != lastWristDetection)
    		change = 2;//
    	else
    		change = 0;
    	
    	SmartDashboard.putNumber("Elevator Position", elevator.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("Wrist Position", wrist.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("leftSide", leftSide.get());
    	SmartDashboard.putNumber("targetWristPos", targetWristPos);
    	SmartDashboard.putNumber("targetElevatorPos", targetElevatorPos);
    	SmartDashboard.putNumber("ahrs", ahrs.getAngle());
    	SmartDashboard.putNumber("forward", xBox.getRawAxis(1));
    	SmartDashboard.putNumber("turn", xBox.getRawAxis(0));
    	SmartDashboard.putBoolean("wristMax", wristMax.get());
    	SmartDashboard.putNumber("change", change);
    }
    
    public void testPeriodic(){
    	System.out.println(wristMax.get());
    }
}
