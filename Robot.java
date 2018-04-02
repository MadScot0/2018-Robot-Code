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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;				
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//                                2018 Bot Code// v1.3.4.2 //added fine tuning to RTU
									//removed automatic write to talon on wrist tuning
									//remeber to rework auto?
									//fixed safety timeout bug, and attempted to fix a bug with
									//the motor safety helper
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
	
	boolean twoLetUp2;
	boolean twoLetUp4;
	boolean twoLetUp5;
	boolean twoLetUp6;
	boolean twoLetUp8;
	boolean twoLetUp10;
	
	double targetTurn;
	
	//PowerDistributionPanel pdp = new PowerDistributionPanel();
	//potential for graphing current draw
	boolean setpointReached;
	boolean wristFlushed;
	
	double multMode;//the multiplier for the multiplier increment in PID tuning
	//cycle through 1, .5, .1, and 2
	
	Timer testTimer;
	int testLoopCount;
	
	boolean startTestAuto = true;
	boolean runningTestAuto;
	
	boolean letUp4;
	boolean letUp5;
	boolean letUp6;
	boolean changedGain;
	double currentPValue;
	double currentIValue;
	double currentDValue;
	double currentMultP = 1, currentMultI = 1, currentMultD = 1;
	int pidGain = 1;
	static final int pidChoice = 6;//1=wristDown,2=elevatorUp,3=elevatorDown, 4=positionPID, 5=wristUp, 6=turningPID
	
	boolean setElevatorDown;
	
	double arrivalThreshold;
	
	double navXCorrect = 10;//16;			// XXX should be ~ 30. Converts heading error (in degrees) into 
										// appropriate -1 to 1 range arcade drive direction control argument
	double directionControl;
	
	double positionP=0.0117, positionI = 0.03488265, positionD = 0.0015475;
	double turningP=0.1, turningI=2.4, turningD=0.04;	
	double basePositionP=positionP, basePositionI = positionI, basePositionD = positionD;
	double baseTurningP=turningP, baseTurningI=turningI, baseTurningD=turningD;
	
	boolean withinRange;//are we within an inch of the setpoint
	double timeReached;//the time we found we were within an inch of the setpoint
	
	final static boolean enableManual = false;
	final static boolean testing = true;//
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
	
	double positionArrivalThreshold;
	boolean potentiallyArrived;
	double startTime;
	double settleTime;
	
	long currentPosition;
	double currentTime;
	double arrivalTime;
	double currentAngle;
	double currentAngleDifference;
	
    RobotDrive robot;
	Joystick xBox;
	Joystick tuningController;

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

	DigitalInput highElevator;
	DigitalInput lowElevator;
	
	double angleNow, scaledAngle; //the current angle and the angle to feed into the PID

	boolean goToHigh;
	boolean letUp2;
	
	Timer t;

	int autoID;
	double turnSetpoint;
	double positionSetpoint;
	
	double timeNow;
	boolean firstSample;

	public enum autoStates { 	Done,
						BasicDrive, DriveFarther,  							
						Turn, TurnOpposite, Turn45, TurnOpposite45, CrossField,
						InitialDrive, SwitchApproach, SwitchPlace, FarSideBackToSwitch, 
						NearScaleInitial, ScaleApproach, ScalePlace, FarSideOnToScale,  
						DriveToSwitchSide, InitialCenterL, SecondaryCenterL, FinalCenter,												
						InitialCenterR, SecondaryCenterR, raiseForSwitch, raiseForScale,// XXX Added new distance to allow straight move
																						// and cube placement on near Switch without turns
																						// Limited to starting Position 'M', with no alliance
																						// robot to our right
																			// Add steps here needed to pick up a cube from the floor (locate, 
																			// acquire, and stow it for driving) to allow for Stage 3 auto tasks; 
																			// will need additional autoIDs since getting a cube
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
	
	autoStates	auto[][] = new autoStates[23][10];			// first index is autoID, second index is place in list (autoStep)
	autoStates autoCase;
	
	private SendableChooser placeCube; 			//Do we place a cube?
	private SendableChooser startingLocation;	//Where do we start?
	private SendableChooser priorityType;		//Decide based on side or scale/switch
	private SendableChooser priorityPlace;		//Decide based on scale or switch
	private SendableChooser testMode;			//Are we in testing mode
	private SendableChooser testCase;			//What case are we testing
	private SendableChooser farSideAccess;		//Are we allowed to cross the field
													//(only in side pref)
	
	SRF_PID positionPID;
	SRF_PID turningPID;

	Encoder leftSide;
										
	double countsPerInch = 19.36;//18.96;
	double output;
	double turningMult;    		//the variable used to mirror our operations depending on robot placement
	
	int targetWristPos;			//variable for where the wrist will move to
	int targetElevatorPos; 		//variable for where the elevator will move to
	
	int turnDirection;
	
	//the variables for the constants in each PID loop
	//double upElevatorP = 0.0058, upElevatorI = 0.0000000000001, upElevatorD = 0.008;
	//double downElevatorP = 0.0035, downElevatorI = 0.000002, downElevatorD  = 0.003;
	double upElevatorP = 0.0058, upElevatorI = 0, upElevatorD = 0.001;//
	double downElevatorP = 0.0035, downElevatorI = 0, downElevatorD  = 0.000003;
	
	double baseUpElevatorP = upElevatorP, baseUpElevatorI = upElevatorI, baseUpElevatorD = upElevatorD;
	double baseDownElevatorP = downElevatorP, baseDownElevatorI = downElevatorI, baseDownElevatorD  = downElevatorD;
	
	double downWristP = 0.00512,  downWristI = 0.00004,  downWristD = 0.00096;//old comp
	//double   upWristP = 0.0297,   upWristI = 0.00187,     upWristD = 0.0000;
	double   upWristP = 0.032,   upWristI = 0.00008,     upWristD = 0.000015;
	
	double baseDownWristP = downWristP,  baseDownWristI = downWristI,  baseDownWristD = downWristD;
	double   baseUpWristP = upWristP,   baseUpWristI = upWristI,     baseUpWristD = upWristD;
	
	int elevatorUp = -1600000, elevatorMid = -475000, elevatorLowScale = -1370288, elevatorDown = -34000;//was -23500
	int    wristUp = -6000,        wristMiddle = -89250,     wristDown = -136000;
	int wristMode, elevatorMode;
	
	boolean manualCon = false; //Used to change whether using manual controls for teleop
	boolean letUp10;
	
	int up = 0, mid = 1, lowScale = 2, down = 3;
	
	public void robotInit() {
		highElevator = new DigitalInput(3);
		lowElevator = new DigitalInput(4);
		
		hook = new Victor(4);
		winch = new Victor(7);
		
		elevator = new TalonSRX(2);
		elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		elevator.setSelectedSensorPosition(0,0,0);
		elevator.config_kP(0, upElevatorP, 10); 
		elevator.config_kI(0, upElevatorI, 10);
		elevator.config_kD(0, upElevatorD, 10);
		elevator.configPeakOutputForward(0.8, 5);//1 is too big
		elevator.configPeakOutputReverse(-0.8, 5);
			
		wrist = new TalonSRX(1);
		wrist.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		wrist.setSelectedSensorPosition(0,0,0);
		wrist.config_kP(0, upWristP, 10); 
		wrist.config_kI(0, upWristI, 10);
		wrist.config_kD(0, upWristD, 10);
		wrist.configPeakOutputForward(0.5, 5);
		wrist.configPeakOutputReverse(-0.5, 5);
		
		intakeL = new Victor(5);
		intakeR = new Victor(6);
		
    	frontLeft = new Talon(2);
    	rearLeft = new Talon(3);
    	frontRight = new Talon(0);
    	rearRight = new Talon(1);
    	
    	robot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
    	xBox = new Joystick(0);
    	if(testing)tuningController = new Joystick(1);
    	
    	ahrs = new AHRS(SPI.Port.kMXP);
    	
    	t = new Timer();
    	t.start();
    	if(enableManual)
    	{
    		testTimer = new Timer();
    		testTimer.start();
    	}
		/*UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);
        camera.setExposureManual(45);
        camera.setFPS(30);*/
	
    	
    	//initialize choosers
    	placeCube = new SendableChooser();
    	placeCube.addObject("Place Cube", 1);
    	placeCube.addDefault("Drive", 2);
    	SmartDashboard.putData("placeCube", placeCube);

    	farSideAccess = new  SendableChooser();
    	farSideAccess.addDefault("Disable Cross Field", 0);
    	farSideAccess.addObject("Enable Cross Field", 1);
    	SmartDashboard.putData("farSideAccess", farSideAccess);
    	
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
    	testCase.addDefault("Basic Drive", 7);							
    	testCase.addObject("Initial Drive", 8);
    	testCase.addObject("Turn", 9);
    	testCase.addObject("Turn Opposite", 10);
    	testCase.addObject("Turn45", 21);
    	testCase.addObject("Switch Approach", 11);
    	testCase.addObject("Switch Place", 12);
    	testCase.addObject("Drive Farther", 13);
    	testCase.addObject("Near Scale Initial", 14);
    	testCase.addObject("Scale Place", 15);
    	testCase.addObject("Cross Field", 16);
    	testCase.addObject("FarSideBackToSwitch", 17);
    	testCase.addObject("Scale Approach", 18);
    	testCase.addObject("FarSideOnToScale", 19);
    	testCase.addObject("DriveToSwitchSide", 20);
    	SmartDashboard.putData("testCase", testCase);

    	positionPID = new SRF_PID();
    	positionPID.setLimits(0.7, -0.7);
    	positionPID.setReverse(true);
    	positionPID.setPID(positionP, positionI, positionD);
    	leftSide = new Encoder(0, 1);
    	
    	turningPID = new SRF_PID();
    	turningPID.setReverse(true);
    	turningPID.setLimits(0.9, -0.9);
    	turningPID.setPID(turningP, turningI, turningD);
    	
    	if (debug) System.out.println("End Robot Init");
    }

	
	
    /**
     * This function is run once each time the robot enters autonomous mode
     */

    public void autonomousInit() {
    	elevatorMode = down;
    	wristFlushed = false;
    	wrist.set(ControlMode.PercentOutput, 0.2);
		Timer.delay(.2);
		wrist.setSelectedSensorPosition(0, 0, 0);
		wrist.set(ControlMode.PercentOutput, 0);
    	
    	startTestAuto = false;
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

    	//initialize potential autonomous steps
    	auto[0][0] = autoStates.BasicDrive; //drive forward over the auto line
    	auto[0][1] = autoStates.Done;
		auto[0][2] = autoStates.NA;
		auto[0][3] = autoStates.NA;
		auto[0][4] = autoStates.NA;
		auto[0][5] = autoStates.NA;
		auto[0][6] = autoStates.NA;
		auto[0][7] = autoStates.NA;
		auto[0][8] = autoStates.NA;
		auto[0][9] = autoStates.NA;
		
    	auto[1][0] = autoStates.InitialDrive;		// Place cube on near switch
    	auto[1][1] = autoStates.Turn;
    	auto[1][2] = autoStates.SwitchApproach;
    	auto[1][3] = autoStates.SwitchPlace;
    	auto[1][4] = autoStates.Done;
		auto[1][5] = autoStates.NA;
		auto[1][6] = autoStates.NA;
		auto[1][7] = autoStates.NA;
		auto[1][8] = autoStates.NA;
		auto[1][9] = autoStates.NA;
		
    	auto[2][0] = autoStates.DriveFarther;		//place cube on far switch platform
    	auto[2][1] = autoStates.Turn;
    	auto[2][2] = autoStates.CrossField;
    	auto[2][3] = autoStates.Turn;
    	auto[2][4] = autoStates.FarSideBackToSwitch;
    	auto[2][5] = autoStates.Turn;
    	auto[2][6] = autoStates.SwitchApproach;		
    	auto[2][7] = autoStates.SwitchPlace;		
		auto[2][8] = autoStates.Done;
		auto[2][9] = autoStates.NA;
		
    	auto[3][0] = autoStates.NearScaleInitial;	//place cube on near scale platform
    	auto[3][1] = autoStates.Turn45;
    	auto[3][2] = autoStates.raiseForScale;
    	auto[3][3] = autoStates.ScaleApproach;
    	auto[3][4] = autoStates.ScalePlace;
		auto[3][5] = autoStates.Done;
		auto[3][6] = autoStates.NA;
		auto[3][7] = autoStates.NA;
		auto[3][8] = autoStates.NA;
		auto[3][9] = autoStates.NA;
		
    	auto[4][0] = autoStates.DriveFarther;		//place cube on far scale platform
    	auto[4][1] = autoStates.Turn;
    	auto[4][2] = autoStates.CrossField;
    	auto[4][3] = autoStates.TurnOpposite;
    	auto[4][4] = autoStates.ScaleApproach;
    	auto[4][5] = autoStates.ScalePlace;		
    	auto[4][6] = autoStates.Done;
    	auto[4][7] = autoStates.NA;
    	auto[4][8] = autoStates.NA;
    	auto[4][9] = autoStates.NA;
													// Don't worry about this and the following new autoID until District Championships
    	auto[5][0] = autoStates.DriveToSwitchSide;	// place cube on Near Switch platform, starting from 'M'
    	auto[5][1] = autoStates.SwitchPlace;
    	auto[5][2] = autoStates.Done;
    	auto[5][3] = autoStates.NA;
    	auto[5][4] = autoStates.NA;
    	auto[5][5] = autoStates.NA;
    	auto[5][6] = autoStates.NA;
    	auto[5][7] = autoStates.NA;
    	auto[5][8] = autoStates.NA;
    	auto[5][9] = autoStates.NA;
													
    	auto[6][0] = autoStates.InitialCenterL;		//place cube on left switch
    	auto[6][1] = autoStates.TurnOpposite45;
    	auto[6][2] = autoStates.SecondaryCenterL;
    	auto[6][3] = autoStates.Turn45;
    	auto[6][4] = autoStates.raiseForSwitch;
    	auto[6][5] = autoStates.FinalCenter;
    	auto[6][6] = autoStates.SwitchPlace;
    	auto[6][7] = autoStates.Done;
    	auto[6][8] = autoStates.NA;
    	auto[6][9] = autoStates.NA;
    	
    	auto[22][0] = autoStates.InitialCenterR;		//place cube on right switch
    	auto[22][1] = autoStates.Turn45;
    	auto[22][2] = autoStates.SecondaryCenterR;
    	auto[22][3] = autoStates.TurnOpposite45;
    	auto[22][4] = autoStates.raiseForSwitch;
    	auto[22][5] = autoStates.FinalCenter;
    	auto[22][6] = autoStates.SwitchPlace;
    	auto[22][7] = autoStates.Done;
    	auto[22][8] = autoStates.NA;
    	auto[22][9] = autoStates.NA;
		
		//testing sequences
 		auto[7][0] = autoStates.BasicDrive; 
    	auto[7][1] = autoStates.Done;
		auto[7][2] = autoStates.NA;
		auto[7][3] = autoStates.NA;
		auto[7][4] = autoStates.NA;
		auto[7][5] = autoStates.NA;
		auto[7][6] = autoStates.NA;
		auto[7][7] = autoStates.NA;
		auto[7][8] = autoStates.NA;		
		auto[7][9] = autoStates.NA;

 		auto[8][0] = autoStates.InitialDrive; 
    	auto[8][1] = autoStates.Done;
		auto[8][2] = autoStates.NA;
		auto[8][3] = autoStates.NA;
		auto[8][4] = autoStates.NA;
		auto[8][5] = autoStates.NA;
		auto[8][6] = autoStates.NA;
		auto[8][7] = autoStates.NA;
		auto[8][8] = autoStates.NA;
		auto[8][9] = autoStates.NA;

 		auto[9][0] = autoStates.Turn;
 		
 		auto[9][1] = autoStates.Done;
		auto[9][2] = autoStates.NA;
		auto[9][3] = autoStates.NA;
		auto[9][4] = autoStates.NA;
 		
    	/*auto[9][1] = autoStates.Turn;
		auto[9][2] = autoStates.Turn;
		auto[9][3] = autoStates.Turn;
		auto[9][4] = autoStates.Done;
		
		*/auto[9][5] = autoStates.NA;
		auto[9][6] = autoStates.NA;
		auto[9][7] = autoStates.NA;
		auto[9][8] = autoStates.NA;		
		auto[9][9] = autoStates.NA;

 		auto[10][0] = autoStates.TurnOpposite; 
    	auto[10][1] = autoStates.TurnOpposite;
		auto[10][2] = autoStates.TurnOpposite;
		auto[10][3] = autoStates.TurnOpposite;
		auto[10][4] = autoStates.Done;
		auto[10][5] = autoStates.NA;
		auto[10][6] = autoStates.NA;
		auto[10][7] = autoStates.NA;
		auto[10][8] = autoStates.NA;		
		auto[10][9] = autoStates.NA;

 		auto[11][0] = autoStates.SwitchApproach; 
    	auto[11][1] = autoStates.Done;
		auto[11][2] = autoStates.NA;
		auto[11][3] = autoStates.NA;
		auto[11][4] = autoStates.NA;
		auto[11][5] = autoStates.NA;
		auto[11][6] = autoStates.NA;
		auto[11][7] = autoStates.NA;
		auto[11][8] = autoStates.NA;		
		auto[11][9] = autoStates.NA;
		
 		auto[12][0] = autoStates.SwitchPlace; 
    	auto[12][1] = autoStates.Done;
		auto[12][2] = autoStates.NA;
		auto[12][3] = autoStates.NA;
		auto[12][4] = autoStates.NA;
		auto[12][5] = autoStates.NA;
		auto[12][6] = autoStates.NA;
		auto[12][7] = autoStates.NA;
		auto[12][8] = autoStates.NA;
		auto[12][9] = autoStates.NA;

 		auto[13][0] = autoStates.DriveFarther; 
    	auto[13][1] = autoStates.Done;
		auto[13][2] = autoStates.NA;
		auto[13][3] = autoStates.NA;
		auto[13][4] = autoStates.NA;
		auto[13][5] = autoStates.NA;
		auto[13][6] = autoStates.NA;
		auto[13][7] = autoStates.NA;
		auto[13][8] = autoStates.NA;		
		auto[13][9] = autoStates.NA;

 		auto[14][0] = autoStates.NearScaleInitial; 
    	auto[14][1] = autoStates.Done;
		auto[14][2] = autoStates.NA;
		auto[14][3] = autoStates.NA;
		auto[14][4] = autoStates.NA;
		auto[14][5] = autoStates.NA;
		auto[14][6] = autoStates.NA;
		auto[14][7] = autoStates.NA;
		auto[14][8] = autoStates.NA;		
		auto[14][9] = autoStates.NA;

 		auto[15][0] = autoStates.ScalePlace; 
    	auto[15][1] = autoStates.Done;
		auto[15][2] = autoStates.NA;
		auto[15][3] = autoStates.NA;
		auto[15][4] = autoStates.NA;
		auto[15][5] = autoStates.NA;
		auto[15][6] = autoStates.NA;
		auto[15][7] = autoStates.NA;
		auto[15][8] = autoStates.NA;
		auto[15][9] = autoStates.NA;

 		auto[16][0] = autoStates.CrossField; 
    	auto[16][1] = autoStates.Done;
		auto[16][2] = autoStates.NA;
		auto[16][3] = autoStates.NA;
		auto[16][4] = autoStates.NA;
		auto[16][5] = autoStates.NA;
		auto[16][6] = autoStates.NA;
		auto[16][7] = autoStates.NA;
		auto[16][8] = autoStates.NA;		
		auto[16][9] = autoStates.NA;

 		auto[17][0] = autoStates.FarSideBackToSwitch; 
    	auto[17][1] = autoStates.Done;
		auto[17][2] = autoStates.NA;
		auto[17][3] = autoStates.NA;
		auto[17][4] = autoStates.NA;
		auto[17][5] = autoStates.NA;
		auto[17][6] = autoStates.NA;
		auto[17][7] = autoStates.NA;
		auto[17][8] = autoStates.NA;		
		auto[17][9] = autoStates.NA;

 		auto[18][0] = autoStates.ScaleApproach; 
    	auto[18][1] = autoStates.Done;
		auto[18][2] = autoStates.NA;
		auto[18][3] = autoStates.NA;
		auto[18][4] = autoStates.NA;
		auto[18][5] = autoStates.NA;
		auto[18][6] = autoStates.NA;
		auto[18][7] = autoStates.NA;
		auto[18][8] = autoStates.NA;		
		auto[18][9] = autoStates.NA;
		
 		auto[19][0] = autoStates.FarSideOnToScale; 
    	auto[19][1] = autoStates.Done;
		auto[19][2] = autoStates.NA;
		auto[19][3] = autoStates.NA;
		auto[19][4] = autoStates.NA;
		auto[19][5] = autoStates.NA;
		auto[19][6] = autoStates.NA;
		auto[19][7] = autoStates.NA;
		auto[19][8] = autoStates.NA;		
		auto[19][9] = autoStates.NA;
		
		auto[20][0] = autoStates.DriveToSwitchSide; 
    	auto[20][1] = autoStates.Done;
		auto[20][2] = autoStates.NA;
		auto[20][3] = autoStates.NA;
		auto[20][4] = autoStates.NA;
		auto[20][5] = autoStates.NA;
		auto[20][6] = autoStates.NA;
		auto[20][7] = autoStates.NA;
		auto[20][8] = autoStates.NA;	
		auto[20][9] = autoStates.NA;
		
		auto[21][0] = autoStates.Turn45; 
    	auto[21][1] = autoStates.Turn45;
		auto[21][2] = autoStates.Turn45;
		auto[21][3] = autoStates.Turn45;
		auto[21][4] = autoStates.Done;
		auto[21][5] = autoStates.NA;
		auto[21][6] = autoStates.NA;
		auto[21][7] = autoStates.NA;
		auto[21][8] = autoStates.NA;	
		auto[21][9] = autoStates.NA;
		
		
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
    		if(ourSide == 'M')
    		{
    			if(switchSide == 'L')
    				autoID = 6;
    			else
    				autoID = 22;
    		}
    		else if((int) priorityType.getSelected() == 1) //if looking for closest side
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
    					else if((int) farSideAccess.getSelected() == 1)//go to the far scale
    						autoID = 4;									//if we can
    					else
    						autoID = 0;
    				}
    				else // going to the switch (default mode - override to scale via sendable chooser for playoffs)
    				{
    					if(switchSide == ourSide)	//if the switch is on our side
    						autoID = 1;
    					else if((int) farSideAccess.getSelected() == 1)//go to far switch if we
    						autoID = 2;									//are allowed to
    					else
    						autoID = 0;
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
		
 		//initialize turning direction multiplier
		if(ourSide == 'R')
    		turningMult = -1;
    	else
    		turningMult = 1;
    	
    	ahrs.reset();
		turnSetpoint = 0;
		
    	leftSide.reset();
    	if(debug)System.out.println(516);
    	elevator.setSelectedSensorPosition(0, 0, 0);
    	wristMode = up;
		
    	targetWristPos = wristUp;
    	targetElevatorPos = elevatorDown;

    	autoStep = -1;
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
		positionPID.setLimits(0.4, -0.4);
		safetyTimeout = ((targetDistance/12)/(maxFeetPerSec*0.2)+.5); // Set drive to position safety timeouts by using .2 of max speed 															  
		// Data - to travel 10 feet takes approx 2 seconds with 2 CIM drive
		// to travel 24 feet takes approx 4 seconds with 2 CIM drive
		// Both observations were with the throttle restricted to .9
		// Max effective speed is thus about 6 feet per sec, or 1/3 of theoretical max of 16 ft/sec. Use .2 factor to avoid early terminations
																	  // To this add a half second fudge factor. 
		positionPID.setPID(positionP, positionI, positionD);//this could be moved to autonomousInit() (or RobotInit())
	}
	
	
	
	public void advanceToNextAutoCase() //correct these values. May be mirrored by turningMult.
	{
		targetTurn = 0;
		setpointReached = false;
		intakeL.set(-0.2);
		intakeR.set(-0.2);
		targetWristPos = wristUp;
		wristMode = up;
		
		turnDirection = 1; //Used to control CW or CCW turns
	
		if(debug) System.out.println("stopping robot before new case");
		robot.arcadeDrive(0,0);							// cancel any residual motion of robot.
		
		lastCaseEndingEncoderCount = leftSide.get();
		lastCaseEndingTime = t.get();
		lastCaseEndingHeading = ahrs.getAngle();
		
		withinRange = false;
		timeReached = 0;
		
		if (debug) System.out.println("autoStepAdvance");
		
		autoStep++;
		if ((autoID < 0) || (autoID > 22) || (autoStep < 0) || (autoStep > 9))//error checks on array indices
			autoCase = autoStates.NA;
		else
			autoCase = auto[autoID][autoStep];
			
		safetyTimeout = 5;				// Just in case SafetyTimeout is NOT specifically set by the new case,
										// (which is usually done) be sure a reasonable default timeout is set.
		
		switch(autoCase)
		{
			case InitialCenterL:
				prepareToMove(22.29);
				break;
			case InitialCenterR:
				prepareToMove(22.45);
				break;
			case SecondaryCenterL:
				prepareToMove(85);
				break;
			case SecondaryCenterR:
				prepareToMove(67.75);
				break;
			case raiseForSwitch:
				safetyTimeout = 5;
				elevator.set(ControlMode.Position, elevatorMid);
				targetElevatorPos = elevatorMid;
				elevatorMode = mid;
				break;
			case raiseForScale:
				safetyTimeout = 5;
				targetElevatorPos = elevatorUp;
				elevator.set(ControlMode.Position, elevatorUp);
				elevatorMode = up;
				break;
			case FinalCenter:
				prepareToMove(18);
				break;
			case TurnOpposite:
				turnDirection = -1;
			case Turn:
				targetTurn = 90;
				turnSetpoint += (90*turnDirection*turningMult);
				if(testing)
					safetyTimeout = 30;
				else
					safetyTimeout = 2;
				turningPID.setSetpoint(turnSetpoint);
				break;
			case TurnOpposite45:
				turnDirection = -1;
			case Turn45:
				targetTurn = 45;
				turnSetpoint+=(45*turningMult*turnDirection);
				if(testing)
					safetyTimeout = 30;
				else
					safetyTimeout = 2;
				turningPID.setSetpoint(turnSetpoint);
				break;
			case BasicDrive:
				safetyTimeout = 2.5;
				break;
			case InitialDrive:		//Drive forward on the current side, in preparation to placing cube on switch
				prepareToMove(148.5);
				break;
			case SwitchApproach:
				prepareToMove(18.5);
				break;
			case DriveFarther:
				prepareToMove(216);
				break;
			case NearScaleInitial:
				prepareToMove(256);
				break;
			//REMOVED BECAUSE WE'RE ALREADY REALLY CLOSE
			case ScaleApproach: //Adjust position relative to near scale plate
				prepareToMove(11);		//make biggerand use for approaching the side of the
				break;
			case CrossField:
				prepareToMove(230);
				break;
			case FarSideOnToScale:		// Removed?
				prepareToMove(89);
				break;
			case FarSideBackToSwitch:		// XXX New - for cross field autonomous
				prepareToMove(67.25);
				break;
			case DriveToSwitchSide:
				prepareToMove(96);			// XXX Allows 4.5" clearance for cube projection. Actual distance 8' 4.75"
				break;
			case SwitchPlace:	//Place cube being carried on switch
				wrist.config_kP(0, downWristP, 10); 
	    		wrist.config_kI(0, downWristI, 10);
	    		wrist.config_kD(0, downWristD, 10);
				elevator.set(ControlMode.Position, elevatorMid);
				if(!testing)
					safetyTimeout = 4;
				else
					safetyTimeout = 30;
				break;
			case ScalePlace:  	//Place cube being carried on the scale
				wrist.config_kP(0, downWristP, 10); 
	    		wrist.config_kI(0, downWristI, 10);
	    		wrist.config_kD(0, downWristD, 10);
				safetyTimeout = 6;
				break;
			case NA:
				System.out.println("Unexpected NA case in advanceToNextAutoCase");	
				intakeL.set(0);
				intakeR.set(0);
				elevator.set(ControlMode.PercentOutput,0);			// XXX Why not move elevator and wrist to default positions here,
				wrist.set(ControlMode.PercentOutput,0);				// instead of just turning them off?
				break;
			case Done:
				intakeL.set(0);
				intakeR.set(0);
				//elevator.set(ControlMode.Position,elevator.getSelectedSensorPosition(0));
				//wrist.set(ControlMode.Position,wrist.getSelectedSensorPosition(0));
				break;
			default:
				System.out.println("Unexpected default case in advanceToNextAutoCase");	
				break;
		}
		
		firstSample = true;	//used to sample the start time of cube placement actions

		if(autoCase!=autoStates.Done) 
		{
			t.reset();
			leftSide.reset();
		}
		
		if (debug) System.out.println("637:"+autoID+":"+autoStep+":"+autoCase);

		positionArrivalThreshold = 1;	// If within one inch, for 100 ms, we're there.
										// Threshold is increased to 3 if safetyTimeout occurs.
		potentiallyArrived = false;		// flag for use in ending PID controlled autoSteps.
		startTime = t.get();			// startTime allows for "safety" timeouts. 
	}
	
	
/**
 * This function is called periodically during autonomous
 */

	public void autonomousPeriodic()
	{
		SmartDashboard.putNumber("leftSide", leftSide.get());
		
		currentTime = t.get();
		currentAngle = ahrs.getAngle();
		currentPosition = leftSide.get();
		
		//cycle counter
		counter++;

		if (debug) System.out.println("658:"+autoID+":"+autoStep+":"+autoCase);

		switch(autoCase)				// For higher level autonomous behavior, consider creating a separate 
										// staging monitor or "executive", and when any autoID is "Done", don't quit before calling that
										// Executive so it can decide if a new autoID task should be started. That executive would 
										// consider various factors: time left in auto? Do we control our switch? Do we control our Scale? 
										// Is this a qual or playoff match? It should then set autoID (and autoStep) appropriately, 
										// store whatever states or flags that are needed, and return. The switch/case here then 
										// continues to serve just as before, sequencing though the steps of the specific new autoID 
										// that the executive decided on.
		{
			case BasicDrive: //Just drive forward
				robot.arcadeDrive(-0.65,ahrs.getAngle()*.03);//we don't need turnSetpoint because it
															//starts at 0
				if(debug){
					System.out.println(currentTime+":"+startTime+":"+safetyTimeout);
				}
				if((currentTime - startTime) > safetyTimeout)
				{
					if (debug) System.out.println("End BD");
					advanceToNextAutoCase();			
				}
				break;
			case SwitchApproach:
				targetElevatorPos = elevatorMid;
				elevatorMode = mid;
			case ScaleApproach:
			case InitialCenterL:
			case InitialCenterR:
			case SecondaryCenterL:
			case SecondaryCenterR:
			case FinalCenter:
			case InitialDrive:
			case DriveFarther:
			case NearScaleInitial:				
			case CrossField:
			case FarSideBackToSwitch:
			case FarSideOnToScale:
			case DriveToSwitchSide:
				
				if(leftSide.get() > 8*countsPerInch)
					positionPID.setLimits(0.8, -0.8);
				else if(leftSide.get() > 4*countsPerInch)
					positionPID.setLimits(0.6, -0.6);
				else
					positionPID.setLimits(0.4, -0.4);
				
				if(!potentiallyArrived)
				{
					//positionPID.setLimits(0.5,-0.5);
					output = positionPID.computePID(currentPosition, currentTime);
					directionControl = -0.3-(turnSetpoint - ahrs.getAngle())/navXCorrect;
					//directionControl = 0;
				
					robot.arcadeDrive(output, directionControl);
				}
				else
				{
					if(currentPosition < positionSetpoint)
					{
						if(setpointReached)
							advanceToNextAutoCase();
						else
							robot.arcadeDrive(-0.35, 0);
					}
					else
					{
						setpointReached = true;
						robot.arcadeDrive(0.35, 0);
					}
				}
				
				if(debug && (counter % 20 == 0))
					System.out.println(output+"::"+positionSetpoint+":"+currentPosition+"::"+currentTime);

				// 	if(positionSetpoint-currentPosition<48*countsPerInch)
				//		positionPID.setPID(positionCloseP, positionCloseI, positionCloseD);

				// If safety timeout exceeded, pretend we are close, set potentiallyArrived flag and arrivalTime normally, but set 
				// special values for tolerance and settle timeout.
				// Otherwise, test for normal arrival such that encoder > (setPoint less 3" allowance), 
				// in which case again set the "potentiallyArrived" flag, the arrival time, and the normal tolerance and settle timeout.
				// Once potentiallyArrived is set, safetyTimeout is no longer checked, only settleTime.
				// 1 second should be more than enough PID settle time, if tuned well.
				// We then test for position being within one inch for 100 ms. If so, advance to next autoStep.

				// To cover the case where the setPoint is never actually crossed (because the robot stops short),
				// also test for being within a small threshold distance of the positionSetpoint (1" ?)
				// If a timeout occurs without meeting setpoint threshold, something is wrong. Make a last ditch attempt to salvage 
				// something by enlarging the thresholds. If still no luck, advance the state to "N/A" (assumed to always be
				// stored as the last autoStep index) to avoid unknown difficulties (and maybe arena fouls).

				if (! potentiallyArrived)
				{
					if ((currentTime-startTime) > safetyTimeout)	//safety timeout now triggers potential arrival as last chance try
					{
						autoStep = 8;
						advanceToNextAutoCase();
						// Always print on timeout condition, DEBUG defined or not!
						/*System.out.println("Pos safety timeout "+autoID+":"+autoStep+" "+positionSetpoint+" : "+currentPosition+" :: "+(currentTime-startTime));			
						potentiallyArrived = true;
						arrivalTime = currentTime;
						settleTime = 0.5;					// On safety timeout, be more stingy on settle time allowed,
						positionArrivalThreshold = 3;		// but be more generous on allowed tolerance.
						withinRange = false;*/
					}//3
					else if (currentPosition > positionSetpoint*0.5)//if the robot is 3 inches short
					{
						potentiallyArrived = true;
						/*arrivalTime = currentTime;
						settleTime = 2;					// On regular arrival trigger, be more generous on settle time,
						arrivalThreshold = 1;	// but more stingy on allowed tolerance.
						withinRange = false;*/
					}
				} /*else
				{								// "potentiallyArrived" flag is now true, so at the least we are close. 
												// Test for final arrival (within threshold for more than 100 ms),
												// and end case if so.
												// But first check if settle time has expired, and end if so
					/*if((currentTime - arrivalTime) > settleTime)
					{
						System.out.println("Pos settle timeout "+autoID+":"+autoStep+" "+positionSetpoint+" : "+currentPosition+" :: "+(currentTime-startTime));	
						if (Math.abs(currentPosition-positionSetpoint)>(6*countsPerInch)) // On settle timeout, increase tolerance to 6"!
																						  // Risky, but it might pay off!
							autoStep = 8;												  // However, if greater than 6" off, abandon goal.
						advanceToNextAutoCase();										  // Either way, we're finally done here.
					} else if (! withinRange )
					{	// We think we are out of range, see if the PID has taken us into (or back into) range.
						if (Math.abs(currentPosition-positionSetpoint)<=(arrivalThreshold*countsPerInch))
						{
							withinRange=true;
							timeReached = currentTime;
						}
					} else {					// we were withinRange earlier, see if we still are
						if (Math.abs(currentPosition-positionSetpoint) > (arrivalThreshold*countsPerInch)) {
							withinRange=false;	// Nope, clear the flag and try again next time through the periodic loop
						} else if ((currentTime-timeReached) > .1)
						{						// Yes, we're still within allowed tolerance, and have been so for 100 ms. or more.
												// Call it successfully done!
							advanceToNextAutoCase();
							if (debug) System.out.println("End "+autoID+":"+autoStep+" "+output+" :: "+positionSetpoint+" : "+currentPosition+" :: "+currentTime+" : "+arrivalTime);
						}
					}
				}*/
				break;
			case TurnOpposite:
			case Turn:
			case TurnOpposite45:
			case Turn45:
				//if you haven't gone half way then compute the PID
				if(Math.abs(currentAngle - turnSetpoint) > targetTurn*0.5)
				{
					directionControl = turningPID.computePID(currentAngle, currentTime);
					robot.arcadeDrive(0, directionControl);
				}
				else
				{
					if(turnDirection*turningMult==1)//turning right
					{
						if(currentAngle < turnSetpoint)
						{
							if(setpointReached)
								advanceToNextAutoCase();
							else
								robot.arcadeDrive(0, -0.6);
						}
						else
						{
							setpointReached = true;
							robot.arcadeDrive(0, 0.55);
						}
					}
					else
					{
						if(currentAngle > turnSetpoint)
						{
							if(setpointReached)
								advanceToNextAutoCase();
							else
								robot.arcadeDrive(0, 0.6);
						}
						else
						{
							setpointReached = true;
							robot.arcadeDrive(0, -0.55);
						}
					}
				}
				break;
			case SwitchPlace: //Place cube
				targetWristPos = wristDown;
				wristMode = down;
				
				// see if we're ready to place
				if((Math.abs(elevator.getSelectedSensorPosition(0)-elevatorMid) < 25000)
				   && 
				   (Math.abs(wrist.getSelectedSensorPosition(0)-wristDown) < 10000))
				{
					if(firstSample)
					{
						firstSample = false;
						timeReached = currentTime;
						if (debug) System.out.println("Ready to place cube " + timeReached);
					}
					
					intakeL.set(0.5);
					intakeR.set(0.5);
					
					if(currentTime - timeReached > 1)
						advanceToNextAutoCase();
				}
				
				if (currentTime - startTime > safetyTimeout)
				{
					autoStep = 8;
					advanceToNextAutoCase();
				}
				break;
			case ScalePlace: //place a cube on the scale
				//if(Math.abs(elevator.getSelectedSensorPosition(0)-elevatorMid) < 25000)
				//{
					targetWristPos = wristDown;
					
					if(Math.abs(wrist.getSelectedSensorPosition(0)-wristDown) < 25000)
					{
						if(firstSample)
						{
							firstSample = false;
							timeReached = currentTime;
						}
							
						intakeL.set(0.5);
						intakeR.set(0.5);
						
						if(currentTime - timeReached > 1)
							advanceToNextAutoCase();
					}
				//}
				
				if((currentTime - startTime) > safetyTimeout)
				{
					autoStep = 8;
					advanceToNextAutoCase();
				}
				break;
			case raiseForSwitch:
				/*if(currentTime - startTime > safetyTimeout)
				{
					autoStep = 8;
					advanceToNextAutoCase();
				}
				else */if(Math.abs(elevator.getSelectedSensorPosition(0)-elevatorMid) < 25000)
					advanceToNextAutoCase();
				break;
			case raiseForScale:
				/*if(currentTime - startTime > safetyTimeout)
				{
					autoStep = 8;
					advanceToNextAutoCase();
				}
				else */if(Math.abs(elevator.getSelectedSensorPosition(0)-elevatorUp) < 25000)
					advanceToNextAutoCase();
				break;
			case NA: //the case after Done; nothing to do, but generally unexpected.
				if(debug && (counter % 20 == 0)) System.out.println("The state \"NA\" has been reached");
				// intentional fall through here
			case Done: //Do nothing
				robot.arcadeDrive(0,0);
				startTestAuto = true;
				runningTestAuto = false;
			default:
				break;
		}
		
		wrist.set(ControlMode.Position, targetWristPos);
		
		if(elevatorMode == down && (highElevator.get() || lowElevator.get()))
		{
			if(lowElevator.get() && highElevator.get())
	    	{
	    		elevator.setSelectedSensorPosition(-34000, 0, 10);
	    		elevator.set(ControlMode.PercentOutput, -0.005);
	    	}
	    	else if(lowElevator.get())
	    		elevator.set(ControlMode.PercentOutput, -0.15);
	    	else if(highElevator.get())
	    		elevator.set(ControlMode.PercentOutput, 0.15);
		}
		else
		{
			elevator.set(ControlMode.Position, targetElevatorPos);
		}
			
		SmartDashboard.putNumber("autoID", autoID);
		SmartDashboard.putNumber("positionSetpoint", positionSetpoint);
		SmartDashboard.putNumber("directionControl", directionControl);
		SmartDashboard.putNumber("leftSide", leftSide.get());
		SmartDashboard.putNumber("ahrs", ahrs.getAngle());
		SmartDashboard.putNumber("t", currentTime);
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
    	if(testing)
    	{
    		wrist.set(ControlMode.PercentOutput, 0.4);
    		Timer.delay(.2);
    		wrist.setSelectedSensorPosition(0, 0, 0);//take these out
    		wrist.set(ControlMode.PercentOutput, 0);
    		elevator.setSelectedSensorPosition(0, 0, 0);//take these out
    	}
    	targetWristPos = wristUp;
    	targetElevatorPos = elevatorDown;
		wrist.set(ControlMode.Position, 0);
		ahrs.reset();
		letUp10 = true;
		manualCon = false;
		manualWrist = false;
		wristMode = up;
		letUp2 = true;
		goToHigh= true;
		elevatorMode = down;
		setElevatorDown = false;
		letUp4 = true;
		letUp5 = true;
		letUp6 = true;
		changedGain = false;
		runningTestAuto = false;
		startTestAuto = true;
		multMode = 1;
    }


    /**
     * This function is called periodically during operator control
     */

    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj.IterativeRobot#teleopPeriodic()
     */

    public void teleopPeriodic()
    {
    	if(!manualCon || !enableManual)
    	{	
	    	if(!xBox.getRawButton(9) && Math.abs(xBox.getRawAxis(1)) > 0.2 || Math.abs(xBox.getRawAxis(0)) > 0.2)
	    	{
	    		if(elevatorMode == down)
	    			robot.arcadeDrive(xBox.getRawAxis(1), -xBox.getRawAxis(0));
	    		else
	    			robot.arcadeDrive(xBox.getRawAxis(1)*0.7, -xBox.getRawAxis(0));
	    	}
	    	else
	    		 robot.arcadeDrive(0,0);
	    	
	    	
	    	//cube intake
	    	if(xBox.getRawAxis(2) > 0.2)//intake
	    	{
	    		intakeL.set(-xBox.getRawAxis(2));
	    		intakeR.set(-xBox.getRawAxis(2));
	    	}
	    	else if(xBox.getRawAxis(3) > 0.2)//eject
	    	{
	    		intakeL.set(xBox.getRawAxis(3)/2);
	    		intakeR.set(xBox.getRawAxis(3)/2);
	    	}
	    	else
	    	{
	    		intakeL.set(-0.2);
	    		intakeR.set(-0.2);
	    	}
	    	
	    	//test wrist code//
	    	if(xBox.getRawButton(3) && !manualWrist && wristMode!=up)//X
	    	{
	    		wristFlushed = false;
	    		wrist.config_kP(0, upWristP, 10); 
	    		wrist.config_kI(0, upWristI, 10);
	    		wrist.config_kD(0, upWristD, 10);
	    		targetWristPos = wristUp;
	    		wristMode = up;
	    	}
	    	else if(xBox.getRawButton(8))//Start
	    	{
	    		wristFlushed = false;
	    		if(!manualWrist && letUp8)
	    			manualWrist = true;
	    		else if(letUp8)
	    			manualWrist = false;
	    		
	    		letUp8 = false;
	    	}
	    	else if(xBox.getRawButton(7) && !manualWrist &&wristMode!=down)//Back
	    	{
	    		wrist.config_kP(0, downWristP, 10); 
	    		wrist.config_kI(0, downWristI, 10);
	    		wrist.config_kD(0, downWristD, 10);
	    		targetWristPos = wristDown;
	    		wristMode = down;
	    	}
	    	
	    	if(wristMode == down && !wristFlushed && wrist.getSelectedSensorPosition(0) < -110000)
	    	{
	    		wristFlushed = true;
	    		wrist.setIntegralAccumulator(0, 0, 0);
	    	}
	    	
	    	if(debug)System.out.println(":"+manualWrist);
	    		
	    	if(!manualWrist)
	    		wrist.set(ControlMode.Position, targetWristPos);
	    	else
	    	{
	    		manualWristPower = xBox.getRawAxis(5);
	    		
	    		if(Math.abs(manualWristPower) > 0.2)
	    		{
	    			if(manualWristPower > 0 && wrist.getSelectedSensorPosition(0) > wristUp)
	    				wrist.set(ControlMode.PercentOutput, 0);
	    			else if(manualWristPower < 0 && wrist.getSelectedSensorPosition(0) < wristDown)
	    				wrist.set(ControlMode.PercentOutput, 0);
	    			else
	    				wrist.set(ControlMode.PercentOutput, manualWristPower);
	    		}
	    		else
	    			wrist.set(ControlMode.PercentOutput, 0);
	    	}

	    	if(debug)System.out.println("814:"+xBox.getRawButton(4));
	    	//Elevator code
	    	if(xBox.getRawButton(4) /*&& letUp4*/)//Y
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
	    	else if(xBox.getRawButton(2) && letUp2 && elevatorMode == down)//B
	    	{
	    		letUp2 = false;
	    	/*	if(elevatorMode == up/* || elevatorMode == lowScale*//*)
	    		{
	    			if(debug)System.out.println("dropping to mid");
	    			elevator.config_kP(0, downElevatorP, 10); 
	       			elevator.config_kI(0, downElevatorI, 10);
	       			elevator.config_kD(0, downElevatorD, 10);
	    		}*/
	    	//	else
	    	//	{
	    			elevator.config_kP(0, upElevatorP, 10); 
	        		elevator.config_kI(0, upElevatorI, 10);
	        		elevator.config_kD(0, upElevatorD, 10);
	    	//	}
	    		
	    		elevatorMode = mid;
	    		targetElevatorPos = elevatorMid;
	    	}
	   		else if(xBox.getRawButton(1))//A
	   		{
	  			if(elevatorMode == up)
	   			{
	   				setElevatorDown = true;
	   				elevator.config_kI(0, 0, 10);
	   				elevator.config_kD(0, 0, 10);
	   			}
	   			else
	   			{
	   				elevator.config_kP(0, downElevatorP, 10); 
	   				elevator.config_kI(0, downElevatorI, 10);
	   				elevator.config_kD(0, downElevatorD, 10);
	   			}
	  			elevatorMode = down;
	   			targetElevatorPos = elevatorDown; //go to bottom
	   		}
	    	
	    	if(setElevatorDown && elevator.getSelectedSensorPosition(0) > elevatorUp / 2)
	    	{
	    		setElevatorDown = false;
	    		elevator.config_kP(0, downElevatorP, 10); 
   				elevator.config_kI(0, downElevatorI, 10);
   				elevator.config_kD(0, downElevatorD, 10);
	    	}
	    	
	    	if((!lowElevator.get() && !highElevator.get()) || elevatorMode != down)
	    		elevator.set(ControlMode.Position, targetElevatorPos);	    	
	    	else if(lowElevator.get() && highElevator.get())
	    	{
	    		elevator.setSelectedSensorPosition(-34000, 0, 10);
	    		elevator.set(ControlMode.PercentOutput, -0.005);
	    	}
	    	else if(lowElevator.get())
	    		elevator.set(ControlMode.PercentOutput, -0.15);
	    		//elevator.set(ControlMode.PercentOutput, -0.00000294*Math.abs(elevator.getSelectedSensorPosition(0)+34000));
	    	else if(highElevator.get())
	    		//elevator.set(ControlMode.PercentOutput, 0.00000294*Math.abs(elevator.getSelectedSensorPosition(0)+34000));
	    		elevator.set(ControlMode.PercentOutput, 0.15);
	    		
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
	    		testLoopCount = 0;
	    		letUp10 = false;
	    		manualCon = true;
	    		manualWrist = false;
	    	}   
    	}
	    else
	    {
	    	//manual elevator controls
	    	if(pidChoice != 2 || pidChoice != 3)
	    	{
	    		if(Math.abs(xBox.getRawAxis(5)) > 0.2 && !xBox.getRawButton(10)) //right stick y-axis
	    			elevator.set(ControlMode.PercentOutput, xBox.getRawAxis(5));
	    		else
	    			elevator.set(ControlMode.PercentOutput, -0.1);
	    	}
	    	
	    	//cube intake
	    	if(xBox.getRawAxis(3) > 0.2)//intake
	    	{
	    		intakeL.set(-.5);
	    		intakeR.set(-.5);
	    	}
	    	else if(xBox.getRawAxis(2) > 0.2 && (wristMode != up || (wristMode == up && manualWrist)))//eject
	    	{
	    		if(elevatorMode == down)
	    		{
	    			intakeL.set(xBox.getRawAxis(2));
	    			intakeR.set(xBox.getRawAxis(2));
	    		}
	    		else
	    		{
	    			intakeL.set(xBox.getRawAxis(2)*0.75);
	    			intakeR.set(xBox.getRawAxis(2)*0.75);
	    		}
	    	}
	    	else
	    	{
	    		intakeL.set(0);
	    		intakeR.set(0);
	    	}
		   
		    if(xBox.getRawButton(5) && letUp5)//LB
		    {
		    	letUp5 = false;
		    	if(pidGain == 1)
		    		currentMultP-=(0.1*multMode);
		    	else if(pidGain == 2)
		    		currentMultI-=(0.1*multMode);
		    	else if(pidGain == 3)
		    		currentMultD-=(0.1*multMode);
		    	changedGain = true;
		    }
		    else if(xBox.getRawButton(6) && letUp6)//RB
		    {
		    	letUp6 = false;
		    	if(pidGain==1)
		    		currentMultP+=(0.1*multMode);
		    	else if(pidGain==2)
		    		currentMultI+=(0.1*multMode);
		    	else if(pidGain==3)
		    		currentMultD+=(0.1*multMode);
		    	changedGain = true;
		    }
		    	
		    if(changedGain)
		    {
		    	changedGain = false;
		    	if(pidChoice == 1)//wristDown
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * baseDownWristP;
		    			downWristP = currentPValue;
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * baseDownWristI;
		    			downWristI = currentIValue;
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * baseDownWristD;
		    			downWristD = currentDValue;
		    		}
		    		else
		    			System.out.println("Problem with with gain choice:"+pidGain);
		    	}
		    	else if(pidChoice == 2)//elevatorUp
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * baseUpElevatorP;
		    			upElevatorP = currentPValue;
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * baseUpElevatorI;
		    			upElevatorI = currentIValue;
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * baseUpElevatorD;
		    			upElevatorD = currentDValue;
		    		}
		    		else
		    			System.out.println("Problem with with gain choice:"+pidGain);
		    	}
		    	else if(pidChoice == 3)//elevatorDown
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * baseDownElevatorP;
		    			downElevatorP = currentPValue;
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * baseDownElevatorI;
		    			downElevatorI = currentIValue;
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * baseDownElevatorD;
		    			downElevatorD = currentDValue;
		    		}
		    		else
		    			System.out.println("Problem with with gain choice:"+pidGain);
		    	}
		    	else if(pidChoice == 4)//positionPID
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * basePositionP;
		    			positionP = currentPValue;
		    			positionPID.setP(positionP);
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * basePositionI;
		    			positionI = currentIValue;
		    			positionPID.setI(positionI);
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * basePositionD;
		    			positionD = currentDValue;
		    			positionPID.setD(positionD);
		    		}
		    		else
		    			System.out.println("Problem with with gain choice:"+pidGain);
		    	}
		    	else if(pidChoice == 5)//wristUp
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * baseUpWristP;
		    			upWristP = currentPValue;
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * baseUpWristI;
		    			upWristI = currentIValue;
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * baseUpWristD;
		    			upWristD = currentDValue;
		    		}
		    		else
		    			System.out.println("Problem with with gain choice"+pidGain);
		    	}
		    	else if(pidChoice == 6)//turningPID
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * baseTurningP;
		    			turningPID.setP(currentPValue);
		    			turningP = currentPValue;
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * baseTurningI;
		    			turningPID.setI(currentIValue);
		    			turningI = currentIValue;
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * baseTurningD;
		    			turningPID.setD(currentDValue);
		    			turningD = currentDValue;
		    		}
		    		else
		    			System.out.println("Problem with with gain choice"+pidGain);
		    	}
		    	else
		    		System.out.println("Problem with tuning choice:"+pidChoice);
		    }
		    
		    
		    if(xBox.getRawButton(4) && letUp4)
		    {
		    	letUp4 = false;
		    	pidGain+=1;
		    	if(pidGain > 3)
		    		pidGain = 1;
		    }
		   
		    if(xBox.getRawButton(1))
		    {
		    	if(pidChoice == 1)//downWrist
		    	{
		    		if(pidGain == 1 && baseDownWristP == 0)//P
		    			baseDownWristP = 0.001;
		    		else if(pidGain == 2 && baseDownWristI == 0)//I
		    			baseDownWristI = 0.001;
		    		else if(pidGain == 3 && baseDownWristD == 0)//D
		    			baseDownWristD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else if(pidChoice == 2)//upElevator
		    	{
		    		if(pidGain == 1 && baseUpElevatorP == 0)//P
		    			baseUpElevatorP = 0.001;
		    		else if(pidGain == 2 && baseUpElevatorI == 0)//I
		    			baseUpElevatorI = 0.001;
		    		else if(pidGain == 3 && baseUpElevatorD == 0)//D
		    			baseUpElevatorD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else if(pidChoice == 3)//downElevator
		    	{
		    		if(pidGain == 1 && baseDownElevatorP == 0)//P
		    			baseDownElevatorP = 0.001;
		    		else if(pidGain == 2 && baseDownElevatorI == 0)//I
		    			baseDownElevatorI = 0.001;
		    		else if(pidGain == 3 && baseDownElevatorD == 0)//D
		    			baseDownElevatorD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else if(pidChoice == 4)//positionPID
		    	{
		    		if(pidGain == 1 && basePositionP == 0)//P
		    			basePositionP = 0.001;
		    		else if(pidGain == 2 && basePositionI == 0)//I
		    			basePositionI = 0.001;
		    		else if(pidGain == 3 && basePositionD == 0)//D
		    			basePositionD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else if(pidChoice == 5)//upWrist
		    	{
		    		if(pidGain == 1 && baseUpWristP == 0)//P
		    			baseUpWristP = 0.001;
		    		else if(pidGain == 2 && baseUpWristI == 0)//I
		    			baseUpWristI = 0.001;
		    		else if(pidGain == 3 && baseUpWristD == 0)//D
		    			baseUpWristD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else if(pidChoice == 6)//turningPID
		    	{
		    		if(pidGain == 1 && baseTurningP == 0)//P
		    			baseTurningP = 0.001;
		    		else if(pidGain == 2 && baseTurningI == 0)//I
		    			baseTurningI = 0.001;
		    		else if(pidGain == 3 && baseTurningD == 0)//D
		    			baseTurningD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else
		    		System.out.println("Problem with selected PID setup:"+pidChoice);
		    }
		    
		    if(pidChoice == 1 || pidChoice == 5)
		    {
		    	if(xBox.getRawButton(3) && !manualWrist && wristMode!=up)//X
		    	{
		    		wristFlushed = false;
		    		wrist.config_kP(0, upWristP, 10); 
		    		wrist.config_kI(0, upWristI, 10);
		    		wrist.config_kD(0, upWristD, 10);
		    		targetWristPos = wristUp;
		    		wristMode = up;
		    	}
		    	else if(xBox.getRawButton(7) && !manualWrist && wristMode!=down)//Back
		    	{
		    		wrist.config_kP(0, downWristP, 10); 
		    		wrist.config_kI(0, downWristI, 10);
		    		wrist.config_kD(0, downWristD, 10);
		    		targetWristPos = wristDown;
		    		wristMode = down;
		    	}
	    	
		    	if(wristMode == down && !wristFlushed && wrist.getSelectedSensorPosition(0) < -110000)
		    	{
		    		wristFlushed = true;
		    		wrist.setIntegralAccumulator(0, 0, 0);
		    	}
		    	
		    	if(debug)System.out.println(":"+manualWrist);
	    		
		    	wrist.set(ControlMode.Position, targetWristPos);
	    	}
		    else if(pidChoice == 2 || pidChoice == 3)//elevator
		    {
		    	if(xBox.getRawButton(3))
		    	{
		    		elevatorMode = up;
		    		if(debug)System.out.println("moving elevator up");
		    		elevator.config_kP(0, upElevatorP, 10); 
		    		elevator.config_kI(0, upElevatorI, 10);
		    		elevator.config_kD(0, upElevatorD, 10);
		   			targetElevatorPos = elevatorUp;//go to top (this is a test value use:
		    	}
		    	else if(xBox.getRawButton(8) && letUp2 && elevatorMode == down)
		    	{
		    		letUp2 = false;
		    		elevator.config_kP(0, upElevatorP, 10);
		        	elevator.config_kI(0, upElevatorI, 10);
		        	elevator.config_kD(0, upElevatorD, 10);
		    		elevatorMode = mid;
		    		targetElevatorPos = elevatorMid;
		    	}
		   		else if(xBox.getRawButton(7))
		   		{
		  			if(elevatorMode == up)
		   			{
		   				setElevatorDown = true;
		   				elevator.config_kI(0, 0, 10);
		   				elevator.config_kD(0, 0, 10);
		   			}
		   			else
		   			{
		   				elevator.config_kP(0, downElevatorP, 10);
		   				elevator.config_kI(0, downElevatorI, 10);
		   				elevator.config_kD(0, downElevatorD, 10);
		   			}
		  			elevatorMode = down;
		   			targetElevatorPos = elevatorDown; //go to bottom
		   		}
		    	
		    	if(setElevatorDown && elevator.getSelectedSensorPosition(0) > elevatorUp / 2)
		    	{
		    		setElevatorDown = false;
		    		elevator.config_kP(0, downElevatorP, 10); 
	   				elevator.config_kI(0, downElevatorI, 10);
	   				elevator.config_kD(0, downElevatorD, 10);
		    	}
		    	
		    	if((!lowElevator.get() && !highElevator.get()) || elevatorMode != down)
		    		elevator.set(ControlMode.Position, targetElevatorPos);	    	
		    	else if(lowElevator.get() && highElevator.get())
		    	{
		    		elevator.setSelectedSensorPosition(-34000, 0, 10);
		    		elevator.set(ControlMode.PercentOutput, -0.005);
		    	}
		    	else if(lowElevator.get())
		    		elevator.set(ControlMode.PercentOutput, -0.15);
		    		//elevator.set(ControlMode.PercentOutput, -0.00000294*Math.abs(elevator.getSelectedSensorPosition(0)+34000));
		    	else if(highElevator.get())
		    		//elevator.set(ControlMode.PercentOutput, 0.00000294*Math.abs(elevator.getSelectedSensorPosition(0)+34000));
		    		elevator.set(ControlMode.PercentOutput, 0.15);
		    }
		    else if(pidChoice == 4 || pidChoice == 6)//positionPID
		    {
		    	if(xBox.getRawButton(3))
		    		runningTestAuto = true;
		    	
		    	if(runningTestAuto)
		    	{
		    		if(startTestAuto)
		    			autonomousInit();
		    	
		    		autonomousPeriodic();
		    	}
		    }
		    else
		    	System.out.println("Improper pidChoice:"+pidChoice);
		    
			winch.set(0);
			
			if(xBox.getRawButton(2) && letUp2)
			{
				letUp2 = false;
				
				if(multMode == 2)
					multMode = 1;
				else if(multMode == 1)
					multMode = 0.5;
				else if(multMode == 0.5)
					multMode = 0.1;
				else if(multMode == 0.1)
					multMode = 10;
				else if(multMode == 10)
					multMode = 2;
				else
					System.out.println("Error with multMode:"+multMode);
			}
			
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
    	
    	if(!manualCon && testing)
    	{		   
		    if(tuningController.getRawButton(5) && letUp5)//LB
		    {
		    	letUp5 = false;
		    	if(pidGain == 1)
		    		currentMultP-=(0.1*multMode);
		    	else if(pidGain == 2)
		    		currentMultI-=(0.1*multMode);
		    	else if(pidGain == 3)
		    		currentMultD-=(0.1*multMode);
		    	changedGain = true;
		    }
		    else if(tuningController.getRawButton(6) && letUp6)//RB
		    {
		    	letUp6 = false;
		    	if(pidGain==1)
		    		currentMultP+=(0.1*multMode);
		    	else if(pidGain==2)
		    		currentMultI+=(0.1*multMode);
		    	else if(pidGain==3)
		    		currentMultD+=(0.1*multMode);
		    	changedGain = true;
		    }   
		    
		    if(changedGain)
		    {
		    	changedGain = false;
		    	if(pidChoice == 1)//wristDown
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * baseDownWristP;
		    			downWristP = currentPValue;
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * baseDownWristI;
		    			downWristI = currentIValue;
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * baseDownWristD;
		    			downWristD = currentDValue;
		    		}
		    		else
		    			System.out.println("Problem with with gain choice:"+pidGain);
		    	}
		    	else if(pidChoice == 2)//elevatorUp
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * baseUpElevatorP;
		    			upElevatorP = currentPValue;
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * baseUpElevatorI;
		    			upElevatorI = currentIValue;
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * baseUpElevatorD;
		    			upElevatorD = currentDValue;
		    		}
		    		else
		    			System.out.println("Problem with with gain choice:"+pidGain);
		    	}
		    	else if(pidChoice == 3)//elevatorDown
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * baseDownElevatorP;
		    			downElevatorP = currentPValue;
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * baseDownElevatorI;
		    			downElevatorI = currentIValue;
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * baseDownElevatorD;
		    			downElevatorD = currentDValue;
		    		}
		    		else
		    			System.out.println("Problem with with gain choice:"+pidGain);
		    	}
		    	else if(pidChoice == 4)//positionPID
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * basePositionP;
		    			positionP = currentPValue;
		    			positionPID.setP(positionP);
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * basePositionI;
		    			positionI = currentIValue;
		    			positionPID.setI(positionI);
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * basePositionD;
		    			positionD = currentDValue;
		    			positionPID.setD(positionD);
		    		}
		    		else
		    			System.out.println("Problem with with gain choice:"+pidGain);
		    	}
		    	else if(pidChoice == 5)//wristUp
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * baseUpWristP;
		    			upWristP = currentPValue;
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * baseUpWristI;
		    			upWristI = currentIValue;
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * baseUpWristD;
		    			upWristD = currentDValue;
		    		}
		    		else
		    			System.out.println("Problem with with gain choice"+pidGain);
		    	}
		    	else if(pidChoice == 6)//turningPID
		    	{
		    		if(pidGain == 1)//P
		    		{
		    			currentPValue = currentMultP * baseTurningP;
		    			turningPID.setP(currentPValue);
		    			turningP = currentPValue;
		    		}
		    		else if(pidGain == 2)//I
		    		{
		    			currentIValue = currentMultI * baseTurningI;
		    			turningPID.setI(currentIValue);
		    			turningI = currentIValue;
		    		}
		    		else if(pidGain == 3)//D
		    		{
		    			currentDValue = currentMultD * baseTurningD;
		    			turningPID.setD(currentDValue);
		    			turningD = currentDValue;
		    		}
		    		else
		    			System.out.println("Problem with with gain choice"+pidGain);
		    	}
		    	else
		    		System.out.println("Problem with tuning choice:"+pidChoice);
		    }
		    
		    
		    if(tuningController.getRawButton(4) && letUp4)
		    {
		    	letUp4 = false;
		    	pidGain+=1;
		    	if(pidGain > 3)
		    		pidGain = 1;
		    }
		   
		    if(tuningController.getRawButton(1))
		    {
		    	if(pidChoice == 1)//downWrist
		    	{
		    		if(pidGain == 1 && baseDownWristP == 0)//P
		    			baseDownWristP = 0.001;
		    		else if(pidGain == 2 && baseDownWristI == 0)//I
		    			baseDownWristI = 0.001;
		    		else if(pidGain == 3 && baseDownWristD == 0)//D
		    			baseDownWristD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else if(pidChoice == 2)//upElevator
		    	{
		    		if(pidGain == 1 && baseUpElevatorP == 0)//P
		    			baseUpElevatorP = 0.001;
		    		else if(pidGain == 2 && baseUpElevatorI == 0)//I
		    			baseUpElevatorI = 0.001;
		    		else if(pidGain == 3 && baseUpElevatorD == 0)//D
		    			baseUpElevatorD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else if(pidChoice == 3)//downElevator
		    	{
		    		if(pidGain == 1 && baseDownElevatorP == 0)//P
		    			baseDownElevatorP = 0.001;
		    		else if(pidGain == 2 && baseDownElevatorI == 0)//I
		    			baseDownElevatorI = 0.001;
		    		else if(pidGain == 3 && baseDownElevatorD == 0)//D
		    			baseDownElevatorD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else if(pidChoice == 4)//positionPID
		    	{
		    		if(pidGain == 1 && basePositionP == 0)//P
		    			basePositionP = 0.001;
		    		else if(pidGain == 2 && basePositionI == 0)//I
		    			basePositionI = 0.001;
		    		else if(pidGain == 3 && basePositionD == 0)//D
		    			basePositionD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else if(pidChoice == 5)//upWrist
		    	{
		    		if(pidGain == 1 && baseUpWristP == 0)//P
		    			baseUpWristP = 0.001;
		    		else if(pidGain == 2 && baseUpWristI == 0)//I
		    			baseUpWristI = 0.001;
		    		else if(pidGain == 3 && baseUpWristD == 0)//D
		    			baseUpWristD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else if(pidChoice == 6)//turningPID
		    	{
		    		if(pidGain == 1 && baseTurningP == 0)//P
		    			baseTurningP = 0.001;
		    		else if(pidGain == 2 && baseTurningI == 0)//I
		    			baseTurningI = 0.001;
		    		else if(pidGain == 3 && baseTurningD == 0)//D
		    			baseTurningD = 0.001;
		    		else
		    			System.out.println("Problem with creating base value:"+pidGain);
		    	}
		    	else
		    		System.out.println("Problem with selected PID setup:"+pidChoice);
		    }
		    
			if(tuningController.getRawButton(2) && twoLetUp2)
			{
				letUp2 = false;
				
				if(multMode == 2)
					multMode = 1;
				else if(multMode == 1)
					multMode = 0.5;
				else if(multMode == 0.5)
					multMode = 0.1;
				else if(multMode == 0.1)
					multMode = 10;
				else if(multMode == 10)
					multMode = 2;
				else
					System.out.println("Error with multMode:"+multMode);
			}
    	}
    	
    	if(debug)System.out.println(manualCon);
    	
    	if(!tuningController.getRawButton(2))
    		letUp2 = true;
    	
    	if(!xBox.getRawButton(4))
	    	letUp4 = true;
    	
    	if(!xBox.getRawButton(5))
	    	letUp5 = true;
	    
	    if(!xBox.getRawButton(6))
	    	letUp6 = true;
	    
	    if(!xBox.getRawButton(8))
    		letUp8 = true;
    	
    	if(!xBox.getRawButton(10))
    		letUp10 = true;
    	
    	if(!tuningController.getRawButton(2))
    		twoLetUp2 = true;
    	
    	if(!tuningController.getRawButton(4))
	    	twoLetUp4 = true;
    	
    	if(!tuningController.getRawButton(5))
	    	twoLetUp5 = true;
	    
	    if(!tuningController.getRawButton(6))
	    	twoLetUp6 = true;
	    
	    if(!tuningController.getRawButton(8))
    		twoLetUp8 = true;
    	
    	if(!tuningController.getRawButton(10))
    		twoLetUp10 = true;
    	
    	testLoopCount++;
    	
    	SmartDashboard.putNumber("multMode", multMode*0.1);
		SmartDashboard.putNumber("PID Choice", pidChoice);//1 is down wrist
	    SmartDashboard.putNumber("PID Gain", pidGain);
	    SmartDashboard.putNumber("currentMultP", currentMultP);
	    SmartDashboard.putNumber("currentMultI", currentMultI);
	    SmartDashboard.putNumber("currentMultD", currentMultD);
	    SmartDashboard.putNumber("Current P Value", currentPValue);
	    SmartDashboard.putNumber("Current I Value", currentIValue);
	    SmartDashboard.putNumber("Current D Value", currentDValue);
    	SmartDashboard.putNumber("elevatorMode", elevatorMode);
    	SmartDashboard.putNumber("Elevator Position", elevator.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("Wrist Position", wrist.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("leftSide", leftSide.get());
    	SmartDashboard.putNumber("targetWristPos", targetWristPos);
    	SmartDashboard.putNumber("targetElevatorPos", targetElevatorPos);
    	SmartDashboard.putNumber("ahrs", ahrs.getAngle());
    	SmartDashboard.putNumber("forward", xBox.getRawAxis(1));
    	SmartDashboard.putNumber("turn", xBox.getRawAxis(0));
    	SmartDashboard.putBoolean("lowElevator", lowElevator.get());
    	SmartDashboard.putBoolean("highElevator", highElevator.get());//
    }
    
    public void testInit(){
    	leftSide.reset();
    }
    
    public void testPeriodic(){
    	//System.out.println(wrist.getSelectedSensorPosition(0));
    	//System.out.println(leftSide.get());
    	//System.out.println(lowElevator.get()+":"+highElevator.get());
    }
}
