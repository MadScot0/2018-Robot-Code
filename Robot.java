package org.usfirst.frc.team3826.robot;

import com.kauailabs.navx.frc.AHRS;

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
//                                2018 Bot Code// v1.1
			/* Implemented turning PID
			 * Untested
			 * 
			 * 
			 * 
			 */
/**
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {


	@SuppressWarnings("deprecation")
	RobotDrive robot;
	Joystick xBox;
	
	//MotorType kFrontLeft;
	Talon frontLeft = new Talon(0);
	Talon rearLeft = new Talon(1); 
	Talon frontRight = new Talon(2); 
	Talon rearRight = new Talon(3);
	
	SRF_PID turningPID;
	
	AHRS ahrs;
	double angleNow, scaledAngle; //the current angle and the angle to feed into the PID
	
	Timer t = new Timer();
	
    public void robotInit() {
    	robot = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
    	turningPID = new SRF_PID();
    	xBox = new Joystick(0);
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	turningPID.setPID(3, 0.2, 0.1);
    	turningPID.setLimits(1, -1);
    	turningPID.setSetpoint(0.25);
    	t.start();
    }
    
    /**
     * This function is called periodically during autonomous
     */
	public void autonomousPeriodic() {
		
		robot.arcadeDrive(0,turningPID.computePID(ahrs.getAngle()/360, t.get()));
    }
    	
    
    /**
     * This function is called once each time the robot enters teleoperated mode
     */
    public void teleopInit(){
    	
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
    		robot.arcadeDrive(-xBox.getRawAxis(1), -xBox.getRawAxis(0));
    	}
    	else
    	{
    		robot.arcadeDrive(0, 0);
    	}
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
