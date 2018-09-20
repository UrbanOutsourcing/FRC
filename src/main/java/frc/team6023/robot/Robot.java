/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team6023.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	// Controller 
	private Joystick driveJoy, gripJoy;
	
	// Plunger
	int plunger_counter =0; 
	int plunger_state =0;
	
	// Moters
	private Spark leftDrive1, leftDrive2, rightDrive1, rightDrive2;
	private SpeedControllerGroup leftDrive, rightDrive, lift;
	private Spark gripLift, liftArm;
	private Spark lift1, lift2;
	
	private DoubleSolenoid gripper, lifter;
	private CameraServer camera;
	
	private Compressor comp;
	// Drive
	private DifferentialDrive drive;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		//Set up joy1 on port 0
		driveJoy = new Joystick(0);
		gripJoy = new Joystick(1);
		
		comp = new Compressor();
		comp.setClosedLoopControl(true);
		
		//Set up left drive on pwm 1 and right on pwm 2
		leftDrive1 = new Spark(1);
		leftDrive2 = new Spark(2);
		rightDrive1 = new Spark(3);
		rightDrive2 = new Spark(4);
		
		leftDrive = new SpeedControllerGroup(leftDrive1, leftDrive2);
		leftDrive.setInverted(true);
		rightDrive = new SpeedControllerGroup(rightDrive1, rightDrive2);
		rightDrive.setInverted(true);
		gripLift = new Spark (5);
		liftArm = new Spark(6);
		lift1 = new Spark(7);
		lift2 = new Spark(8);
		lift = new SpeedControllerGroup(lift1, lift2);
		
		gripper = new DoubleSolenoid(0,1);
		lifter = new DoubleSolenoid(2,3);
		
		drive = new DifferentialDrive(leftDrive, rightDrive);
		drive.setSafetyEnabled(true);
		
		camera = CameraServer.getInstance();
		camera.startAutomaticCapture(0);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		
		
		drive.tankDrive(-0.75, -0.75);
		Timer.delay(3.2);
		drive.tankDrive(0, 0);
		plunger_state=0;
		plunger_counter=0;
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
		switch (plunger_state) {
		case 0:
			if (++plunger_counter > 150)       //* increments of 20mseconds 
			{
				lifter.set(DoubleSolenoid.Value.kForward);
				plunger_state = 1;
			} 
			break;
			
		case 1:
			break;
			
		}
		
	}
	
	double driveY, driveX;
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		driveY = driveJoy.getY();
		driveX = -driveJoy.getX();
		
		drive.arcadeDrive(driveY/3*4, driveX);
		
		if(driveJoy.getRawButton(5)) {
			while(driveJoy.getRawButton(5)) {};
			if(lifter.get() == DoubleSolenoid.Value.kForward) {
				lifter.set(DoubleSolenoid.Value.kReverse);
			}
			else {
				lifter.set(DoubleSolenoid.Value.kForward);
			}
		}
		
		
		if(gripJoy.getRawButton(4)) {
			while(gripJoy.getRawButton(4)) {};
			if(gripper.get() == DoubleSolenoid.Value.kForward) {
				gripper.set(DoubleSolenoid.Value.kReverse);
			}
			else {
				gripper.set(DoubleSolenoid.Value.kForward);
			}
		}
		
		if(gripJoy.getRawButton(6)) {
			gripLift.set(-0.5);
		}
		else {
			gripLift.set(0);
		}
		
		/*
		if(gripJoy.getRawButton(3)) {
			lift.set(0.60);
		}
		else if(gripJoy.getRawButton(5)) {
			lift.set(-0.60);
		}
		else {
			lift.set(0);
		} */
		
		lift.set(-gripJoy.getY());
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
