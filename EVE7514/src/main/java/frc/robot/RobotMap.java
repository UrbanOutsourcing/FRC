/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// Motors
	public static final int DRIVETRAIN_LEFT_FRONT = 0;
	public static final int DRIVETRAIN_LEFT_BACK = 1;
	public static final int DRIVETRAIN_RIGHT_FRONT = 2;
	public static final int DRIVETRAIN_RIGHT_BACK = 3;
	public static final int LIFT = 4;

	//Encoders
	public static final int DRIVETRAIN_LEFT_CHANNELA = 1;
    public static final int DRIVETRAIN_LEFT_CHANNELB = 2;
    public static final int DRIVETRAIN_RIGHT_CHANNELB= 4;
    public static final int DRIVETRAIN_RIGHT_CHANNELA = 3;
	public static final int LIFT_CHANNELA = 5;
	public static final int LIFT_CHANNELB = 6;
	
	// Joystick
	public static final int OI_DRIVER_CONTROLLER = 0;
	public static final int DRIVER_CONTROLLER_MOVE_AXIS = 1;
	public static final int DRIVER_CONTROLLER_ROTATE_AXIS = 2;

	// Solenoids
	public static final int HATCHARM_SOLENOID_DEPLOY = 1;
	public static final int HATCHARM_SOLENOID_RETRACT = 0;

	//Limit Switches
	public static final int TOP_LIMITSWITCH = 0;
    public static final int BOTTOM_LIMITSWITCH = 1;
	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
