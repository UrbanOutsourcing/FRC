/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HatchArm extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	DoubleSolenoid armSolenoid = null;

	public HatchArm() {
		super();
		armSolenoid = new DoubleSolenoid(RobotMap.HATCHARM_SOLENOID_DEPLOY,
				RobotMap.HATCHARM_SOLENOID_RETRACT);
		addChild("Arm", armSolenoid);	
		Robot.m_compressor.enabled();
		Robot.m_compressor.start();
	}

	public void Deploy() {
		
		//armSolenoid.set(Value.kForward);
		armSolenoid.set(0);
	}

	public void Retract() {
		armSolenoid.set(Value.kReverse);
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	public void log() {
		SmartDashboard.putData("Hatch Arm", (DoubleSolenoid) armSolenoid);
		
	  }
}
