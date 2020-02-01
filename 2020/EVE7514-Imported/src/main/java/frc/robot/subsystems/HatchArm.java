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
				armSolenoid.set(Value.kOff);
		addChild("Arm", armSolenoid);
		SmartDashboard.putData("Hatch Arm", (DoubleSolenoid) armSolenoid);
	
	}

	public void Deploy() {
		
		//armSolenoid.set(Value.kOff);
		armSolenoid.set(Value.kForward);
	}

	public void Retract() {
		//armSolenoid.set(Value.kOff);
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
