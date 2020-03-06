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

 public class Hook extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	DoubleSolenoid hookSolenoid = null;
	

	public Hook() {
		super();
		hookSolenoid = new DoubleSolenoid(RobotMap.HOOK_SOLENOID_DEPLOY,
				RobotMap.HOOK_SOLENOID_RETRACT);
				hookSolenoid.set(Value.kOff);
		addChild("Hook",hookSolenoid);
		SmartDashboard.putData("Hook", (DoubleSolenoid) hookSolenoid);
	
	}

	public void Deploy() {
		
		//pivotSolenoid.set(Value.kOff);
		hookSolenoid.set(Value.kForward);
	}

	public void Retract() {
		//pivotSolenoid.set(Value.kOff);
		hookSolenoid.set(Value.kReverse);
		
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	public void log() {
		SmartDashboard.putData("Hook", (DoubleSolenoid) hookSolenoid);
		
	}
}
