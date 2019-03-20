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
public class HabClimb extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	DoubleSolenoid habSolenoid = null;

	public HabClimb() {
		super();
		habSolenoid = new DoubleSolenoid(RobotMap.HABCLIMB_SOLENOID_DEPLOY,
				RobotMap.HABCLIMB_SOLENOID_RETRACT);
				habSolenoid.set(Value.kOff);
		addChild("Hab", habSolenoid);
		SmartDashboard.putData("Hab Climb", (DoubleSolenoid) habSolenoid);
	
	}

	public void Deploy() {
		
		//armSolenoid.set(Value.kOff);
		habSolenoid.set(Value.kForward);
	}

	public void Retract() {
		//armSolenoid.set(Value.kOff);
		habSolenoid.set(Value.kReverse);
		
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	public void log() {
		SmartDashboard.putData("Hab Climb", (DoubleSolenoid) habSolenoid);
		
	}
}
