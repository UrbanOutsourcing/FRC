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

 public class LiftArmPivot extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	DoubleSolenoid pivotSolenoid = null;
	

	public LiftArmPivot() {
		super();
		pivotSolenoid = new DoubleSolenoid(RobotMap.LIFTARMPIVOT_SOLENOID_DEPLOY,
				RobotMap.LIFTARMPIVOT_SOLENOID_RETRACT);
				pivotSolenoid.set(Value.kOff);
		addChild("LiftArmPivot", pivotSolenoid);
		SmartDashboard.putData("Lift Arm Pivot", (DoubleSolenoid) pivotSolenoid);
	
	}

	public void Deploy() {
		
		//pivotSolenoid.set(Value.kOff);
		pivotSolenoid.set(Value.kForward);
	}

	public void Retract() {
		//pivotSolenoid.set(Value.kOff);
		pivotSolenoid.set(Value.kReverse);
		
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	public void log() {
		SmartDashboard.putData("Lift Arm Pivot", (DoubleSolenoid) pivotSolenoid);
		
	}
}
