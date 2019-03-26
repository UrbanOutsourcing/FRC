/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class ShooterShift extends InstantCommand {
	/**
	 * Add your docs here.
	 */
	private final double m_direction;
	public ShooterShift(double direction) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		m_direction = direction;
		//requires(Robot.m_edrivetrain);
	}

	// Called once when the command executes
	@Override
	protected void initialize() {
		//Constants.setkMotorDamp(++Constants.kMotorDamp);
	}
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (m_direction < 0) {
			//Constants.setkMotorDamp(++Constants.kMotorDamp);
			Constants.setShooterDamp(2);
		} else {
			Constants.setShooterDamp(1);
		}
			
		
	}
}
