/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;


/**
 * Add your docs here.
 */
public class DriveStraight extends InstantCommand {
	/**
	 * Add your docs here.
	 */
	private final double m_distance;
	public DriveStraight(double distance) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		m_distance = distance;
		//requires(Robot.m_edrivetrain);
	}

	// Called once when the command executes
	@Override
	protected void initialize() {
		//Robot.m_edrivetrain.driveto(m_distance);
	}
}
