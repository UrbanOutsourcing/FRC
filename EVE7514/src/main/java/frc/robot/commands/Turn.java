/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Turn extends Command {
	/**
	 * Add your docs here.
	 */
	private final double m_degree;
	public Turn(double degrees) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		m_degree = degrees;
		//Robot.m_mdrivetrain.zeroSensors();
		requires(Robot.m_mdrivetrain);
	}

		
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.m_mdrivetrain.driveto(0,m_degree);
	}
  
	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {

	  return Robot.m_mdrivetrain.ontarget();
	}
  
	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.m_mdrivetrain.zeroSensors();
	}
}
