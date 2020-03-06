/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class HookRetract extends InstantCommand {
	/**
	 * Add your docs here.
	 */
	
	public HookRetract() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		//requires(Robot.m_liftarmpivot);
	}

	// Called once when the command executes
	@Override
	protected void execute() {
		Robot.m_hook.Retract();
	}

		// Make this return true when this Command no longer needs to run execute()
		@Override
		protected boolean isFinished() {
		  return true; // Runs until interrupted
		}
	  
		// Called once after isFinished returns true
		@Override
		protected void end() {
		
		}


}
