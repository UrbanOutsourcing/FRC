/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Operate to Shoot or Intake until interrupted
 */
public class ShootWithTriggers extends Command {
  public ShootWithTriggers() {
    requires(Robot.m_shooter);
    }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   
    switch ( (int) Robot.m_oi.m_joystick.getRawAxis(3)) {
      case 1:
       new Shoot();
       break;
      case -1:
       new Intake();
       break;
      default:
    }

  }
 
    // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
      return false; //run until interrupted
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
}
}
