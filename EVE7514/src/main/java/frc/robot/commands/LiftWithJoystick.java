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
 * Have the robot drive tank style using the PS3 Joystick until interrupted.
 */
public class LiftWithJoystick extends Command {
  public LiftWithJoystick() {
    requires(Robot.m_lift);
    }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Robot.m_drivetrain.drive(Robot.m_oi.getJoystick());
    Robot.m_lift.move(Robot.m_oi.m_joystick.getRawAxis(1));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Robot.m_lift.m_toplimitswitch.get()) { 
      return Robot.m_lift.m_toplimitswitch.get();
    } else if (Robot.m_lift.m_bottomlimitswitch.get()) { 
      return Robot.m_lift.m_bottomlimitswitch.get();
    }
    return false; //run until interrupted
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_lift.move(0);
  }
}
