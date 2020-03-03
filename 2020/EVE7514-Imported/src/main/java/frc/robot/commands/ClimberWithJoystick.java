/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Operate to Shoot or Intake until interrupted
 */
public class ClimberWithJoystick extends Command {
  public ClimberWithJoystick() {
    requires(Robot.m_climber);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Robot.m_climber.move(Robot.m_oi.m_joystick.getRawAxis(RobotMap.RIGHT_STICK_X_AXIS));


    /*if (Robot.m_oi.m_joystick.getRawAxis(RobotMap.LEFT_TRIGGER_AXIS) > 0) {
      Robot.m_shooter.move(Robot.m_oi.m_joystick.getRawAxis(RobotMap.LEFT_TRIGGER_AXIS));
    } else {
      Robot.m_shooter.move(-1*Robot.m_oi.m_joystick.getRawAxis(RobotMap.RIGHT_TRIGGER_AXIS));
    }*/

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false; // run until interrupted
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

  }
}
