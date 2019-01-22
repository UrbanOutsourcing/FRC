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
 * Move the elevator to a given location. This command finishes when it is
 * within the tolerance, but leaves the PID loop running to maintain the
 * position. Other commands using the elevator should make sure they disable
 * PID!
 */
public class SetPivotSetpoint extends Command {
  private final double m_setpoint;

  public SetPivotSetpoint(double setpoint) {
    m_setpoint = setpoint;
    requires(Robot.m_pivot);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_lift.enable();
    Robot.m_lift.setSetpoint(m_setpoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_pivot.onTarget();
  }
}
