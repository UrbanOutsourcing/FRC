/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.commands.*;

/**
 * The lift subsystem uses PID to go to a given height. Unfortunately, in it's
 * current state PID values for simulation are different than in the real world
 * do to minor differences.
 */
public class LimeLight extends Subsystem {
  

  // public final DigitalInput m_toplimitswitch,m_bottomlimitswitch ;

  /**
   * Create a new pivot subsystem.
   */
  public LimeLight() {
    super();


    NetworkTable table = NetworkTable.getTable("limelight");
    double targetOffsetAngle_Horizontal = table.getNumber("tx", 0);
    double targetOffsetAngle_Vertical = table.getNumber("ty", 0);
    double targetArea = table.getNumber("ta", 0);
    double targetSkew = table.getNumber("ts", 0);

    
  }

  @Override
  public void initDefaultCommand() {
   setDefaultCommand(new PivotWithJoystick());
  }

  /**
   * The log method puts interesting information to the SmartDashboard.
   */
  public void log() {

    //SmartDashboard.putNumber("Pivot Target", m_motor.getClosedLoopTarget(Constants.kPIDPivot));
    //SmartDashboard.putNumber("Pivot Position",;
  }

  
}
