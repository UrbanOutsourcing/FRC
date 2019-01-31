/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * The lift subsystem uses PID to go to a given height. Unfortunately, in
 * it's current state PID values for simulation are different than in the real
 * world do to minor differences.
 */
public class Lift extends PIDSubsystem {
  private final Spark m_motor;
  private final Encoder m_encoder;
 
  private static final double kP_real = 4;
  private static final double kI_real = 0.07;
  private static final double kP_simulation = 18;
  private static final double kI_simulation = 0.2;

  /**
   * Create a new lift subsystem.
   */
  public Lift() {
    super(kP_real, kI_real, 0);
    if (Robot.isSimulation()) { // Check for simulation and update PID values
      getPIDController().setPID(kP_simulation, kI_simulation, 0, 0);
    }
    setAbsoluteTolerance(0.005);

    m_motor = new Spark(RobotMap.LIFT);
    
    m_encoder = new Encoder(RobotMap.LIFT_CHANNELA, RobotMap.LIFT_CHANNELB);

    // Conversion value of potentiometer varies between the real world and
    // simulation
    if (Robot.isReal()) {
      m_encoder.setDistancePerPulse(0.042);
      
    } else {
      // Circumference in ft = 4in/12(in/ft)*PI
      m_encoder.setDistancePerPulse((4.0 / 12.0 * Math.PI) / 360.0);
      
    }

    // Let's name everything on the LiveWindow
    //addChild("Motor", (Sendable) m_motor);
    //addChild("Motor", (Sendable) m_motor);
    addChild("Encoder", m_encoder);
  }

  @Override
  public void initDefaultCommand() {
  }

  /**
   * The log method puts interesting information to the SmartDashboard.
   */
  public void log() {
    SmartDashboard.putData("Lift Encoder", (Encoder) m_encoder);
  }

  /**
   * Use the encoder as the PID sensor. This method is automatically
   * called by the subsystem.
   */
  @Override
  protected double returnPIDInput() {
    return m_encoder.getDistance();
  }

  /**
   * Use the motor as the PID output. This method is automatically called by
   * the subsystem.
   */
  @Override
  protected void usePIDOutput(double power) {
    m_motor.set(power);
  }
  public void move(double power) {
    m_motor.set(power);
  }
}