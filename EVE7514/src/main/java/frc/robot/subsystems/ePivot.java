/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
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
public class ePivot extends Subsystem {
  private final TalonSRX m_motor;
  // public final DigitalInput m_toplimitswitch,m_bottomlimitswitch ;

  /**
   * Create a new pivot subsystem.
   */
  public ePivot() {
    super();

    m_motor = new TalonSRX(RobotMap.PIVOT);

    // m_toplimitswitch = new DigitalInput(RobotMap.TOP_LIMITSWITCH);
    // m_bottomlimitswitch = new DigitalInput(RobotMap.BOTTOM_LIMITSWITCH);

    // *TalonSRX Encoder Configuration
    // *Config the sensor used for Primary PID and sensor direction

    m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDPivot,
        Constants.kTimeoutMs);

    // Ensure sensor is positive when output is positive

    m_motor.setSensorPhase(Constants.kSensorPhase);
    /**
     * Set based on what direction you want forward/positive to be. This does not
     * affect sensor phase.
     */
    m_motor.setInverted(Constants.kMotorInvert);

    // * Config the peak and nominal outputs, 12V means full
    m_motor.configNominalOutputForward(0, Constants.kTimeoutMs);
    m_motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    m_motor.configPeakOutputForward(1, Constants.kTimeoutMs);
    m_motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral
     * within this range. See Table in Section 17.2.1 for native units per rotation.
     */
    m_motor.configAllowableClosedloopError(0, Constants.kPIDPivot, Constants.kTimeoutMs);

    // * Config Position Closed Loop gains in slot0, tsypically kF stays zero.
    m_motor.config_kF(Constants.kPIDPivot, Constants.kGains.kF, Constants.kTimeoutMs);
    m_motor.config_kP(Constants.kPIDPivot, Constants.kGains.kP, Constants.kTimeoutMs);
    m_motor.config_kI(Constants.kPIDPivot, Constants.kGains.kI, Constants.kTimeoutMs);
    m_motor.config_kD(Constants.kPIDPivot, Constants.kGains.kD, Constants.kTimeoutMs);

    /**
     * Grab the 360 degree position of the MagEncoder's absolute position, and
     * intitally set the relative sensor to match.
     */
    int absolutePosition = m_motor.getSensorCollection().getPulseWidthPosition();

    /* Mask out overflows, keep bottom 12 bits */
    absolutePosition &= 0xFFF;
    if (Constants.kSensorPhase) {
      absolutePosition *= -1;
    }
    if (Constants.kMotorInvert) {
      absolutePosition *= -1;
    }

    /* Set the quadrature (relative) sensor to match absolute */
    m_motor.setSelectedSensorPosition(absolutePosition, Constants.kPIDPivot, Constants.kTimeoutMs);
  }

  @Override
  public void initDefaultCommand() {
   setDefaultCommand(new PivotWithJoystick());
  }

  /**
   * The log method puts interesting information to the SmartDashboard.
   */
  public void log() {

    SmartDashboard.putNumber("Pivot Target", m_motor.getClosedLoopTarget(Constants.kPIDPivot));
    SmartDashboard.putNumber("Pivot Position", m_motor.getSelectedSensorPosition(Constants.kPIDPivot));
  }

  public void move(double power) {

    m_motor.set(ControlMode.PercentOutput, (power / 3));

  }

  public void moveto(double degrees) {

    m_motor.set(ControlMode.Position, (degrees / 360) * Constants.kSensorUnitsPerRotation);

  }

  public boolean ontarget() {
    if (m_motor.getClosedLoopError(Constants.kPIDPivot) < 1) {
      return true;
    }
    return false;
  }
}
