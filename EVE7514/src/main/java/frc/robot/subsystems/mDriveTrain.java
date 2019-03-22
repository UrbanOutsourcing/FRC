/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.commands.*;

/**
 * The DriveTrain subsystem uses PID to go a given distance. Using the TalonSRX
 * Encoders, PID Libraries and Motion Magic Control Mode
 */
public class mDriveTrain extends Subsystem {
  private final BaseMotorController m_leftrear;
  private final TalonSRX m_leftmaster;
  private final BaseMotorController m_rightrear;
  private final TalonSRX m_rightmaster;

  /**
   * Create a new pivot subsystem.
   */
  public mDriveTrain() {
    super();

    m_leftmaster = new TalonSRX(RobotMap.DRIVETRAIN_LEFT_FRONT);
    m_rightmaster = new TalonSRX(RobotMap.DRIVETRAIN_RIGHT_FRONT);
    m_leftrear = new VictorSPX(RobotMap.DRIVETRAIN_LEFT_BACK);
    m_rightrear = new VictorSPX(RobotMap.DRIVETRAIN_RIGHT_BACK);

    m_leftrear.configFactoryDefault();
    m_rightrear.configFactoryDefault();
    
    m_leftmaster.configFactoryDefault();
    m_rightmaster.configFactoryDefault();

    m_leftrear.follow(m_leftmaster);
    m_rightrear.follow(m_rightmaster);

    // *TalonSRX Encoder Configuration
    // *Config the sensor used for Primary PID and sensor direction

    m_leftmaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.PID_PRIMARY,
        Constants.kTimeoutMs);
    m_rightmaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.PID_PRIMARY,
        Constants.kTimeoutMs);

    /**
     * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
     * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
     * sensor to have positive increment when driving Talon Forward (Green LED)
     */
    m_leftmaster.setSensorPhase(true);
    m_leftmaster.setInverted(false);
    m_rightmaster.setSensorPhase(true);
    m_rightmaster.setInverted(false);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    m_leftmaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    m_leftmaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    m_rightmaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    m_rightmaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    /* Set the peak and nominal outputs */
    m_leftmaster.configNominalOutputForward(0, Constants.kTimeoutMs);
    m_leftmaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
    m_leftmaster.configPeakOutputForward(1, Constants.kTimeoutMs);
    m_leftmaster.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    m_rightmaster.configNominalOutputForward(0, Constants.kTimeoutMs);
    m_rightmaster.configNominalOutputReverse(0, Constants.kTimeoutMs);
    m_rightmaster.configPeakOutputForward(1, Constants.kTimeoutMs);
    m_rightmaster.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    m_leftmaster.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    m_leftmaster.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    m_leftmaster.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    m_leftmaster.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    m_leftmaster.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    m_rightmaster.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    m_rightmaster.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    m_rightmaster.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    m_rightmaster.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    m_rightmaster.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    m_leftmaster.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    m_leftmaster.configMotionAcceleration(6000, Constants.kTimeoutMs);
    m_rightmaster.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    m_rightmaster.configMotionAcceleration(6000, Constants.kTimeoutMs);

    /* Zero the sensor */
    zeroSensors();
  }

  /**
   * When no other command is running let the operator drive around using the PS3
   * joystick.
   */
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TankDriveWithJoystick());
  }

  /**
   * The log method puts interesting information to the SmartDashboard.
   */
  public void log() {

    //SmartDashboard.putNumber("DriveTrain Right Target", m_rightmaster.getClosedLoopTarget(Constants.PID_PRIMARY));
    SmartDashboard.putNumber("DriveTrain Right Position",
        m_rightmaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
    //SmartDashboard.putNumber("DriveTrain Left Target", m_leftmaster.getClosedLoopTarget(Constants.PID_PRIMARY));
    SmartDashboard.putNumber("DriveTrain Left Position",
        m_rightmaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
  }

  /* Zero quadrature encoders on Talons */
  void zeroSensors() {
    m_leftmaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    m_rightmaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param left  Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public void drive(double left, double right) {
    SmartDashboard.putNumber("Left Power", left);
    SmartDashboard.putNumber("Right Power", right);
    m_rightmaster.set(ControlMode.PercentOutput, -right);
    m_leftmaster.set(ControlMode.PercentOutput, left);
    //m_rightrear.set(ControlMode.PercentOutput, right);
    //m_leftrear.set(ControlMode.PercentOutput, left);

  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param joy The ps3 style joystick to use to drive tank style.
   */
  public void drive(Joystick joy) {
    drive(-joy.getY(), -joy.getThrottle());
  }

  public void drive(double left, double right, double rotate) {
    SmartDashboard.putNumber("Turn Degrees", rotate);
    // m_drive.arcadeDrive(left, rotate);
  }

  public void driveto(double distance, double degrees) {
    SmartDashboard.putNumber("Drive Distance", distance);
    SmartDashboard.putNumber("Turn Degree", degrees);

    if (distance > 0) {
      /* calculate targets */
      double m_target_sensorUnits = (distance * 12) * Constants.kWSensorUnitsPerInch;
      m_leftmaster.set(ControlMode.MotionMagic, m_target_sensorUnits);
      m_rightmaster.set(ControlMode.MotionMagic, m_target_sensorUnits);
    } else if (Math.abs(degrees) > 0) {
      /* calculate targets */
      double m_target_sensorUnits = degrees * Constants.kWSensorUnitsPerDegree;
      m_leftmaster.set(ControlMode.MotionMagic, m_target_sensorUnits);
      m_rightmaster.set(ControlMode.MotionMagic, -m_target_sensorUnits);
    }
  }

  /**
   * Get the robot's heading.
   *
   * @return The robots heading in degrees.
   */
  public double getHeading() {
    // return m_gyro.getAngle();
    return 1;
  }

}
