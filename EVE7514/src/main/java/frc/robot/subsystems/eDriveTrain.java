/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
public class eDriveTrain extends Subsystem {
  private final TalonSRX m_leftrear;
  private final TalonSRX m_leftmaster;
  private final TalonSRX m_rightrear;
  private final TalonSRX m_rightmaster;

  private final SpeedController m_leftMotor = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_FRONT),
      new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_BACK));
  private final SpeedController m_rightMotor = new SpeedControllerGroup(
      new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_FRONT), new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_BACK));

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  //private final AnalogGyro m_gyro = new AnalogGyro(2);
  

  /**
   * Create a new pivot subsystem.
   */
  public eDriveTrain() {
    super();

    m_leftmaster = new TalonSRX(RobotMap.DRIVETRAIN_LEFT_FRONT);
    m_rightmaster = new TalonSRX(RobotMap.DRIVETRAIN_RIGHT_FRONT);
    m_leftrear = new TalonSRX(RobotMap.DRIVETRAIN_LEFT_BACK);
    m_rightrear = new TalonSRX(RobotMap.DRIVETRAIN_RIGHT_BACK);

    // *TalonSRX Encoder Configuration
    // *Config the sensor used for Primary PID and sensor direction

    m_leftmaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.PID_PRIMARY,
        Constants.kTimeoutMs);
    m_rightmaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.PID_PRIMARY,
        Constants.kTimeoutMs);

    m_leftrear.configRemoteFeedbackFilter(m_leftmaster.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor,
        Constants.REMOTE_0, Constants.kTimeoutMs);
    m_rightrear.configRemoteFeedbackFilter(m_rightmaster.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor,
        Constants.REMOTE_0, Constants.kTimeoutMs);

    /* Setup Sum signal to be used for Distance */
    m_leftmaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs); // Feedback
                                                                                                        // Device of
                                                                                                        // Remote Talon
    m_leftmaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs); // Quadrature
                                                                                                                   // Encoder
                                                                                                                   // of
                                                                                                                   // current
                                                                                                                   // Talon

    /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
    m_leftmaster.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY, Constants.kTimeoutMs);

    /* Scale Feedback by 0.5 to half the sum of Distance */
    m_leftmaster.configSelectedFeedbackCoefficient(0.5, // Coefficient
        Constants.PID_PRIMARY, // PID Slot of Source
        Constants.kTimeoutMs); // Configuration Timeout

    /* Setup Sum signal to be used for Distance */
    m_rightmaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs); // Feedback
                                                                                                         // Device of
                                                                                                         // Remote Talon
    m_rightmaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs); // Quadrature
                                                                                                                    // Encoder
                                                                                                                    // of
                                                                                                                    // current
                                                                                                                    // Talon

    /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
    m_rightmaster.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY, Constants.kTimeoutMs);

    /* Scale Feedback by 0.5 to half the sum of Distance */
    m_rightmaster.configSelectedFeedbackCoefficient(0.5, // Coefficient
        Constants.PID_PRIMARY, // PID Slot of Source
        Constants.kTimeoutMs); // Configuration Timeout

    /* Configure output and sensor direction */
    m_leftmaster.setInverted(false);
    m_leftmaster.setSensorPhase(true);
    m_rightmaster.setInverted(true);
    m_rightmaster.setSensorPhase(true);

    /* Set status frame periods to ensure we don't have stale data */
    m_rightmaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
    m_rightmaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    m_leftmaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

    /* Configure neutral deadband */
    m_rightmaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
    m_leftmaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

    /* Motion Magic Configurations */
    m_rightmaster.configMotionAcceleration(2000, Constants.kTimeoutMs);
    m_rightmaster.configMotionCruiseVelocity(2000, Constants.kTimeoutMs);

    /**
     * Max out the peak output (for all modes). However you can limit the output of
     * a given PID object with configClosedLoopPeakOutput().
     */
    m_leftmaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
    m_leftmaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
    m_rightmaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
    m_rightmaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

    /* FPID Gains for Motion Magic servo */
    m_rightmaster.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
    m_rightmaster.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
    m_rightmaster.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
    m_rightmaster.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
    m_rightmaster.config_IntegralZone(Constants.kSlot_Distanc, Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
    m_rightmaster.configClosedLoopPeakOutput(Constants.kSlot_Distanc, Constants.kGains_Distanc.kPeakOutput,
        Constants.kTimeoutMs);
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

    SmartDashboard.putNumber("DriveTrain Right Target", m_rightmaster.getClosedLoopTarget(Constants.PID_PRIMARY));
    SmartDashboard.putNumber("DriveTrain Right Position",  m_rightmaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
    SmartDashboard.putNumber("DriveTrain Right Target", m_leftmaster.getClosedLoopTarget(Constants.PID_PRIMARY));
    SmartDashboard.putNumber("DriveTrain Left Target", m_leftmaster.getClosedLoopTarget(Constants.PID_PRIMARY));
    SmartDashboard.putNumber("DriveTrain Left Position",  m_rightmaster.getSelectedSensorPosition(Constants.PID_PRIMARY));
  }

  /* Zero quadrature encoders on Talons */
  void zeroSensors() {
    m_leftmaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
    m_rightmaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
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
    m_drive.tankDrive(left, right);
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
    m_drive.arcadeDrive(left, rotate);
  }

  public void driveto(double distance) {
    SmartDashboard.putNumber("Drive Distance", distance);

    /* calculate targets from gamepad inputs */
    double target_sensorUnits = (distance / 12) * Constants.kRotationsPerInch;

    m_rightmaster.set(ControlMode.Position, target_sensorUnits);
    m_leftmaster.follow(m_rightmaster);
    m_leftrear.follow(m_rightmaster);
    m_rightrear.follow(m_rightmaster);
  }

  /**
   * Get the robot's heading.
   *
   * @return The robots heading in degrees.
   */
  public double getHeading() {
    //return m_gyro.getAngle();
    return 1;
  }

}
