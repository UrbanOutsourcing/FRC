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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.commands.*;

/**
 * The lift subsystem uses PID to go to a given height. Unfortunately, in
 * it's current state PID values for simulation are different than in the real
 * world do to minor differences.
 */
public class Pivot extends PIDSubsystem {
  private final TalonSRX m_motor;
  private final Encoder m_encoder;
  //public final DigitalInput m_toplimitswitch,m_bottomlimitswitch ;
  private static final double kP_real = 4;
  private static final double kI_real = 0.07;
  private static final double kP_simulation = 18;
  private static final double kI_simulation = 0.2;

  /**
   * Create a new lift subsystem.
   */
  public Pivot() {
    super(kP_real, kI_real, 0);
    if (Robot.isSimulation()) { // Check for simulation and update PID values
      getPIDController().setPID(kP_simulation, kI_simulation, 0, 0);
    }
    setAbsoluteTolerance(0.005);

    m_motor = new TalonSRX(RobotMap.PIVOT);
    m_encoder = new Encoder(RobotMap.PIVOT_CHANNELA, RobotMap.PIVOT_CHANNELB);
    
    //m_toplimitswitch = new DigitalInput(RobotMap.TOP_LIMITSWITCH);
    //m_bottomlimitswitch = new DigitalInput(RobotMap.BOTTOM_LIMITSWITCH);

    // Conversion value of potentiometer varies between the real world and
    // simulation
    if (Robot.isReal()) {
      m_encoder.setDistancePerPulse(0.042);
      
    } else {
      // Circumference in ft = 4in/12(in/ft)*PI
      m_encoder.setDistancePerPulse((4.0 / 12.0 * Math.PI) / 360.0);
      
    }
      /*TalonSRX Encoder Configuration

      /* Config the sensor used for Primary PID and sensor direction 
      m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
      Constants.kPIDLoopIdx,
      Constants.kTimeoutMs);

      /* Ensure sensor is positive when output is positive 
      m_motor.setSensorPhase(Constants.kSensorPhase);

      /**
      * Set based on what direction you want forward/positive to be.
      * This does not affect sensor phase. 
      */
      m_motor.setInverted(Constants.kMotorInvert);

      /* Config the peak and nominal outputs, 12V means full 
      m_motor.configNominalOutputForward(0, Constants.kTimeoutMs);
      m_motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
      m_motor.configPeakOutputForward(1, Constants.kTimeoutMs);
      m_motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
      */
		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		  //m_motor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. 
		m_motor.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		m_motor.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		m_motor.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		m_motor.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    */
       
    
    // Let's name everything on the LiveWindow
      //addChild("Encoder", m_encoder);
  }

  @Override
  public void initDefaultCommand() {
   // setDefaultCommand(new PivotWithJoystick());
  }

  /**
   * The log method puts interesting information to the SmartDashboard.
   */
  public void log() {
    SmartDashboard.putData("Pivot Encoder", (Encoder) m_encoder);
    SmartDashboard.putNumber("Pivot Power", m_motor.getMotorOutputPercent());
  }

  /**
   * Use the encoder as the PID sensor. This method is automatically
   * called by the subsystem.
   */
  @Override
  protected double returnPIDInput() {
    return m_encoder.getDistance();
    //return m_motor.getSensorCollection().getQuadraturePosition();
    
  }

  /**
   * Use the motor as the PID output. This method is automatically called by
   * the subsystem.
   */
  @Override
  protected void usePIDOutput(double power) {
    m_motor.set(null, power);
  }
  public void move(double power) {
    
    m_motor.set(ControlMode.PercentOutput, power/2);
    
  }
  public void moveto(double degrees) {
          
    m_motor.set(ControlMode.Position, (degrees/360)*4096);
    
  }
}
