/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  
  private final WPI_VictorSPX m_motor1;
  private final WPI_VictorSPX m_motor2;

  
  
  //private final WPI_VictorSPX m_motor = new WPI_VictorSPX(7);
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeWithJoystick());
  }
  public Intake() {
    super();

    // Let's name everything on the LiveWindow
    m_motor1 = new WPI_VictorSPX(RobotMap.INTAKE1);
    m_motor2 = new WPI_VictorSPX(RobotMap.INTAKE2);
     
  }
  
  public void move(double power) {
    
    m_motor1.set(ControlMode.PercentOutput, power);
    m_motor2.set(ControlMode.PercentOutput, -power);
  }
  public void stop() {
    
    m_motor1.set(ControlMode.PercentOutput, 0);
    m_motor2.set(ControlMode.PercentOutput, 0);

  }
  public void log() {
   
    SmartDashboard.putNumber("Intake Power", m_motor1.getMotorOutputPercent());
    
  }
}
