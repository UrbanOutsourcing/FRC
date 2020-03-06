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
public class Climber extends Subsystem {
  
  private final WPI_VictorSPX m_motor3;
 

  
  
  //private final WPI_VictorSPX m_motor = new WPI_VictorSPX(7);
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ClimberWithJoystick());
  }
  public Climber() {
    super();

    // Let's name everything on the LiveWindow
    m_motor3 = new WPI_VictorSPX(RobotMap.CLIMBER);
   
  }
  
  public void move(double power) {
    
    m_motor3.set(ControlMode.PercentOutput, power);
  }
  public void stop() {
    
    m_motor3.set(ControlMode.PercentOutput, 0);
  
  }
  public void log() {
   
    SmartDashboard.putNumber("Intake Power", m_motor3.getMotorOutputPercent());
    
  }
}
