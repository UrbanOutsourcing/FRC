/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final Spark m_motor = new Spark(RobotMap.SHOOTER);
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public Shooter() {
    super();

    // Let's name everything on the LiveWindow
    addChild("Motor", m_motor);
     
  }
  public void eject() {
    m_motor.set(1);
  }
  public void intake() {
    m_motor.set(-1);
  }
  public void stop() {
    m_motor.set(0);
  }
  public void log() {
    SmartDashboard.putNumber("Shooter Speed", m_motor.get());
    
  }
}
