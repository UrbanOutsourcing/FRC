/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.HatchArmDeploy;
import frc.robot.commands.HatchArmRetract;
import frc.robot.commands.SetLiftSetpoint;




/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private final Joystick m_drive_joystick = new Joystick(0);
  public final Joystick m_joystick = new Joystick(1);
  

  /**
   * Construct the OI and all of the buttons on it.
   */
  public OI() {
    // Put Some buttons on the SmartDashboard
    
    SmartDashboard.putData("Lift Bottom", new SetLiftSetpoint(0));
    SmartDashboard.putData("Lift  Middle", new SetLiftSetpoint(2));
    SmartDashboard.putData("Lift Top", new SetLiftSetpoint(4));
    SmartDashboard.putData("Hatch Arm Deploy", new HatchArmDeploy());
    SmartDashboard.putData("Hatch Arm Retract", new HatchArmRetract());
    
    // Create some buttons
    final JoystickButton dpadUp = new JoystickButton(m_joystick, 5);
    final JoystickButton dpadRight = new JoystickButton(m_joystick, 6);
    final JoystickButton dpadDown = new JoystickButton(m_joystick, 7);
    final JoystickButton dpadLeft = new JoystickButton(m_joystick, 8);
    final JoystickButton l2 = new JoystickButton(m_joystick, 9);
    final JoystickButton r2 = new JoystickButton(m_joystick, 10);
    final JoystickButton l1 = new JoystickButton(m_joystick, 11);
    final JoystickButton r1 = new JoystickButton(m_joystick, 12);

    // Connect the buttons to commands
    dpadUp.whenPressed(new SetLiftSetpoint(6));
    dpadDown.whenPressed(new SetLiftSetpoint(2));
    dpadLeft.whenPressed(new HatchArmDeploy());
    dpadRight.whenPressed(new HatchArmRetract());
  }

  public Joystick getJoystick() {
    return m_drive_joystick;
  }
}
