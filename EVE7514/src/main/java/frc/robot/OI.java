/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.commands.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private final Joystick m_drive_joystick = new Joystick(RobotMap.OI_DRIVER_CONTROLLER);
  public final Joystick m_joystick = new Joystick(RobotMap.OI_ATTACHMENTS_CONTROLLER);
  public final XboxController m_xjoystick = new XboxController(RobotMap.OI_ATTACHMENTS_CONTROLLER);
  

  /**
   * Construct the OI and all of the buttons on it.
   */
  public OI() {
    // Put Some buttons on the SmartDashboard
    
    SmartDashboard.putData("Hatch Arm Deploy", new HatchArmDeploy());
    SmartDashboard.putData("Hatch Arm Retract", new HatchArmRetract());
    SmartDashboard.putData("Hab Climb Deploy", new HabClimbDeploy());
    SmartDashboard.putData("Hab Climb Retract", new HabClimbRetract());
    SmartDashboard.putData("Pivot  45", new SetPivotSetpoint(45));
    SmartDashboard.putData("Pivot  90", new SetPivotSetpoint(720));
    SmartDashboard.putNumber("Motor Damp",Constants.kMotorDamp);
    
    //SmartDashboard.putData("Drive Straight 12", new eDriveStraight(12));

    // Gamepad 1 Buttons
    
    final JoystickButton rtrigger = new JoystickButton(m_drive_joystick, 3);
    final JoystickButton ltrigger = new JoystickButton(m_drive_joystick, 2);
    final JoystickButton rbumper = new JoystickButton(m_drive_joystick, 6);    
    final JoystickButton lbumper = new JoystickButton(m_drive_joystick, 5);
    // Gamepad 2 Buttons
    
    final JoystickButton buttona = new JoystickButton(m_joystick, 1);
    final JoystickButton buttonx = new JoystickButton(m_joystick, 2);
    final JoystickButton buttonb = new JoystickButton(m_joystick, 3);
    final JoystickButton buttony = new JoystickButton(m_joystick, 4);

    
    

    final JoystickButton dpadUp = new JoystickButton(m_joystick, 7);
    final JoystickButton dpadDown = new JoystickButton(m_joystick, 8);
    final JoystickButton l2 = new JoystickButton(m_joystick, 9);
    final JoystickButton r2 = new JoystickButton(m_joystick, 10);
    final JoystickButton l1 = new JoystickButton(m_joystick, 11);
    final JoystickButton r1 = new JoystickButton(m_joystick, 12);

    // Connect the buttons to commands
    
    buttona.whenPressed(new HatchArmDeploy());
    buttony.whenPressed(new HatchArmRetract());
    buttonx.whenPressed(new HabClimbDeploy());
    buttonb.whenPressed(new HabClimbRetract());


    rbumper.whenPressed(new GearShift(-1));
    lbumper.whenPressed(new GearShift(1));
    
       
  }

  public Joystick getJoystick() {
    return m_drive_joystick;
  }
}
