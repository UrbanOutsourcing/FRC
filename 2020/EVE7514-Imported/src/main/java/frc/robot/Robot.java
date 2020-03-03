/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  Command m_autonomousCommand;

  public static DriveTrain m_drivetrain;
  public static eDriveTrain m_edrivetrain;
  public static mDriveTrain m_mdrivetrain;
  public static Pivot m_pivot;
  public static Lift m_lift;
  public static Intake m_intake;
  public static GearShift m_gearshift;
  public static LiftArmPivot m_liftarmpivot;
  public static Hook m_hook;
  public static OI m_oi;
  

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {//
    // Initialize all subsystems
    m_drivetrain = new DriveTrain();
    //m_edrivetrain = new eDriveTrain();
     //m_edrivetrain = new mDriveTrain();
    m_pivot = new Pivot();
    m_lift = new Lift();
    m_intake = new Intake();
    m_gearshift = new GearShift();
    m_oi = new OI();
    m_liftarmpivot = new LiftArmPivot();
    m_hook = new Hook();

    //CameraServer.getInstance().startAutomaticCapture();

    // instantiate the command used for the autonomous period

    // Show what command your subsystem is running on the SmartDashboard
    //SmartDashboard.putData(m_edrivetrain);
    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_lift);
    SmartDashboard.putData(m_pivot);
    SmartDashboard.putData(m_gearshift);
    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_liftarmpivot);
    SmartDashboard.putData(m_hook);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand.start(); // schedule the autonomous command (example)
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    log();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //m_autonomousCommand.cancel();
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    log();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * The log method puts interesting information to the SmartDashboard.
   */
  private void log() {
    m_lift.log();
    m_pivot.log();
    m_drivetrain.log();
    //m_edrivetrain.log();
    //m_mdrivetrain.log();
    m_intake.log();
    m_gearshift.log();
    m_liftarmpivot.log();
    m_hook.log();

  }
}
