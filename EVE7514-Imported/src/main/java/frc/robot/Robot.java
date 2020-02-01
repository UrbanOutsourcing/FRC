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

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.cameraserver.*;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.HatchArm;
import frc.robot.commands.*;
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

  
  public static  DriveTrain m_drivetrain;
  public static Pivot m_pivot;
  public static Shooter m_shooter;
  public static HatchArm m_hatcharm;
  public static HabClimb m_habclimb;
  public static OI m_oi;
  
  

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {//
    // Initialize all subsystems
    m_drivetrain = new DriveTrain();
   
    m_pivot = new Pivot();
    m_shooter = new Shooter();
    m_hatcharm = new HatchArm();
    m_habclimb = new HabClimb();
    m_oi = new OI();
    m_autonomousCommand = new EVEAuto();
    //CameraServer.getInstance().startAutomaticCapture();

    // CameraServer Instantiation 

    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);
      
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("EVECam1", 640, 480);
      
      Mat source = new Mat();
      Mat output = new Mat();
      
      while(!Thread.interrupted()) {
          cvSink.grabFrame(source);
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }
  }).start();


    // Show what command your subsystem is running on the SmartDashboard
    SmartDashboard.putData(m_drivetrain);
    //SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_pivot);
    SmartDashboard.putData(m_hatcharm);
    SmartDashboard.putData(m_shooter);
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
    
    //teleopPeriodic();
    Scheduler.getInstance().run();
    log();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_autonomousCommand.cancel();
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
    m_pivot.log();
    //m_drivetrain.log();
    m_drivetrain.log();
    m_shooter.log();
    m_hatcharm.log();

  }
}
