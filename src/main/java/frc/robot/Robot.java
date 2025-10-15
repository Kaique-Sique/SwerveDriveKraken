// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Limelight.LimelightHelpers;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * MNL 06/26/2025
   * This is the constructor for the Robot class. It sets the period for the robot
   * to 10ms (100Hz).
   */

  // Creates UsbCamera
  UsbCamera usbCamera;

  public Robot() {
    // Creates UsbCamera
    // Configure camera
    usbCamera = CameraServer.startAutomaticCapture();
    // Set the resolution
    usbCamera.setResolution(640, 480);
    // Set the frames per second
    usbCamera.setFPS(30);
    // Set the pixel format
    usbCamera.setPixelFormat(PixelFormat.kMJPEG);

    usbCamera.setExposureAuto();
    usbCamera.setBrightness(10);
    usbCamera.setExposureManual(50);
    usbCamera.setWhiteBalanceAuto();
    // Call the superclass constructor with a period of 0.01 seconds (10ms).
    // super(0.01); // Set the period to 10ms (100Hz)
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // This command will not control your robot, it will simply run through
    // a full path following command to warm up the library.
    FollowPathCommand.warmupCommand().schedule();
    PathfindingCommand.warmupCommand().schedule();
    // Reset all encoders
    RobotContainer.swerveDrive.zeroHeading();
    /**
     * Zeroing / Seeding Limelight initial orientation -
     * To reset the internal IMU's fused robot yaw to the yaw submitted via
     * SetRobotOrientation()
     * IMU Modes:
     * 0 - Use external IMU yaw submitted via SetRobotOrientation() for MT2
     * localization. The internal IMU is ignored entirely.
     * 1 - Use external IMU yaw submitted via SetRobotOrientation(), and configure
     * the LL4 internal IMU's fused yaw to match the submitted yaw value.
     * 2 - Use internal IMU for MT2 localization.
     */
    LimelightHelpers.SetIMUMode(DriveConstants.limelightFront, 1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Batery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    // System.out.println("Touch X: " +
    // SelectScore.poseToScore(RobotContainer.touchScreenInterface).getX());
    // System.out.println("Touch Y: " +
    // SelectScore.poseToScore(RobotContainer.touchScreenInterface).getY());
    // System.out.println("Touch Rot: " +
    // SelectScore.poseToScore(RobotContainer.touchScreenInterface).getRotation().getDegrees());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.touchInterf.disableInterface();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    LimelightHelpers.SetIMUMode(DriveConstants.limelightFront, 2);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.touchInterf.enableInterface();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
