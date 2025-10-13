// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//WPILIB Imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//Robot Imports
import frc.robot.Constants.OIConstants.JoystickDriverConstants;
import frc.robot.Limelight.LimelightHelpers;
import frc.robot.Utils.PoseInterface;
import frc.robot.Utils.TouchScreenInterface;
import frc.robot.subsystems.LedControl;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Driver Controller
  public static CommandXboxController driverJoystick = new CommandXboxController(
      JoystickDriverConstants.kDriverControllerPort);

  // Put all subsystems here...
  public static LimelightHelpers limelightHelpers = new LimelightHelpers();
  public static LedControl ledControl = new LedControl();

  //touchScreenInterface instance
  public static TouchScreenInterface touchInterf = new TouchScreenInterface();
  public static PoseInterface poseInterf = new PoseInterface();

  // Swerve Drive
  public static SwerveSubsystem swerveDrive = new SwerveSubsystem();

  // Put all commands here...

  // Create an auto chooser to autonomus mode
  //SendableChooser<Command> chooserAuto;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //SmartDashboard.putData("Autonomous", chooserAuto);

    //chooserAuto = AutoBuilder.buildAutoChooser();
    // Configure the button bindings
    configureBindings();

    // *******set default command to drive with joystick************/
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    swerveDrive.setDefaultCommand(new RunCommand(() -> swerveDrive.driveRobotOriented(() -> driverJoystick.getLeftY(),
        () -> driverJoystick.getLeftX(),
        () -> -driverJoystick.getRightX(),
        () -> true),
        swerveDrive)// .onlyIf(()-> !driverJoystick.getHID().getXButton())
    );
  }

  /** 
   * use this method to configure triggers and operations 
   * on teleop mode
   */
  private void configureBindings() {
    /**** Swerve Commands ****/
    // Changes robot speed - slow, fast, max
    driverJoystick.leftBumper().whileTrue(
        new InstantCommand(() -> {
          swerveDrive.robotSlower();
          ledControl.setWhiteLed();
        }));
    driverJoystick.rightBumper().whileTrue(
        new InstantCommand(() -> {
          swerveDrive.robotFast();
          ledControl.setYellowLed();
        }));
    driverJoystick.leftBumper().and(driverJoystick.rightBumper())
        .whileFalse(new InstantCommand(() -> {
          swerveDrive.robotMaxSpeed();
          ledControl.setRedLed();
        }));

    // zero heading - swerve modules to 0 degrees
    driverJoystick.start().onTrue(new InstantCommand(swerveDrive::zeroHeading));


    /**** Teleop Commands ****/
    //exemple driveToThePoint command with interface position
    driverJoystick.x().whileTrue(
                swerveDrive.driveToThePoint(()-> poseInterf.GetPoseChoosed()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
