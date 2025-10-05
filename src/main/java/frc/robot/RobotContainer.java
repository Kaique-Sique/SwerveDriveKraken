// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//WPILIB Imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//Robot Imports
import frc.robot.Constants.OIConstants.JoystickDriverConstants;
import frc.robot.Limelight.LimelightHelpers;
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
  // The robot's subsystems and commands are defined here...
  // Operator Controller

  public static CommandXboxController driverJoystick = new CommandXboxController(
      JoystickDriverConstants.kDriverControllerPort);

  public static CommandXboxController operatorJoystick = new CommandXboxController(1);

  // Put all subsystems here
  public static LimelightHelpers limelightHelpers = new LimelightHelpers();
  public static LedControl ledControl = new LedControl();

  // Swerve Drive
  public static SwerveSubsystem swerveDrive = new SwerveSubsystem();

  SendableChooser<Command> chooserAuto;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("Autonomous", chooserAuto);

    // Configure the button bindings
    configureBindings();

    // *******set default command to drive with joystick************/
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    /**
     * DEFAULT COMMANDS
     */
    swerveDrive.setDefaultCommand(new RunCommand(() -> swerveDrive.driveRobotOriented(() -> driverJoystick.getLeftY(),
        () -> driverJoystick.getLeftX(),
        () -> -driverJoystick.getRightX(),
        () -> true),
        swerveDrive)// .onlyIf(()-> !driverJoystick.getHID().getXButton())
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
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

    // here you can implement generic positons to go to with buttons
    if (swerveDrive.getBlueAlliance()) {
      // here to blue aliance positions
    } else {
      // here to red aliance positions
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // choose the auto command from dashboard
    return chooserAuto.getSelected();
  }
}
