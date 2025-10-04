// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Java Imports
import com.pathplanner.lib.auto.AutoBuilder;

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
import frc.robot.Constants.Pose2dConstansts.BluePose2d;
import frc.robot.Constants.Pose2dConstansts.RedsPose2d;
import frc.robot.Limelight.LimelightHelpers;
import frc.robot.Utils.touchScreenInterface;
import frc.robot.commands.AutoTrajectory1;
import frc.robot.commands.RampCmd;
import frc.robot.commands.autos.redTest;
import frc.robot.commands.liftCommands.liftHome;
import frc.robot.commands.liftCommands.liftLevelDashboard;
import frc.robot.commands.sequencialPositions.normalPosition;
import frc.robot.commands.sequencialPositions.rampIntakePosition;
import frc.robot.commands.swerve.AimAndRangeCmd;
import frc.robot.commands.swerve.AlignToReefTagRelative;
import frc.robot.subsystems.LedControl;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.algae.algaeArticulador;
import frc.robot.subsystems.coral.ArticuladorCoral;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.RampSubsystem;
import frc.robot.subsystems.liftSubsystem.LiftSubsystem;

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

  //articulador Coral
  public static ArticuladorCoral aCoral = new ArticuladorCoral();

  /**
   * COMMANDS
   */
  /**
   * SEQUENTIAL COMMANDS
   */
  public static AutoTrajectory1 autoTrajectory1 = new AutoTrajectory1();
  // public static ScoreAutoCmd scoreAutoCmd = new ScoreAutoCmd();
  // Swerve commands
  public static AimAndRangeCmd aimAndRangeCmd = new AimAndRangeCmd();
  public static AlignToReefTagRelative alignToReefTagRelative = new AlignToReefTagRelative(true, swerveDrive);

  //Lift Subsystem
  public static final LiftSubsystem liftSubsystem = new LiftSubsystem();

  public static final Coral m_coral = new Coral();

  public static final algaeArticulador aAlgae = new algaeArticulador();

  //public static final AlgleIntake algaeCmd = new AlgleIntake(0);

  public static RampSubsystem m_rampSubsystem = new RampSubsystem();

  public static final RampCmd m_rampCmd = new RampCmd(0);

  public static final rampIntakePosition coralPosition = new rampIntakePosition();
  public static final normalPosition normal_positon = new normalPosition();


  public static final touchScreenInterface m_interface = new touchScreenInterface();

  // public static PathCoralStationCmd pathCoralStationAuto = new
  // PathCoralStationCmd(1.5);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // public static XboxController driverJoystick;
  // Controllers


  /**
   * Declare class SendableChoser()
   * Pop up selection of option on the SmartDashBoard
   */
  SendableChooser<Command> chooserAuto;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Swerve Commands
    aimAndRangeCmd.addRequirements(swerveDrive);
    alignToReefTagRelative.addRequirements(swerveDrive);

    m_rampCmd.addRequirements(m_rampSubsystem);
    
    //algaeCmd.addRequirements(aAlgae);

    autoTrajectory1.addRequirements(swerveDrive);

    // Build an auto chooser. This will use Commands.none() as the default option.
    chooserAuto = AutoBuilder.buildAutoChooser();
    coralPosition.addRequirements(m_rampSubsystem, aCoral, aAlgae);
    normal_positon.addRequirements(m_rampSubsystem, aCoral, aAlgae);

    SmartDashboard.putData("Autonomous", chooserAuto);

    // Configure the button bindings
    configureBindings();

    // *******set default command to drive with joystick************/
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.

    /**
     * DEFAULT COMMANDS
     */
    // liftSubsystem.setDefaultCommand(liftCmd);

    // Runnable command
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
    /**
     * XBOX CONTROLLER BUTTONS
     */

    //new Trigger(() -> liftSubsystem.isL1()).onTrue(new RunCommand(() -> 
      //                aCoral.holdMotorsOnPosition(57), aCoral));

    //new Trigger(() -> driverJoystick.y().getAsBoolean()).onTrue(new RunCommand(() -> 
    //                  aCoral.holdMotorsOnPosition(0), aCoral));

    // Swerve Drive Buttons
    // Changes robot speed - speeding up
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

    driverJoystick.start().onTrue(new InstantCommand(swerveDrive::zeroHeading));

    driverJoystick.leftBumper().and(driverJoystick.rightBumper())
        .whileFalse(new InstantCommand(() -> {
          swerveDrive.robotMaxSpeed();
          ledControl.setRedLed();
        }));

    //lift commands
    //lift to home position
    driverJoystick.y().onTrue(
      new RunCommand(() -> aCoral.holdMotorsOnPosition(0), aCoral).withTimeout(0.5)
        .andThen(new liftHome())
        .andThen(new RunCommand(() -> aCoral.holdMotorsOnPosition(0), aCoral).withTimeout(1)));

    //lift to dashboard position
    driverJoystick.a().onTrue(new liftLevelDashboard().andThen(new RunCommand(() -> 
        aCoral.holdMotorsOnPosition(57), aCoral).withTimeout(2))
       .alongWith(new InstantCommand (()-> swerveDrive.robotLiftVelocity(), swerveDrive)));

    //driverJoystick.b().onTrue(new RunCommand(() -> aCoral.holdMotorsOnPosition(57), aCoral).withTimeout(0.8));


    //position commands
    //driverJoystick.x().onTrue(new RunCommand(() -> aCoral.holdMotorsOnPosition(0), aCoral).withTimeout(1));
    if (swerveDrive.getBlueAlliance())
    {
        driverJoystick.x().whileTrue(
                swerveDrive.driveToThePoint(()-> BluePose2d.coralA));

        driverJoystick.b().whileTrue(
              swerveDrive.driveToThePoint(()-> BluePose2d.driverStation)
              .alongWith(new RunCommand(() -> aCoral.holdMotorsOnPosition(57), aCoral).withTimeout(1)));
    }
    else
    {
        driverJoystick.x().whileTrue(
                swerveDrive.driveToThePoint(()-> RedsPose2d.coralA));


        driverJoystick.b().whileTrue(
              swerveDrive.driveToThePoint(()-> RedsPose2d.driverStation)
              .alongWith(new RunCommand(() -> aCoral.holdMotorsOnPosition(57), aCoral).withTimeout(1)));
    }


    //triggers comands
    new Trigger(()-> liftSubsystem.isHomed() && !aCoral.getCoralInput()).onTrue(
        new RunCommand(()-> aCoral.holdMotorsOnPosition(0), aCoral).withTimeout(0.8));

    new Trigger(()-> liftSubsystem.isL1() && aCoral.getCoralInput() && m_coral.coralShooter()).onTrue(
      new RunCommand(() -> aCoral.holdMotorsOnPosition(0), aCoral).withTimeout(0.5)
        .andThen(new liftHome()));

    new Trigger(() -> liftSubsystem.isHomed()).onTrue(new InstantCommand(()-> m_coral.resetEncoder()));


    //// operator controls
    operatorJoystick.b().onTrue(new InstantCommand(()-> m_coral.resetEncoder(), m_coral)
      .andThen(new RunCommand(()-> m_coral.setPosition(5), m_coral).withTimeout(1)
      .andThen(new InstantCommand(()-> m_coral.stopMotors(), m_coral))));

    operatorJoystick.x().onTrue(new InstantCommand(()-> m_coral.resetEncoder(), m_coral)
        .andThen(new RunCommand(()-> m_coral.setPosition(-5), m_coral).withTimeout(1)
        .andThen(new InstantCommand(()-> m_coral.stopMotors(), m_coral))));

    operatorJoystick.leftBumper().onTrue(new rampIntakePosition());
    operatorJoystick.rightBumper().onTrue(new normalPosition());

    //operatorJoystick.povUp().onTrue(new AlgleIntake(0));
    operatorJoystick.povDown().whileTrue(new RunCommand(() -> aAlgae.setVelocity(0.07), aCoral)
    ).onFalse(
      new InstantCommand(() -> aAlgae.stopMotors(), aAlgae)
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return chooserAuto.getSelected();
    return new redTest();
  }
}
