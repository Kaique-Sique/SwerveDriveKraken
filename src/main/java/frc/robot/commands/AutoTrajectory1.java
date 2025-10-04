// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTrajectory1 extends SequentialCommandGroup 
{

  private static SwerveSubsystem swerveDrive = RobotContainer.swerveDrive;
  /** Creates a new AutoTrajectory1. */
  public AutoTrajectory1() 
  {
    // Create config for trajectory
    TrajectoryConfig config = 
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                            .setKinematics(DriveConstants.kDriveKinematics);

     // An example trajectory to follow. All units in meters.
    Trajectory firstTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            //Pose2d.kZero,
            new Pose2d(1.1,2.58, new Rotation2d(Units.degreesToRadians(180))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1.48, 3.0), new Translation2d(1.75, 3.87)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2.79, 4.00, new Rotation2d(Units.degreesToRadians(0))),
            config);
    Trajectory secondTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            //Pose2d.kZero,
            new Pose2d(2.79,4.0, new Rotation2d(Units.degreesToRadians(0))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1.75, 4.08), new Translation2d(1.37, 4.48)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1.13, 4.91, new Rotation2d(Units.degreesToRadians(-90))),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            firstTrajectory,
            swerveDrive::getPoseEstimator, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            swerveDrive::setModuleStates,
            swerveDrive);
    SwerveControllerCommand swerveControllerCommand2 =
            new SwerveControllerCommand(
                secondTrajectory,
                swerveDrive::getPoseEstimator, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                swerveDrive::setModuleStates,
                swerveDrive);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands
    (
      new InstantCommand (()-> swerveDrive.resetOdometry(firstTrajectory.getInitialPose())),
      swerveControllerCommand,
      //new WaitCommand(1.0),
      swerveControllerCommand2,	
      new InstantCommand(swerveDrive::stopModules)



    );
  }
}
