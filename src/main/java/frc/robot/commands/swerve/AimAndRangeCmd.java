// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;





// WPI imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// Robot imports
import frc.robot.RobotContainer;

import frc.robot.Limelight.LimelightHelpers;

import frc.robot.subsystems.Swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndRangeCmd extends Command 
{
  // Instantiate the object
  private final static SwerveSubsystem swerveDrive = RobotContainer.swerveDrive;
  
  private  Pose2d targetPosition = new Pose2d(2.79, 4.0, Rotation2d.fromDegrees(0)); // Example target position, replace with actual values
    
  private static String limeLightNameFront = "limelight-front";

 

  // Variables
  //private final double kX, kY, kRot; // Replace with the correct class ID for the game piece
  private static boolean finish = false; 

  //Max speed variables
  private final double kMaxSpeed2Drive = 0.5; //0.15;  // Simple speed limit so we don't drive too fast
  private final double kMaxSpeed2Steer = 0.5; //0.15
  private final double kMaxSpeed2Rot = 0.5; //0.05

  //PID offset variables
  private final double kPsteer = 2.0;  //2.0                  // how hard to turn toward the target
  private final double kPdrive = 2.0; //2.0 
  private final double kProt = 0.1; //0.01//0.26 

  //PID controller
  private PIDController pidDrive =  new PIDController(kPdrive, 0, 0);
  private PIDController pidSteer = new PIDController(kPsteer,0,0);
  private PIDController pidRot = new PIDController(kProt,0,0);

  //Trapezoidal constants
  //private static double kDt = 0.02; //The period between controller updates in seconds. 
  //private static double kMaxVelocity = 4;
  //private static double kMaxAcceleration = 1;
  

  //private final TrapezoidProfile.Constraints m_constraints =  new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  //private final ProfiledPIDController mController = new ProfiledPIDController(KP, KI, KD, m_constraints,kDt);

  

  //Trajectory.State goal = trajectory.sample(3.4);

  //double kX, double kY , double kRot
  /** Creates a new AimAndRangeCmd. */
  public AimAndRangeCmd()
  {
    //AimingUtils.trackReefPosition(limelightName,Id1, Id2);
   

    // Set the class ID
    /*this.kX = kX;
    this.kY = kY;
    this.kRot = kRot;*/
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
      // Switch to pipeline 1
    //LimelightHelpers.setPipelineIndex(limelightName, 0);
    System.out.println("Aim and Range Command Started!!");

    
    // Set variables
    finish = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    LimelightHelpers.PoseEstimate frontLimelight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(DriveConstants.limelightFront);
    // Get the current robot pose from the pose estimator
    robotPosition();
    Pose2d distanceToTarget = LimelightHelpers.getBotPose2d(limeLightNameFront);

    if (frontLimelight.tagCount > 0  && frontLimelight.avgTagDist <3 )
    {
        System.out.println("Aim and Range initialized!!");
            double steer_cmd = kMaxSpeed2Steer * pidSteer.calculate(robotPosition().getY(),targetPosition.getY());
            // try to drive forward until the target area reaches our desired area
            //double drive_cmd = MathUtil.clamp(pidDrive.calculate(distanceToTarget, kDistanceSetpoint),-kMaxSpeed2Drive,kMaxSpeed2Drive);
            double drive_cmd = kMaxSpeed2Drive *pidDrive.calculate(robotPosition().getX(),targetPosition.getX());

            double rot_cmd = kMaxSpeed2Rot *pidRot.calculate(robotPosition().getRotation().getDegrees(),targetPosition.getRotation().getDegrees());
            // Drive the swerve subsystem with the calculated speeds
            
            // try to drive forward until the target area reaches our desired area
            //double drive_cmd = MathUtil.clamp(pidDrive.calculate(distanceToTarget, kDistanceSetpoint),-kMaxSpeed2Drive,kMaxSpeed2Drive);
        
            // Drive the swerve subsystem with the calculated speeds
            swerveDrive.driveToTarget(() -> drive_cmd, () -> steer_cmd, () -> rot_cmd, () -> true);

            // Get the adjusted speeds. Here, we want the robot to be facing
            // 70 degrees (in the field-relative coordinate system).
            //ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(drive_cmd, steer_cmd, rot_cmd, swerveDrive.getGyroRotation2d());
            
            //swerveDrive.drive(adjustedSpeeds);

            if (frontLimelight.avgTagDist < 0.02 && distanceToTarget.getRotation().getDegrees()<0.1)
            
            //if (distanceToTarget.getX() < 0.01 && distanceToTarget.getY() < 0.01 && distanceToTarget.getRotation().getDegrees() < 0.51)
            {
                finish = true;
                System.out.println("Aim and Range Command Done!!");
            }
            else
            {
                finish = false;
            }
    

    }
    
    else
     
    {
        finish =true;
        System.out.println("Out of Range Try again!!");
    }
    
    
      
  }

  //Helper method to calculate robot position

  /**
   * Method to get the robot's current position
   * @return robotPosition
   */
  public Pose2d robotPosition()
  {

    // Get the current robot pose from the pose estimator
    Pose2d currentPose = swerveDrive.getPoseEstimator();

    // Get the robot's current position
    //Translation2d robotPosition = currentPose.getTranslation();
    //targetPosition = currentPose;
    return currentPose;
  }
 
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    swerveDrive.stopModules();
    System.out.println("Motors stopped!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
   
        
        return finish ;//|| (distanceToTarget.getX() < 0.1 && distanceToTarget.getY() < 0.1 && distanceToTarget.getRotation().getDegrees() < 5); // Example threshold distance in meters
  }

}
