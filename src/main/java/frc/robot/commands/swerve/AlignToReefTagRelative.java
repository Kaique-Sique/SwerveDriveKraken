// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

//wpilib imports
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//local imports
import frc.robot.Limelight.LimelightHelpers;
import frc.robot.subsystems.Swerve.SwerveSubsystem;


public class AlignToReefTagRelative extends Command 
{
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = 18;//-1
  private boolean finish=false;
  
  private double setpointX = -0.81; // Vertical pose
  private double setpointY = -0.15; // Horizontal pose
  private double setpointRot = 2.70; // Angle pose

  private double errorToleranceX = 0.02; // Tolerance for X position
  private double errorToleranceY = 0.02; // Tolerance for Y position
  
  private double errorToleranceRot = 0.5; // Tolerance for rotation angle
  private double errorToleranceRotExt = 2.0; // Tolerance for rotation angle


  private final Debouncer autoEjectDebouncer = new Debouncer(0.1);

  //Trapezoidal constants
  //private static double kDt = 0.02; //The period between controller updates in seconds. 
  //private static double kMaxVelocity = 1.0;
  // static double kMaxAcceleration = 0.25;
  

  //private final TrapezoidProfile.Constraints m_constraints =  new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  //private final ProfiledPIDController motionControllerX, motionControllerY ,motionControllerRot;//   = new ProfiledPIDController(KP, KI, KD, m_constraints,kDt);
/**
 * 
 * dontSeeTagTimer - this timer counts the time since we last saw the tag we align to and 
 * if we haven’t seen for for some time it automatically stops the command 
 * 
 * stopTimer - if the pid is not perfect we might overshot and miss the target this timer is used so that 
 * the robot waits a bit when its aligned to make sure it stopped and is truly aligned 
 * 
 * tagID - the ID of the tag we align to. *its the first tag saw when we started aligning
 * 
 */

  /**
   * Aligns the robot to the reef tag relative to its current position.
   * @param isRightScore
   * @param drivebase
   */

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase) 
  {
    xController = new PIDController(0.86, 0.0, 0);  // 3.3Vertical movement
    yController = new PIDController(0.86, 0, 0);  // 3.3 Rotation
    rotController = new PIDController(0.023452, 0, 0);  // 0.058, 0.045,0.048, 0.05///0.023452 angle

    /*motionControllerX = new ProfiledPIDController(0.26, 0.0, 0, m_constraints, kDt);
    motionControllerY = new ProfiledPIDController(0.26, 0, 0, m_constraints, kDt);
    motionControllerRot = new ProfiledPIDController(0.0452, 0, 0, m_constraints, kDt);*/
    
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() 
  {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.finish=false;
    this.autoEjectDebouncer.calculate(false);
    /**
     * reset the PID controllers
     * this is important to make sure the robot starts from a clean state
     */
    /*this.motionControllerRot.reset(0.0);
    this.motionControllerX.reset(0.0);
    this.motionControllerY.reset(0.0);*/
   


    /*
    * the setpoints: put your robot on the field where you want it to be at the end of the aligning (where it is when it scores)
    * then open the limelight web interface and go to the advanced tab and get the robot position from there
    * make sure you set the view to “Robot pose in Target Space” set each setpoint to its relative value in the web interface
    *   x stepoint = TX
    *   y setpoint = TZ*
    *   rot setpoint = RY
    *
    * if your camera isn’t centered with your scoring system you would need to have 2 different values for the right and left reef pipes. 
    * if it is centered you can just use the same value but but negative (-Constants.Y_SETPOINT_REEF_ALIGNMENT)
    * 
    * the tolerance: this is how much error we are allowing. the smaller the value the more accurate 
    * we want to be but the harder it is to tune. 
    * I recommend putting this value a bit smaller than how much the robot needs to be aligned.
    * don’t make this value too large as than the aligning will stop to early and the robot won’t be 
    * aligned and don’t make it too small as than it will be really hard to tune, and also d you really 
    * need to be accurate to the mm or maybe 5 or 10 mm would be close enough 
    *
    */
    rotController.setSetpoint(setpointRot);//0.0 zero rotation2.70
    rotController.setTolerance(errorToleranceRot);//2.0, 1.0, 

    //motionControllerRot.setGoal(2.70);
    //motionControllerRot.setTolerance(1.0);

    xController.setSetpoint(setpointX);  // Vertical pose);
    xController.setTolerance(errorToleranceX);//0.1, 0.05

    //motionControllerX.setGoal(-0.81);
    //motionControllerX.setTolerance(0.02);

    yController.setSetpoint(isRightScore ? setpointY : setpointY);// Horizontal pose
    yController.setTolerance(errorToleranceY);//0.1, 0.05

    //motionControllerY.setGoal(isRightScore ? -0.15 : -0.15);  
    //motionControllerY.setTolerance(0.02);

    tagID = LimelightHelpers.getFiducialID("limelight-front");
  }

  @Override
  public void execute() 
  {
    /**
     * the if is checking if we see a valid target and if the target has the id we are aligning to
     */
    if (LimelightHelpers.getTV("limelight-front")
        && LimelightHelpers.getFiducialID("limelight-front") == tagID) 
    {
      /**
       * we reset the dontSeeTagTimer because we saw a tag
       */
      this.dontSeeTagTimer.reset();

      /**
       * we are getting the position of the robot in the target space
       * we get the pose of the robot relative to the tag as an array and put in the variable positions
       */

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-front");
      SmartDashboard.putNumber("x", postions[2]);
      SmartDashboard.putNumber("y", postions[0]);
      SmartDashboard.putNumber("rot", postions[4]);

      /**
       * we are calculating the speed of the robot in the target space
       * we use the PID controllers to calculate the speed of the robot in the target space
       * we put the speed in the variables xSpeed, ySpeed and rotValue
       * we also put the speed in the smart dashboard so we can see it in the dashboard
       */

      double xSpeed = -xController.calculate(postions[2]);
      //double xSpeed = -motionControllerX.calculate(postions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);

      double ySpeed = yController.calculate(postions[0]);
      //double ySpeed = motionControllerY.calculate(postions[0]);
      SmartDashboard.putNumber("yspeed", ySpeed);

      double rotValue = -rotController.calculate(postions[4]);
      //double rotValue = -motionControllerRot.calculate(postions[4]);
      //double rotValuePigeon = rotController.calculate(drivebase.getGyroRotation2d().getDegrees());
      SmartDashboard.putNumber("rotValue", rotValue);

      //drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);
      //!!make sure your drive function is robot relative!!!
     

      /**
       * “yController.getError() < Constants.Y_TOLERANCE_REEF_ALIGNMENT ? xSpeed : 0” - 
       * this line in the drive code is used to make sure we are aligned in the y direction before we start alignin in the x direction
       * if you want both alignment to happen at the same time replace this with xSpeed
       */
      //MNL 08/11/2025 - changed the drive function to use the driveToTarget function
      //drivebase.driveToTarget(yController.getError() < 0.1 ? () -> xSpeed : () -> 0.0, () -> ySpeed, () -> rotValue, () -> false);
      
      //MNL 08/12/2025 - changed the drive function to use the driveToTarget function within if condition below
      //drivebase.driveToTarget( () -> xSpeed, () -> ySpeed, () -> rotValue, () -> false);
      

      /**
       * this checks if we are aligned and if not it reset the stop timer
       */

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) 
      {
        if (yController.atSetpoint()&&
              xController.atSetpoint() && errorToleranceRotExt > Math.abs(rotController.getPositionError()))
        {
          //drivebase.stopModules();
          finish=true;
        }
          //drivebase.stopModules();
        //if we are not aligned we drive to the target
        drivebase.driveToTarget( () -> xSpeed, () -> ySpeed, () -> rotValue, () -> false);
        stopTimer.reset();
      } 
      else 
      {
        //drivebase.stopModules();
        finish=true;
      }
      
    } 
    else 
    {
      /**
       * if we don’t see a tag we stop the robot and reset the dontSeeTagTimer
       * this is important so that we don’t keep driving when we don’t see a tag
       */
      drivebase.drive(new ChassisSpeeds(0, 0, 0));
      //drivebase.driveToTarget(() -> 0.0, () -> 0.0, ()->0.0, ()->false);//test 1 MNL 08/11/2025 - nothing changed
      finish=true;
      //drivebase.stopModules();
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) 
  {
    drivebase.stopModules();
  }

  @Override
  public boolean isFinished() 
  {
    
    /**
     * DONT_SEE_TAG_WAIT_TIME = 1.0; // seconds
     * POSE_VALIDATION_TIME = 0.3;
     * check if we either didn’t see a tag in some time or we are aligned and if so stops the command
     * Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
     */
    //MNL 08/11/2025 - changed finish condition to also include finish variable
    return finish||this.dontSeeTagTimer.hasElapsed(1.0);// ||stopTimer.hasElapsed(0.2);
    
  }
}