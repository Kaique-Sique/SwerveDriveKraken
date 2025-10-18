// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.Set;
//JAVA Imports
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.ctre.phoenix6.StatusCode;

//CTRE imports
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

//WPI Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;

//Robot Imports
import frc.robot.Constants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Limelight.LimelightHelpers;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;

public class SwerveSubsystem extends SubsystemBase {
  /**************************
   * Create 4 Swerve Modules*
   **************************/
  public final SwerveModule frontLeftModule = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
      DriveConstants.angleOffsetFLTurning,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule frontRightModule = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
      DriveConstants.angleOffsetFRTurning,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule backLeftModule = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
      DriveConstants.angleOffsetBLTurning,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule backRightModule = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightDriveAbsoluteEncoderPort,
      DriveConstants.angleOffsetBRTurning,
      DriveConstants.kBackRightChassisAngularOffset);

  SwerveModuleState[] states = new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
  };

  // Create Pigeon gyro
  Pigeon2 gyro = new Pigeon2(DriveConstants.kPigeonPort, "rio");

  // Create a slew rate limiter to limit the rate of change of the commanded robot
  // velocity
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
      DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(); // Initialize chassis speeds to zero
  // private final Field2d field2d;
  // Array of modules
  SwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };

  /** Create speed booleans */
  private boolean throttleSlow = false, throttleFast = false, throttleMax = false, // Create speed booleans
      isBlue = false, blueAlliance = false, isAlliancePresent = false;

  // PathPlanner Config
  private RobotConfig config;

  // Select alliance heading - default is blue
  private double allianceHeading = 0;

  private boolean deploy = true;
  private int countDeploy = 0;

  // Create a new SwerveDrivePoseEstimator object
  private final SwerveDrivePoseEstimator m_poseEstimator;

  // Create a NetworkTable publisher to publish the pose to NetworkTables
  // You can use this values on advanced scope from wpi

  /**** Robot Poses 2D ****/
  // Robot Pose2d
  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("Swerve/MyPose", Pose2d.struct).publish();

  /** Swerve NT Data publisher - swerveModule states **/
  StructArrayPublisher<SwerveModuleState> swPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Swerve/MyStatesMeasured", SwerveModuleState.struct).publish();

  StructArrayPublisher<SwerveModuleState> swDesiredPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Swerve/MyDesiredStates", SwerveModuleState.struct).publish();

  StructPublisher<ChassisSpeeds> chassisPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("Swerve/ChassisSpeeds", ChassisSpeeds.struct)
      .publish();

  StructPublisher<Rotation2d> rotation2dPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("Swerve/MyRotation2d", Rotation2d.struct)
      .publish();

  // Autopilot for path following we must setup all of our motion constraints
  private static final APConstraints kConstraints = new APConstraints()
      .withVelocity(3.0)
      .withAcceleration(3.0)// 5
      .withJerk(1.0);// 2

  // Create a new APProfile with the constraints and other parameters
  private static final APProfile kProfile = new APProfile(kConstraints)
      .withErrorXY(Centimeters.of(2))// 2
      .withErrorTheta(Degrees.of(0.5))// 0.5
      .withBeelineRadius(Centimeters.of(8));// 8

  // Create a new Autopilot instance with the profile
  public static final Autopilot kAutopilot = new Autopilot(kProfile);

  // Create a target pose for the Autopilot
  Pose2d targetPose = new Pose2d(2.79, 4.0, Rotation2d.fromDegrees(0));
  APTarget target = new APTarget(targetPose).withEntryAngle(Rotation2d.fromDegrees(0));

  public SwerveSubsystem() {

    initializePigeon2();

    // field2d = new Field2d();
    // SmartDashboard.putData("Field", field2d);

    setLimlightsPose();

    // Initialize pose estimator ALWAYS towards to red wall
    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        },
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    // Initialize the PathPlanner config and AutoBuilder
    /**
     * If the robot's configuration file is missing or corrupted, this code will:
     * Catch the exception thrown by RobotConfig.fromGUISettings().
     * Report the error to the Driver Station, allowing the team to investigate and
     * fix the issue.
     */

    try {
      config = new RobotConfig(
          PathPlannerConstants.robotMass,
          PathPlannerConstants.MOI,

          new ModuleConfig(ModuleConstants.kWheelDiameterMeters / 2,
              ModuleConstants.maxDriveVelocityMPS,
              ModuleConstants.wheelCOF,
              DCMotor.getKrakenX60(1).withReduction(ModuleConstants.gearBox),
              ModuleConstants.driveCurrentLimit, 1),
          DriveConstants.kDriveKinematics.getModules());

      AutoBuilder.configure(
          this::getPoseEstimator, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                       // optionally outputs individual module feedforwards
          PathPlannerConstants.AutoConfig,
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  public void setLimlightsPose() {
    LimelightHelpers.SetIMUMode(DriveConstants.limelightFront, 1);
    LimelightHelpers.setCameraPose_RobotSpace(DriveConstants.limelightFront, 0.110, 0.259, 0.410, 0, 0, 0);
    LimelightHelpers.setCameraPose_RobotSpace(DriveConstants.limelightLeft, 0.191, -0.25, 0.524, 0, 0, 0);
    LimelightHelpers.setCameraPose_RobotSpace(DriveConstants.limelightBack, 0.130, 0.259, 0.870, 0, 25, 180);
    LimelightHelpers.SetIMUMode(DriveConstants.limelightFront, 2);
  }

  @Override
  public void periodic() {

    initializeAliance();

    this.updatePoseEstimator();
    this.addPoseVisionNew();

    // Output to SmartDashboard
    SmartDashboard.putNumber("PoseEstimator /Pose X (meters)", this.getPoseEstimator().getX());
    SmartDashboard.putNumber("PoseEstimator /rPose Y (meters)", this.getPoseEstimator().getY());
    // SmartDashboard.putNumber("PoseEstimator /BotPose (degrees)",
    // LimelightHelpers.getBotPose(DriveConstants.limelightFront)[5]);
    SmartDashboard.putNumber("PoseEstimator /Rotation (degrees)", this.getPoseEstimator().getRotation().getDegrees());
    SmartDashboard.putBoolean("isBlue", isBlue);
    SmartDashboard.putNumber("Pigeon 7563 Yaw", gyro.getYaw().getValueAsDouble());
    SmartDashboard.putNumber("Pigeon 7563 Rotation", gyro.getRotation2d().getDegrees());

    // gyro Calibration with no motion
    SmartDashboard.putNumber("Pigeon 7563 No motion", gyro.getNoMotionCount().getValueAsDouble());

    SmartDashboard.putNumber("Translation 2d X", this.getFieldRelativeSpeeds().getX());
    SmartDashboard.putNumber("Translation 2d Y", this.getFieldRelativeSpeeds().getY());

    if (deploy) {
      countDeploy++;
      if (countDeploy == 5) {
        deploy = false;
      }
      System.out.println("Deploy1");
    }

    for (int i = 0; i < modules.length; i++) {
      SmartDashboard.putNumber("Module " + i + " /Drive Motor Current (Amp)", modules[i].getDriveCurrent());
      SmartDashboard.putNumber("Module " + i + " /Turn Motor Current (Amp)", modules[i].getTurnCurrent());
      SmartDashboard.putNumber("Module " + i + " /Drive Motor Temperature (℃)",
          modules[i].getDriveTemperature());
      SmartDashboard.putNumber("Module " + i + " /Turning Motor Heading (Degrees)",
          modules[i].getTurningHeadingDegrees());
      SmartDashboard.putNumber("Module " + i + " /CanCoder Heading (rad)",
          modules[i].canCoderRad());
      SmartDashboard.putNumber("Module " + i + " /CanCoder Heading (Degrees)",
          modules[i].canCoderDegrees());
      SmartDashboard.putNumber("Module " + i + " /CanCoder Angle value (rad)",
          modules[i].getTurningPosition());

      if (!modules[i].getCanCoderIsValid()) {
        for (int j = 0; j < 3; j++) {
          System.err.println("Absolute Encoder Error: Returning default angle.");
          DriverStation.reportError("Absolute Encoder Error: Returning default angle.", null);
        }
      }
    }

    // Output Module States to SmartDashboard
    SwerveModuleState[] moduleStates = getModuleStates();
    for (int i = 0; i < moduleStates.length; i++) {
      SmartDashboard.putNumber("Module " + i + " /Speed (m/s)", moduleStates[i].speedMetersPerSecond);
      // SmartDashboard.putNumber("Module " + i + " /Angle (rad)",
      // moduleStates[i].angle.getRadians());
    }
    // Publish the pose to NetworkTables
    publisher.set(getPoseEstimator());
    chassisPublisher.set(this.getChassisSpeeds());
    swPublisher.set(moduleStates);
    swDesiredPublisher.set(states);
    rotation2dPublisher.set(this.getPoseEstimator().getRotation());
  }

  private void initializeAliance() {
    if (!isAlliancePresent && DriverStation.getAlliance().isPresent()) {
      Translation2d posPose = this.getPoseEstimator().getTranslation();
      m_poseEstimator.resetPosition(gyro.getRotation2d(),
          getModulesPosition(),
          new Pose2d(posPose,
              new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue
                  ? Math.PI
                  : 0)));
      isAlliancePresent = true;
    }
  }

  /** Updates the field relative position of the robot. */
  public void updatePoseEstimator() {
    // Update the pose estimator with the latest sensor measurements
    m_poseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        });
  }

  /**
   * update pose estimator using absoule coordenates from limelight
   */
  public void addPoseVisionNew() {
    boolean doRejectUpdate = false;

    try {
      // List of cameras to process
      List<String> cameras = List.of(
          DriveConstants.limelightFront,
          DriveConstants.limelightBack,
          DriveConstants.limelightLeft);

      // Set robot orientation for all cameras
      double robotRotationDegrees = m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
      for (String camera : cameras) {
        LimelightHelpers.SetRobotOrientation(camera, robotRotationDegrees, 0, 0, 0, 0, 0);
      }

      // Get pose estimates for all cameras
      List<LimelightHelpers.PoseEstimate> poseEstimates = cameras.stream()
          .map(LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2)
          .filter(Objects::nonNull) // Filter out null estimates
          .collect(Collectors.toList());

      // Reject vision updates if angular velocity is too high
      if (Math.abs(gyro.getAngularVelocityZDevice().refresh().getValueAsDouble()) > 720) {
        doRejectUpdate = true;
      }

      // Reject updates if no valid measurements are available
      if (poseEstimates.isEmpty()) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {
        // Select the best measurement based on tag count, distance, and area
        LimelightHelpers.PoseEstimate bestMeasurement = poseEstimates.stream()
            .filter(p -> p.tagCount > 0 && p.avgTagDist < 3) // Valid measurements
            .max(Comparator.comparingDouble(p -> p.avgTagArea)) // Select the one with the largest tag area
            .orElse(null);

        if (bestMeasurement != null) {
          // Add the best measurement to the pose estimator
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
          m_poseEstimator.addVisionMeasurement(bestMeasurement.pose, bestMeasurement.timestampSeconds);
        }
      }
    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to get Limelight botpose", e.getStackTrace());
    }
  }

  /**
   * 
   * @return blue alliance true/false
   */
  public boolean getBlueAlliance() {
    return blueAlliance;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    for (int i = 0; i < 4; i++) {
      System.out.println("Reset Encoders");
      modules[i].resetDriveEncoders();
    }
  }

  /**
   * Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
   * Path Planner uses
   * 
   * @param chassisSpeeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;

    SwerveModuleState[] swerveModuleStates = Constants.DriveConstants.kDriveKinematics
        .toSwerveModuleStates(chassisSpeeds);

    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    this.setModuleStates(swerveModuleStates);
  }

  // Method to get the ROBOT RELATIVE ChassisSpeeds
  public ChassisSpeeds getChassisSpeeds() {
    // Relative to robot
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  // reset the pose Estimator to a new location
  /**
   * MNL10/23/2024
   * 
   * @param pose The pose to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        gyro.getRotation2d(), // arrow usa gyro.getAngle() ///getGyroYaw()
        this.getModulesPosition(),
        pose);
  }

  /**
   * 
   * @return estimated position
   */
  public Pose2d getPoseEstimator() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Stop all modules
   */
  public void stopModules() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].stopMotors();
    }
  }

  /**
   * @ disable all modules with no BRAKES
   */
  public void disableModules() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].disableMotors();
    }
  }

  /**
   * 
   * @return throttle slow down the robot speed
   */
  public void robotSlower() {
    throttleSlow = true;
    throttleFast = false;
    throttleMax = false;

    for (int i = 0; i < 3; i++) {
      System.out.println("Speed up velocity - 30%");
    }
  }

  /**
   * 
   * @return throttle slow down the robot speed
   */
  public void robotFast() {
    throttleSlow = false;
    throttleFast = true;
    throttleMax = false;

    for (int i = 0; i < 3; i++) {
      System.out.println("Speed up velocity - 50%");
    }
  }

  /**
   * 
   * @return throttle slow down the robot speed
   */
  public void robotMaxSpeed() {
    throttleSlow = false;
    throttleFast = false;
    throttleMax = true;

    for (int i = 0; i < 3; i++) {
      System.out.println("Velocity 100% Max");
    }
  }

  // Set the wheels in an "X" formation to resist being pushed
  public void setX() {
    SwerveModuleState[] xStates = new SwerveModuleState[4];
    xStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    xStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    xStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    xStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    setModuleStates(xStates);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // normalize the wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);

    // Output Module States to each one
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }

    states = desiredStates;
  }

  /**
   * Get swerve module status on real time
   * 
   * @return states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulesPosition() {
    SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /**
   * Get Swerve module desired status
   * 
   * @return desired status
   */
  public SwerveModuleState[] getModuleDisiredStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getDesiredState();
    }
    return states;
  }

  /**
   * Method drive with joystick
   * The use of these parameters as suppliers to dynamically provide speed and
   * field orientation values.
   * Suppliers are especially useful in cases where you need to calculate values
   * dynamically based on factors that can change over time.
   * The Supplier interface gives you a powerful mechanism to make your FRC robot
   * code more flexible and adaptable.
   * Suppliers are incredibly useful in command-based FRC programming because they
   * allow you to:
   * Decouple Logic: You can separate the logic for calculating drive parameters
   * (speeds, orientation)
   * from the actual drive command. This makes your code cleaner and more
   * maintainable.
   * Dynamic Values: You can easily update the speed and orientation values
   * on-the-fly based on real-time conditions,
   * such as sensor feedback or joystick input.
   * 
   * @param xSpdFunction          Speed of the robot in the x direction (forward).
   * @param ySpdFunction          Speed of the robot in the y direction
   *                              (sideways).
   * @param turningSpdFunction    Angular rate of the robot. rad/s
   * @param fieldOrientedFunction Boolean indicating if speeds are relative to the
   *                              field or to therobot.
   * 
   **/
  public void driveRobotOriented(Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction) {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    // boolean fieldOriented = fieldOrientedFunction.get();
    double throttle = 0.2; // initial speed

    // Selects speed
    // If its necessary you can add another option of speed,
    // on the 2025 season we use it when we call the lift
    if (throttleSlow) {
      throttle = 0.30;
    } else if (throttleFast) {
      throttle = 0.65;
    } else if (throttleMax) {
      throttle = 0.9;
    }
    // 2. Apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    // 3. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * throttle;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * throttle;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond
        * throttle;

    // 4. Construct desired chassis speeds
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,
            ySpeed,
            turningSpeed,
            gyro.getRotation2d()));

    // 5. Convert chassis speeds to individual module states
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
    // DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    // 6. Output each module states to wheels
    this.setModuleStates(swerveModuleStates);
  }

  /**
   * Initialize Pigeon2 device from the configurator object
   * 
   * @param cfg Configurator of the Pigeon2 device
   */
  private void initializePigeon2() {
    // Create a new Pigeon2Configurator object
    Pigeon2Configuration configs = new Pigeon2Configuration();
    // Clear the sticky faults
    gyro.clearStickyFaults();

    // Apply the configuration to the Pigeon2 device
    gyro.getConfigurator().apply(new Pigeon2Configuration());

    configs.Pigeon2Features.DisableNoMotionCalibration = false;

    // gyro calibration values for acumulate error in a rotation
    configs.GyroTrim.GyroScalarX = -2.9779;
    configs.GyroTrim.GyroScalarY = -2.9779;
    configs.GyroTrim.GyroScalarZ = -2.9779;

    configs.Pigeon2Features.DisableTemperatureCompensation = false;
    configs.Pigeon2Features.EnableCompass = false;
    /*
     * Pigeon2FeaturesConfigs features = new Pigeon2FeaturesConfigs();
     * features.DisableNoMotionCalibration = false;
     * features.DisableTemperatureCompensation = false;
     */

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = gyro.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply gyro configs, error code: " + status.toString());
    }
    gyro.setYaw(0, 0);
  }

  /**
   * Method drive to target
   * The use of these parameters as suppliers to dynamically provide speed and
   * field orientation values.
   * Suppliers are especially useful in cases where you need to calculate values
   * dynamically based on factors that can change over time.
   * The Supplier interface gives you a powerful mechanism to make your FRC robot
   * code more flexible and adaptable.
   * Suppliers are incredibly useful in command-based FRC programming because they
   * allow you to:
   * Decouple Logic: You can separate the logic for calculating drive parameters
   * (speeds, orientation)
   * from the actual drive command. This makes your code cleaner and more
   * maintainable.
   * Dynamic Values: You can easily update the speed and orientation values
   * on-the-fly based on real-time conditions,
   * such as sensor feedback or joystick input.
   * 
   * @ without DeadBand
   * 
   * @param xSpdFunction          Speed of the robot in the x direction (forward).
   * @param ySpdFunction          Speed of the robot in the y direction
   *                              (sideways).
   * @param turningSpdFunction    Angular rate of the robot. rad/s
   * @param fieldOrientedFunction Boolean indicating if speeds are relative to the
   *                              field or to therobot.
   * 
   **/
  public void driveToTarget(Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction) {
    // 1. Get real-time joystick inputs
    double xSpeed = -xSpdFunction.get();
    double ySpeed = -ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    boolean fieldOriented = fieldOrientedFunction.get();

    // 2. Apply deadband
    // xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    // ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    // turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed
    // : 0.0;

    // 3. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // 4. Construct desired chassis speeds

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(fieldOriented
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,
            ySpeed,
            turningSpeed,
            gyro.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, turningSpeed));// Do this if fielOrientation is false

    // 5. Convert chassis speeds to individual module states
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
    // DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    // 6. Output each module states to wheels
    this.setModuleStates(swerveModuleStates);// */
  }

  /**
   * Get the gyro yaw
   * 
   * @return
   */
  public Rotation2d getGyroRotation2d() {
    return gyro.getRotation2d();
  }

  public Command align() {
    return this.run(() -> {
      Translation2d velocities = this.getFieldRelativeSpeeds();
      Pose2d pose = this.getPoseEstimator();

      APResult output = kAutopilot.calculate(pose, velocities, target);

      /* these speeds are field relative */
      LinearVelocity veloX = output.vx();
      LinearVelocity veloY = output.vy();
      System.out.println("VeloX: " + veloX.baseUnitMagnitude() + " VeloY: " + veloY.baseUnitMagnitude());

      Rotation2d headingReference = output.targetAngle();

      ChassisSpeeds setControl = ChassisSpeeds.fromFieldRelativeSpeeds(veloX.baseUnitMagnitude(),
          veloY.baseUnitMagnitude(),
          headingReference.getRadians(),
          gyro.getRotation2d());

      this.drive(setControl);
      /*
       * this.setControl(m_fieldRelativeRequest
       * .withVelocityX(veloX)
       * .withVelocityY(veloY)
       * .withTargetDirection(headingReference));
       */
    })
        .until(() -> kAutopilot.atTarget(this.getPoseEstimator(), target))
        .finallyDo(this::stopModules);
  }

  /**
   * Calculates and returns the robot's field-relative speeds as a Translation2d.
   * 
   * @return Field-relative speeds as a Translation2d.
   */
  public Translation2d getFieldRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = this.getChassisSpeeds();
    return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    // .rotateBy(gyro.getRotation2d());
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a
   * given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per
   *                               second
   * @return a Command that drives the swerve drive to a specific distance at a
   *         given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> this.getPoseEstimator()
            .getTranslation()
            .getDistance(new Translation2d(0, 0)) > distanceInMeters);
  }

  public Command driveToThePoint(Supplier<Pose2d> poseSupplier) {
    PathPlannerPath.clearCache();
    PathConstraints telePathConstraints = new PathConstraints(1,
        1,
        Math.PI,
        Math.PI);

    return new DeferredCommand(() -> AutoBuilder
        .pathfindToPose(poseSupplier.get(), telePathConstraints), Set.of(this));
  }
}