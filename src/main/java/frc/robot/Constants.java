// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//CTRE Imports
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
//REV Imports
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//WPI Imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
// Constants class
public final class Constants {
  public static final class ModuleConstants {
    public static final double kDrivingMotorFreeSpeedRps = KrakenMotorConstants.kFreeSpeedRpm / 60;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);// 0.1016m
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;// 0.1016*3.14=

    public static final double kDriveL3 = 1.0 / 6.12;
    public static final double kTurningL3 = 1.0 / 12.8;

    public static final double kDriveL4 = 1.0 / 5.14;
    public static final double kTurningL4 = 1.0 / 12.8;

    public static final double kDriveMotorGearRatio = kDriveL3; // 1 / 5.14; //L4 L3=1/6.12
    public static final double kTurningMotorGearRatio = kTurningL3; // 1 /12.8; // 1/18.0

    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * (2 * Math.PI);

    public static final double kTurningMotorCurrentLimit = 60; // amps

    // public static final double kTurningEncoderPositionFactor =2 * Math.PI; //
    // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        * kDriveMotorGearRatio;
    public static final double kDriveRPM = kDriveEncoderRPM2MeterPerSec * 60;
    public static final double kDriveRPMRatio = kDriveRPM / kDriveMotorGearRatio;
    public static final double kDriveRPMPi = kDriveRPMRatio / Math.PI;
    public static final double kDriveRPMSpeed = kDriveRPMPi / kWheelDiameterMeters;

    /**************************************
     * Turning REV SparkMax PID settings *
     **************************************/

    // 14,285%
    public static final double kPTurning = 22.88;// ANTERIOR 0,5 //0,25 ; 0.2142875; 0.175; 0.125; 0.23
    public static final double kITurning = 0;
    public static final double kDTurning = 0.3762;

    // The F parameter should only be set when using a velocity-based PID
    // controller,
    // and should be set to zero otherwise to avoid unwanted behavior.
    public static final double kTurningFF = 0; // 1.0/473.0; //

    public static final double kTurningKS = 1.08;
    public static final double kTurningKV = 1.376;
    public static final double kTurningKA = 0.0;

    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    /**********************************
     * Driving TALON Fx PID settings *
     *********************************/
    /**************************************************************
     * @category Drive Motor Characterization Values From SYSID
     *           TO DO: This must be tuned to specific robot
     ************************************************************/
    public static final double driveKS = 0.2020;// <-MNL 11/11/2024 0.1; //0.32; // Add 0.1 V output to overcome static
                                                // friction
    public static final double driveKV = 0.7524; // 1.51; // A velocity target of 1 rps results in 0.12 V output
    public static final double driveKA = 0.0; // 0.27;

    // 11,69%
    public static final double kPdriving = 0.6;// <--MNL11/11/2024 0.0665;//ANTERIOR 0,0665 ; 0.05872615 // kP = 0.11 An
                                               // error of 1 rps results in 0.11 V output
    public static final double kIdriving = 0.0;
    public static final double kDdriving = 0.0;

    public static final double kFFdriving = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final NeutralModeValue turningNeutralMode = NeutralModeValue.Coast; // TalonFx

    /* Swerve Current Limiting */
    // TalonFX
    public static final int driveCurrentLimit = 70; // supply current
    public static final int driveCurrentThreshold = 120; // stator current

    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    public static double kDriveClosedLoopRamp = 0.25;

    /* Neutral Modes */
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast; // template was coast SparkMax
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake; // TalonFx
  }

  // Drive Constants
  public static final class DriveConstants {
    /**************************************
     * Specify the kinematics of our robot*
     ************************************/
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(21);
    // Distance between front and back wheels
    public static final double kDriveRadius = Math.hypot(kTrackWidth / 2, kWheelBase / 2);
    // Distance from robot center to furthest module.
    public static final double parametro = 2;

    /**********************************************************************
     * Swerve Drive Object - It specifies the location of each swerve *
     * module on the robot this way the wpi library can construct the *
     * geometry of our robot setup and do all the calculations *
     * 
     * @see Modules Location: FL= +X,+Y; FR= +X,-Y; BL=-X, +Y; BR=-X, -Y,*
     **********************************************************************/
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // + - antes
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // + + antes
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // - - antes
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) // - + antes
    );

    //
    /**
     * MNL 17/10/2024
     * kFrontLeftChassisAngularOffset = 0: This means that when your front left
     * module's turning motor is at its zero position,
     * the module is pointing straight forward relative to the chassis.
     * kFrontRightChassisAngularOffset = 0: This suggests your front right module is
     * also pointing forward when its turning motor
     * is at zero.
     * kBackLeftChassisAngularOffset = Math.PI: This means that your back left
     * module is pointing 180 degrees from the front left
     * module. So, when its turning motor is at zero, the module is pointing
     * straight backward relative to the chassis.
     * kBackRightChassisAngularOffset = Math.PI: This means that your back right
     * module is also pointing straight backward
     * when its turning motor is at zero.
     ******/
    public static final double kFrontLeftChassisAngularOffset = 0;//
    public static final double kFrontRightChassisAngularOffset = Math.PI;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = Math.PI;

    // Portas dos Kraken
    public static final int kFrontLeftDriveMotorPort = 15;
    public static final int kBackLeftDriveMotorPort = 12;
    public static final int kFrontRightDriveMotorPort = 10;
    public static final int kBackRightDriveMotorPort = 14;

    // Portas dos Sparks
    public static final int kFrontLeftTurningMotorPort = 16;
    public static final int kBackLeftTurningMotorPort = 11;
    public static final int kFrontRightTurningMotorPort = 9;
    public static final int kBackRightTurningMotorPort = 13;

    // CanCoder ports
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 5;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 4;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 7;
    public static final int kBackRightDriveAbsoluteEncoderPort = 6;
    public static final int kPigeonPort = 30;// 7563

    /**
     * SINTONIA DAS RODAS
     * FL => FRONT LEFT
     * FR => FRONT RIGHT
     * BL => BACK LEFT
     * BR => BACK RIGHT
     */

    public static final Rotation2d angleOffsetFLTurning = Rotation2d.fromDegrees(-4.85);
    public static final Rotation2d angleOffsetFRTurning = Rotation2d.fromDegrees(-77.43);
    public static final Rotation2d angleOffsetBLTurning = Rotation2d.fromDegrees(142.207);
    public static final Rotation2d angleOffsetBRTurning = Rotation2d.fromDegrees(-91.14);

    /*******************************************************
     * constante que limita a velocidade maxima drive teleop*
     ******************************************************/
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5; // kRAKEN - GEAR RATIO L4
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;// 2*2* Math.PI; //

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.052;// anterior
                                                                                                            // 1.175
                                                                                                            // 4,25531914893617
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
        / 1.75;// anterior 2

    // Slew Rate adjustments
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2.5;// anterior 2 - 2/12/2024 MNL
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2.5;// anterior 2 2/12/2024 MNL

    public static final String limelightFront = "limelight-front";
    public static final String limelightBack = "limelight-back";
    public static final String limelightLeft = "limelight-left";
  }

  /*************************************************************
   * constante que limita a velocidade maxima drive autonomous *
   *************************************************************/
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 5.0; // 4
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond
        / 2;// div/2//10,20

    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0; // 3
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2; // pi/2//4

    public static final double kPXController = 10;// 1.5
    public static final double kPYController = 10;// 1.5
    public static final double kPThetaController = 2 * Math.PI;

    public static final double kOffset = Units.inchesToMeters(9);
    public static final double kOffsetSide = Units.inchesToMeters(3);

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  // Path Planner Constants
  public static final class PathPlannerConstants {
    public static final double kPTranslationPath = 2.5;// Anterior 0.125,
    public static final double kITranslationPath = 0;
    public static final double kPRotationPath = 1.4;// 3.0; //3.0; 1.6

    public static final PPHolonomicDriveController AutoConfig = new PPHolonomicDriveController( // PPHolonomicController
                                                                                                // is the built in path
                                                                                                // following controller
                                                                                                // for holonomic drive
                                                                                                // trains
        new PIDConstants(kPTranslationPath, kITranslationPath, 0), // Translation PID constants
        new PIDConstants(kPRotationPath, 0.0, 0.0) // Rotation PID constants
    );

    public static final double maxAccelerationPath = 1;// 1.75//1.0//5.0 //3.0
    public static final double maxAngularVelocityRadPerSec = Units.degreesToRadians(540);
    public static final double maxAngularAccelerationRadPerSecSq = Units.degreesToRadians(720);
  }

  // OI Constants
  public static final class OIConstants {

    public static final class JoystickDriverConstants {
      // Joystick Driver
      public static final int kDriverControllerPort = 0;

      public static final int kDriverYAxis = 1;
      public static final int kDriverXAxis = 0;

      public static final int kDriverFieldOrientedButtonIdx = 1;

      public static final double kDeadband = 0.1;
    }
    public static final double kDeadband = 0.08;
  }

  /***********************************************************************************************************************************
   * @param https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
   * 
   * @param https://store.ctr-electronics.com/announcing-kraken-x60/?srsltid=AfmBOorh3sPSXQ-WmuWYeJlxrIkATC1wRVPc0V65woNtynzQ1Sil1Ueh
   *
   *************************************************************************************************************************************/
  public static final class KrakenMotorConstants {
    public static final double kFreeSpeedRpm = 6000;
  }

  // Poses Constants for Autonomous and Teleop positions
  public static final class Pose2dConstansts {
    // Poses to alliance red
    public static final class RedsPose2d {
      //implement the positions here
    }

    // Poses to alliance blue
    public static final class BluePose2d {
      //implement the positions here
    }
  }
}
