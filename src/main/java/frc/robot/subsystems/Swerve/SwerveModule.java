// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Swerve;

//CTRE imports
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

//WPI imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Local imports
import frc.robot.Constants.ModuleConstants;
import frc.robot.Utils.Conversions;

public class SwerveModule extends SubsystemBase {
  private final TalonFX driveMotor; // Declare motor Dirves
  private final TalonFX turningMotor; // declare turning motor

  public final CANcoder absoluteCaNcoder; // declare absolute canCoder

  // velocity request drive
  private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
  // position request turning
  private final PositionVoltage mmrequest = new PositionVoltage(0).withSlot(0);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private SwerveModuleState m_desiredState; // = new SwerveModuleState(0.0, new Rotation2d());

  private Rotation2d angleOffset; // absolute encoder offset
  private double m_chassisAngularOffset = 0; // chassis angular offset

  /**
   * Constructs a SwerveModule with a drive motor, turning motor,
   * drive encoder and turning encoder.
   * 
   * @param driveMotorId
   * @param turningMotorId
   * @param absoluteEncoderId
   * @param absoluteEncoderOffset
   * @param chassiAngularOffset
   */
  public SwerveModule(int driveMotorId,
      int turningMotorId,
      int absoluteEncoderId,
      Rotation2d absoluteEncoderOffset,
      double chassisAngularOffset) {
    this.angleOffset = absoluteEncoderOffset; // that is in rad

    // motors instances
    this.driveMotor = new TalonFX(driveMotorId, "rio");
    this.turningMotor = new TalonFX(turningMotorId, "rio");

    // canCoder instance
    this.absoluteCaNcoder = new CANcoder(absoluteEncoderId);

    // intantiate SwerveModuleState
    this.m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    canCoderConfigs(); // 2.absolute encoder config

    initializeTurningMotor(); // 3. turning motor config
    initializeDriveMotor(); // 4. drive motor config

    m_chassisAngularOffset = chassisAngularOffset; // 5. Initialize chassi offset value config (rotation)

    resetToAbsolute(); // 6.Load absolute CanCoder value to turning encoder

    m_desiredState.angle = new Rotation2d(this.getTurningPositionRad()); // 7. Create desired angle
  }

  /**
   * Returns the current state of the module. (position and angle)
   *
   * @return The current state of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        this.getDrivePositionMeters(),
        new Rotation2d(this.getTurningPositionRad() - m_chassisAngularOffset));
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /**
   * Returns the current state of the module. (velocity and angle)
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(this.getDriveVelocityMPS(),
        new Rotation2d(this.getTurningRotation() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   * 
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState(); // instantiate new state
    correctedDesiredState.speedMetersPerSecond = state.speedMetersPerSecond; // copy speed
    correctedDesiredState.angle = state.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    // add chassis offset to angle

    /**
     * SwerveModuleState optimizedDesiredState =
     * SwerveModuleState.optimize(correctedDesiredState,
     * new Rotation2d(turningEncoder.getPosition()));
     */
    correctedDesiredState.optimize(new Rotation2d(this.getTurningPositionRad())); // optimize the state
    correctedDesiredState.cosineScale(new Rotation2d(this.getTurningPositionRad())); // cosine scale the state

    /*
     * double driveVelocityRPS =
     * Conversions.MPSToRPS(optimizedDesiredState.speedMetersPerSecond,
     * ModuleConstants.kWheelCircumferenceMeters,
     * ModuleConstants.kDriveMotorGearRatio);
     */
    double driveVelocityRPS = Conversions.MPSToRPS(correctedDesiredState.speedMetersPerSecond,
        ModuleConstants.kWheelCircumferenceMeters,
        ModuleConstants.kDriveMotorGearRatio);

    // Set the motor outputs from the optimized state.
    driveMotor.setControl(m_request.withVelocity(driveVelocityRPS)); // set drive velocity
    turningMotor.setControl(mmrequest.withPosition(correctedDesiredState.angle.getRotations())); // set turnig pid angle
                                                                                                 // reference
    // Save the desired state.
    m_desiredState = state;
  }

  private void initializeTurningMotor() {
    // factore default
    turningMotor.clearStickyFaults();
    this.turningMotor.getConfigurator().apply(new TalonFXConfiguration());

    turningMotor.setNeutralMode(ModuleConstants.turningNeutralMode); // neutralMode = coast

    TalonFXConfiguration tConfiguration = new TalonFXConfiguration(); // create a new config object
    CurrentLimitsConfigs currentLimitsConfigs = tConfiguration.CurrentLimits;
    currentLimitsConfigs.StatorCurrentLimit = ModuleConstants.kTurningMotorCurrentLimit; // set current limit 60
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    tConfiguration.Feedback.SensorToMechanismRatio = 1 / ModuleConstants.kTurningMotorGearRatio; // set gear ratio
    tConfiguration.ClosedLoopGeneral.ContinuousWrap = true; // enable continuous wrap on the pid controller

    // tConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // put pid values constants
    tConfiguration.Slot0.kP = ModuleConstants.kPTurning;
    tConfiguration.Slot0.kD = ModuleConstants.kDTurning;
    tConfiguration.Slot0.kI = ModuleConstants.kITurning;
    tConfiguration.Slot0.kS = ModuleConstants.kTurningFF;

    StatusCode status = StatusCode.StatusCodeNotInitialized; // init status code
    /* Retry config apply up to 5 times, report if failure */
    for (int i = 0; i < 5; ++i) {
      status = turningMotor.getConfigurator().apply(tConfiguration);
      if (status.isOK())
        break; // if apply is ok break the loop
    }
    if (!status.isOK()) { // if not ok print the error
      System.out.println("Could not apply swerve module configs, error code: " + status.toString());
    }
  }

  /**
   * https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
   * Config drive motors
   * 
   * @category TalonFx settings for drive motors
   */

  private void initializeDriveMotor() {
    // Factory Default
    driveMotor.clearStickyFaults();
    this.driveMotor.getConfigurator().apply(new TalonFXConfiguration());

    driveMotor.setNeutralMode(ModuleConstants.driveNeutralMode); // NeutralMode = Brake

    /* Configure a stator limit of 20 amps */
    TalonFXConfiguration toConfigure = new TalonFXConfiguration(); // create a new config object
    CurrentLimitsConfigs currentLimitConfigs = toConfigure.CurrentLimits;
    currentLimitConfigs.StatorCurrentLimit = ModuleConstants.driveCurrentThreshold; // 120 - current limit
    currentLimitConfigs.StatorCurrentLimitEnable = ModuleConstants.driveEnableCurrentLimit; // true

    toConfigure.Feedback.SensorToMechanismRatio = 1 / ModuleConstants.kDriveMotorGearRatio; // set gear ratio
    toConfigure.ClosedLoopGeneral.ContinuousWrap = true; // no continuous wrap on the pid controller
    toConfigure.MotorOutput.NeutralMode = ModuleConstants.driveNeutralMode; // NeutralMode = Brake
    toConfigure.MotorOutput.ControlTimesyncFreqHz = 100; // 100Hz is the default, but can be changed if needed

    // MNL 01/28/2025
    toConfigure.Slot0.kS = ModuleConstants.driveKS; // 0.0; // Add 0.1 V output to overcome static friction
    toConfigure.Slot0.kV = ModuleConstants.driveKV; // 0.124; // A velocity target of 1 rps results in 0.12 V output
    toConfigure.Slot0.kP = ModuleConstants.kPdriving; // 0.1; // An error of 1 rps results in 0.11 V output
    toConfigure.Slot0.kI = ModuleConstants.kIdriving; // 0; // no output for integrated error
    toConfigure.Slot0.kD = ModuleConstants.kDdriving; // 0; // no output for error derivative

    // MNL 01/28/2025
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = driveMotor.getConfigurator().apply(toConfigure); // apply the config to the motor
      if (status.isOK()) // if ok - break
        break;
    }
    if (!status.isOK()) { // if not ok print the error
      System.out.println("Could not apply swerve module configs, error code: " + status.toString());
    }

    driveMotor.setPosition(0); // reset encoder
  }

  /**
   * @category CanCoder configutation
   */
  public void canCoderConfigs() {
    // Factory Default
    absoluteCaNcoder.clearStickyFaults();
    absoluteCaNcoder.getConfigurator().apply(new CANcoderConfiguration());

    var configs = new CANcoderConfiguration(); // new config object

    // Sensor direction
    configs.withMagnetSensor(
        new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    // Speed up signals to an appropriate rate
    absoluteCaNcoder.getPosition().setUpdateFrequency(100);
    absoluteCaNcoder.getVelocity().setUpdateFrequency(100);

    // Set the absolute sensor range to 0-360 degrees.
    configs.MagnetSensor.withMagnetOffset(0);

    // aply new config
    absoluteCaNcoder.getConfigurator().apply(configs);
  }

  /**
   * Get canCoder value
   * 
   * @return canCoder value as Rotation2d
   */
  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(absoluteCaNcoder.getAbsolutePosition().getValueAsDouble());
  }

  /**
   * Checks if cancoder getAbsolutePosition have an OK error code.
   * 
   * @return true if all is good otherwise false
   */
  public boolean getCanCoderIsValid() {
    return BaseStatusSignal.isAllGood(absoluteCaNcoder.getAbsolutePosition());
  }

  /* Initialize wheels positions */
  public void resetToAbsolute() {
    // put absolute canCoder value to turning motor
    turningMotor.setPosition(getCANcoder().getRotations() - angleOffset.getRotations());
  }

  /**
   * @return canCoder degrees (0-360)°
   */
  public double canCoderDegrees() {
    return getRawHeading() * 360;
  }

  /**
   * @return canCoder degrees radians
   */
  public double canCoderRad() {
    return getRawHeading() * 2 * Math.PI;
  }

  /**
   * Get the heading of the canCoder - will also include the offset
   *
   * @return Returns the raw heading of the canCoder (rotations)
   */
  public double getRawHeading() {
    return absoluteCaNcoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Get the heading of the swerve module
   * 
   * @return Returns the heading of the module in radians as a double
   */
  public double getTurningHeading() {
    // double heading = Units.degreesToRadians(canCoderDegrees() -
    // absoluteOffsetEncoderDegrees());
    double heading = canCoderRad() - absoluteOffsetEncoderRadians();
    heading %= 2 * Math.PI;
    return heading;
  }

  /**
   * @return offset angle of the module in degrees
   */
  public double absoluteOffsetEncoderDegrees() {
    return angleOffset.getDegrees();
  }

  /**
   * @return offset angle of the module in radians
   */
  public double absoluteOffsetEncoderRadians() {
    return m_chassisAngularOffset;
  }

  /**
   * Get the heading of the swerve module
   * 
   * @return Returns the heading of the module in degrees as a double
   */
  public double getTurningHeadingDegrees() {
    double heading = (canCoderDegrees() - absoluteOffsetEncoderDegrees()); // * (absoluteEncoderReversed ? -1.0: 1.0);
    heading %= 360;
    return heading;
  }

  @Override
  public void periodic() {
  }

  /**
   * @return driveMotor position as double
   */
  public double getDrivePositionMeters() {
    return Conversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(),
        ModuleConstants.kWheelCircumferenceMeters);
  }

  /**
   * Get the turning position of the module
   * 
   * @return turning position in degrees (0-360)°
   */
  public double getTurningPosition() {
    return turningMotor.getPosition().getValueAsDouble() * 360;
  }

  /**
   * Get the turning position of the module
   * 
   * @return turning position in rotations
   */
  public double getTurningRotation() {
    return turningMotor.getPosition().getValueAsDouble();
  }

  /**
   * Get the turning position of the module
   * 
   * @return getting position in radians
   */
  public double getTurningPositionRad() {
    return turningMotor.getPosition().getValueAsDouble() * (2 * Math.PI);
  }

  /**
   * Get the drive velocity of the module
   * 
   * @return drive velocity in meters per second
   */
  public double getDriveVelocityMPS() {
    return Conversions.RPSToMPS(driveMotor.getRotorVelocity().getValueAsDouble(),
        ModuleConstants.kWheelCircumferenceMeters);
  }

  /**
   * Get the turning velocity of the module
   * 
   * @return turning velocity in degrees per second
   */
  public double getTurningVelocity() {
    return turningMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Get the drive motor current
   * 
   * @return drive motor current in amps
   */
  public double getDriveCurrent() {
    return driveMotor.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Get the turning motor current
   * 
   * @return turning motor current in amps
   */
  public double getTurnCurrent() {
    return turningMotor.get();
  }

  /**
   * Get the drive motor temperature
   * 
   * @return drive motor temperature in celsius
   */
  public double getDriveTemperature() {
    return driveMotor.getDeviceTemp().getValueAsDouble();
  }

  /**
   * get drive motor voltage
   * 
   * @return drive motor voltage as a double
   */
  public double getDriveVoltage() {
    return driveMotor.getMotorVoltage().getValueAsDouble();
  }

  /**
   * @drive brake mode
   * @turn disable turning motor
   */
  public void stopMotors() {
    driveMotor.setControl(m_brake);
    turningMotor.set(0);
  }

  /**
   * @ disable motors
   */
  public void disableMotors() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  /**
   * Resets the drive encoder
   */
  public void resetDriveEncoders() {
    driveMotor.setPosition(0);
  }
}
