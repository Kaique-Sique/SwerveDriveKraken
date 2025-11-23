# Base Swerve Drive Code
* Code by Megazord 7563

## Systens Overview
### Swerve Drive
* The swerve drive allowed robot move to in any direction and rotate simultaneously and independently, thats a system indicated to improve his robot mobility with more accuracy.

#### 1. Swerve Module
* This is used to create the modules, and control output and postion of each one, this part of code configure each motors and encoder, above set his position and output using a swerveModuleStates argument

```java
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
```

* Optimaze function: Go to turning position in the fastest and shortest way

<img width="1000" height="400" alt="image" src="https://github.com/user-attachments/assets/d7deda26-1478-42ce-9f6e-743c4e1707be" />

This function help us to move turn of module position as fast as possible, it's improve a controlled move and increase issues with accuracy, above this, if its a angle so different, the module set reverse direction on the module to reduse the size of move, like the exemple on the image.
