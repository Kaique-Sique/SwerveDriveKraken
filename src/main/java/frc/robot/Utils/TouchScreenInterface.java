// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of

package frc.robot.Utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TouchScreenInterface extends SubsystemBase {

  boolean enableInterface;

  boolean btnValues[] = new boolean[16]
  /**
   * @ this method set buttons option on the code
   */
  public enum screenButtons()
  {
    //A buttom add option
    kA("A"),
    //A buttom add option
    kB("B")

    public final String value;

    screenButtons(String value)
    {
      this.value = value;
    }

    @Override
    public String toString()
    {
      return this.name().substring(1) + "Button";
    }
  }

  public TouchScreenInterface() 
  {
    for(screenButtons buttons : screenButtons.values())
    {
      //put on array buttons values
      btnValues[buttons.ordinal()] = false;

      //put NT buttons to false
      SmartDashboard.putBoolean(buttom.toString() + "Value", btnValues[buttons.ordinal()]);
    }
  }

  @Override
  public void periodic() {
    enableInterface = SmartDashboard.putBoolean("enableInterface", isInterfaceEnabled());

    for(screenButtons buttons : screenButtons.values())
    {
      btnValues[buttons.ordinal()] = SmartDashboard.getBoolean(buttons.toString() + "Value", 
                                                                btnValues[buttons.ordinal()]);
    }
  }

  public void enableInterface()
  {
    enableInterface = true;
  }

  public void disableInterface()
  {
    enableInterface = false;
  }

  public boolean isEnableInterface()
  {
    return enableInterface;
  }
}
