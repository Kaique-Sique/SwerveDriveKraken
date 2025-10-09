// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.Utils.TouchScreenInterface.screenButtons;

public class PoseInterface {
    private static TouchScreenInterface touchInterf = RobotContainer.touchInterf;
    public Pose2d GetPoseChoosed()
    {
        if(touchInterf.getVirtualButton(screenButtons.kA))
        {
            return FieldPoses.front18;
        }
        else if(touchInterf.getVirtualButton(screenButtons.kB))
        {
            return FieldPoses.front18;
        }
        return RobotContainer.swerveDrive.getPoseEstimator();
    }
}
