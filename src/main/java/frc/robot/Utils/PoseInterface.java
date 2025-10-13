// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.Utils.TouchScreenInterface.screenButtons;

public class PoseInterface {
    private static TouchScreenInterface touchInterf = RobotContainer.touchInterf;
    public Pose2d GetPoseChoosed() {
        var alliance = DriverStation.getAlliance();
        Pose2d targetPose2d;
        //Blue Poses
        if (alliance.get() == DriverStation.Alliance.Blue) {
            if (touchInterf.getVirtualButton(screenButtons.kA)) {
                targetPose2d = FieldPoses.bluePoses.reefA;
            } else if (touchInterf.getVirtualButton(screenButtons.kB)) {
                targetPose2d = FieldPoses.bluePoses.reefB;
            } else {
            targetPose2d = RobotContainer.swerveDrive.getPoseEstimator();
            }
        }
        // Red Poses
        else
        {
            if (touchInterf.getVirtualButton(screenButtons.kA)) {
                targetPose2d = FieldPoses.redPoses.reefA;
            } else if (touchInterf.getVirtualButton(screenButtons.kB)) {
                targetPose2d = FieldPoses.bluePoses.reefB;
            } else {
            targetPose2d = RobotContainer.swerveDrive.getPoseEstimator();
            }
        }
        return targetPose2d;
    }
}
