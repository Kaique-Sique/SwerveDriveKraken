// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldPoses {
    public static class bluePoses {
        public static final Pose2d reefA = new Pose2d(
            new Translation2d(3.05, 4.18), Rotation2d.fromDegrees(0));

        public static final Pose2d reefB = new Pose2d(
            new Translation2d(3.05, 3.87), Rotation2d.fromDegrees(0));
    }

    public static class redPoses {
        public static final Pose2d reefA = new Pose2d(
            new Translation2d(14.47, 3.87), Rotation2d.fromDegrees(180));

        public static final Pose2d reefB = new Pose2d(
            new Translation2d(14.47,4.190), Rotation2d.fromDegrees(180));
    }
}
