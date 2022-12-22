// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class Utilities {
    public Utilities() {
    }

    public void setPoseFromVisionData(RobotContainer robotContainer) {
    }

    public static Translation2d getRobotTranslation2d() {
        Translation2d res = new Translation2d(0, 0);
        return res;
    }

    public Pose2d getRobotPose() {
        // TODO return real pose
        Pose2d p = new Pose2d(new Translation2d(56, 36), new Rotation2d(5));
        return p;
    }

    public double getDistancePidLogic(double error) {
        double minPowerForMOvement = Constants.FF_VALUE;
        double res = 0;
        if (Math.abs(error) >= 0.5) {
            res = Math.signum(error) * minPowerForMOvement;
        }
        return res;
    }

}
