// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
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
public final class Constants {
    public static final class DriveConstants {

        public static double kMaxAccelerationMetersPerSecondSquared = 3.0;
        public static double kMaxSpeedMetersPerSecond = 3.0;

        public static final int fr_motor = 2;
        public static final int fl_motor = 4;

        public static final int br_motor = 1;
        public static final int bl_motor = 3;

        public static final int XboxPort = 0;

        public static final double centerOfWheel = ((51.5 - 44) / 2);

        // distance to center (right and left)
        public static final double kTrackWidth = (44 + centerOfWheel * 2) / 100; // convert it into meters

        // distance to center (front and back)
        public static final double kWheelBase = (52) / 100; // convert it into meters;

        public static final double w = 1 / 10.71 * Units.inchesToMeters(8) * Math.PI;

        public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
                new Translation2d(
                        kTrackWidth , kWheelBase),
                new Translation2d(
                        kTrackWidth, -kWheelBase),
                new Translation2d(
                        -kTrackWidth, kWheelBase),
                new Translation2d(
                        -kTrackWidth, -kWheelBase));

        public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(br_motor, bl_motor,
                XboxPort);

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPFrontLeftVel = 0.5;
        public static final double kPRearLeftVel = 0.5;
        public static final double kPFrontRightVel = 0.5;
        public static final double kPRearRightVel = 0.5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPThetaController = 1.0;

        public static final double kPXController = 0.5;
        public static final double kPYController = 0.5;
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
