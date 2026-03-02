// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LoggedTunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DriveConstants {

    public static final double kMaxAngularSpeedMultiplierDefault = 1.0; //Use this to change the default max angular speed multiplier

    public static final double kMaxAngularSpeedMultiplierWhileShooting = 0.25; //Use this to change the default max angular speed multiplier

    public static double kMaxSpeedMultiplier = 1.0; // DO NOT CHANGE THIS VALUE
    public static double kMaxAngularSpeedMultiplier = 1.0; // DO NOT CHANGE THIS VALUE

    public static double kMaxSpeed = DriveConstants.kMaxSpeedMultiplier * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double kMaxAngularRate = RotationsPerSecond.of(DriveConstants.kMaxAngularSpeedMultiplier).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final int kDriverControllerPort = 0;
    public static final double kTranslationDeadband = 0.1;
    public static final double kRotationDeadband = 0.1;
  }

  public static class ColorConstants {
    public static final Color green = new Color(0, 255, 68);
    public static final Color white = new Color(255, 255, 255);
    public static final Color blue = new Color(0, 57, 162);
  }


  public static class PoseConstants {
    public static final Pose2d BLUE_HUB = new Pose2d(4.625,4.035, Rotation2d.kZero);
    public static final Pose2d RED_HUB = FlippingUtil.flipFieldPose(BLUE_HUB);

    public static final Pose2d BLUE_OUTPOST_SHUTTLING = new Pose2d(2.500, 2.000, Rotation2d.kZero);
    public static final Pose2d BLUE_DEPOT_SHUTTLING = new Pose2d(2.500, 6.000, Rotation2d.kZero);

    public static final Pose2d RED_DEPOT_SHUTTLING = FlippingUtil.flipFieldPose(BLUE_DEPOT_SHUTTLING);
    public static final Pose2d RED_OUTPOST_SHUTTLING = FlippingUtil.flipFieldPose(BLUE_OUTPOST_SHUTTLING);

    public static final double kBlueAllianceZoneLineX = 4;
    public static final double kRedAllianceZoneLineX = 12.5;

    public static final double kFieldMidlineY = 4;

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  }

      public static class ZoneConstants {

        public static final Distance TRENCH_BUMP_X = Inches.of(181.56); // x position of the center of the trench and bump

        public static final Distance TRENCH_WIDTH = Inches.of(49.86); // y width of the trench

        public static final Distance TRENCH_BUMP_LENGTH = Inches.of(65); // x length of the trench and bump

        public static final Distance TRENCH_BLOCK_WIDTH = Inches.of(12); // y width of block separating bump and trench

        public static final Distance BUMP_WIDTH = Inches.of(73); // y width of bump

        // Time to look ahead for predictive zone entry (seconds)
        public static final double TRENCH_ALIGN_TIME_SECONDS = 0.5;
        public static final double BUMP_ALIGN_TIME_SECONDS = 0.5;

        public static final LoggedTunableNumber BUMP_Y_KP = new LoggedTunableNumber("Tuning/BumpYController/KP", 2.0,true);
        public static final LoggedTunableNumber BUMP_Y_KI = new LoggedTunableNumber("Tuning/BumpYController/KI", 0.0,true);
        public static final LoggedTunableNumber BUMP_Y_KD = new LoggedTunableNumber("Tuning/BumpYController/KD", 0.0,true);

        public static double getBumpYkP() {
          return BUMP_Y_KP.get();
        }

        public static double getBumpYkI() {
          return BUMP_Y_KI.get();
        }

        public static double getBumpYkD() {
          return BUMP_Y_KD.get();
        }

        public static final double BUMP_Y_TOLERANCE = 0.05; // meters

        public static final LoggedTunableNumber ROTATION_KP = new LoggedTunableNumber("Tuning/RotationYController/KP", 2.0,true);
        public static final LoggedTunableNumber ROTATION_KI = new LoggedTunableNumber("Tuning/RotationYController/KI", 0.0,true);
        public static final LoggedTunableNumber ROTATION_KD = new LoggedTunableNumber("Tuning/RotationYController/KD", 0.0,true);

        public static double getRotationkP() {
          return ROTATION_KP.get();
        }

        public static double getRotationkI() {
          return ROTATION_KI.get();
        }

        public static double getRotationkD() {
          return ROTATION_KD.get();
        }

        public static final double ROTATION_TOLERANCE = 0.05; // radians

        // Speed multiplier when in trench zone (0.0 - 1.0)
        public static final double TRENCH_SPEED_FACTOR = 0.5;
        public static final double BUMP_SPEED_FACTOR = 0.3;
        public static final double SHOOTING_SPEED_FACTOR = 0.25;

        // Center Y of bump (computed from zone geometry: trench + block + half bump width)
        public static final Distance BUMP_CENTER_Y = TRENCH_WIDTH.plus(TRENCH_BLOCK_WIDTH).plus(BUMP_WIDTH.div(2));



    }







}