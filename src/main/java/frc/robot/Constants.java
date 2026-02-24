// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.generated.TunerConstants;

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
  }







}