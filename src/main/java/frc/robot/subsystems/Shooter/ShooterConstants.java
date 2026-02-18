// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShooterConstants {
    public static final int kMotorId = 23;

    public static final int kSupplyCurrentLimit = 35;

    public static final double kP = 0.18249;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.26444;
    public static final double kA = 0.0076942;
    public static final double kV = 0.12232;

    public static final int kAcceleration = 1000;
    public static final int kJerk = 1000;

    public static final double kFeedforward = 0.0;

    public static final double kVelocityTolerance = 1;

    public static final double kPrepSpeed = 0.25;

    public static final double kMotorSpeed = 0.60;

    public static InterpolatingDoubleTreeMap kShooterMap = new InterpolatingDoubleTreeMap(); //may need to change data types

    public static void setupShooterMap() {
        kShooterMap.put(1.0, 1.0);
        kShooterMap.put(2.0, 2.1);
        kShooterMap.put(3.0, 3.2);
        kShooterMap.put(4.0, 4.0);
        kShooterMap.put(5.0, 5.0);
    }
}

