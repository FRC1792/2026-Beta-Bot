// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

/** Add your docs here. */
public class TurretConstants {
    public static final int kMotorId = 22;

    public static final double kMinAngle = -180;
    public static final double kMaxAngle = 180;

    public static final double kP = 200;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kA = 0; // 0.50555;
    public static final double kV = 0; //1.4999;
    public static final double kS = 0; //-0.34727;



    public static final double kCruiseVelocity = 1000;
    public static final double kAcceleration = 5000;
    public static final double kJerk = 0;

    public static final double kTurretTransformInchesX = 6.5;
    public static final double kTurretTransformInchesY = 5.5;

    public static final double kFeedforward = 0.0;
    
    public static final double kTolerance = 5;

    public static final double kSupplyCurrentLimit = 35;

    public static final double kSensorToMechanismRatio = 12.5;

    public static final double kSpeed = 0.15;
}
