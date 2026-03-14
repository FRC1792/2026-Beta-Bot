// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeConstants {
    public static final int kIntakeMotorId = 18;

    public static final int kIntakeSupplyCurrentLimit = 35;

    public static final double kIntakeInSpeed = 0.7;
    public static final double kIntakeOutSpeed = -kIntakeInSpeed;

    public static final int kPivotMotorId = 21;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kCruiseVelocity = 1000;
    public static final double kAcceleration = 1000;

    public static final double kPivotTolerance = 0.01;

    public static final double kPivotSupplyCurrentLimit = 35;

    public static final double kIntakePivotStowPosition = -0.2;
    public static final double kIntakePivotIntakePosition = 1;
}