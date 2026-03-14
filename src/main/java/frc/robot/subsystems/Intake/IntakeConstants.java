// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeConstants {
    public static final int kIntakeMotorId = 18;

    public static final int kPivotMotorId = 21;

    public static final int kIntakeSupplyCurrentLimit = 35;

    public static final int kPivotSupplyCurrentLimit = 35;

    public static final double kIntakeInSpeed = 0.7;
    public static final double kIntakeOutSpeed = -kIntakeInSpeed;

    public static final double kPivotInSpeed = 0.3;
    public static final double kPivotOutSpeed = -kPivotInSpeed;

    public static final double kTolerance = 5;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final int kAcceleration = 100;
    public static final int kJerk = 100;
    public static final int kVelocity = 100;
}