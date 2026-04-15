// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeConstants {
    public static final int kIntakeMotorId = 18;
    public static final int kIntakeMotor2Id = 25;

    public static final int kIntakeSupplyCurrentLimit = 35;
    public static final int kIntakeStatorCurrentLimit = 120;

    public static final double kIntakeInSpeed = 0.6;
    public static final double kIntakeOutSpeed = -kIntakeInSpeed;

    public static final int kPivotMotorId = 21;

    public static final double kP = 8;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kCruiseVelocity = 200;
    public static final double kAcceleration = 500;

    public static final double kPivotTolerance = 0.2;

    public static final double kPivotSupplyCurrentLimit = 35;

    public static final int kEncoderId = 27;
    public static final double kEncoderOffset = 0.955566;

    public static final double kIntakePivotStowPosition = -0.7;
    public static final double kIntakePivotAgitatePosition = -0.2;
    public static final double kIntakePivotIntakePosition = 0;

}