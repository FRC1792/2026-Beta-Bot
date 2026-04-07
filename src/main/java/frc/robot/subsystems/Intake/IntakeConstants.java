// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeConstants {
    public static final int kIntakeMotorId = 18;

    public static final int kIntakeSupplyCurrentLimit = 35;
    public static final int kIntakeStatorCurrentLimit = 120;

    public static final double kIntakeInSpeed = 1;
    public static final double kIntakeOutSpeed = -kIntakeInSpeed;

    public static final int kPivotMotorId = 21;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kCruiseVelocity = 100;
    public static final double kAcceleration = 100;

    public static final double kPivotTolerance = 3;

    public static final double kPivotSupplyCurrentLimit = 35;

    public static final double kIntakePivotStowPosition = -28;
    public static final double kIntakePivotAgitatePosition = -10;
    public static final double kIntakePivotIntakePosition = 1;

    // Crescendo agitation constants (oscillates from intake position upward toward stow)
    public static final double kCrescendoStartPosition = 0;
    public static final double kCrescendoMinAmplitude = 5.0;
    public static final double kCrescendoMaxAmplitude = 28.0;
    public static final double kCrescendoAmplitudeStep = 4.0;
}