// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;


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

    public static final double kTurretTransformInchesX = 6.5; // Inches
    public static final double kTurretTransformInchesY = 5.5; // Inches

    
    public static final double kTolerance = 5;

    public static final double kSupplyCurrentLimit = 35;

    public static final double kSensorToMechanismRatio = 12.5;

    public static final double kSpeed = 0.15;

    private static final InterpolatingDoubleTreeMap kAirTimeMap = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber kAirTime0 = new LoggedTunableNumber( "Shooter/Air Time/0",   0, false);
    private static final LoggedTunableNumber kAirTime2 = new LoggedTunableNumber("Shooter/Air Time/2", 0.71, false);
    private static final LoggedTunableNumber kAirTime25 = new LoggedTunableNumber("Shooter/Air Time/2.5", 0.83, false);
    private static final LoggedTunableNumber kAirTime3 = new LoggedTunableNumber("Shooter/Air Time/3", 1.03, false);
    private static final LoggedTunableNumber kAirTime35 = new LoggedTunableNumber("Shooter/Air Time/3.5", 1.01, false);
    private static final LoggedTunableNumber kAirTime4 = new LoggedTunableNumber( "Shooter/Air Time/4",   1.14, false);
    private static final LoggedTunableNumber kAirTime45 = new LoggedTunableNumber( "Shooter/Air Time/4.5",   1.38, false);
    private static final LoggedTunableNumber kAirTime5 = new LoggedTunableNumber( "Shooter/Air Time/5",   1.3, false);


    public static double getAirTime(double dist) {
        kAirTimeMap.put(0.0,  kAirTime0.get());
        kAirTimeMap.put(2.0,  kAirTime2.get());
        kAirTimeMap.put(2.5,  kAirTime25.get());
        kAirTimeMap.put(3.0,  kAirTime3.get());
        kAirTimeMap.put(3.5,  kAirTime35.get());
        kAirTimeMap.put(4.0,  kAirTime4.get());
        kAirTimeMap.put(4.5,  kAirTime45.get());
        kAirTimeMap.put(5.0,  kAirTime5.get());

        return kAirTimeMap.get(dist);
    }
}
