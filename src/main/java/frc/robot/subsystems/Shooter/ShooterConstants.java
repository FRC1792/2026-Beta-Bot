// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class ShooterConstants {
    public static final int kMotor1Id = 23;
    public static final int kMotor2Id = 24;

    public static final int kSupplyCurrentLimit = 35;

    public static final double kP = 0.18249;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.26444;
    public static final double kA = 0.0076942;
    public static final double kV = 0.12232;

    public static final int kAcceleration = 1000;
    public static final int kJerk = 1000;

    public static final double kVelocityTolerance = 2; // RPS 1-100


    public static final double kIdleSpeed = 10; // RPS 1-100

    public static final double kManualTowerSpeed = 50; // RPS 1-100


    public static InterpolatingDoubleTreeMap kShooterHubMap = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber kshooter15 = new LoggedTunableNumber("Shooter/Hub/1.5", 44,true);
    private static final LoggedTunableNumber kshooter175 = new LoggedTunableNumber("Shooter/Hub/1.75", 40,true);
    private static final LoggedTunableNumber kshooter20 = new LoggedTunableNumber("Shooter/Hub/2.0", 42,true);
    private static final LoggedTunableNumber kshooter225 = new LoggedTunableNumber("Shooter/Hub/2.25", 40,true);
    private static final LoggedTunableNumber kshooter25 = new LoggedTunableNumber("Shooter/Hub/2.5", 43,true);
    private static final LoggedTunableNumber kshooter275 = new LoggedTunableNumber("Shooter/Hub/2.75", 44,true);
    private static final LoggedTunableNumber kshooter30 = new LoggedTunableNumber("Shooter/Hub/3.0", 47,true);
    private static final LoggedTunableNumber kshooter325 = new LoggedTunableNumber("Shooter/Hub/3.25", 48,true);
    private static final LoggedTunableNumber kshooter35 = new LoggedTunableNumber("Shooter/Hub/3.5", 49,true);
    private static final LoggedTunableNumber kshooter375 = new LoggedTunableNumber("Shooter/Hub/3.75", 55,true);
    private static final LoggedTunableNumber kshooter40 = new LoggedTunableNumber("Shooter/Hub/4.0", 58,true);
    private static final LoggedTunableNumber kshooter425 = new LoggedTunableNumber("Shooter/Hub/4.25", 58,true);
    private static final LoggedTunableNumber kshooter45 = new LoggedTunableNumber("Shooter/Hub/4.5", 58,true);
    private static final LoggedTunableNumber kshooter475 = new LoggedTunableNumber("Shooter/Hub/4.75", 60,true);
    private static final LoggedTunableNumber kshooter50 = new LoggedTunableNumber("Shooter/Hub/5.0", 63,true);
    private static final LoggedTunableNumber kshooter525 = new LoggedTunableNumber("Shooter/Hub/5.25", 63,true);
    private static final LoggedTunableNumber kshooter55 = new LoggedTunableNumber("Shooter/Hub/5.5", 65,true);

    
    public static InterpolatingDoubleTreeMap kShooterNeutralMap = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber kshooterNeutral30 = new LoggedTunableNumber("Shooter/Neutral/3.0", 50,true);
    private static final LoggedTunableNumber kshooterNeutral85 = new LoggedTunableNumber("Shooter/Neutral/8.5", 90,true);


    public static double getShooterHubVelocity(double distance) {
        
        kShooterHubMap.put(1.5, 44.0);
        kShooterHubMap.put(1.75, 40.0);
        kShooterHubMap.put(2.0, 42.0);
        kShooterHubMap.put(2.25, 44.0);
        kShooterHubMap.put(2.5, 45.0);
        kShooterHubMap.put(2.75, 48.0);
        kShooterHubMap.put(3.0, 49.0);
        kShooterHubMap.put(3.25, 51.0);
        kShooterHubMap.put(3.5, 54.0);
        kShooterHubMap.put(3.75, 56.0);
        kShooterHubMap.put(4.0, 57.0);
        kShooterHubMap.put(4.25, 57.0);
        kShooterHubMap.put(4.5, 59.0);
        kShooterHubMap.put(4.75, 60.0);
        kShooterHubMap.put(5.0, 62.0);
        kShooterHubMap.put(5.25, 64.0);
        kShooterHubMap.put(5.5, 66.0);


        // kShooterHubMap.put(1.5, kshooter15.get());
        // kShooterHubMap.put(1.75, kshooter175.get());
        // kShooterHubMap.put(2.0, kshooter20.get());
        // kShooterHubMap.put(2.25, kshooter225.get());
        // kShooterHubMap.put(2.5, kshooter25.get());
        // kShooterHubMap.put(2.75, kshooter275.get());
        // kShooterHubMap.put(3.0, kshooter30.get());
        // kShooterHubMap.put(3.25, kshooter325.get());
        // kShooterHubMap.put(3.5, kshooter35.get());
        // kShooterHubMap.put(3.75, kshooter375.get());
        // kShooterHubMap.put(4.0, kshooter40.get());
        // kShooterHubMap.put(4.25, kshooter425.get());
        // kShooterHubMap.put(4.5, kshooter45.get());
        // kShooterHubMap.put(4.75, kshooter475.get());
        // kShooterHubMap.put(5.0, kshooter50.get());
        // kShooterHubMap.put(5.25, kshooter525.get());
        // kShooterHubMap.put(5.5, kshooter55.get());

        return kShooterHubMap.get(distance);

    }

    public static double getShooterNeutralVelocity(double distance) {
        
        // kShooterNeutralMap.put(3.0, 50.0);
        // kShooterNeutralMap.put(8.5, 90.0);

        kShooterNeutralMap.put(3.0, kshooterNeutral30.get());
        kShooterNeutralMap.put(8.5, kshooterNeutral85.get());
        kShooterNeutralMap.put(10.0, 65.0);

        return kShooterNeutralMap.get(distance);

    }
}

