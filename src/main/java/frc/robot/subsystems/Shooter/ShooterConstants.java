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
    public static final int kStatorCurrentLimit = 25;

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

    // private static final LoggedTunableNumber kshooter15 = new LoggedTunableNumber("Shooter/Hub/1.5", 44,false);
    // private static final LoggedTunableNumber kshooter175 = new LoggedTunableNumber("Shooter/Hub/1.75", 40,false);
    // private static final LoggedTunableNumber kshooter20 = new LoggedTunableNumber("Shooter/Hub/2.0", 42,false);
    // private static final LoggedTunableNumber kshooter225 = new LoggedTunableNumber("Shooter/Hub/2.25", 40,false);
    // private static final LoggedTunableNumber kshooter25 = new LoggedTunableNumber("Shooter/Hub/2.5", 43,false);
    // private static final LoggedTunableNumber kshooter275 = new LoggedTunableNumber("Shooter/Hub/2.75", 44,false);
    // private static final LoggedTunableNumber kshooter30 = new LoggedTunableNumber("Shooter/Hub/3.0", 47,false);
    // private static final LoggedTunableNumber kshooter325 = new LoggedTunableNumber("Shooter/Hub/3.25", 48,false);
    // private static final LoggedTunableNumber kshooter35 = new LoggedTunableNumber("Shooter/Hub/3.5", 49,false);
    // private static final LoggedTunableNumber kshooter375 = new LoggedTunableNumber("Shooter/Hub/3.75", 55,false);
    // private static final LoggedTunableNumber kshooter40 = new LoggedTunableNumber("Shooter/Hub/4.0", 58,false);
    // private static final LoggedTunableNumber kshooter425 = new LoggedTunableNumber("Shooter/Hub/4.25", 58,false);
    // private static final LoggedTunableNumber kshooter45 = new LoggedTunableNumber("Shooter/Hub/4.5", 58,false);
    // private static final LoggedTunableNumber kshooter475 = new LoggedTunableNumber("Shooter/Hub/4.75", 60,false);
    // private static final LoggedTunableNumber kshooter50 = new LoggedTunableNumber("Shooter/Hub/5.0", 63,false);
    // private static final LoggedTunableNumber kshooter525 = new LoggedTunableNumber("Shooter/Hub/5.25", 63,false);
    // private static final LoggedTunableNumber kshooter55 = new LoggedTunableNumber("Shooter/Hub/5.5", 65,false);
    // private static final LoggedTunableNumber kshooter575 = new LoggedTunableNumber("Shooter/Hub/5.75", 75,true);
    // private static final LoggedTunableNumber kshooter60 = new LoggedTunableNumber("Shooter/Hub/6.0", 80,true);
    // private static final LoggedTunableNumber kshooter625 = new LoggedTunableNumber("Shooter/Hub/6.25", 83,true);
    // private static final LoggedTunableNumber kshooter65 = new LoggedTunableNumber("Shooter/Hub/6.5", 85,true);
    // private static final LoggedTunableNumber kshooter675 = new LoggedTunableNumber("Shooter/Hub/6.75", 85,true);
    // private static final LoggedTunableNumber kshooter70 = new LoggedTunableNumber("Shooter/Hub/7.0", 85,true);

    
    public static InterpolatingDoubleTreeMap kShooterNeutralMap = new InterpolatingDoubleTreeMap();


    // private static final LoggedTunableNumber kshooterNeutral3 = new LoggedTunableNumber("Shooter/Neutral/3.5", 50,true);
    // private static final LoggedTunableNumber kshooterNeutral85 = new LoggedTunableNumber("Shooter/Neutral/8.5", 90,true);

    public static double getShooterHubVelocity(double distance) {
        
        kShooterHubMap.put(1.5, 44.0);
        kShooterHubMap.put(1.75, 40.0);
        kShooterHubMap.put(2.0, 42.0);
        kShooterHubMap.put(2.25, 40.0);
        kShooterHubMap.put(2.5, 43.0);
        kShooterHubMap.put(2.75, 44.0);
        kShooterHubMap.put(3.0, 47.0);
        kShooterHubMap.put(3.25, 48.0);
        kShooterHubMap.put(3.5, 49.0);
        kShooterHubMap.put(3.75, 55.0);
        kShooterHubMap.put(4.0, 58.0);
        kShooterHubMap.put(4.25, 58.0);
        kShooterHubMap.put(4.5, 58.0);
        kShooterHubMap.put(4.75, 60.0);
        kShooterHubMap.put(5.0, 63.0);
        kShooterHubMap.put(5.25, 63.0);
        kShooterHubMap.put(5.5, 65.0);

        return kShooterHubMap.get(distance);

    }

    public static double getShooterNeutralVelocity(double distance) {
        
        kShooterNeutralMap.put(3.0, 50.0);
        kShooterNeutralMap.put(8.5, 90.0);

        return kShooterNeutralMap.get(distance);

    }
}

