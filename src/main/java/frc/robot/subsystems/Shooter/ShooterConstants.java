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

    private static final LoggedTunableNumber kshooter15 = new LoggedTunableNumber("Shooter/Hub/1.5", 20,true);//TODO: set to false for comp
    private static final LoggedTunableNumber kshooter20 = new LoggedTunableNumber("Shooter/Hub/2.0", 33,true);
    private static final LoggedTunableNumber kshooter25 = new LoggedTunableNumber("Shooter/Hub/2.5", 38,true);
    private static final LoggedTunableNumber kshooter30 = new LoggedTunableNumber("Shooter/Hub/3.0", 40,true);
    private static final LoggedTunableNumber kshooter35 = new LoggedTunableNumber("Shooter/Hub/3.5", 45,true);
    private static final LoggedTunableNumber kshooter40 = new LoggedTunableNumber("Shooter/Hub/4.0", 47,true);
    private static final LoggedTunableNumber kshooter45 = new LoggedTunableNumber("Shooter/Hub/4.5", 55,true);
    private static final LoggedTunableNumber kshooter50 = new LoggedTunableNumber("Shooter/Hub/5.0", 65,true);
    private static final LoggedTunableNumber kshooter55 = new LoggedTunableNumber("Shooter/Hub/5.5", 70,true);
    private static final LoggedTunableNumber kshooter60 = new LoggedTunableNumber("Shooter/Hub/6.0", 80,true);
    private static final LoggedTunableNumber kshooter65 = new LoggedTunableNumber("Shooter/Hub/6.5", 85,true);
    private static final LoggedTunableNumber kshooter70 = new LoggedTunableNumber("Shooter/Hub/7.0", 85,true);

    
    public static InterpolatingDoubleTreeMap kShooterNeutralMap = new InterpolatingDoubleTreeMap();


    private static final LoggedTunableNumber kshooterNeutral35 = new LoggedTunableNumber("Shooter/Neutral/3.5", 40,true);//TODO: set to false for comp 
    private static final LoggedTunableNumber kshooterNeutral40 = new LoggedTunableNumber("Shooter/Neutral/4.0", 40,true);
    private static final LoggedTunableNumber kshooterNeutral45 = new LoggedTunableNumber("Shooter/Neutral/4.5", 40,true);
    private static final LoggedTunableNumber kshooterNeutral50 = new LoggedTunableNumber("Shooter/Neutral/5.0", 40,true);
    private static final LoggedTunableNumber kshooterNeutral55 = new LoggedTunableNumber("Shooter/Neutral/5.5", 40,true);
    private static final LoggedTunableNumber kshooterNeutral60 = new LoggedTunableNumber("Shooter/Neutral/6.0", 40,true);
    private static final LoggedTunableNumber kshooterNeutral65 = new LoggedTunableNumber("Shooter/Neutral/6.5", 40,true);
    private static final LoggedTunableNumber kshooterNeutral70 = new LoggedTunableNumber("Shooter/Neutral/7.0", 40,true);

    public static double getShooterHubVelocity(double distance) {
        
        kShooterHubMap.put(1.5, kshooter15.get());
        kShooterHubMap.put(2.0, kshooter20.get());
        kShooterHubMap.put(2.5, kshooter25.get());
        kShooterHubMap.put(3.0, kshooter30.get());
        kShooterHubMap.put(3.5, kshooter35.get());
        kShooterHubMap.put(4.0, kshooter40.get());
        kShooterHubMap.put(4.5, kshooter45.get());
        kShooterHubMap.put(5.0, kshooter50.get());
        kShooterHubMap.put(5.5, kshooter55.get());
        kShooterHubMap.put(6.0, kshooter60.get());
        kShooterHubMap.put(6.5, kshooter65.get());
        kShooterHubMap.put(7.0, kshooter70.get());

        return kShooterHubMap.get(distance);

    }

    public static double getShooterNeutralVelocity(double distance) {
        
        kShooterNeutralMap.put(3.5, kshooterNeutral35.get());
        kShooterNeutralMap.put(4.0, kshooterNeutral40.get());
        kShooterNeutralMap.put(4.5, kshooterNeutral45.get());
        kShooterNeutralMap.put(5.0, kshooterNeutral50.get());
        kShooterNeutralMap.put(5.5, kshooterNeutral55.get());
        kShooterNeutralMap.put(6.0, kshooterNeutral60.get());
        kShooterNeutralMap.put(6.5, kshooterNeutral65.get());
        kShooterNeutralMap.put(7.0, kshooterNeutral70.get());

        return kShooterNeutralMap.get(distance);

    }
}

