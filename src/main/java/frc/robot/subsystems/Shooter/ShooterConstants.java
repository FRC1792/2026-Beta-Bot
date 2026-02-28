// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;

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

    public static final double kVelocityTolerance = 2; // RPS 1-100


    public static final double kPrepSpeed = 10; // RPS 1-100

    public static final double kShooterShuttleSpeed = 50; // RPS 1-100


    public static InterpolatingDoubleTreeMap kShooterHubMap = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber kshooter15 = new LoggedTunableNumber("Shooter/Hub/1.5", 30,true);
    private static final LoggedTunableNumber kshooter20 = new LoggedTunableNumber("Shooter/Hub/2.0", 43,true);
    private static final LoggedTunableNumber kshooter25 = new LoggedTunableNumber("Shooter/Hub/2.5", 48,true);
    private static final LoggedTunableNumber kshooter30 = new LoggedTunableNumber("Shooter/Hub/3.0", 50,true);
    private static final LoggedTunableNumber kshooter35 = new LoggedTunableNumber("Shooter/Hub/3.5", 55,true);
    private static final LoggedTunableNumber kshooter40 = new LoggedTunableNumber("Shooter/Hub/4.0", 57,true);
    private static final LoggedTunableNumber kshooter45 = new LoggedTunableNumber("Shooter/Hub/4.5", 65,true);
    private static final LoggedTunableNumber kshooter50 = new LoggedTunableNumber("Shooter/Hub/5.0", 75,true);
    private static final LoggedTunableNumber kshooter55 = new LoggedTunableNumber("Shooter/Hub/5.5", 80,true);
    private static final LoggedTunableNumber kshooter60 = new LoggedTunableNumber("Shooter/Hub/6.0", 90,true);
    private static final LoggedTunableNumber kshooter65 = new LoggedTunableNumber("Shooter/Hub/6.5", 95,true);
    private static final LoggedTunableNumber kshooter70 = new LoggedTunableNumber("Shooter/Hub/7.0", 95,true);

    
    public static InterpolatingDoubleTreeMap kShooterNeutralMap = new InterpolatingDoubleTreeMap();


    private static final LoggedTunableNumber kshooterNeutral35 = new LoggedTunableNumber("Shooter/Neutral/3.5", 50,true);
    private static final LoggedTunableNumber kshooterNeutral40 = new LoggedTunableNumber("Shooter/Neutral/4.0", 50,true);
    private static final LoggedTunableNumber kshooterNeutral45 = new LoggedTunableNumber("Shooter/Neutral/4.5", 50,true);
    private static final LoggedTunableNumber kshooterNeutral50 = new LoggedTunableNumber("Shooter/Neutral/5.0", 50,true);
    private static final LoggedTunableNumber kshooterNeutral55 = new LoggedTunableNumber("Shooter/Neutral/5.5", 50,true);
    private static final LoggedTunableNumber kshooterNeutral60 = new LoggedTunableNumber("Shooter/Neutral/6.0", 50,true);
    private static final LoggedTunableNumber kshooterNeutral65 = new LoggedTunableNumber("Shooter/Neutral/6.5", 50,true);
    private static final LoggedTunableNumber kshooterNeutral70 = new LoggedTunableNumber("Shooter/Neutral/7.0", 50,true);

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

