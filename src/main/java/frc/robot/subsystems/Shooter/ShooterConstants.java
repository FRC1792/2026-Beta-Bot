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

    private static final LoggedTunableNumber kshooter15 = new LoggedTunableNumber("Shooter/Hub/1.5", 44,true);//TODO: set to false for comp
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
    // private static final LoggedTunableNumber kshooter575 = new LoggedTunableNumber("Shooter/Hub/5.75", 75,true);
    // private static final LoggedTunableNumber kshooter60 = new LoggedTunableNumber("Shooter/Hub/6.0", 80,true);
    // private static final LoggedTunableNumber kshooter625 = new LoggedTunableNumber("Shooter/Hub/6.25", 83,true);
    // private static final LoggedTunableNumber kshooter65 = new LoggedTunableNumber("Shooter/Hub/6.5", 85,true);
    // private static final LoggedTunableNumber kshooter675 = new LoggedTunableNumber("Shooter/Hub/6.75", 85,true);
    // private static final LoggedTunableNumber kshooter70 = new LoggedTunableNumber("Shooter/Hub/7.0", 85,true);

    
    public static InterpolatingDoubleTreeMap kShooterNeutralMap = new InterpolatingDoubleTreeMap();


    private static final LoggedTunableNumber kshooterNeutral35 = new LoggedTunableNumber("Shooter/Neutral/3.5", 64,true);//TODO: set to false for comp 
    private static final LoggedTunableNumber kshooterNeutral375 = new LoggedTunableNumber("Shooter/Neutral/3.75", 64,true);
    private static final LoggedTunableNumber kshooterNeutral40 = new LoggedTunableNumber("Shooter/Neutral/4.0", 66,true);
    private static final LoggedTunableNumber kshooterNeutral425 = new LoggedTunableNumber("Shooter/Neutral/4.25", 60,true);
    private static final LoggedTunableNumber kshooterNeutral45 = new LoggedTunableNumber("Shooter/Neutral/4.5", 62,true);
    private static final LoggedTunableNumber kshooterNeutral475 = new LoggedTunableNumber("Shooter/Neutral/4.75", 60,true);
    private static final LoggedTunableNumber kshooterNeutral50 = new LoggedTunableNumber("Shooter/Neutral/5.0", 67,true);
    private static final LoggedTunableNumber kshooterNeutral525 = new LoggedTunableNumber("Shooter/Neutral/5.25", 68,true);
    private static final LoggedTunableNumber kshooterNeutral55 = new LoggedTunableNumber("Shooter/Neutral/5.5", 68,true);
    private static final LoggedTunableNumber kshooterNeutral575 = new LoggedTunableNumber("Shooter/Neutral/5.75", 70,true);
    private static final LoggedTunableNumber kshooterNeutral60 = new LoggedTunableNumber("Shooter/Neutral/6.0", 73,true);
    private static final LoggedTunableNumber kshooterNeutral625 = new LoggedTunableNumber("Shooter/Neutral/6.25", 73,true);
    private static final LoggedTunableNumber kshooterNeutral65 = new LoggedTunableNumber("Shooter/Neutral/6.5", 74,true);
    private static final LoggedTunableNumber kshooterNeutral675 = new LoggedTunableNumber("Shooter/Neutral/6.75", 75,true);
    private static final LoggedTunableNumber kshooterNeutral70 = new LoggedTunableNumber("Shooter/Neutral/7.0", 75,true);

    public static double getShooterHubVelocity(double distance) {
        
    kShooterHubMap.put(1.5, kshooter15.get());
    kShooterHubMap.put(1.75, kshooter175.get());
    kShooterHubMap.put(2.0, kshooter20.get());
    kShooterHubMap.put(2.25, kshooter225.get());
    kShooterHubMap.put(2.5, kshooter25.get());
    kShooterHubMap.put(2.75, kshooter275.get());
    kShooterHubMap.put(3.0, kshooter30.get());
    kShooterHubMap.put(3.25, kshooter325.get());
    kShooterHubMap.put(3.5, kshooter35.get());
    kShooterHubMap.put(3.75, kshooter375.get());
    kShooterHubMap.put(4.0, kshooter40.get());
    kShooterHubMap.put(4.25, kshooter425.get());
    kShooterHubMap.put(4.5, kshooter45.get());
    kShooterHubMap.put(4.75, kshooter475.get());
    kShooterHubMap.put(5.0, kshooter50.get());
    kShooterHubMap.put(5.25, kshooter525.get());
    kShooterHubMap.put(5.5, kshooter55.get());
    // kShooterHubMap.put(5.75, kshooter575.get());
    // kShooterHubMap.put(6.0, kshooter60.get());
    // kShooterHubMap.put(6.25, kshooter625.get());
    // kShooterHubMap.put(6.5, kshooter65.get());
    // kShooterHubMap.put(6.75, kshooter675.get());
    // kShooterHubMap.put(7.0, kshooter70.get());

        return kShooterHubMap.get(distance);

    }

    public static double getShooterNeutralVelocity(double distance) {
        
    kShooterNeutralMap.put(3.5, kshooterNeutral35.get());
    kShooterNeutralMap.put(3.75, kshooterNeutral375.get());
    kShooterNeutralMap.put(4.0, kshooterNeutral40.get());
    kShooterNeutralMap.put(4.25, kshooterNeutral425.get());
    kShooterNeutralMap.put(4.5, kshooterNeutral45.get());
    kShooterNeutralMap.put(4.75, kshooterNeutral475.get());
    kShooterNeutralMap.put(5.0, kshooterNeutral50.get());
    kShooterNeutralMap.put(5.25, kshooterNeutral525.get());
    kShooterNeutralMap.put(5.5, kshooterNeutral55.get());
    kShooterNeutralMap.put(5.75, kshooterNeutral575.get());
    kShooterNeutralMap.put(6.0, kshooterNeutral60.get());
    kShooterNeutralMap.put(6.25, kshooterNeutral625.get());
    kShooterNeutralMap.put(6.5, kshooterNeutral65.get());
    kShooterNeutralMap.put(6.75, kshooterNeutral675.get());
    kShooterNeutralMap.put(7.0, kshooterNeutral70.get());

        return kShooterNeutralMap.get(distance);

    }
}

