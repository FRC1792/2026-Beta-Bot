// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class IndexerConstants {
    public static final int kIndexerMotorId = 20;
    public static final int kSpindexerMotorId = 19;

    public static final int kIndexerSupplyCurrentLimit = 35;
    public static final int kSpindexerSupplyCurrentLimit = 35;

    public static final int kIndexerSensorMinSignalStrength = 2000;
    public static final double kIndexerSensorProximityThreshold = Units.inchesToMeters(0.1);

    public static final double kIndexerInSpeed = 1;
    public static final double kIndexerOutSpeed = -kIndexerInSpeed;

    public static final LoggedTunableNumber kSpindexerTuningSpeed = new LoggedTunableNumber("SpindexerSpeed", 0.35,true);

    public static double kSpindexerInSpeed = kSpindexerTuningSpeed.get();
    public static final double kSpindexerOutSpeed = -kSpindexerInSpeed;

    public static final int kIndexerSensorId = 21;
}
