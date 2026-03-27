// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LimelightHelpers;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    // Cache Limelight IMU modes to avoid unnecessary NetworkTable writes
    private int cachedFrontIMUMode = -1;
    private int cachedLeftIMUMode = -1;

    public Robot() {
        Logger.recordMetadata("Beta-Bot", "MyProject"); // Set a metadata value

        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.updateSingleColorView();
        m_robotContainer.updateShiftHelpers();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.intake.setPivotBrakeMode(false);
    }

    @Override
    public void disabledPeriodic() {
        setLimelightIMUMode(1); // Seeding mode when disabled
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_robotContainer.intake.setPivotBrakeMode(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        // Use seeding mode when not rotating, use IMU mode when rotating
        int targetMode = m_robotContainer.drivetrain.notRotating() ? 1 : 2;
        setLimelightIMUMode(targetMode);
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        m_robotContainer.intake.setPivotBrakeMode(true);
    }

    @Override
    public void teleopPeriodic() {
        // Use seeding mode when not rotating, use IMU mode when rotating
        int targetMode = m_robotContainer.drivetrain.notRotating() ? 1 : 2;
        setLimelightIMUMode(targetMode);
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    /**
     * Sets Limelight IMU mode only if it has changed, avoiding unnecessary NetworkTable writes.
     * @param mode The IMU mode to set (1 = seeding, 2 = using IMU)
     */
    private void setLimelightIMUMode(int mode) {
        if (cachedFrontIMUMode != mode) {
            LimelightHelpers.SetIMUMode("limelight-front", mode);
            cachedFrontIMUMode = mode;
        }
        if (cachedLeftIMUMode != mode) {
            LimelightHelpers.SetIMUMode("limelight-left", mode);
            cachedLeftIMUMode = mode;
        }
    }
}
