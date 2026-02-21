// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.util.ShiftHelpers;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Turret turret = new Turret(drivetrain);
    public final Shooter shooter = new Shooter(drivetrain);
    public final Indexer indexer = new Indexer();
    public final Intake intake = new Intake();
    public final Climber climber = new Climber();
    public final Vision vision = new Vision(
                                    drivetrain::addVisionMeasurement,
                                    new VisionIOLimelight(VisionConstants.camera0Name, () -> drivetrain.getState().Pose.getRotation()),
                                    new VisionIOLimelight(VisionConstants.camera1Name, () -> drivetrain.getState().Pose.getRotation()));

    public final AutoFactory autoFactory = new AutoFactory(drivetrain, intake, indexer, shooter, turret);

    public final ShiftHelpers shiftHelpers = new ShiftHelpers();

    public RobotContainer() {

        configureIdealBindings();
        // configureTestBindings();
        setupShiftHelpers();
        setupSingleColorView();
    }

    private void setupShiftHelpers() {
        Logger.recordOutput("ShiftHelpers/CurrentShiftIsYours", shiftHelpers.currentShiftIsYours());
        Logger.recordOutput("ShiftHelpers/TimeLeftInCurrentShift", shiftHelpers.timeLeftInShiftSeconds(DriverStation.getMatchTime()));
        Logger.recordOutput("ShiftHelpers/CurrentShift", shiftHelpers.getCurrentShiftState());
    }

    private void setupSingleColorView(){
        Color green = new Color(0, 255, 68);
        Color white = new Color(255, 255, 255);
        Color blue = new Color(0, 57, 162);

        if (turret.isAtSetpoint() && shooter.isAtSetpoint() && joystick.rightTrigger().getAsBoolean()) { // If we're in a good shooting state, show green
            Logger.recordOutput("SingleColorView", green.toHexString());
        }else if (shiftHelpers.timeLeftInShiftSeconds(DriverStation.getMatchTime()) <= 5) { // If we're in the last 5 seconds of the shift
            Logger.recordOutput("SingleColorView", white.toHexString());
        } else { // Otherwise, show blue
            Logger.recordOutput("SingleColorView", blue.toHexString());
        }
    }

    private void configureIdealBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        // Reset the field-centric heading on button A press.
        joystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


        joystick.leftTrigger()
            .onTrue(
                intake.runOnce(()-> intake.setGoal(IntakeState.INTAKE)))
            .onFalse(
                intake.runOnce(()-> intake.setGoal(IntakeState.STOP)));



        joystick.rightTrigger()
            .onTrue(
                shooter.runOnce(()-> shooter.setAutoGoalEnabled(true)))
            .onFalse(
                shooter.runOnce(()-> {
                                        shooter.setAutoGoalEnabled(false); 
                                        shooter.setGoal(ShooterState.IDLE);
                                      })
            );

        joystick.leftBumper()
            .onTrue(
                intake.runOnce(()-> intake.setGoal(IntakeState.OUTTAKE)))
            .onFalse(
                intake.runOnce(()-> intake.setGoal(IntakeState.STOP)));

        joystick.rightBumper()
            .onTrue(
                indexer.runOnce(()-> indexer.setGoal(IndexerState.OUTTAKE)))
            .onFalse(
                indexer.runOnce(()-> indexer.setGoal(IndexerState.STOP)));

        joystick.rightTrigger()
            // .and(
            //     ()-> shooter.isAtSetpoint())
            .and(
                ()-> turret.isAtSetpoint())

            .onTrue(
                indexer.runOnce(()-> indexer.setGoal(IndexerState.SPINDEX))
            ).onFalse(
                indexer.runOnce(()-> indexer.setGoal(IndexerState.STOP)));


        joystick.start().onTrue(
            climber.runOnce(()-> climber.setGoal(ClimberState.EXTEND)));
            
        joystick.back().onTrue(
            climber.runOnce(()-> climber.setGoal(ClimberState.RETRACT)));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureTestBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.x()
            .onTrue(turret.runOnce(() -> turret.turretTurnLeft()))
            .onFalse(turret.runOnce(() -> turret.turretStop()));
        
        joystick.b()
            .onTrue(turret.runOnce(() -> turret.turretTurnRight()))
            .onFalse(turret.runOnce(() -> turret.turretStop()));

        joystick.y()
            .onTrue(turret.runOnce(() -> turret.setPivotPosition(90)));

        joystick.leftTrigger()
            .onTrue(intake.runOnce(() -> intake.setGoal(IntakeState.INTAKE)))
            .onFalse(intake.runOnce(() -> intake.setGoal(IntakeState.STOP)));

        
        joystick.rightTrigger()
            .onTrue(shooter.runOnce
            (() -> shooter.setAutoGoalEnabled(true)))
            .onFalse(shooter.runOnce(() -> {
                                        shooter.setAutoGoalEnabled(false);
                                        shooter.setGoal(ShooterState.IDLE);
                                      })
            );

        joystick.leftBumper()
            .onTrue(indexer.runOnce(() -> indexer.setGoal(IndexerState.SPINDEX)))
            .onFalse(indexer.runOnce(() -> indexer.setGoal(IndexerState.STOP)));
        
        joystick.rightBumper()
            .onTrue(indexer.runOnce(() -> indexer.setGoal(IndexerState.OUTTAKE)))
            .onFalse(indexer.runOnce(() -> indexer.setGoal(IndexerState.STOP)));
    

        joystick.povUp()
            .onTrue(climber.runOnce(() -> climber.setGoal(ClimberState.EXTEND)))
            .onFalse(climber.runOnce(() -> climber.setGoal(ClimberState.OFF)));

        joystick.povDown()
            .onTrue(climber.runOnce(() -> climber.setGoal(ClimberState.RETRACT)))
            .onFalse(climber.runOnce(() -> climber.setGoal(ClimberState.OFF)));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.povLeft()
            .onTrue(turret.runOnce(() -> turret.zeroTuret()));

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return null;
    }
}
