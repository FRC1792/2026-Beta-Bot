// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.auto.shootIntoHub;
import frc.robot.generated.TunerConstants;
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
import frc.robot.util.Zones;

public class RobotContainer {

    private final Telemetry logger = new Telemetry(DriveConstants.kMaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(DriveConstants.kDriverControllerPort);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Turret turret = new Turret(drivetrain);
    public final Shooter shooter = new Shooter(drivetrain);
    public final Indexer indexer = new Indexer();
    public final Intake intake = new Intake();
    // public final Climber climber = new Climber();
    public final Vision vision = new Vision(
                                    drivetrain::addVisionMeasurement,
                                    new VisionIOLimelight(VisionConstants.camera0Name, () -> drivetrain.getState().Pose.getRotation()),
                                    new VisionIOLimelight(VisionConstants.camera1Name, () -> drivetrain.getState().Pose.getRotation()));

    public final AutoFactory autoFactory = new AutoFactory(drivetrain, intake, indexer, shooter);

    public final ShiftHelpers shiftHelpersUtil = new ShiftHelpers();

    public final Zones zonesUtil = new Zones();

    public final TeleopDrive teleopDrive = new TeleopDrive(drivetrain, m_driverController);

    public final shootIntoHub shootIntoHub = new shootIntoHub(shooter, indexer);

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {

        autoChooser = new SendableChooser<Command>();    
        
        autoChooser.setDefaultOption("Shoot Into Hub", shootIntoHub);
        autoChooser.setDefaultOption("Left Mobility Auto", autoFactory.getLeftMobilityAuto());
        autoChooser.addOption("Left Mobility Auto", autoFactory.getLeftMobilityAuto());
        autoChooser.addOption("Translation Tuning Auto", autoFactory.getTranslationTuningAuto());
        autoChooser.addOption("Straight Auto", autoFactory.getStraightAuto());
        autoChooser.addOption("Rotation Tuning Auto", autoFactory.getRotationTuningAuto());
        autoChooser.addOption("Neutral Auto", autoFactory.getNeutralAuto());
        autoChooser.addOption("Middle Depot P2 Auto", autoFactory.getMiddleDepotP2Auto());
        autoChooser.addOption("Neutral Zone Pickup P1 Auto", autoFactory.getNeutralZonePickupP1Auto());
        autoChooser.addOption("Trench To Trench Auto", autoFactory.getTrenchToTrenchAuto());

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureIdealBindings();
        // configureTestBindings();
        setupShiftHelpers();
        setupSingleColorView();
        zonesUtil.logAllZones();
    }

    private void setupShiftHelpers() {
        Logger.recordOutput("ShiftHelpers/CurrentShiftIsYours", shiftHelpersUtil.currentShiftIsYours());
        Logger.recordOutput("ShiftHelpers/TimeLeftInCurrentShift", shiftHelpersUtil.timeLeftInShiftSeconds(DriverStation.getMatchTime()));
        Logger.recordOutput("ShiftHelpers/CurrentShift", shiftHelpersUtil.getCurrentShiftState());
    }

    private void setupSingleColorView(){

        if (turret.isAtSetpoint() && shooter.isAtSetpoint() && m_driverController.rightTrigger().getAsBoolean()) { // If we're in a good shooting state, show green
            Logger.recordOutput("SingleColorView", ColorConstants.green.toHexString());
        }else if (shiftHelpersUtil.timeLeftInShiftSeconds(DriverStation.getMatchTime()) <= 5) { // If we're in the last 5 seconds of the shift
            Logger.recordOutput("SingleColorView", ColorConstants.white.toHexString());
        } else { // Otherwise, show blue
            Logger.recordOutput("SingleColorView", ColorConstants.blue.toHexString());
        }
    }

    private void configureIdealBindings() {

        // TeleopDrive handles field-centric driving with trench/bump auto-alignment
        drivetrain.setDefaultCommand(teleopDrive);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading.
        m_driverController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //Intake
        m_driverController.leftTrigger()
            .onTrue(
                intake.runOnce(()-> intake.setGoal(IntakeState.INTAKE)))
            .onFalse(
                intake.runOnce(()-> intake.setGoal(IntakeState.STOP)));
        
        //Spindex
        m_driverController.rightBumper()
            .onTrue(
                indexer.runOnce(()-> indexer.setGoal(IndexerState.SPINDEX))
            ).onFalse(
                indexer.runOnce(()-> indexer.setGoal(IndexerState.STOP)));


        //Shooter auto-goal, with the condition that the turret is at setpoint
        m_driverController.rightTrigger()
            .onTrue(
                shooter.runOnce(()-> {
                    shooter.setAutoGoalEnabled(true);
                }))
            .onFalse(
                shooter.runOnce(()-> {
                                        shooter.setAutoGoalEnabled(false); 
                                        shooter.setGoal(ShooterState.IDLE);
                                      })
            );

        //Outtake Intake and Indexer
        m_driverController.leftBumper()
            .onTrue(
                intake.runOnce(()-> {
                    intake.setGoal(IntakeState.OUTTAKE);
                    indexer.setGoal(IndexerState.OUTTAKE);
                }))
            .onFalse(
                intake.runOnce(()-> {
                    intake.setGoal(IntakeState.STOP);
                    indexer.setGoal(IndexerState.STOP);
                }));
        
        // //Climber Extend
        // m_driverController.start()
        //     .onTrue(climber.runOnce(() -> climber.setGoal(ClimberState.EXTEND)))
        //     .onFalse(climber.runOnce(() -> climber.setGoal(ClimberState.OFF)));
        
        // //Climber Retract
        // m_driverController.back()
        //     .onTrue(climber.runOnce(() -> climber.setGoal(ClimberState.RETRACT)))
        //     .onFalse(climber.runOnce(() -> climber.setGoal(ClimberState.OFF)));
        
        //Turret Stop Override
        m_driverController.x().onTrue(turret.runOnce(() -> turret.turretStop()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureTestBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(teleopDrive);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        m_driverController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        m_driverController.x()
            .onTrue(turret.runOnce(() -> turret.turretTurnLeft()))
            .onFalse(turret.runOnce(() -> turret.turretStop()));
        
        m_driverController.b()
            .onTrue(turret.runOnce(() -> turret.turretTurnRight()))
            .onFalse(turret.runOnce(() -> turret.turretStop()));

        m_driverController.y()
            .onTrue(turret.runOnce(() -> turret.setPivotPosition(90)));

        m_driverController.leftTrigger()
            .onTrue(intake.runOnce(() -> intake.setGoal(IntakeState.INTAKE)))
            .onFalse(intake.runOnce(() -> intake.setGoal(IntakeState.STOP)));

        
        m_driverController.rightTrigger()
            .onTrue(shooter.runOnce
            (() -> shooter.setAutoGoalEnabled(true)))
            .onFalse(shooter.runOnce(() -> {
                                        shooter.setAutoGoalEnabled(false);
                                        shooter.setGoal(ShooterState.IDLE);
                                      })
            );

        m_driverController.leftBumper()
            .onTrue(indexer.runOnce(() -> indexer.setGoal(IndexerState.SPINDEX)))
            .onFalse(indexer.runOnce(() -> indexer.setGoal(IndexerState.STOP)));
        
        m_driverController.rightBumper()
            .onTrue(indexer.runOnce(() -> indexer.setGoal(IndexerState.OUTTAKE)))
            .onFalse(indexer.runOnce(() -> indexer.setGoal(IndexerState.STOP)));
    

        // m_driverController.povUp()
        //     .onTrue(climber.runOnce(() -> climber.setGoal(ClimberState.EXTEND)))
        //     .onFalse(climber.runOnce(() -> climber.setGoal(ClimberState.OFF)));

        // m_driverController.povDown()
        //     .onTrue(climber.runOnce(() -> climber.setGoal(ClimberState.RETRACT)))
        //     .onFalse(climber.runOnce(() -> climber.setGoal(ClimberState.OFF)));

        drivetrain.registerTelemetry(logger::telemeterize);

        m_driverController.povLeft()
            .onTrue(turret.runOnce(() -> turret.zeroTuret()));

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
