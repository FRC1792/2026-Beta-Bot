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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.teleopDrive;
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
import frc.robot.subsystems.LEDs.LEDStates;
import frc.robot.subsystems.LEDs.LEDs;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShiftHelpers;

public class RobotContainer {

    private final Telemetry logger = new Telemetry(DriveConstants.kMaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(DriveConstants.kDriverControllerPort);
    
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
    public final LEDs LEDs = new LEDs();

    public final teleopDrive teleopDrive = new teleopDrive(drivetrain, m_driverController);

    public final AutoFactory autoFactory = new AutoFactory(drivetrain, intake, indexer, shooter, climber);

    public final ShiftHelpers shiftHelpers = new ShiftHelpers();

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        ShotCalculator.getInstance().init(drivetrain);
        setupAutoChooser();
        configureIdealBindings();
        // configureTestBindings();
    }

    private void setupAutoChooser() {
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Right Swipe Outpost", autoFactory.getRightNeutralSwipeOutpostAuto());
        autoChooser.addOption("Left Swipe Depot", autoFactory.getLeftNeutralSwipeDepotAuto());
        autoChooser.addOption("None", Commands.none());

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureIdealBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(teleopDrive);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        // Reset the field-centric heading.
        m_driverController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        
        m_driverController.y()
            .onTrue(
                drivetrain.runOnce(() -> teleopDrive.setBumpTrenchAssistEnabled(false))
            ).onFalse(
                drivetrain.runOnce(() -> teleopDrive.setBumpTrenchAssistEnabled(true)));

        //Intake
        m_driverController.leftTrigger()
            .onTrue(
                intake.runOnce(()-> intake.setGoal(IntakeState.OUTTAKE))
                    .andThen(new WaitCommand(0.1))
                    .andThen(intake.runOnce(()-> intake.setGoal(IntakeState.INTAKE)))
            )
            .onFalse(
                intake.runOnce(()-> intake.setGoal(IntakeState.STOP))
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

             m_driverController.rightTrigger()
            .whileTrue(
                shooter.runOnce(() -> shooter.setAutoGoalEnabled(true))
                .andThen(Commands.waitUntil(shooter::isAtSetpoint))
                .andThen(indexer.runOnce(() -> indexer.setGoal(IndexerState.OUTTAKE)))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(indexer.runOnce(() -> indexer.setGoal(IndexerState.SPINDEX)))
                .andThen(
                    Commands.repeatingSequence(
                        Commands.either(
                            Commands.none(),
                            Commands.runOnce(() -> intake.setGoal(IntakeState.AGITATE)),
                            intake::isIntaking
                        ),
                        Commands.waitSeconds(0.5),
                        Commands.either(
                            Commands.none(),
                            Commands.runOnce(() -> intake.setGoal(IntakeState.DOWN)),
                            intake::isIntaking
                        ),
                        Commands.waitSeconds(0.5)
                    )
                )
                .finallyDo(() -> {
                    shooter.setAutoGoalEnabled(false);
                    indexer.setGoal(IndexerState.STOP);
                    intake.setGoal(IntakeState.DOWN);
                })
            );

            m_driverController.rightBumper()
            .onTrue(
                intake.runOnce(()-> intake.setGoal(IntakeState.STOW))
            ).onFalse(
                intake.runOnce(()-> intake.setGoal(IntakeState.DOWN))
            );
        
        
        //Climber Extend
        m_driverController.start()
            .onTrue(climber.runOnce(() -> climber.setGoal(ClimberState.EXTEND)))
            .onFalse(climber.runOnce(() -> climber.setGoal(ClimberState.STOP)));
        
        //Climber Retract
        m_driverController.back()
            .onTrue(climber.runOnce(() -> climber.setGoal(ClimberState.RETRACT)))
            .onFalse(climber.runOnce(() -> climber.setGoal(ClimberState.STOP)));
        
        //Climber Manual UP
        m_driverController.povUp()
            .onTrue(climber.runOnce(() -> climber.setGoal(ClimberState.MANUAL_UP)))
            .onFalse(climber.runOnce(() -> climber.setGoal(ClimberState.STOP)));
            
        //Climber Manual DOWN
        m_driverController.povDown()
            .onTrue(climber.runOnce(() -> climber.setGoal(ClimberState.MANUAL_DOWN)))
            .onFalse(climber.runOnce(() -> climber.setGoal(ClimberState.STOP)));
        
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

    
    public void updateShiftHelpers() {
        Logger.recordOutput("ShiftHelpers/CurrentShiftIsYours", shiftHelpers.currentShiftIsYours());
        Logger.recordOutput("ShiftHelpers/TimeLeftInCurrentShift", shiftHelpers.timeLeftInShiftSeconds(DriverStation.getMatchTime()));
        Logger.recordOutput("ShiftHelpers/CurrentShift", shiftHelpers.getCurrentShiftState());
    }

    public void updateSingleColorView(){

        if(!DriverStation.isEnabled()){
            LEDs.setPattern(LEDStates.DEFAULT);
        }else{
            if (shooter.isAtSetpoint() && m_driverController.rightTrigger().getAsBoolean()) { // If we're in a good shooting state, show green
                Logger.recordOutput("SingleColorView", ColorConstants.green.toHexString());
                LEDs.setPattern(LEDStates.CAN_SHOOT);
            }else if (shiftHelpers.timeLeftInShiftSeconds(DriverStation.getMatchTime()) <= 10) { // If we're in the last 10 seconds of the shift
                Logger.recordOutput("SingleColorView", ColorConstants.white.toHexString());
                LEDs.setPattern(LEDStates.TEN_SECONDS);
            } else { // Otherwise, show blue
                Logger.recordOutput("SingleColorView", ColorConstants.blue.toHexString());
                LEDs.setPattern(LEDStates.DEFAULT);
            }
        }
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

}
