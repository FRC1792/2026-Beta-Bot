package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;

public class AutoFactory extends SubsystemBase{

    private CommandSwerveDrivetrain m_swerveSubsystem;

    private Climber m_climber;
    private Intake m_intake;
    private Indexer m_indexer;
    private Shooter m_shooter;

    private PIDController translationController = new PIDController(2.53, 0.0, 0.0);
    private PIDController rotationController = new PIDController(4.2, 0.0, 0.25);
    private PIDController crossTrackController = new PIDController(1.03, 0.0, 0.0);
    

    private ApplyRobotSpeeds m_driveRequest = new ApplyRobotSpeeds()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // .withDesaturateWheelSpeeds(true);

    private FollowPath.Builder pathBuilder;

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public AutoFactory(CommandSwerveDrivetrain swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter, Climber climber){
        m_swerveSubsystem = swerveSubsystem;
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
        m_climber = climber;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        pathBuilder = new FollowPath.Builder(
                m_swerveSubsystem,
                () -> m_swerveSubsystem.getState().Pose,
                () -> m_swerveSubsystem.getState().Speeds,
                (speeds) -> m_swerveSubsystem.setControl(m_driveRequest.withSpeeds(speeds)),
                translationController, //pid loop for translation
                rotationController, //pid loop for rotation
                crossTrackController //pid loop for cross track
            ).withDefaultShouldFlip();


        FollowPath.registerEventTrigger("Intake", intake.runOnce(()-> intake.setGoal(IntakeState.INTAKE)));
        FollowPath.registerEventTrigger("Stow", intake.runOnce(() -> intake.setGoal(IntakeState.STOW)));
        FollowPath.registerEventTrigger("AutoGoalEnable", shooter.runOnce(()-> shooter.setAutoGoalEnabled(true)));
        FollowPath.registerEventTrigger("Spindex", indexer.runOnce(()-> indexer.setGoal(IndexerState.SPINDEX)));
        FollowPath.registerEventTrigger("ShooterNoIntake", shooter.runOnce(() -> shooter.setAutoGoalEnabled(true))
                .andThen(Commands.waitUntil(shooter::isAtSetpoint))
                .andThen(indexer.runOnce(() -> indexer.setGoal(IndexerState.OUTTAKE)))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(indexer.runOnce(() -> indexer.setGoal(IndexerState.SPINDEX))));
        FollowPath.registerEventTrigger("Shooter", shooter.runOnce(() -> shooter.setAutoGoalEnabled(true))
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
                })
            );
    }

    public Command getLeftDepotAuto(){
        Path LeftDepot = new Path("LeftToDepot");
        Rotation2d initialDirection = LeftDepot.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));

        return Commands.sequence(
            pathBuilder.build(LeftDepot),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP))
        );
    }

    public Command getLeftIntoNeutralPickupAuto(){
        Path LeftBumpNeutral = new Path("LeftIntoNeutral");
        Rotation2d initialDirection = LeftBumpNeutral.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));

        return Commands.sequence(
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(true)),
            new WaitCommand(1),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.SPINDEX)),
            new WaitCommand(7),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.STOP)),
            new WaitCommand(2),
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(false)),
            pathBuilder.build(LeftBumpNeutral),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP))
        );
    }

    public Command getLeftNeutralAuto() {
          Path RightIntakePath = new Path("RightIntoNeutral");
          RightIntakePath.mirror();
        Path RightReturnToShootPath = new Path("NeutralIntoRight");
        RightReturnToShootPath.mirror();
        return Commands.sequence(
            Commands.runOnce(()-> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(RightIntakePath),
            pathBuilder.build(RightReturnToShootPath),
            new WaitCommand(3),
            Commands.runOnce(()-> m_intake.setGoal(IntakeState.STOW)),
            new WaitCommand(2),
            Commands.runOnce(()-> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(RightIntakePath)
        );
    }

    public Command getRightNeutralSwipeOutpostAuto(){
        Path RightIntakePath = new Path("RightIntoNeutral");
        Path RightReturnToShootPath = new Path("RightReturnToOutpost");
        return Commands.sequence(
            Commands.runOnce(()-> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(RightIntakePath),
            pathBuilder.build(RightReturnToShootPath)
        );
    }

    public Command getRightIntoNeutralPickupAuto() {
        Path RightBumpNeutral = new Path("RightIntoNeutral");
        Rotation2d initialDirection = RightBumpNeutral.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));

        return Commands.sequence(
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(true)),
            new WaitCommand(1),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.SPINDEX)),
            new WaitCommand(7),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.STOP)),
            new WaitCommand(2),
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(false)),
            pathBuilder.build(RightBumpNeutral),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP))
        );
    }

    public Command getRightNeutralAuto() {
          Path RightIntakePath = new Path("RightIntoNeutral");
        Path RightReturnToShootPath = new Path("NeutralIntoRight");
        return Commands.sequence(
            Commands.runOnce(()-> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(RightIntakePath),
            pathBuilder.build(RightReturnToShootPath),
            new WaitCommand(3),
            Commands.runOnce(()-> m_intake.setGoal(IntakeState.STOW)),
            new WaitCommand(4),
            Commands.runOnce(()-> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(RightIntakePath)
        );
    }

    public Command getLeftNeutralSwipeDepotAuto(){
        Path RightIntoNeutral = new Path("RightIntoNeutral");
        RightIntoNeutral.mirror();
        Path NeutralToLeftToDepot = new Path("NeutralToLeftToDepot");
        return Commands.sequence(
            Commands.runOnce(()-> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(RightIntoNeutral),
            pathBuilder.build(NeutralToLeftToDepot)
        );
    }

    @Override
    public void periodic(){
        // SmartDashboard.putData("Tuning/Auto Translation Controller", translationController);//TODO: Remove for Comp
        // SmartDashboard.putData("Tuning/Auto Rotation Controller", rotationController);//TODO: Remove for Comp
        // SmartDashboard.putData("Tuning/Auto Cross Track Controller", crossTrackController);//TODO: Remove for Comp

        FollowPath.setPoseLoggingConsumer(pair -> {
            Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });

        FollowPath.setTranslationListLoggingConsumer(pair -> {
            Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });

        FollowPath.setDoubleLoggingConsumer(pair -> {
            Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });
    }
}
