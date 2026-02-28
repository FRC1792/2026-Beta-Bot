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
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;

public class AutoFactory extends SubsystemBase{

    private CommandSwerveDrivetrain m_swerveSubsystem;

    private Intake m_intake;
    private Indexer m_indexer;
    private Shooter m_shooter;
    private Turret m_turret;

    private PIDController translationController = new PIDController(5.3, 0.0, 0.0);
    private PIDController rotationController = new PIDController(2.5, 0.0, 0.0);
    private PIDController crossTrackController = new PIDController(1, 0.0, 0.0);
    

    private ApplyRobotSpeeds m_driveRequest = new ApplyRobotSpeeds()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // .withDesaturateWheelSpeeds(true);

    private FollowPath.Builder pathBuilder;

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public AutoFactory(CommandSwerveDrivetrain swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter, Turret turret){
        m_swerveSubsystem = swerveSubsystem;
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
        m_turret = turret;


        pathBuilder = new FollowPath.Builder(
                m_swerveSubsystem,
                () -> m_swerveSubsystem.getState().Pose,
                () -> m_swerveSubsystem.getState().Speeds,
                (speeds) -> m_swerveSubsystem.setControl(m_driveRequest.withSpeeds(speeds)),
                translationController,
                rotationController,
                crossTrackController
            ).withDefaultShouldFlip();


        FollowPath.registerEventTrigger("Intake", Commands.print("Intake Running"));
    }


    public Command getStraightAuto(){
        Path firstPath = new Path("StraightPath");
        Rotation2d initialDirection = firstPath.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));
        
        return Commands.sequence(
            pathBuilder.build(firstPath)
        );
    }

    public Command getTranslationTuningAuto(){
        Path TranslationTuningPath = new Path("TranslationTuning");
        Rotation2d initialDirection = TranslationTuningPath.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));
        
        return Commands.sequence(
            pathBuilder.build(TranslationTuningPath)
        );
    }

    public Command getRotationTuningAuto(){
        Path RotationTuningPath = new Path("RotationTuning");
        Rotation2d initialDirection = RotationTuningPath.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));
        
        return Commands.sequence(
            pathBuilder.build(RotationTuningPath)
        );
    }

    public Command getNeutralAuto(){
        Path RightIntakePath = new Path("RightIntakeNeutral");
        Path RightReturnToShootPath = new Path("RightReturnToShoot");
        Rotation2d initialDirection = RightIntakePath.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));
        
        return Commands.sequence(
            pathBuilder.build(RightIntakePath),
            pathBuilder.build(RightReturnToShootPath)

        );
    }

    public Command getMiddleDepotAuto(){
        Path MiddleDepotPath = new Path("middle_depot_pickup");
        Path MiddleDepotPath2 = new Path("middle_depot_pickup_2");
        Rotation2d initialDirection = MiddleDepotPath.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));

        return Commands.sequence(
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(true)),
            new WaitCommand(2),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.SPINDEX)),
            new WaitCommand(10),

            pathBuilder.build(MiddleDepotPath),

            Commands.runOnce(() -> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(MiddleDepotPath2),
            new WaitCommand(10),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP)),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.STOP)),
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(false))
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putData("Auto Translation Controller", translationController);
        SmartDashboard.putData("Auto Rotation Controller", rotationController);
        SmartDashboard.putData("Auto Cross Track Controller", crossTrackController);

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
