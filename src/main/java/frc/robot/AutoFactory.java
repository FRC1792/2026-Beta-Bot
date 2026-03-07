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

    // private Climber m_climber;
    private Intake m_intake;
    private Indexer m_indexer;
    private Shooter m_shooter;

    private PIDController translationController = new PIDController(1, 0.0, 0.0);
    private PIDController rotationController = new PIDController(0.25, 0.0, 0.25);
    private PIDController crossTrackController = new PIDController(0.025, 0.0, 0.0);
    

    private ApplyRobotSpeeds m_driveRequest = new ApplyRobotSpeeds()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // .withDesaturateWheelSpeeds(true);

    private FollowPath.Builder pathBuilder;

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public AutoFactory(CommandSwerveDrivetrain swerveSubsystem, Intake intake, Indexer indexer, Shooter shooter){
        m_swerveSubsystem = swerveSubsystem;
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;


        pathBuilder = new FollowPath.Builder(
                m_swerveSubsystem,
                () -> m_swerveSubsystem.getState().Pose,
                () -> m_swerveSubsystem.getState().Speeds,
                (speeds) -> m_swerveSubsystem.setControl(m_driveRequest.withSpeeds(speeds)),
                translationController, //pid loop for translation
                rotationController, //pid loop for rotation
                crossTrackController //pid loop for cross track
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

    public Command getTrenchToTrenchAuto(){
        Path TrenchToTrench = new Path("trench_to_trench");
        Rotation2d initialDirection = TrenchToTrench.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));

        return Commands.sequence(
            pathBuilder.build(TrenchToTrench)
        );
            
    }

    public Command getMiddleDepotP2Auto(){
        Path MiddleDepotPath = new Path("middle_to_depot");
        Path MiddleDepotPath2 = new Path("middle_to_depot_2");
        Rotation2d initialDirection = MiddleDepotPath.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));

        return Commands.sequence(
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(true)),
            new WaitCommand(1),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.SPINDEX)),
            new WaitCommand(7),
            pathBuilder.build(MiddleDepotPath),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(MiddleDepotPath2),
            new WaitCommand(10),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP)),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.STOP)),
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(false))
        );
    }

    public Command getLeftDepot2pAuto(){
        Path LeftDepot = new Path("left_to_depot");
        Rotation2d initialDirection = LeftDepot.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));

        return Commands.sequence(
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(true)),
            new WaitCommand(1.5),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.SPINDEX)),
            new WaitCommand(2.5),
            //Commands.runOnce(() -> m_indexer.setGoal(IndexerState.STOP)),
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(false)),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.INTAKE)),
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(true)),
            pathBuilder.build(LeftDepot),
            //Commands.runOnce(() -> m_indexer.setGoal(IndexerState.SPINDEX)),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP))
        );
    }

    public Command getNeutralZoneLeftPickup1pAuto(){
        Path LeftBumpNeutral = new Path("left_bump_into_neutral");
        Path NeutralFromLeft = new Path("through_neutral_from_left");
        Rotation2d initialDirection = LeftBumpNeutral.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));

        return Commands.sequence(  //to be changed very much tomorrow
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(true)),
            new WaitCommand(1),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.SPINDEX)),
            new WaitCommand(7),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.STOP)),
            new WaitCommand(2),
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(false)),
            pathBuilder.build(LeftBumpNeutral),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(NeutralFromLeft),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP))
        );
    }
    
    public Command getRightNeutral1pAuto() {
        Path RightBumpNeutral = new Path("right_bump_into_neutral");
        Path NeutralFromRight = new Path("through_neutral_from_right");
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
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(NeutralFromRight),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP))
        );
    }

    public Command getLeftDepotNeutral3pAuto() {
        Path LeftToDepot = new Path("left_to_depot");
        Path DepotNeutralLeftBump = new Path("depot_into_neutral_over_left_bump");
        Path NeutralFromLeft = new Path("through_neutral_from_left");
        Path NeutralToLeftBump = new Path("neutral_into_left_bump");
        Rotation2d initialDirection = LeftToDepot.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));

        return Commands.sequence(
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(true)),
            new WaitCommand(1),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.SPINDEX)),
            new WaitCommand(3),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(LeftToDepot),
            new WaitCommand(5),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP)),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.STOP)),
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(false)),
            pathBuilder.build(DepotNeutralLeftBump),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(NeutralFromLeft),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP)),
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(true)),
            pathBuilder.build(NeutralToLeftBump),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.SPINDEX))
        );
    }

    public Command getRightNeutral2pAuto() {
        Path RightBumpNeutral = new Path("right_bump_into_neutral");
        Path NeutralFromRight = new Path("through_neutral_from_right");
        Path NeutralRightBump = new Path("neutral_into_right_bump");
        Rotation2d initialDirection = RightBumpNeutral.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));
        
        return Commands.sequence(
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(true)),
            new WaitCommand(1),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.SPINDEX)),
            new WaitCommand(3),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(RightBumpNeutral),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.STOP)),
            new WaitCommand(3),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP)),
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(false)),
            pathBuilder.build(NeutralFromRight),
            Commands.runOnce(() -> m_shooter.setAutoGoalEnabled(true)),
            new WaitCommand(1),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.SPINDEX)),
            new WaitCommand(3),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.INTAKE)),
            pathBuilder.build(NeutralRightBump),
            Commands.runOnce(() -> m_indexer.setGoal(IndexerState.STOP)),
            new WaitCommand(3),
            Commands.runOnce(() -> m_intake.setGoal(IntakeState.STOP)),
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
