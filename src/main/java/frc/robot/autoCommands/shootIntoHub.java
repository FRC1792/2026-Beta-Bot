// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootIntoHub extends ParallelCommandGroup {
  /** Creates a new shootIntoHub1. */
  public shootIntoHub(Shooter shooter, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(() -> shooter.setAutoGoalEnabled(true)),
      Commands.sequence(
        new WaitCommand(2),
        Commands.runOnce(() -> indexer.setGoal(IndexerState.SPINDEX))),
      Commands.sequence(
        new WaitCommand(10),
        Commands.runOnce(() -> indexer.setGoal(IndexerState.STOP))),
      Commands.sequence(
        new WaitCommand(15),
        Commands.runOnce(() -> shooter.setAutoGoalEnabled(false)))
    );
  }
}
