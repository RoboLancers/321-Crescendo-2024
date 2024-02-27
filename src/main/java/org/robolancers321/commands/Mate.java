/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.launcher.Launcher;

public class Mate extends SequentialCommandGroup {
  private Intake intake;
  private Launcher launcher;

  public Mate() {
    this.intake = Intake.getInstance();
    this.launcher = Launcher.getInstance();

    // TODO: use actual fb instead of time out on accept command

    addCommands(
        new ParallelCommandGroup(
            this.intake.retractor.moveToMating(), this.launcher.pivot.moveToMating()),
        new ParallelRaceGroup(
            this.intake.sucker.out(), this.launcher.acceptHandoff(), new WaitCommand(0.6)));
  }
}
