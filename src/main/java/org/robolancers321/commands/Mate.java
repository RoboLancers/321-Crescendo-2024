/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.DoubleSupplier;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.launcher.Launcher;

public class Mate extends SequentialCommandGroup {
  private final Intake intake;
  private final Launcher launcher;

  public Mate(DoubleSupplier distanceSupplier) {
    this.intake = Intake.getInstance();
    this.launcher = Launcher.getInstance();
    
    addCommands(
        new ParallelCommandGroup(this.intake.retractor.moveToMating(), this.launcher.pivot.moveToMating()),
        new ParallelRaceGroup(this.intake.sucker.out(), this.launcher.acceptHandoff())
    );

    addRequirements(this.intake, this.launcher);
  }
}