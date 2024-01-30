/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.DoubleSupplier;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.launcher.Launcher;

public class MateAndShoot extends SequentialCommandGroup {

  private final Launcher launcher;
  private final Intake intake;

  public MateAndShoot(DoubleSupplier distanceSupplier) {
    launcher = Launcher.getInstance();
    intake = Intake.getInstance();

    addRequirements(launcher, intake);
    // Both mating commands have an end condition
    addCommands(
        new ParallelCommandGroup(launcher.pivot.goToMating(), intake.retractor.moveToMating()),
        new ParallelCommandGroup(intake.sucker.out(), launcher.yeetSpeaker(distanceSupplier)),
        intake.goAway(launcher::getBeamBreakState));
  }
}
