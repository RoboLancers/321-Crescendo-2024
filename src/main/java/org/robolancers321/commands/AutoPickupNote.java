/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class AutoPickupNote extends SequentialCommandGroup {
  private Drivetrain drivetrain;
  private Retractor retractor;
  private Sucker sucker;

  public AutoPickupNote() {
    this.drivetrain = Drivetrain.getInstance();
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();

    this.addCommands(
        new ParallelRaceGroup(
            this.sucker.in(),
            new SequentialCommandGroup(
                this.retractor.moveToIntake(),
                new WaitCommand(0.5),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        // this.drivetrain.turnToNote(),
                        // this.drivetrain
                        //     .driveCommand(0.0, 2.0, 0.0, false)
                        //     .until(() -> !this.drivetrain.seesNote())
                        //     .withTimeout(1.5),
                        this.drivetrain.driveIntoNote().raceWith(new WaitUntilCommand(this.sucker::noteDetected)),
                        this.drivetrain
                            .driveCommand(0.0, 1.5, 0.0, false)
                            .until(this.sucker::noteDetected)
                            .withTimeout(1.0),
                        this.drivetrain.stop(),
                        this.retractor.moveToMating()),
                    new InstantCommand(),
                    this.drivetrain::seesNote))),
        this.sucker.offInstantly());
  }
}
