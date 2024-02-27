/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Intake {
  /*
   * Singleton
   */

  private static Intake instance = null;

  public static Intake getInstance() {
    if (instance == null) instance = new Intake();

    return instance;
  }

  /*
   * Constants
   */

  // TODO: implement beam break

  /*
   * Implementation
   */

  public final Retractor retractor;
  public final Sucker sucker;

  private Intake() {
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
  }

  // TODO: use beam break to auto retract and rumble
  public Command deployIntake() {
    return new ParallelCommandGroup(this.retractor.moveToIntake(), this.sucker.in());
  }

  public Command outtakeNote() {
    return new SequentialCommandGroup(this.retractor.moveToIntake(), this.sucker.out());
  }
}
