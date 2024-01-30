/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase {
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

  // TODO: beam break port and any other resources/information that interops with retractor and
  // sucker

  /*
   * Implementation
   */

  public Retractor retractor;
  public Sucker sucker;

  private Intake() {
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
  }

  public Command goAway(BooleanSupplier beamBreakState) {
    return new SequentialCommandGroup(
        new WaitUntilCommand(beamBreakState), retractor.moveToRetracted());
  }
}
