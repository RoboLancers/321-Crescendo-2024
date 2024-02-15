/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
}
