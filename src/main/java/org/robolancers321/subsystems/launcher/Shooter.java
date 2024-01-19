/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static Shooter INSTANCE;

  public static Shooter getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Shooter();
    }
    return INSTANCE;
  }

  private Shooter() {
    // TODO: Set the default command, if any, for this subsystem by calling
    // setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the
    // subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
  }
}
