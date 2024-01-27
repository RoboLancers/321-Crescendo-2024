/* (C) Robolancers 2024 */
package org.robolancers321.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

public class TeleopDrive extends FeedForwardDrive {
  private static final double kTeleopMaxTranslationalSpeed = 2.5;
  private static final double kTeleopMaxRotationalSpeed = 1.0;

  public TeleopDrive(XboxController controller, boolean fieldCentric) {
    super(
        () -> kTeleopMaxTranslationalSpeed * MathUtil.applyDeadband(controller.getLeftY(), 0.2),
        () -> -kTeleopMaxTranslationalSpeed * MathUtil.applyDeadband(controller.getLeftX(), 0.2),
        () -> kTeleopMaxRotationalSpeed * MathUtil.applyDeadband(controller.getRightX(), 0.2),
        () -> fieldCentric);
  }

  public TeleopDrive(XboxController controller) {
    this(controller, true);
  }
}
