/* (C) Robolancers 2024 */
package org.robolancers321.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopDrive extends FeedForwardDrive {
  private static final double kTeleopMaxTranslationalSpeed = 1.5;
  private static final double kTeleopMaxRotationalSpeed = 1.0;
  
  public TeleopDrive(XboxController controller, boolean fieldCentric) {
    super(
        () -> SmartDashboard.getNumber("max drive speed", 1.5) * MathUtil.applyDeadband(controller.getLeftY(), 0.2),
        () -> -SmartDashboard.getNumber("max drive speed", 1.5) * MathUtil.applyDeadband(controller.getLeftX(), 0.2),
        () -> kTeleopMaxRotationalSpeed * MathUtil.applyDeadband(controller.getRightX(), 0.2),
        () -> fieldCentric);
  }

  public TeleopDrive(XboxController controller) {
    this(controller, true);
  }

  @Override
  public void initialize(){
    SmartDashboard.putNumber("max drive speed", SmartDashboard.getNumber("max drive speed", 1.5));
  }
}
