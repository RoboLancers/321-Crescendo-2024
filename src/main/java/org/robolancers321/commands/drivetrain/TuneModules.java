/* (C) Robolancers 2024 */
package org.robolancers321.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.drivetrain.SwerveModule;

public class TuneModules extends Command {
  private Drivetrain drivetrain;

  public TuneModules() {
    this.drivetrain = Drivetrain.getInstance();

    SwerveModule.initTuning();

    SmartDashboard.putNumber("target module angle (deg)", 0.0);
    SmartDashboard.putNumber("target module vel (m/s)", 0.0);

    this.addRequirements(this.drivetrain);
  }

  @Override
  public void execute() {
    double theta = SmartDashboard.getNumber("target module angle (deg)", 0.0);
    double vel = SmartDashboard.getNumber("target module vel (m/s)", 0.0);

    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      states[i] = new SwerveModuleState(vel, Rotation2d.fromDegrees(theta));
    }

    this.drivetrain.tuneDrive(states);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.drive(0.0, 0.0, 0.0, false);
  }
}
