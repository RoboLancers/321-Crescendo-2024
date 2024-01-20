/* (C) Robolancers 2024 */
package org.robolancers321.subsystems;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeRetractor extends SubsystemBase {

    public CANSparkMax retractorMotor;
    private DigitalInput retractorLimitSwitch;
    private DigitalInput retractorBeamBreak;
    private Trigger retractorTrigger;
    SparkPIDController retractorPIDController;
    AbsoluteEncoder retractorEncoder1;
    RelativeEncoder retractorEncoder2;

    public double referencePosition = 0;
    public double retractPosition = 90;

    public static final double kP = 0.000;
    public static final double kI = 0.000;
    public static final double kD = 0.000;
    public static final double kFF = 0.0001;
    public static final double kErrorThreshold = 2.0;

    public static final int kPort = 0;
    public static final int kRetractorLimitSwitch = 0;
    private static final Type kDutyCycle = null;


  public IntakeRetractor() {
    this.retractorMotor = new CANSparkMax(kPort, MotorType.kBrushless);
    this.retractorEncoder1 = retractorMotor.getAbsoluteEncoder(kDutyCycle);
    this.retractorEncoder2 = retractorMotor.getEncoder();
    retractorPIDController = retractorMotor.getPIDController();
    retractorLimitSwitch = new DigitalInput(0);
    retractorBeamBreak = new DigitalInput(0);

    this.retractorEncoder1.setPositionConversionFactor(1.0);

    retractorPIDController.setD(kD);
    retractorPIDController.setP(kP);
    retractorPIDController.setI(kI);

    this.retractorMotor.setSmartCurrentLimit(20);

    
  //   this.retractorLimitSwitch = new DigitalInput(kRetractorLimitSwitch);
  //       this.retractorTrigger = new Trigger(this::limitSwitchTriggered);
  //       this.retractorTrigger.whileTrue(new InstantCommand(() -> {
  //           this.resetEncoder();
  //       }, this));
  }

  @Override
  public void periodic() {

    retractorPIDController.setD(kD);
    retractorPIDController.setP(kP);
    retractorPIDController.setI(kI);
    retractorPIDController.setFF(kFF);

    retractorMotor.getOutputCurrent();
    retractorMotor.set(100);

    // pivotcontrolleroutput = controller.calculate(stuff)
    // pivotMotor.set(Output)
 
  }

//   public boolean limitSwitchTriggered() {
//     return !this.retractorLimitSwitch.get();
// }


public void setRetractorPosition() {
  retractorEncoder2.setPosition(retractPosition);
}

public double getRetractPosition() {
  return retractorEncoder2.getPosition();
}

public boolean isRetractorAtCorrectPos(double position) {
  return Math.abs(this.getRetractPosition() - position) < kErrorThreshold;
}
    

public void resetEncoder() {
  retractorEncoder2.setPosition(0);
}

  public boolean isBeamBroken() {
    return !this.retractorBeamBreak.get();
  }

}
