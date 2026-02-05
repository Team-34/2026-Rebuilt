// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.ctre.phoenix6.controls.DutyCycleOut;


public class Climber extends SubsystemBase {
  
  private final TalonFX leftMotor = new TalonFX(40);
  private final TalonFX rightMotor = new TalonFX(41);
  private final DutyCycleOut motorControl = new DutyCycleOut(0);

  private DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  public Climber() {
    this.rightMotor.setControl(new Follower(this.leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    piston.set(DoubleSolenoid.Value.kReverse);
  }
  
  public Command extendCommand() {
    return runOnce(
        () -> {
          leftMotor.setControl(motorControl.withOutput(0.25));
        });
  }

    public Command retractCommand() {
      return runOnce(
          () -> {
            leftMotor.setControl(motorControl.withOutput(-0.25));
          });
    }

    public Command toggleCommand() {
      return runOnce(() -> piston.toggle()); // test pt2
    }

    public boolean isExtended() {
      return piston.get() == DoubleSolenoid.Value.kForward;
    }
}
