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
  
  public static Climber climber = new Climber();

  private TalonFX leftMotor = new TalonFX(40);
  private TalonFX rightMotor = new TalonFX(41);
  private DutyCycleOut leftMotorControl = new DutyCycleOut(40);
  //private DutyCycleOut rightMotorControl = new DutyCycleOut(41); /* Not being used now but may need it later on */

  private DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  public Climber() {
    this.rightMotor.setControl(new Follower(this.leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }
  
  public Command extendCommand() {
    return runOnce(
        () -> {
          leftMotor.setControl(leftMotorControl.withOutput(0.25));
        });
  }

    public Command retractCommand() {
      return runOnce(
          () -> {
            leftMotor.setControl(leftMotorControl.withOutput(-0.25));
          });
    }

    public Command toggleSolenoid() {
      doubleSolenoid.toggle();
      return null;
    }

    public DoubleSolenoid.Value getSolenoidState() {
      return doubleSolenoid.get();
    }

    public void setSolenoidOff() {
      doubleSolenoid.set(DoubleSolenoid.Value.kOff);
    }
}
