// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  enum Speed {
    STOP(0.0), HALF(0.5), FULL(1.0);

    public final double value;

    Speed(double value) {
      this.value = value;
    }
  }

  private Speed speed = Speed.STOP;
  private final TalonFX master = new TalonFX(20);//left
  private final TalonFX padawan = new TalonFX(21);//right
  private final TalonSRX aimer = new TalonSRX(22);
  /**
 * 
 */
public Shooter() {
    TalonFXConfiguration master_config = new TalonFXConfiguration();
    master_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    master.getConfigurator().apply(master_config);
    padawan.setControl(new Follower(master.getDeviceID(),MotorAlignmentValue.Opposed));
    
  
    aimer.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    aimer.config_kP(0, 0.1, 0);
    aimer.config_kI(0, 0, 0);
    aimer.config_kD(0, 0, 0);
    aimer.config_kF(0, 0, 0);
    aimer.configMotionCruiseVelocity(1000, 10);
    aimer.configMotionAcceleration(500, 10);


  }
  //right motor should go clockwise
  /**
   * Cyles thought 3 different speeds. Stop -> Half -> Full -> Stop -> ...
   *
   * @returns The command that switches the speed to the next in the cycle.
   */

  public Command cycleSpeedCommand() {   // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
      this.speed = switch (this.speed) {
        case STOP -> Speed.HALF;
        case HALF -> Speed.FULL;
        case FULL -> Speed.STOP;
        default -> Speed.STOP;
      };
      runFiringMotor(this.speed.value);
    });
  }
  private void runFiringMotor(double motor_speed) {
    this.master.set(motor_speed);
  }

  public Command setPosition(){
     return runOnce(() -> {
        moveAimingMotorRot(2);
     });
  }
  public Command setPrecent(double run_speed){
     return runOnce(() -> {
      moveAimingMotor(run_speed);   
     });
  }
  private void moveAimingMotorRot(double motor_rot) {
    
    double position = motor_rot * 4096; 
    this.aimer.set(TalonSRXControlMode.Position, position);
    // this moves the motor to 2048 encoder units. still figuring out how fast it reaches that position
    // and if we can control this speed...
  }
  private void moveAimingMotor(double motor_speed) {
    
    this.aimer.set(TalonSRXControlMode.PercentOutput, motor_speed);
    // this moves the motor to 2048 encoder units. still figuring out how fast it reaches that position
    // and if we can control this speed...
  }


  
  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed: ", master.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
