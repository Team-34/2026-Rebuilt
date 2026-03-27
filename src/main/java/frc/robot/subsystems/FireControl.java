package frc.robot.subsystems;

import frc.firecontrol.FuelPhysicsSim;
import frc.firecontrol.ProjectileSimulator;
import frc.firecontrol.ShotCalculator;
import frc.firecontrol.ShotLUT;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision;

public class FireControl extends SubsystemBase {

  private Vision vision;

  public FireControl(Vision vision) {
    
    this.vision = vision;

    var latency = vision.getTotalCameraLatency();

    ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX = 0.0; // how far forward the launcher is from robot center (m)
    config.launcherOffsetY = 0.0; // how far left, 0 if centered
    config.phaseDelayMs = latency; // your vision pipeline latency
    config.mechLatencyMs = 20.0; // how long the mechanism takes to respond
    config.maxTiltDeg = 5.0; // suppress firing when chassis tilts past this (bumps/ramps)
    config.headingSpeedScalar = 1.0; // heading tolerance tightens with robot speed (0 to disable)
    config.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub

    ShotCalculator shotCalc = new ShotCalculator(config);
  }
}
