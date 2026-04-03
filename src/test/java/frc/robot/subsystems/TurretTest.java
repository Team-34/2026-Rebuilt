// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Inches;
// import static org.junit.jupiter.api.Assertions.assertEquals;
// import java.util.stream.Stream;

// import edu.wpi.first.units.measure.Angle;
// import org.junit.jupiter.params.ParameterizedTest;
// import org.junit.jupiter.params.provider.Arguments;
// import org.junit.jupiter.params.provider.MethodSource;

// import edu.wpi.first.units.measure.Distance;

// public class TurretTest {
//   Game game = new DriverStationGame();
//   Vision vision = new Vision(game);
//   Turret turret = new Turret(game, vision);

//   static Stream<Arguments> turretPosToSetpoint_testCases() {
//     return Stream.of(
//       Arguments.of(Degrees.of(90), Degrees.zero()),
//       Arguments.of(Degrees.of(120), Degrees.zero()),
//       Arguments.of(Degrees.of(190), Degrees.of(-170)),
//       Arguments.of(Degrees.of(-90), Degrees.zero()),
//       Arguments.of(Degrees.of(-120), Degrees.zero()),
//       Arguments.of(Degrees.of(-190), Degrees.of(170))
//     );
//   }

//   @ParameterizedTest
//   @MethodSource("turretPosToSetpoint_testCases")
//   public void turretPosToSetpoint_testCases(
//     final double expected
//   ) {
//     assertEquals(expected, turret.wrapAround().magnitude(), 0.01);
//   }
// }
