package frc.team449.robot2024.constants.subsystem

import edu.wpi.first.math.InterpolatingMatrixTreeMap
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Nat
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import kotlin.math.PI

object ShooterConstants {
  const val EFFICIENCY = 0.8
  const val RIGHT_MOTOR_ID = 45
  const val RIGHT_MOTOR_INVERTED = true
  const val LEFT_MOTOR_ID = 46
  const val LEFT_MOTOR_INVERTED = false
  const val CURRENT_LIMIT = 40
  const val BRAKE_MODE = false

  val SUBWOOFER_LEFT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(4414.0)
  val SUBWOOFER_RIGHT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(2056.0)
  val AUTO_LEFT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(4750.0)
  val AUTO_RIGHT_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(3650.0)
  const val AUTO_SHOOT_TOL = 30.0

  val BRAKE_RATE_LIMIT = Units.rotationsPerMinuteToRadiansPerSecond(3750.0)

  val SHOOTING_MAP = InterpolatingMatrixTreeMap<Double, N3, N1>()

  const val LEFT_KS = (0.10843 + 0.11944) / 2
  const val RIGHT_KS = (0.11509 + 0.11287) / 2
  const val LEFT_KV = (0.0204 + 0.020384) / 2
  const val RIGHT_KV = (0.021071 + 0.021073) / 2
  const val LEFT_KA = (0.0056603 + 0.0053949) / 2
  const val RIGHT_KA = (0.0053768 + 0.0055329) / 2

  const val AMP_SCORE_VOLTAGE = 4.144
  const val DURING_INTAKE_VOLTAGE = -1.0

  /** In meters from the ground */
  const val SHOOTER_HEIGHT = 0.25

  const val IN_TOLERANCE = 20.0

  /** These constants are PER SIDE of the shooter */
  const val NUM_MOTORS = 1

  const val MODEL_VEL_STDDEV = 3.0
  const val ENCODER_VEL_STDDEV = 0.075
  const val LQR_VEL_TOL = 10.0
  const val LQR_MAX_VOLTS = 12.0
  const val MAX_VOLTAGE = 12.0

  const val MIN_RAMP_VEL = 50.0

  /** Encoder stuff */
  const val INTERNAL_ENC_DEPTH = 2
  const val INTERNAL_MEASUREMENT_PD = 16
  const val LEFT_CHANNEL_A = 1
  const val LEFT_CHANNEL_B = 2

  // check
  const val RIGHT_CHANNEL_A = 3
  const val RIGHT_CHANNEL_B = 4
  const val CPR = 2048
  const val UPR = 2 * PI
  const val GEARING = 1.0 / 1.0
  const val LEFT_ENCODER_INVERTED = false
  const val RIGHT_ENCODER_INVERTED = false
  const val SAMPLES_TO_AVERAGE = 127

  init {
    /**
     * Fill with values of optimized left/right and pivot angles
     *  for a given distance to the Speaker
     *
     * It may be better to mathematically calculate pivot angle,
     *  this is something to test
     *
     * Data is entered as following:
     *  Right shooter speed, left shooter speed, pivot angle
     */
    SHOOTING_MAP.put(
      0.0,
      MatBuilder.fill(
        Nat.N3(),
        Nat.N1(),
        0.0,
        0.0,
        0.0
      )
    )
  }
}
