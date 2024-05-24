package frc.team449.robot2024.constants.drives

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import frc.team449.robot2024.constants.MotorConstants
import frc.team449.robot2024.constants.RobotConstants

object SwerveConstants {
  const val EFFICIENCY = 0.95

  /** Drive motor ports */
  const val DRIVE_MOTOR_FL = 4
  const val DRIVE_MOTOR_FR = 20
  const val DRIVE_MOTOR_BL = 1
  const val DRIVE_MOTOR_BR = 17
  const val TURN_MOTOR_FL = 3
  const val TURN_MOTOR_FR = 21
  const val TURN_MOTOR_BL = 2
  const val TURN_MOTOR_BR = 61

  /** Turning encoder channels */
  const val TURN_ENC_CHAN_FL = 6
  const val TURN_ENC_CHAN_FR = 9
  const val TURN_ENC_CHAN_BL = 7
  const val TURN_ENC_CHAN_BR = 8

  /** Offsets for the absolute encoders in rotations. */
  const val TURN_ENC_OFFSET_FL = 0.1348 + 0.480
  const val TURN_ENC_OFFSET_FR = 0.2927 + 0.5 + 0.256
  const val TURN_ENC_OFFSET_BL = 0.4755 - 0.110
  const val TURN_ENC_OFFSET_BR = 0.391 + 0.5 + 0.327

  /** PID gains for turning each module */
  const val TURN_KP = 0.05 //0.95
  const val TURN_KI = 0.0
  const val TURN_KD = 0.0

  // TODO: Figure out this value
  const val STEER_KS = 0.05

  /** PID gains for driving each module*/
  const val DRIVE_KP = 0.35
  const val DRIVE_KI = 0.0
  const val DRIVE_KD = 0.0

  /** Drive configuration, L2 for this bot */
  const val DRIVE_GEARING = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)
  const val DRIVE_UPR = 0.31818905832
  const val TURN_UPR = 2 * Math.PI
  val MAX_ATTAINABLE_MK4I_SPEED = Units.feetToMeters(15.0) // (12 - DRIVE_KS) / DRIVE_KV
  const val DRIVE_CURRENT_LIM = 55
  const val STEERING_CURRENT_LIM = 40
  const val JOYSTICK_FILTER_ORDER = 2
  const val ROT_FILTER_ORDER = 1.25
  const val SKEW_CONSTANT = 11.5

  /** Feed forward values for driving each module */
  const val DRIVE_KS = 0.15
  val DRIVE_KV = 12.0 / MAX_ATTAINABLE_MK4I_SPEED
  val DRIVE_KA = 12.0 / (4 * DCMotor(
    MotorConstants.NOMINAL_VOLTAGE,
    MotorConstants.STALL_TORQUE * EFFICIENCY,
    MotorConstants.STALL_CURRENT,
    MotorConstants.FREE_CURRENT,
    MotorConstants.FREE_SPEED,
    1
  ).getTorque(DRIVE_CURRENT_LIM.toDouble()) /
    (Units.inchesToMeters(2.0) * RobotConstants.ROBOT_WEIGHT * DRIVE_GEARING))

  /** Wheelbase = wheel-to-wheel distance from front to back of the robot */
  /** Trackwidth = wheel-to-wheel distance from side to side of the robot */
  val WHEELBASE = Units.inchesToMeters(16.75) // ex. FL to BL
  val TRACKWIDTH = Units.inchesToMeters(21.75) // ex. BL to BR
  val X_SHIFT = Units.inchesToMeters(27.5 - 22.0) / 2.0
}
