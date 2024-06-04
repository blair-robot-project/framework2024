package frc.team449.control.holonomic

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.robot2024.constants.drives.SwerveConstants
import frc.team449.system.AHRS
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import kotlin.math.hypot

/**
 * A Swerve Drive chassis.
 * @param modules An array of [SwerveModule]s that are on the drivetrain.
 * @param ahrs The gyro that is mounted on the chassis.
 * @param maxLinearSpeed The maximum translation speed of the chassis.
 * @param maxRotSpeed The maximum rotation speed of the chassis.
 * @param field The SmartDashboard [Field2d] widget that shows the robot's pose.
 */
open class SwerveDrive(
  protected val modules: List<SwerveModule>,
  protected val ahrs: AHRS,
  override var maxLinearSpeed: Double,
  override var maxRotSpeed: Double,
  protected val field: Field2d
) : SubsystemBase(), HolonomicDrive {

  /** The kinematics that convert [ChassisSpeeds] into multiple [SwerveModuleState] objects. */
  protected val kinematics = SwerveDriveKinematics(
    *this.modules.map { it.location }.toTypedArray()
  )

  /** The current speed of the robot's drive. */
  var currentSpeeds = ChassisSpeeds()

  /** Current estimated vision pose */

  /** Pose estimator that estimates the robot's position as a [Pose2d]. */
  protected val poseEstimator = SwerveDrivePoseEstimator(
    kinematics,
    ahrs.heading,
    getPositions(),
    RobotConstants.INITIAL_POSE
  )

  init {
    SmartDashboard.putData("Elastic Swerve Drive") { builder: SendableBuilder ->
      builder.setSmartDashboardType("SwerveDrive")
      builder.addDoubleProperty("Front Left Angle", { modules[0].state.angle.radians }, null)
      builder.addDoubleProperty("Front Left Velocity", { modules[0].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Front Right Angle", { modules[1].state.angle.radians }, null)
      builder.addDoubleProperty("Front Right Velocity", { modules[1].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Back Left Angle", { modules[2].state.angle.radians }, null)
      builder.addDoubleProperty("Back Left Velocity", { modules[2].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Back Right Angle", { modules[3].state.angle.radians }, null)
      builder.addDoubleProperty("Back Right Velocity", { modules[3].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Robot Angle", { heading.radians }, null)
    }
  }

  var desiredSpeeds: ChassisSpeeds = ChassisSpeeds()

  protected var maxSpeed: Double = 0.0

  override fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = desiredSpeeds

    // Converts the desired [ChassisSpeeds] into an array of [SwerveModuleState].
    val desiredModuleStates =
      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)

    // Scale down module speed if a module is going faster than the max speed, and prevent early desaturation.
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredModuleStates,
      SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED,

    )

    for (i in this.modules.indices) {
      this.modules[i].state = desiredModuleStates[i]
    }

    for (module in modules)
      module.update()
  }

  fun setVoltage(volts: Double) {
    modules.forEach {
      it.setVoltage(volts)
    }
  }

  /** The (x, y, theta) position of the robot on the field. */
  override var pose: Pose2d
    get() = this.poseEstimator.estimatedPosition
    set(value) {
      this.poseEstimator.resetPosition(
        ahrs.heading,
        getPositions(),
        value
      )
    }

  override fun periodic() {
    // Updates the robot's currentSpeeds.
    currentSpeeds = kinematics.toChassisSpeeds(
      modules[0].state,
      modules[1].state,
      modules[2].state,
      modules[3].state
    )

    val transVel = hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
    if (transVel > maxSpeed) maxSpeed = transVel

    // Update the robot's pose using the gyro heading and the SwerveModulePositions of each module.
    this.poseEstimator.update(
      ahrs.heading,
      getPositions()
    )

    // Sets the robot's pose and individual module rotations on the SmartDashboard [Field2d] widget.
    setRobotPose()
  }

  /** Stops the robot's drive. */
  override fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  /** @return An array of [SwerveModulePosition] for each module, containing distance and angle. */
  protected fun getPositions(): Array<SwerveModulePosition> {
    return Array(modules.size) { i -> modules[i].position }
  }

  /** @return An array of [SwerveModuleState] for each module, containing speed and angle. */
  private fun getStates(): Array<SwerveModuleState> {
    return Array(modules.size) { i -> modules[i].state }
  }

  protected fun setRobotPose() {
    this.field.robotPose = this.pose

    this.field.getObject("FL").pose = this.pose.plus(
      Transform2d(
        Translation2d(SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2),
        this.getPositions()[0].angle
      )
    )

    this.field.getObject("FR").pose = this.pose.plus(
      Transform2d(
        Translation2d(SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2),
        this.getPositions()[1].angle
      )
    )

    this.field.getObject("BL").pose = this.pose.plus(
      Transform2d(
        Translation2d(-SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2),
        this.getPositions()[2].angle
      )
    )

    this.field.getObject("BR").pose = this.pose.plus(
      Transform2d(
        Translation2d(-SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2),
        this.getPositions()[0].angle
      )
    )
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Poses and ChassisSpeeds")
    builder.addDoubleArrayProperty("1.1 Estimated Pose", { doubleArrayOf(pose.x, pose.y, pose.rotation.radians) }, null)
    builder.addDoubleArrayProperty("1.2 Current Chassis Speeds", { doubleArrayOf(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond, currentSpeeds.omegaRadiansPerSecond) }, null)
    builder.addDoubleArrayProperty("1.3 Desired Chassis Speeds", { doubleArrayOf(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond) }, null)
    builder.addDoubleProperty("1.4 Max Recorded Speed", { maxSpeed }, null)

    builder.publishConstString("3.0", "Driving & Steering (Std Order FL, FR, BL, BR)")
    builder.addDoubleArrayProperty(
      "3.1 Current States",
      {
        doubleArrayOf(
          modules[0].state.angle.radians,
          modules[0].state.speedMetersPerSecond,
          modules[1].state.angle.radians,
          modules[1].state.speedMetersPerSecond,
          modules[2].state.angle.radians,
          modules[2].state.speedMetersPerSecond,
          modules[3].state.angle.radians,
          modules[3].state.speedMetersPerSecond,
        )
      },
      null
    )
    builder.addDoubleArrayProperty(
      "3.2 Desired States",
      {
        doubleArrayOf(
          modules[0].desiredState.angle.radians,
          modules[0].desiredState.speedMetersPerSecond,
          modules[1].desiredState.angle.radians,
          modules[1].desiredState.speedMetersPerSecond,
          modules[2].desiredState.angle.radians,
          modules[2].desiredState.speedMetersPerSecond,
          modules[3].desiredState.angle.radians,
          modules[3].desiredState.speedMetersPerSecond,
        )
      },
      null
    )

    builder.addDoubleArrayProperty(
      "3.3 Steering Rotation",
      {
        doubleArrayOf(
          modules[0].state.angle.rotations,
          modules[1].state.angle.rotations,
          modules[2].state.angle.rotations,
          modules[3].state.angle.rotations,
        )
      },
      null
    )

    builder.publishConstString("5.0", "Motor Stats (Standard Order, FL, FR, BL, BR)")
    builder.addDoubleArrayProperty(
      "5.11 Driving Motor Voltage",
      { DoubleArray(modules.size) { index -> modules[index].lastDrivingVoltage() } },
      null
    )
    builder.addDoubleArrayProperty(
      "5.12 Steering Motor Voltage",
      { DoubleArray(modules.size) { index -> modules[index].lastSteeringVoltage() } },
      null
    )
    builder.addDoubleArrayProperty(
      "5.21 Driving Requested Duty Cycle",
      { DoubleArray(modules.size) { index -> modules[index].requestedDutyCycleDriving() } },
      null
    )
    builder.addDoubleArrayProperty(
      "5.22 Steering Requested Duty Cycle",
      { DoubleArray(modules.size) { index -> modules[index].requestedDutyCycleSteering() } },
      null
    )
    builder.addDoubleArrayProperty(
      "5.31 Driving Applied Duty Cycle",
      { DoubleArray(modules.size) { index -> modules[index].appliedDutyCycleDriving() } },
      null
    )
    builder.addDoubleArrayProperty(
      "5.32 Steering Applied Duty Cycle",
      { DoubleArray(modules.size) { index -> modules[index].appliedDutyCycleSteering() } },
      null
    )
    builder.addDoubleArrayProperty(
      "5.41 Driving Bus Voltage",
      { DoubleArray(modules.size) { index -> modules[index].busVoltageDriving() } },
      null
    )
    builder.addDoubleArrayProperty(
      "5.42 Steering Bus Voltage",
      { DoubleArray(modules.size) { index -> modules[index].busVoltageSteering() } },
      null
    )
    builder.addDoubleArrayProperty(
      "5.51 Driving Output Current",
      { DoubleArray(modules.size) { index -> modules[index].outputCurrentDriving() } },
      null
    )
    builder.addDoubleArrayProperty(
      "5.52 Steering Output Current",
      { DoubleArray(modules.size) { index -> modules[index].outputCurrentSteering() } },
      null
    )

    builder.publishConstString("6.0", "AHRS Values")
    builder.addDoubleProperty("6.1 Heading Degrees", { ahrs.heading.degrees }, null)
    builder.addDoubleProperty("6.2 Pitch Degrees", { ahrs.pitch.degrees }, null)
    builder.addDoubleProperty("6.3 Roll Degrees", { ahrs.roll.degrees }, null)
    builder.addDoubleProperty("6.4 Angular X Vel", { ahrs.angularXVel() }, null)
    builder.addBooleanProperty("6.5 Navx Connected", { ahrs.connected() }, null)
    builder.addBooleanProperty("6.6 Navx Calibrated", { ahrs.calibrated() }, null)

    // Note: You should also tune UPR too
    builder.publishConstString("7.0", "Tuning Values")
    builder.addDoubleProperty("7.1 FL Drive P", { modules[0].driveController.p }, { value -> modules[0].driveController.p = value })
    builder.addDoubleProperty("7.2 FL Drive D", { modules[0].driveController.d }, { value -> modules[0].driveController.d = value })
    builder.addDoubleProperty("7.3 FL Turn P", { modules[0].turnController.p }, { value -> modules[0].turnController.p = value })
    builder.addDoubleProperty("7.4 FL Turn D", { modules[0].turnController.d }, { value -> modules[0].turnController.d = value })
    builder.addDoubleProperty("7.5 FR Drive P", { modules[1].driveController.p }, { value -> modules[1].driveController.p = value })
    builder.addDoubleProperty("7.6 FR Drive D", { modules[1].driveController.d }, { value -> modules[1].driveController.d = value })
    builder.addDoubleProperty("7.8 FR Turn P", { modules[1].turnController.p }, { value -> modules[1].turnController.p = value })
    builder.addDoubleProperty("7.9 FR Turn D", { modules[1].turnController.d }, { value -> modules[1].turnController.d = value })
    builder.addDoubleProperty("7.10 BL Drive P", { modules[2].driveController.p }, { value -> modules[2].driveController.p = value })
    builder.addDoubleProperty("7.11 BL Drive D", { modules[2].driveController.d }, { value -> modules[2].driveController.d = value })
    builder.addDoubleProperty("7.12 BL Turn P", { modules[2].turnController.p }, { value -> modules[2].turnController.p = value })
    builder.addDoubleProperty("7.13 BL Turn D", { modules[2].turnController.d }, { value -> modules[2].turnController.d = value })
    builder.addDoubleProperty("7.14 BR Drive P", { modules[3].driveController.p }, { value -> modules[3].driveController.p = value })
    builder.addDoubleProperty("7.15 BR Drive D", { modules[3].driveController.d }, { value -> modules[3].driveController.d = value })
    builder.addDoubleProperty("7.16 BR Turn P", { modules[3].turnController.p }, { value -> modules[3].turnController.p = value })
    builder.addDoubleProperty("7.17 BR Turn D", { modules[3].turnController.d }, { value -> modules[3].turnController.d = value })
  }

  companion object {
    /** Create a [SwerveDrive] using [SwerveConstants]. */
    fun createSwerve(ahrs: AHRS, field: Field2d): SwerveDrive {
      val driveMotorController = { PIDController(SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD) }
      val turnMotorController = { PIDController(SwerveConstants.TURN_KP, SwerveConstants.TURN_KI, SwerveConstants.TURN_KD) }
      val driveFeedforward = SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA)
      val modules = listOf(
        SwerveModule.create(
          "FLModule",
          makeDrivingMotor(
            "FL",
            SwerveConstants.DRIVE_MOTOR_FL,
            inverted = false
          ),
          makeTurningMotor(
            "FL",
            SwerveConstants.TURN_MOTOR_FL,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_FL,
            SwerveConstants.TURN_ENC_OFFSET_FL
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "FRModule",
          makeDrivingMotor(
            "FR",
            SwerveConstants.DRIVE_MOTOR_FR,
            inverted = false
          ),
          makeTurningMotor(
            "FR",
            SwerveConstants.TURN_MOTOR_FR,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_FR,
            SwerveConstants.TURN_ENC_OFFSET_FR
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "BLModule",
          makeDrivingMotor(
            "BL",
            SwerveConstants.DRIVE_MOTOR_BL,
            inverted = false
          ),
          makeTurningMotor(
            "BL",
            SwerveConstants.TURN_MOTOR_BL,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_BL,
            SwerveConstants.TURN_ENC_OFFSET_BL
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(-SwerveConstants.WHEELBASE / 2, SwerveConstants.TRACKWIDTH / 2)
        ),
        SwerveModule.create(
          "BRModule",
          makeDrivingMotor(
            "BR",
            SwerveConstants.DRIVE_MOTOR_BR,
            inverted = false
          ),
          makeTurningMotor(
            "BR",
            SwerveConstants.TURN_MOTOR_BR,
            inverted = true,
            sensorPhase = false,
            SwerveConstants.TURN_ENC_CHAN_BR,
            SwerveConstants.TURN_ENC_OFFSET_BR
          ),
          driveMotorController(),
          turnMotorController(),
          driveFeedforward,
          Translation2d(-SwerveConstants.WHEELBASE / 2, -SwerveConstants.TRACKWIDTH / 2)
        )
      )
      return if (isReal()) {
        SwerveDrive(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          field
        )
      } else {
        SwerveSim(
          modules,
          ahrs,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ROT_SPEED,
          field
        )
      }
    }

    /** Helper to make turning motors for swerve. */
    private fun makeDrivingMotor(
      name: String,
      motorId: Int,
      inverted: Boolean
    ) =
      createSparkMax(
        name = name + "Drive",
        id = motorId,
        enableBrakeMode = true,
        inverted = inverted,
        encCreator =
        NEOEncoder.creator(
          SwerveConstants.DRIVE_UPR,
          SwerveConstants.DRIVE_GEARING
        ),
        currentLimit = SwerveConstants.DRIVE_CURRENT_LIM
      )

    /** Helper to make turning motors for swerve. */
    private fun makeTurningMotor(
      name: String,
      motorId: Int,
      inverted: Boolean,
      sensorPhase: Boolean,
      encoderChannel: Int,
      offset: Double
    ) =
      createSparkMax(
        name = name + "Turn",
        id = motorId,
        enableBrakeMode = false,
        inverted = inverted,
        encCreator = AbsoluteEncoder.creator(
          encoderChannel,
          offset,
          SwerveConstants.TURN_UPR,
          sensorPhase
        ),
        currentLimit = SwerveConstants.STEERING_CURRENT_LIM
      )
  }
}
