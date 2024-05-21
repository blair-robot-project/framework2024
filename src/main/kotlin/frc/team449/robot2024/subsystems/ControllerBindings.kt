package frc.team449.robot2024.subsystems

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure.mutable
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.RobotConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

class ControllerBindings(
  private val driveController: CommandXboxController,
  private val mechanismController: CommandXboxController,
  private val robot: Robot
) {

  val sysIdRoutine = SysIdRoutine(
    SysIdRoutine.Config(),
    Mechanism(
      { voltage: Measure<Voltage> -> robot.drive.setVoltage(voltage.`in`(Volts)) },
      null,
      robot.drive
    )
  )

  private val m_appliedVoltage = mutable(Volts.of(0.0))
  private val m_angle = mutable(Radians.of(0.0))
  private val m_velocity = mutable(RadiansPerSecond.of(0.0))

  val shooterRoutine = SysIdRoutine(
    SysIdRoutine.Config(
      Volts.of(0.5).per(Seconds.of(1.0)),
      Volts.of(3.0),
      Seconds.of(10.0)
    ) { state -> SignalLogger.writeString("state", state.toString()) },
    Mechanism(
      { voltage: Measure<Voltage> -> run { robot.drive.setVoltage(voltage.`in`(Volts)) } },
      null,
      robot.drive,
      "shooter"
    )
  )

  fun robotBindings() {
    /** Characterization */
    // Quasistatic Forwards
    driveController.povUp().onTrue(
      shooterRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    )

    // Quasistatic Reverse
    driveController.povDown().onTrue(
      shooterRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    // Dynamic Forwards
    driveController.povRight().onTrue(
      shooterRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    // Dynamic Reverse
    driveController.povLeft().onTrue(
      shooterRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )
  }

  private fun nonRobotBindings() {
    // slow drive
    driveController.rightBumper().onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 1.0 })
        .andThen(InstantCommand({ robot.drive.maxRotSpeed = PI / 2 }))
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
        .andThen(
          InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED })
        )
    )

    // reset gyro
    driveController.start().onTrue(
      ConditionalCommand(
        InstantCommand({ robot.drive.heading = Rotation2d(PI) }),
        InstantCommand({ robot.drive.heading = Rotation2d() })
      ) { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }
    )

    // introduce "noise" to the simulated pose
//    driveController.b().onTrue(
//      ConditionalCommand(
//        InstantCommand({
//          robot.drive as SwerveSim
//          robot.drive.resetPos()
//        }),
//        InstantCommand()
//      ) { RobotBase.isSimulation() }
//    )
  }

  fun bindButtons() {
    nonRobotBindings()
    robotBindings()
  }
}
