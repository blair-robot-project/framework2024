package frc.team449.robot2024.auto

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Nat
import edu.wpi.first.wpilibj2.command.*
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.constants.auto.AutoConstants
import frc.team449.robot2024.constants.field.FieldConstants
import kotlin.math.PI

object AutoUtil {

  fun autoIntake(robot: Robot): Command {
    return ParallelCommandGroup(
      SequentialCommandGroup(
        robot.undertaker.intake(),
        robot.feeder.autoIntake(),
        WaitUntilCommand { !robot.infrared.get() },
        robot.undertaker.stop(),
        robot.feeder.outtake(),
        WaitUntilCommand { robot.infrared.get() },
        robot.feeder.stop(),
      ),
      robot.shooter.shootSubwoofer(),
    )
  }

  fun autoIntakeAway(robot: Robot): Command {
    return ParallelCommandGroup(
      SequentialCommandGroup(
        robot.pivot.moveStow(),
        robot.undertaker.intake(),
        robot.feeder.autoIntake(),
        WaitUntilCommand { !robot.infrared.get() },
        robot.undertaker.stop(),
        robot.feeder.outtake(),
        robot.pivot.autoAngle(),
        WaitUntilCommand { robot.infrared.get() },
        robot.feeder.stop(),
      ),
      robot.shooter.shootAuto(),
    )
  }

  fun autoShootAway(robot: Robot): Command {
    return ParallelDeadlineGroup(
      SequentialCommandGroup(
        robot.pivot.autoAngle(),
        WaitUntilCommand { robot.shooter.atAutoSetpoint() && robot.pivot.inTolerance() },
        WaitCommand(AutoConstants.SHOOT_AWAY_WAIT),
        robot.feeder.intake(),
        robot.undertaker.intake(),
        WaitCommand(AutoConstants.SHOOT_INTAKE_TIME)
      ),
      robot.shooter.shootAuto()
    )
  }

  fun autoShoot(robot: Robot): Command {
    return ParallelDeadlineGroup(
      SequentialCommandGroup(
        WaitUntilCommand { robot.shooter.atAutoSetpoint() },
        robot.feeder.intake(),
        robot.undertaker.intake(),
        WaitCommand(AutoConstants.SHOOT_INTAKE_TIME)
      ),
      robot.shooter.shootSubwoofer()
    )
  }
  fun transformForPos2(pathGroup: MutableList<ChoreoTrajectory>): MutableList<ChoreoTrajectory> {
    for (index in 0 until pathGroup.size) {
      for (time in pathGroup[index].objectiveTimestamps) {
        val currentMatrix = pathGroup[index].stateMap.get(time)

        val newMatrix = MatBuilder.fill(
          Nat.N2(),
          Nat.N3(),
          currentMatrix[0, 0],
          FieldConstants.fieldWidth - currentMatrix[0, 1],
          -currentMatrix[0, 2],
          currentMatrix[1, 0],
          -currentMatrix[1, 1],
          -currentMatrix[1, 2]
        )

        pathGroup[index].stateMap.put(time, newMatrix)
      }
    }

    return pathGroup
  }

  fun transformForRed(pathGroup: MutableList<ChoreoTrajectory>): MutableList<ChoreoTrajectory> {
    for (index in 0 until pathGroup.size) {
      for (time in pathGroup[index].objectiveTimestamps) {
        val currentMatrix = pathGroup[index].stateMap.get(time)

        val newMatrix = MatBuilder.fill(
          Nat.N2(),
          Nat.N3(),
          FieldConstants.fieldLength - currentMatrix[0, 0],
          currentMatrix[0, 1],
          MathUtil.angleModulus(PI - currentMatrix[0, 2]),
          -currentMatrix[1, 0],
          currentMatrix[1, 1],
          -currentMatrix[1, 2]
        )

        pathGroup[index].stateMap.put(time, newMatrix)
      }
    }

    return pathGroup
  }

  /** Add other methods that return commands that do groups of actions that are done
   * across different auto routines. For Charged UP, these methods were things such as
   * dropping a cone/cube, or getting in ground intake position, etc.
   */
}
