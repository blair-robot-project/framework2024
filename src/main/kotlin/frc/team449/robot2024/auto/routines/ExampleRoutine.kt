package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.control.auto.ChoreoRoutine
import frc.team449.control.auto.ChoreoRoutineStructure
import frc.team449.control.auto.ChoreoTrajectory
import frc.team449.robot2024.Robot
import frc.team449.robot2024.auto.AutoUtil

class ExampleRoutine(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      parallelEventMap = hashMapOf(
        0 to InstantCommand(),
        1 to InstantCommand()
      ),
      stopEventMap = hashMapOf(
        0 to PrintCommand("Going to start moving!"),
        1 to PrintCommand("Finished the 1st trajectory!"),
        2 to PrintCommand("Finished the routine!")
      ),
      debug = false,
      timeout = 1.5
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRed(
        ChoreoTrajectory.createTrajectory(arrayListOf("part1", "part2"), "example")
      )
    } else {
      ChoreoTrajectory.createTrajectory(arrayListOf("part1", "part2"), "example")
    }
}
