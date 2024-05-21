package frc.team449.robot2024.auto.routines

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.robot2024.Robot

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "DoNothing" to DoNothing(robot).createCommand(),
    )
  }

  init {
    updateOptions(true)
  }

  fun updateOptions(isRed: Boolean) {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")

    this.addOption(
      "4 piece",
      if (isRed) "Red4" else "Blue4"
    )

    this.addOption(
      "worlds center first",
      if (isRed) "RedWorld" else "BlueWorld"
    )

    this.addOption(
      "worlds 3",
      if (isRed) "RedWorld3" else "BlueWorld3"
    )

    this.addOption(
      "5 Piece Subwoofer MIDDY",
      if (isRed) "RedSubwoofer5Piece" else "BlueSubwoofer5Piece"
    )

    this.addOption(
      "5 Piece Subwoofer CENTY",
      if (isRed) "RedSubwoofer5PieceCenty" else "BlueSubwoofer5PieceCenty"
    )

    this.addOption(
      "5 Piece Subwoofer FARTHY",
      if (isRed) "RedSubwoofer5PieceFarthy" else "BlueSubwoofer5PieceFarthy"
    )

    this.addOption(
      "3 Piece Mid",
      if (isRed) "Red3PieceMid" else "Blue3PieceMid"
    )

    this.addOption(
      "6 Piece",
      if (isRed) "RedSixPiece" else "BlueSixPiece"
    )

    this.addOption(
      "4 Piece Helper Centerline",
      if (isRed) "RedFourPieceHelper" else "BlueFourPieceHelper"
    )

    this.addOption(
      "4 Piece Helper Amp Side",
      if (isRed) "RedFourPieceAmp" else "BlueFourPieceAmp"
    )
  }
}
