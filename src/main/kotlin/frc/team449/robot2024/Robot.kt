package frc.team449.robot2024

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.RobotBase
import frc.team449.control.holonomic.swerve.SwerveDrive
import frc.team449.control.holonomic.swerve.SwerveOrthogonalCommand
import frc.team449.robot2024.constants.RobotConstants
import frc.team449.system.AHRS
import frc.team449.system.light.Light.Companion.createLight
import monologue.Annotations.Log
import monologue.Logged

class Robot : RobotBase(), Logged {

  val driveController = CommandXboxController(0)

  val mechController = CommandXboxController(1)

  val ahrs = AHRS(SPI.Port.kMXP)

  // Instantiate/declare PDP and other stuff here

  @Log.NT
  override val powerDistribution: PowerDistribution = PowerDistribution(
    RobotConstants.PDH_CAN,
    PowerDistribution.ModuleType.kRev
  )

  @Log.NT
  override val drive = SwerveDrive.createSwerveNEO(ahrs, field)

  @Log.NT
  override val driveCommand = SwerveOrthogonalCommand(drive, driveController.hid)

  val light = createLight()

  /** Example for using infrared sensors
   @Log.NT
   val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
   */
}
