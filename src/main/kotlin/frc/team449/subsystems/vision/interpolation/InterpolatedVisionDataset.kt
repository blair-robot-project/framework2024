// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team449.subsystems.vision.interpolation

import edu.wpi.first.math.geometry.Translation2d

enum class InterpolatedVisionDataset(red: List<VisionInterpolationData>, blue: List<VisionInterpolationData>) {
  HOME1(
    listOf(
      VisionInterpolationData(
        Translation2d(15.2245, 5.522),
        Translation2d(15.215, 5.553),
        "RED_SUBWOOFER"
      ),
      VisionInterpolationData(
        Translation2d(13.0745, 5.522),
        Translation2d(13.351, 5.572),
        "RED_PODIUM_SPEAKER_INTERSECTION"
      ),
      VisionInterpolationData(
        Translation2d(11.059, 6.842),
        Translation2d(11.42, 6.958),
        "RED_WING_LINE_MIDDLE"
      ),
      VisionInterpolationData(
        Translation2d(13.799, 4.202),
        Translation2d(13.929, 4.152),
        "RED_FRONT_PODIUM_MIDDLE"
      )
    ),
    listOf(
      VisionInterpolationData(
        Translation2d(1.315, 5.522),
        Translation2d(1.34, 5.51),
        "BLUE_SUBWOOFER"
      ),
      VisionInterpolationData(
        Translation2d(3.465, 5.522),
        Translation2d(3.295, 5.35),
        "BLUE_PODIUM_SPEAKER_INTERSECTION"
      ),
      VisionInterpolationData(
        Translation2d(5.481, 6.842),
        Translation2d(5.256, 6.751),
        "BLUE_WING_LINE_MIDDLE"
      ),
      VisionInterpolationData(
        Translation2d(2.741, 4.202),
        Translation2d(2.630, 3.974),
        "BLUE_FRONT_PODIUM_MIDDLE"
      )
    )
  ),
  HOME2(
    listOf(
      VisionInterpolationData(
        Translation2d(15.2245, 5.522),
        Translation2d(15.2245, 5.522),
        "SUBWOOFER"
      ),
      VisionInterpolationData(
        Translation2d(13.0745, 5.522),
        Translation2d(13.15, 5.52),
        "PODIUM_SPEAKER_INTERSECTION"
      ),
      VisionInterpolationData(
        Translation2d(11.059, 6.842),
        Translation2d(11.31, 6.81),
        "WING_LINE_MIDDLE"
      ),
      VisionInterpolationData(
        Translation2d(13.799, 4.202),
        Translation2d(13.80, 4.02),
        "FRONT_PODIUM_MIDDLE"
      )
    ),
    listOf(
      VisionInterpolationData(
        Translation2d(1.315, 5.522),
        Translation2d(1.39, 5.53),
        "BLUE_SUBWOOFER"
      ),
      VisionInterpolationData(
        Translation2d(3.465, 5.522),
        Translation2d(3.42, 5.49),
        "BLUE_PODIUM_SPEAKER_INTERSECTION"
      ),
      VisionInterpolationData(
        Translation2d(5.481, 6.842),
        Translation2d(5.37, 6.82),
        "BLUE_WING_LINE_MIDDLE"
      ),
      VisionInterpolationData(
        Translation2d(2.741, 4.202),
        Translation2d(2.79, 4.09),
        "BLUE_FRONT_PODIUM_MIDDLE"
      )
    )
  );

  val redSet: List<VisionInterpolationData>
  val blueSet: List<VisionInterpolationData>

  init {
    redSet = red
    blueSet = blue
  }
}
