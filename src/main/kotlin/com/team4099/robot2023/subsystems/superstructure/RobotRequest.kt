package com.team4099.robot2023.subsystems.superstructure

import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.perSecond

sealed interface RobotRequest {
  sealed interface ArmRobotRequest : RobotRequest {
    class TargetingPosition(val position: Angle) : ArmRobotRequest
    class OpenLoop(val voltage: ElectricalPotential) : ArmRobotRequest
    class ZeroArm : ArmRobotRequest
  }

  sealed interface ElevatorRobotRequest : RobotRequest {
    class TargetingPosition(
      val position: Length,
      val finalVelocity: LinearVelocity = 0.0.inches.perSecond
    ) : ElevatorRobotRequest
    class OpenLoop(val voltage: ElectricalPotential) : ElevatorRobotRequest
    class Home : ElevatorRobotRequest
  }

  sealed interface ManipulatorRobotRequest : RobotRequest {
    class OpenLoop(val rollerVoltage: ElectricalPotential, val wristVoltage: ElectricalPotential) :
      ManipulatorRobotRequest
    class TargetingPosition(val rollerVoltage: ElectricalPotential, val wristPosition: Angle) :
      ManipulatorRobotRequest
    class HomingWrist() : ManipulatorRobotRequest
  }
}
