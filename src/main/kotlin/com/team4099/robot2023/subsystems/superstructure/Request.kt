package com.team4099.robot2023.subsystems.superstructure

import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.perSecond

sealed class Request {
  sealed interface ArmRequest {
    class TargetingPosition(val position: Angle) : ArmRequest
    class OpenLoop(val voltage: ElectricalPotential) : ArmRequest
    class ZeroArm : ArmRequest
  }

  sealed interface ElevatorRequest {
    class TargetingPosition(
      val position: Length,
      val finalVelocity: LinearVelocity = 0.0.inches.perSecond
    ) : ElevatorRequest
    class OpenLoop(val voltage: ElectricalPotential) : ElevatorRequest
    class Home : ElevatorRequest
  }

  sealed interface ManipulatorRequest{
    class OpenLoop(val rollerVoltage: ElectricalPotential, val wristVoltage: ElectricalPotential) : ManipulatorRequest
    class TargetingPosition(val rollerVoltage: ElectricalPotential, val wristPosition: Angle) : ManipulatorRequest
    class HomingWrist() : ManipulatorRequest
  }
}
