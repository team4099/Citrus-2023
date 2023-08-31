package com.team4099.robot2023.superstructure

import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request{

  sealed interface ManipulatorRequest : Request{
    class OpenLoop(val rollerVoltage: ElectricalPotential, val wristVoltage: ElectricalPotential) : ManipulatorRequest
    class TargetingPosition(val rollerVoltage: ElectricalPotential, val wristPosition: Angle) : ManipulatorRequest
    class HomingWrist() : ManipulatorRequest
  }
}
