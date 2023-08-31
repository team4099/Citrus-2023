package com.team4099.robot2023.subsystems.superstructure

import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
  sealed interface ArmRequest {
    class TargetingPosition(val position: Angle) : ArmRequest
    class OpenLoop(val voltage: ElectricalPotential) : ArmRequest
    class ZeroArm : ArmRequest
  }
}
