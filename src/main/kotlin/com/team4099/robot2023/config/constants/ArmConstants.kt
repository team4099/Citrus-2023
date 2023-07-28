package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.gearRatio
import org.team4099.lib.units.derived.volts

object ArmConstants {
  val SPROCKET_RATIO = ((12.0 / 48.0) * (24.0 * 48.0)).gearRatio

  val VOLTAGE_COMPENSATION = 12.volts
  val ARM_CURRENT_LIMIT = 60.amps // TODO TUNE

  val ABSOLUTE_ENCODER_OFFSET = 0.degrees  // No actual value can be determined
}
