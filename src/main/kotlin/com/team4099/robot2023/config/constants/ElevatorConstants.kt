package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.derived.gearRatio
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import kotlin.math.PI

object ElevatorConstants {

  val VOLTAGE_COMPENSATION = 12.volts
  val GEAR_RATIO = ((10.0 / 72.0) * (20.0 / 72.0)).gearRatio

  val SPOOL_RADIUS = 0.005.meters * 24.0 / (2 * PI)

  val PHASE_CURRENT_LIMIT = 80.amps
  val RAMP_RATE = 0.5.percent.perSecond
}
