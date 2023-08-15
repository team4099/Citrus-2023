package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.gearRatio
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import kotlin.math.PI

object ElevatorConstants {
  val REAL_KP = 0.85.volts / 1.inches // TODO: Tune later in sim
  val REAL_KI = 0.0.volts / (1.inches * 1.seconds)
  val REAL_KD = 0.0.volts / (1.inches.perSecond)

  val SIM_KP = 1.5.volts / 1.inches
  val SIM_KI = 0.0.volts / (1.inches * 1.seconds)
  val SIM_KD = 0.25.volts / (1.inches.perSecond)

  val REAL_ELEVATOR_KS_FIRST_STAGE = 0.54.volts  // TODO: Tune later
  val ELEVATOR_KG_FIRST_STAGE = 0.28.volts
  val ELEVATOR_KV_FIRST_STAGE = 0.18.volts / 1.inches.perSecond
  val ELEVATOR_KA_FIRST_STAGE = 0.03.volts / 1.meters.perSecond.perSecond

  val REAL_ELEVATOR_KS_SECOND_STAGE = 0.25.volts
  val ELEVATOR_KG_SECOND_STAGE = 0.36.volts
  val ELEVATOR_KV_SECOND_STAGE = 0.18.volts / 1.inches.perSecond
  val ELEVATOR_KA_SECOND_STAGE = 0.04.volts / 1.meters.perSecond.perSecond

  // Homing constants
  val HOMING_APPLIED_VOLTAGE = -1.volts
  val HOMING_STALL_CURRENT = 20.amps
  val HOMING_STALL_TIME_THRESHOLD = 0.1.seconds

  val VOLTAGE_COMPENSATION = 12.volts
  val GEAR_RATIO = ((10.0 / 72.0) * (20.0 / 72.0)).gearRatio

  val SPOOL_RADIUS = 0.005.meters * 24.0 / (2 * PI)

  val MAX_VELOCITY = 100.inches.perSecond // TODO: Tune in sim
  val MAX_ACCELERATION = 350.inches.perSecond.perSecond

  val PHASE_CURRENT_LIMIT = 60.amps
  val RAMP_RATE = 0.5.percent.perSecond

  val ELEVATOR_MIN_RETRACTION = 0.0.inches
  val ELEVATOR_MAX_EXTENSION = 66.8.inches
  val ELEVATOR_SOFT_LIMIT_RETRACTION = 0.5.inches
  val ELEVATOR_SOFT_LIMIT_EXTENSION = 66.inches
  val ELEVATOR_OPEN_LOOP_SOFTLIMIT_RETRACTION = 5.inches
  val ELEVATOR_OPEN_LOOP_SOFTLIMIT_EXTENSION = 60.inches
  val ELEVATOR_IDLE_HEIGHT = 1.0.inches

  val FIRST_STAGE_HEIGHT = 25.5.inches
  val SECOND_STAGE_HEIGHT = 25.75.inches

  const val ENABLE_ELEVATOR = 1.0
  val ELEVATOR_TOLERANCE = 0.75.inches
}
