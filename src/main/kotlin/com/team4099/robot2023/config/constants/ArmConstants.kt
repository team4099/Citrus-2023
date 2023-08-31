package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.gearRatio
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object ArmConstants {
  object PID {
    val NEO_KP =
      0.1.volts / 1.degrees // TODO: Taken directly from 1678's code but we should tune them.
    val NEO_KI = 0.0.volts / (1.degrees * 1.seconds)
    val NEO_KD = 0.0.volts / (1.degrees.perSecond)

    val SIM_KP = 0.1.volts / 1.degrees
    val SIM_KI = 0.0.volts / (1.degrees * 1.seconds)
    val SIM_KD = 0.01.volts / (1.degrees.perSecond)

    val ARM_KS =
      0.2.volts // TODO: We should tune these feedforward values because they don't seem to exist
    // in 1678's code, so these constants are taken from our ground intake
    // implementation.

    val ARM_KG = 0.86.volts
    val ARM_KV = 1.8.volts / 1.0.radians.perSecond
    val ARM_KA = 0.1.volts / 1.0.radians.perSecond.perSecond
  }
  val SPROCKET_RATIO = ((12.0 / 48.0) * (24.0 * 48.0)).gearRatio

  val VOLTAGE_COMPENSATION = 12.volts
  val ARM_CURRENT_LIMIT = 60.amps // TODO: TUNE

  val ABSOLUTE_ENCODER_OFFSET = 0.degrees // No actual value can be determined
  val HOMING_POSITION = (-9.598).degrees
  val MIN_ROTATION = (-9.598).degrees
  val MAX_ROTATION = 113.706.degrees

  val MAX_VELOCITY = 360.degrees.perSecond // TODO: TUNE
  val MAX_ACCELERATION = 600.degrees.perSecond.perSecond // TODO: TUNE
  val TOLERANCE = 1.degrees // TODO: TUNE
  val MOMENT_OF_INERTIA = 0.0000478.kilo.grams * 1.0.meters.squared
  val ARM_LENGTH = 25.5.inches // Use first stage height
}
