package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.driven
import org.team4099.lib.units.derived.driving
import org.team4099.lib.units.derived.gearRatio
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perSecond

object WristConstants {
  object PID {
    val NEO_KP = 0.0.volts / 1.degrees
    val NEO_KI = 0.0.volts / (1.degrees * 1.seconds)
    val NEO_KD = 0.0.volts / (1.degrees.perSecond)

    val SIM_KP = 0.2.volts / 1.degrees
    val SIM_KI = 0.0.volts / (1.degrees * 1.seconds)
    val SIM_KD = 0.0.volts / (1.degrees.perSecond)

    val ARM_KS = 0.0.volts
    val ARM_KG = 0.2.volts
    val ARM_KV = 1.01.volts / 1.0.radians.perSecond
    val ARM_KA = 0.0.volts / 1.0.radians.perSecond.perSecond
  }

  val TOLERANCE = 1.degrees
  val ARM_VOLTAGE_COMPENSATION = 0.volts
  val MIN_ROTATION = 0.degrees
  val MAX_ROTATION = 215.degrees
  val HOMING_APPLIED_VOLTAGE = 0.volts
  val HOMING_STALL_TIME_THRESHOLD = 0.seconds
  val HOMING_STALL_CURRENT = 0.amps
  val WRIST_MASS = 2.93.pounds
  val WRIST_LENGTH = 15.inches
  val ROLLER_MOMENT_OF_INERTIA = 0.0000478.kilo.grams * 1.0.meters.squared
  val WRIST_MOMENT_OF_INERTIA = WRIST_MASS * WRIST_LENGTH.squared * 1 / 3

  // TODO get the gear ratio
  val ROLLER_GEAR_RATIO = 1.0.gearRatio
  val ROLLER_VOLTAGE_COMPENSATION = 12.volts
  val WRIST_GEAR_RATIO =
    (( 72.0.driven / 10.0.driving) * (72.0.driven / 20.0.driving ) * (48.0.driven / 24.0.driving ))
      .gearRatio
  val WRIST_VOLTAGE_COMPENSATION = 12.volts
  val ROLLER_MOTOR_INVERTED = false
  val INTAKE_ANGLE = 0.degrees
  val OUTTAKE_ANGLE = 0.degrees
  val STOWED_UP_ANGLE = 0.degrees
  val STOWED_DOWN_ANGLE = 0.degrees
  val VOLTAGE_COMPENSATION = 0.volts
  val ROLLER_CURRENT_LIMIT = 0.amps
  val ROLLER_RAMP_RATE = 0.percent.perSecond
  val WRIST_CURRENT_LIMIT = 0.amps
  val WRIST_MOTOR_INVERTED = false
  val INTAKE_VOLTAGE = 0.volts
  val OUTTAKE_VOLTAGE = 0.volts
  val NEUTRAL_VOLTAGE = 0.volts
  val ARM_MAX_ROTATION = 0.radians
  val ARM_MIN_ROTATION = 0.radians
  val ARM_OPEN_LOOP_MAX_ROTATION = 0.radians
  val ARM_OPEN_LOOP_MIN_ROTATION = 0.radians
  val MAX_ARM_VELOCITY = 30.degrees.perSecond
  val MAX_ARM_ACCELERATION = 30.degrees.perSecond.perSecond
  val CONE_CURRENT_THRESHOLD = 0.amps
  val CUBE_CURRENT_THRESHOLD = 0.amps
  val CUBE_IN = 12.volts
  val CONE_IN = -12.volts
  val CUBE_IDLE = 3.volts
  val CONE_IDLE = -3.volts
  val WRIST_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.seconds
}
