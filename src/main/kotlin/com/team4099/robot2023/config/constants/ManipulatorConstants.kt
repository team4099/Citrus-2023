package com.team4099.robot2023.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.percent
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.gearRatio
import org.team4099.lib.units.derived.perRadian
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object ManipulatorConstants {
  object PID{
    val NEO_KP = 0.0.volts / 1.degrees
    val NEO_KI = 0.0.volts / (1.degrees * 1.seconds)
    val NEO_KD = 0.0.volts / (1.degrees.perSecond)
    val ARM_KS = 0.0.volts
    val ARM_KG = 1.582887.volts
    val ARM_KV = 0.87.volts / 1.0.radians.perSecond
    val ARM_KA = 0.04.volts / 1.0.radians.perSecond.perSecond
  }

  // TODO get the gear ratio
  val ROLLER_GEAR_RATIO = 1.0.gearRatio
  val ROLLER_VOLTAGE_COMPENSATION = 12.volts
  val WRIST_GEAR_RATIO = 1.0.gearRatio
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
  val MAX_ARM_VELOCITY = 0.degrees.perSecond
  val MAX_ARM_ACCELERATION = 0.degrees.perSecond.perSecond
  val CONE_CURRENT_THRESHOLD = 0.amps
  val CUBE_CURRENT_THRESHOLD = 0.amps
  val CUBE_IN = 12.volts
  val CONE_IN = -12.volts
  val CUBE_IDLE = 3.volts
  val CONE_IDLE = -3.volts
  val MANIPULATOR_WAIT_BEFORE_DETECT_CURRENT_SPIKE = 0.seconds
}
