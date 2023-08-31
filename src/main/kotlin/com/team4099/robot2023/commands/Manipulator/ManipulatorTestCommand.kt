package com.team4099.robot2023.commands.Manipulator

import com.team4099.robot2023.subsystems.Manipulator.Manipulator
import com.team4099.robot2023.superstructure.Request
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts

class ManipulatorTestCommand(val manipulator: Manipulator): CommandBase() {
  init{
    addRequirements(manipulator)
  }

  override fun execute() {
    manipulator.currentRequest = Request.ManipulatorRequest.TargetingPosition(6.volts, 40.degrees)
  }

  override fun end(interupted: Boolean){
    manipulator.currentRequest = Request.ManipulatorRequest.TargetingPosition(0.volts, 0.degrees)
  }

}
