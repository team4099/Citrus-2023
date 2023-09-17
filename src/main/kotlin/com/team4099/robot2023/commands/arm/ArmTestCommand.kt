package com.team4099.robot2023.commands.arm

import com.team4099.lib.requests.EmptyRequest
import com.team4099.lib.requests.Request
import com.team4099.robot2023.subsystems.arm.Arm
import com.team4099.robot2023.subsystems.wrist.Wrist
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.derived.degrees

class ArmTestCommand(val arm: Arm) : CommandBase() {
  var request: Request = EmptyRequest()

  init {
    addRequirements(arm)
  }

  override fun initialize() {
    request = arm.armClosedLoopRequest(60.degrees)
  }

  override fun execute() {
    request.act()
  }

  override fun isFinished(): Boolean {
    return false
  }
}
