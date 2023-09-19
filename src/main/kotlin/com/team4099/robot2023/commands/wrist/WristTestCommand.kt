package com.team4099.robot2023.commands.wrist

import com.team4099.lib.requests.EmptyRequest
import com.team4099.lib.requests.Request
import com.team4099.robot2023.subsystems.wrist.Wrist
import edu.wpi.first.wpilibj2.command.CommandBase
import org.team4099.lib.units.derived.degrees

class WristTestCommand(val wrist: Wrist) : CommandBase() {
  var request: Request = EmptyRequest()

  init {
    addRequirements(wrist)
  }

  override fun initialize() {
    request = wrist.wristClosedLoopRequest(60.degrees, false)
  }

  override fun execute() {
    request.act()
  }

  override fun isFinished(): Boolean {
    return request.isFinished()
  }
}
