package com.team4099.lib.requests

import org.team4099.lib.hal.Clock
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.seconds

abstract class WaitRequest {
  private var waitTime = 0.0.seconds
  private var startTime = 0.0.seconds

  open fun WaitRequest(waitTime: Time) {
    this.waitTime = waitTime
  }

  open fun act() {
    startTime = Clock.fpgaTime
  }

  open fun isFinished(): Boolean {
    return Clock.fpgaTime - startTime >= waitTime
  }
}
