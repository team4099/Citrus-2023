package com.team4099.lib.requests

abstract class EmptyRequest {
  open fun act() {}

  open fun isFinished(): Boolean {
    return true
  }
}
