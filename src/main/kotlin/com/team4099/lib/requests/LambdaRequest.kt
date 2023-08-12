package com.team4099.lib.requests

abstract class LambdaRequest {
  /**
   * Generates a method denoted by 'f' and is of the type void
   */
  interface VoidInterface {
    fun f()
  }

  /**
   * Makes a variable that holds a method 'f'
   */
  var mF: VoidInterface? = null

  /**
   * Setter for the vairable 'mF'
   * @param f Any function that is of class void
   */
  open fun LambdaRequest(f: VoidInterface?) {
    mF = f
  }

  open fun act() {
    mF!!.f()
  }

  open fun isFinished(): Boolean {
    return true
  }
}
