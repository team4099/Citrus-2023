package com.team4099.lib.requests


abstract class Request {

  /**
   * Empty method that will be filled with all of the executable(s) for the given request.
   */
  abstract fun act()

  /**
   * Abstract method that returns if all of the request parameters have been met.
   * @return Default should be true if there is no check required for the next request to execute. Will run indefinately if this never returns true
   */
  abstract fun isFinished(): Boolean

  /**
   * List that contains all of the passed in prerequisites for a request to run
   */
  var prerequisites: MutableList<Prerequisite> = ArrayList<Prerequisite>()

  /**
   * Adds each individual subsyste prerequisite to a list of prerequisites which is default empty on each request creaetion
   * @param reqs List of prerequisites for a subsystem to meet prior to the request being executed
   */
  open fun addPrerequisites(reqs: List<Prerequisite>) {
    for (req in reqs) {
      prerequisites.add(req)
    }
  }

  /**
   * Adds a specific prerequisite to a list of prerequisites which is default empty on each request creation.
   * @param req A single prerequisite for a subsystem to meet prior to the request being executed.
   */
  open fun addPrerequisite(req: Prerequisite) {
    prerequisites.add(req)
  }

  /**
   * Verifies that all prerequisites have been met prior to allowing the request to execute
   * @return Default is true if no prerequisites are added or need to be evaluated for the subsystem to function.
   */
  open fun allowed(): Boolean {
    var reqMet = true
    for (req in prerequisites) {
      reqMet = reqMet and req.met()
    }
    return reqMet
  }
}
