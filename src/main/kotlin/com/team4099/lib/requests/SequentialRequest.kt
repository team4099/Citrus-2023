package com.team4099.lib.requests

import org.eclipse.jetty.http.HttpURI.Mutable


abstract class SequentialRequest {
  var requests: MutableList<Request>? = null
  var currentRequest: Request? = null

  /**
   * Adds in a list of requests into the local requests list using variable arguments.
   * @param reqs List of Request type arguments delineated by a comma. Can range from 0 - inf.
   */
  fun SequentialRequest(vararg reqs: Request) {
    val requests = mutableListOf<Request>()
    for (req in reqs) {
      requests.add(req)
    }
  }

  /**
   * Adds a list of requets into the local request list using a set Java list.
   * @param reqs List of Request type arguments.
   */
  fun SequentialRequest(reqs: List<Request>) {
    val requests = mutableListOf<Request>()
    for (req in reqs) {
      requests.add(req)
    }
  }

  fun act() {
    currentRequest = requests!!.removeAt(0)
    currentRequest!!.act()
  }

  fun isFinished(): Boolean {
    if (currentRequest == null) {
      if (requests!!.isEmpty()) {
        currentRequest = null
        return true
      } else {
        currentRequest = requests!!.removeAt(0)
        currentRequest!!.act()
      }
    }
    if (currentRequest!!.isFinished()) {
      if (requests!!.isEmpty()) {
        currentRequest = null
        return true
      } else {
        currentRequest = requests!!.removeAt(0)
        currentRequest!!.act()
      }
    }
    return false
  }

  fun getListLength(): Int {
    return requests!!.size
  }

  fun getActiveRequest(): String? {
    return if (currentRequest == null) {
      "null"
    } else {
      currentRequest.toString()
    }
  }
}
