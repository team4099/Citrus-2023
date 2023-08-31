package com.team4099.lib.requests


abstract class ParallelRequest {
  var requests: MutableList<Request> = ArrayList()

  /**
   * Adds any length of requests to the list of requests using variable args.
   * @param requests Can be any amount of requests from 0 - inf. Requests can be passed in with a separation of commas.
   */
  open fun ParallelRequest(vararg reqs: Request) {
    for (request in reqs) {
      requests.add(request)
    }
  }

  /**
   * Appends a list of requests to the empty parallel request list.
   * @param requests List of requests.
   */
  open fun ParallelRequest(reqs: List<Request>) {
    for (request in reqs) {
      requests.add(request)
    }
  }

  open fun act() {
    for (request in requests) {
      request.act()
    }
  }

  open fun isFinished(): Boolean {
    requests.removeIf { r: Request -> r.isFinished() }
    return requests.isEmpty()
  }
}
