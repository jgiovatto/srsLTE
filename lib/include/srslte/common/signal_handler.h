/*
 * Copyright 2013-2020 Software Radio Systems Limited
 *
 * This file is part of srsLTE.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

/**
 * @file signal_handler.h
 * @brief Common signal handling methods for all srsLTE applications.
 */

#ifndef SRSLTE_SIGNAL_HANDLER_H
#define SRSLTE_SIGNAL_HANDLER_H

#include "srslte/common/logger_file.h"
#include <signal.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// ALINK impatient, was 5 sec
#define SRSLTE_TERM_TIMEOUT_S (1)

// static vars required by signal handling
static srslte::logger_file logger_file;
static bool                running = true;

static void srslte_signal_handler(int signal)
{
  switch (signal) {
    case SIGALRM:
      fprintf(stderr, "Couldn't stop after %ds. Forcing exit.\n", SRSLTE_TERM_TIMEOUT_S);
      logger_file.stop();
      raise(SIGKILL);
    default:
      // all other registered signals try to stop the app gracefully
      if (running) {
        running = false;
        fprintf(stdout, "Stopping ..\n");
        alarm(SRSLTE_TERM_TIMEOUT_S);
      } else {
        // already waiting for alarm to go off ..
      }
      break;
  }
}

void srslte_register_signal_handler()
{
  signal(SIGINT, srslte_signal_handler);
  signal(SIGTERM, srslte_signal_handler);
  signal(SIGHUP, srslte_signal_handler);
  signal(SIGALRM, srslte_signal_handler);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SRSLTE_SIGNAL_HANDLER_H
