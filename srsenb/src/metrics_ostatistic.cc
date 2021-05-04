/*
 * Copyright 2013-2019 Software Radio Systems Limited
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

#include "libemanelte/enbstatisticmanager.h"
#include "srsenb/hdr/metrics_ostatistic.h"

namespace srsenb{

metrics_ostatistic::metrics_ostatistic():
  enb(NULL)
{ }

void metrics_ostatistic::set_handle(enb_metrics_interface *enb_)
{
  enb = enb_;
}

void metrics_ostatistic::set_metrics(const enb_metrics_t &m, const uint32_t)
{
  const auto & stack = m.stack;
  const auto & rrc   = stack.rrc;
  const auto & mac   = stack.mac;
  const auto & s1ap  = stack.s1ap;

  ENBSTATS::MACMetrics    macMetrics;

  ENBSTATS::setS1State(s1ap.status == S1AP_ATTACHING ? "ATTACHING" :
                       s1ap.status == S1AP_READY     ? "READY"     : 
                                                       "ERROR");

  for(uint16_t user = 0; user < rrc.ues.size(); ++user)
   {
        const std::string state = rrc.ues[user].state == RRC_STATE_IDLE                            ? "IDLE" :
                                  rrc.ues[user].state == RRC_STATE_WAIT_FOR_CON_SETUP_COMPLETE     ? "WAIT_SETUP_COMP" :
                                  rrc.ues[user].state == RRC_STATE_WAIT_FOR_SECURITY_MODE_COMPLETE ? "WAIT_SECMD_COMP" :
                                  rrc.ues[user].state == RRC_STATE_WAIT_FOR_UE_CAP_INFO            ? "WAIT_CAP_INFO"   :
                                  rrc.ues[user].state == RRC_STATE_WAIT_FOR_CON_RECONF_COMPLETE    ? "WAIT_CON_RECONF" :
                                  rrc.ues[user].state == RRC_STATE_REGISTERED                      ? "REGISTERED"      :
                                  rrc.ues[user].state == RRC_STATE_RELEASE_REQUEST                 ? "RELEASE_REQUEST" : "ERROR";

        macMetrics.emplace_back(ENBSTATS::MACMetric(mac.ues[user].rnti,
                                                    mac.ues[user].tx_pkts,
                                                    mac.ues[user].tx_errors,
                                                    mac.ues[user].tx_brate,
                                                    mac.ues[user].rx_pkts,
                                                    mac.ues[user].rx_errors,
                                                    mac.ues[user].rx_brate,
                                                    mac.ues[user].ul_buffer,
                                                    mac.ues[user].dl_buffer,
                                                    mac.ues[user].dl_cqi,
                                                    mac.ues[user].dl_ri,
                                                    mac.ues[user].dl_pmi,
                                                    mac.ues[user].phr,
                                                    state));
   }

  ENBSTATS::setMACMetrics(macMetrics);
}

} // end namespace srsenb
