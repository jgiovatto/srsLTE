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

#include "libemanelte/uestatisticmanager.h"
#include "srsue/hdr/metrics_ostatistic.h"

namespace srsue{

metrics_ostatistic::metrics_ostatistic():
  ue(NULL)
{ }

void metrics_ostatistic::set_ue_handle(ue_metrics_interface *ue_)
{
  ue = ue_;
}

void metrics_ostatistic::set_metrics(const ue_metrics_t &m, const uint32_t)
{
#if 0 // ALINK_XXX REMOVE
  UESTATS::RLCQueueMetricsList rlcQueueMetrics;
  UESTATS::RLCQueueMetricsList rlcMrbQueueMetrics;

  const auto & stack = m.stack;
  const auto & phy   = m.phy;
  const auto & gw    = m.gw;
  const auto & mac   = stack.mac;
  const auto & rlc   = stack.rlc;

  for(size_t n = 0; n < SRSLTE_N_RADIO_BEARERS; ++n)
    {
      // use capacity to determine if lcid is active
      if(rlc.metrics[n].qmetrics.capacity > 0)
       {
         rlcQueueMetrics.emplace_back(
            UESTATS::RLCQueueMetrics((int)rlc.metrics[n].mode,
                                     rlc.metrics[n].qmetrics.capacity,
                                     rlc.metrics[n].qmetrics.currsize,
                                     rlc.metrics[n].qmetrics.highwater,
                                     rlc.metrics[n].qmetrics.num_cleared,
                                     rlc.metrics[n].qmetrics.num_push,
                                     rlc.metrics[n].qmetrics.num_push_fail,
                                     rlc.metrics[n].qmetrics.num_pop,
                                     rlc.metrics[n].qmetrics.num_pop_fail));
        }
     }

   for(size_t n = 0; n < SRSLTE_N_MCH_LCIDS; ++n)
     {
       // use capacity to determine if lcid is active
       if(rlc.mrb_metrics[n].qmetrics.capacity > 0)
        {
          rlcMrbQueueMetrics.emplace_back(
            UESTATS::RLCQueueMetrics((int)rlc.mrb_metrics[n].mode,
                                     rlc.mrb_metrics[n].qmetrics.capacity,
                                     rlc.mrb_metrics[n].qmetrics.currsize,
                                     rlc.mrb_metrics[n].qmetrics.highwater,
                                     rlc.mrb_metrics[n].qmetrics.num_cleared,
                                     rlc.mrb_metrics[n].qmetrics.num_push,
                                     rlc.mrb_metrics[n].qmetrics.num_push_fail,
                                     rlc.mrb_metrics[n].qmetrics.num_pop,
                                     rlc.mrb_metrics[n].qmetrics.num_pop_fail));
        }
     }

  UESTATS::setRLCMetrics(UESTATS::RLCMetrics(rlc.dl_tput_mbps, 
                                             rlc.ul_tput_mbps,
                                             rlcQueueMetrics,
                                             rlc.dl_tput_mrb_mbps,
                                             rlcMrbQueueMetrics));

  UESTATS::setGWMetrics(UESTATS::GWMetrics(gw.dl_tput_mbps, gw.ul_tput_mbps));

  // list 1 carrier for now
  for(int cc = 0; cc < 1; ++cc)
   {
     UESTATS::setMACMetrics(
      UESTATS::MACMetrics(mac[cc].tx_pkts,
                          mac[cc].tx_errors,
                          mac[cc].tx_brate,
                          mac[cc].rx_pkts,
                          mac[cc].rx_errors,
                          mac[cc].rx_brate,
                          mac[cc].ul_buffer,
                          mac[cc].dl_retx_avg,
                          mac[cc].ul_retx_avg));

      UESTATS::setPHYMetrics(
       UESTATS::PHYMetrics(phy.sync[cc].ta_us,
                           phy.sync[cc].cfo,      
                           phy.sync[cc].sfo,        
                           phy.dl[cc].n,
                           phy.dl[cc].sinr,
                           phy.dl[cc].rsrp,
                           phy.dl[cc].rsrq,
                           phy.dl[cc].rssi,
                           phy.dl[cc].turbo_iters,
                           phy.dl[cc].mcs,
                           phy.dl[cc].pathloss,
                           0,   // was mabr_mbps, now unused
                           phy.ul[cc].mcs,
                           phy.ul[cc].power,
                           0)); // was mabr_mbps, now unused
    }
#endif
}

} // end namespace srsue
