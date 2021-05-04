/**
 *
 * \section COPYRIGHT
 *
 * Copyright (c) 2019 - Adjacent Link LLC, Bridgewater, New Jersey
 *
 * \section LICENSE
 *
 * This file is part of srsRAN.
 *
 * srsUE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsUE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#ifndef EMU_SRSENB_PHY_ADAPTER_H
#define EMU_SRSENB_PHY_ADAPTER_H

#include "srsran/config.h"
#include "srsran/phy/enb/enb_dl.h"
#include "srsran/interfaces/enb_interfaces.h"
#include "srsenb/hdr/phy/phy_interfaces.h"
#include "srsran/interfaces/enb_mac_interfaces.h"

#include "srsenb/hdr/stack/rrc/rrc.h"

#include "libemanelte/mhalconfig.h"

#include <vector>

namespace srsenb {

namespace phy_adapter {

void enb_initialize(uint32_t                       sf_interval,
                    phy_cell_cfg_list_t            cfg_list,
                    EMANELTE::MHAL::mhal_config_t& mhal_config,
                    rrc_cfg_t*                     rrc_cfg);

 void enb_set_frequency(uint32_t cc_idx,
                        double rx_freq_hz,
                        double tx_freq_hz);

 // state start
 void enb_start();

 // state stop
 void enb_stop();

 // set base info
 void enb_dl_cc_tx_init(const srsran_enb_dl_t* q,
                        uint32_t               tti,
                        uint32_t               cfi,
                        uint32_t               cc_idx);

 // send to mhal with sot time
 bool enb_dl_send_signal(time_t sot_sec,
                         float  frac_sec);

 // get from mhal with sot time
 bool enb_ul_get_signal(uint32_t            tti,
                        srsran_timestamp_t * ts);

 // set the power scaling on a per rnti basis
 void enb_dl_set_power_allocation(uint32_t tti, 
                                  uint16_t rnti,
                                  float    rho_a_db,
                                  float    rho_b_db);

 // set dl pdcch
 int enb_dl_cc_put_pdcch_dl(srsran_enb_dl_t*                         q, 
                            srsran_dci_cfg_t*                        dci_cfg,
                            mac_interface_phy_lte::dl_sched_grant_t* grant,
                            uint32_t                                 ref,
                            uint32_t                                 cc_idx);
 // set dl pdsch
 int enb_dl_cc_put_pdsch_dl(srsran_enb_dl_t*                         q, 
                            srsran_pdsch_cfg_t*                      pdsch, 
                            mac_interface_phy_lte::dl_sched_grant_t* grant,
                            uint32_t                                 ref,
                            uint32_t                                 cc_idx);


 // set dl pdsch
 int enb_dl_cc_put_pdsch(srsran_enb_dl_t*    q, 
                         srsran_pdsch_cfg_t* pdsch, 
                         uint8_t*            data[SRSRAN_MAX_CODEWORDS],
                         uint32_t            idx,
                         uint32_t            cc_idx);

 // set dl mch
 int enb_dl_cc_put_pmch(srsran_enb_dl_t*                         q,
                        srsran_pmch_cfg_t*                       pmch_cfg,
                        mac_interface_phy_lte::dl_sched_grant_t* dl_sched_grant,
                        uint32_t                                 cc_idx);

 //set ul pdcch
 int enb_dl_cc_put_pdcch_ul(srsran_enb_dl_t*  q, 
                            srsran_dci_cfg_t* dci_cfg, 
                            srsran_dci_ul_t*  dci_ul,
                            uint32_t          idx,
                            uint32_t          cc_idx);

 // set phich
 int enb_dl_cc_put_phich(srsran_enb_dl_t*                       q, 
                         srsran_phich_grant_t*                  grant,
                         mac_interface_phy_lte::ul_sched_ack_t* ack,
                         uint32_t                               cc_idx);

 // get prach
 int enb_ul_cc_get_prach(uint32_t*  indicies, 
                         float*     offsets,
                         float*     avgs,
                         uint32_t   max_entries,
                         uint32_t & num_detected,
                         uint32_t cc_idx);

 // get pucch
 int enb_ul_cc_get_pucch(srsran_enb_ul_t*    q,
                         srsran_ul_sf_cfg_t* ul_sf,
                         srsran_pucch_cfg_t* cfg,
                         srsran_pucch_res_t* res,
                         uint32_t            cc_idx);

 // get pusch
 int enb_ul_cc_get_pusch(srsran_enb_ul_t*    q,
                         srsran_ul_sf_cfg_t* ul_sf,
                         srsran_pusch_cfg_t* cfg,
                         srsran_pusch_res_t* res,
                         uint16_t            rnti,
                         uint32_t            cc_idx);
 } 
}
#endif //EMU_SRSENB_PHY_ADAPTER_H
