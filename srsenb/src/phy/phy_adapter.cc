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

#include "srsran/config.h"
#include "srsran/srslog/srslog.h"
#include "srsran/phy/adapter/phy_adapter_common.h"

extern "C" {
#include "srsran/phy/phch/ra.h"
#include "srsran/phy/phch/dci.h"
#include "srsran/phy/phch/phich.h"
#include "srsran/phy/phch/pucch.h"
}

#include "lib/include/srsran/phy/phch/pdsch_cfg.h"
#include "srsenb/hdr/phy/phy_adapter.h"

#include "libemanelte/enbotamessage.pb.h"
#include "libemanelte/ueotamessage.pb.h"

#include "libemanelte/mhalenb.h"
#include "libemanelte/enbstatisticmanager.h"
#include "libemanelte/sinrtester.h"

#include <mutex>

// private namespace for misc helpers and state for PHY_ADAPTER
namespace {
  EMANELTE::MHAL::ENB_DL_Message     enb_dl_msg_;
  EMANELTE::MHAL::TxControlMessage   txControl_;

  EMANELTE::MHAL::SINRTester         sinrTester_{{}};

  // ul ue msg
  #define UL_Message_Message(x)    std::get<0>((x))
  #define UL_Message_RxControl(x)  std::get<1>((x))
  #define UL_Message_SINRTester(x) std::get<2>((x))

  using UL_Message = std::tuple<EMANELTE::MHAL::UE_UL_Message, 
                                EMANELTE::MHAL::RxControl,
                                EMANELTE::MHAL::SINRTester>;

  // 0 or more ue ul messages for this tti
  using UL_Messages = std::vector<UL_Message>;

  // search for carrier result
  using CarrierResult = std::pair<bool, const EMANELTE::MHAL::UE_UL_Message_CarrierMessage &>;
 // helpers
#define CarrierResult_Found(x)   std::get<0>((x))
#define CarrierResult_Carrier(x) std::get<1>((x))

  UL_Messages ulMessages_;

  // track carrier index to rx/tx freq
  CarrierIndexFrequencyTable carrierIndexFrequencyTable_;

  // track pci to carrier
  std::map<uint32_t,uint32_t> pciTable_;

  // track carrier to pci
  std::map<uint32_t,uint32_t> carrierTable_;

  uint64_t tx_seqnum_ = 0;
  uint32_t curr_tti_  = 0;
  uint32_t tti_tx_    = 0;

  uint32_t pdcch_ref_ = 0;
  uint32_t pdsch_ref_ = 0;

  // referenceSignalPower as set by sib.conf sib2.rr_config_common_sib.pdsch_cnfg.rs_power
  float pdsch_rs_power_milliwatt_ = 0.0;

  // scaling between pdsch res in symbols with reference signals to symbols without reference signals
  float pdsch_rho_b_over_rho_a_ = 1.0;

  // scaling between reference signal res and pdsch res in symbols without reference signals, by tti and rnti
  using RHO_A_DB_MAP_t = std::map<uint16_t, float>; // map of rnti to rho_a

  RHO_A_DB_MAP_t rho_a_db_map_[10];                 // vector of rho_a maps by subframe number

  // cyclic prefix normal or extended for this cell
  srsran_cp_t cell_cp_ = SRSRAN_CP_NORM;

  inline bool rnti_is_user_i(uint32_t rnti)
   {
     return(rnti == SRSRAN_SIRNTI || 
            rnti == SRSRAN_PRNTI  || 
           (rnti >= SRSRAN_RARNTI_START && rnti <= SRSRAN_RARNTI_END));
   }

  std::mutex dl_mutex_;
  std::mutex ul_mutex_;


  srslog::basic_logger * logger_phy = nullptr;

  const uint8_t zeros_[0xffff] = {0};

  inline void
  initDownlinkChannelMessage(EMANELTE::MHAL::ChannelMessage * channelMessage,
                             EMANELTE::MHAL::CHANNEL_TYPE ctype,
                             EMANELTE::MHAL::MOD_TYPE modType,
                             uint16_t rnti,
                             uint32_t infoBits,
                             float txPowerScaledB = 0.0)
  {
    channelMessage->set_channel_type(ctype);
    channelMessage->set_modulation_type(modType);
    channelMessage->set_number_of_bits(infoBits);
    channelMessage->set_tx_power_scale_db(txPowerScaledB);

    if(rnti)
     {
       channelMessage->set_rnti(rnti);
     }
  }

  inline int bits_to_bytes(int bits) { return bits/8; }
}

#define Error(fmt, ...)                               \
  if (SRSRAN_DEBUG_ENABLED && logger_phy)             \
  logger_phy->error(fmt, ##__VA_ARGS__)
#define Warning(fmt, ...)                             \
  if (SRSRAN_DEBUG_ENABLED && logger_phy)             \
  logger_phy->warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)                                \
  if (SRSRAN_DEBUG_ENABLED && logger_phy)             \
  logger_phy->info(fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)                               \
  if (SRSRAN_DEBUG_ENABLED && logger_phy)             \
  logger_phy->debug(fmt, ##__VA_ARGS__)


namespace srsenb {
namespace phy_adapter {

static inline EMANELTE::MHAL::DCI_FORMAT convert(srsran_dci_format_t format)
 {
   switch(format)
    {
      case SRSRAN_DCI_FORMAT0:
        return(EMANELTE::MHAL::DCI_FORMAT_0);

      case SRSRAN_DCI_FORMAT1:
        return(EMANELTE::MHAL::DCI_FORMAT_1);

      case SRSRAN_DCI_FORMAT1A:
        return(EMANELTE::MHAL::DCI_FORMAT_1A);

      case SRSRAN_DCI_FORMAT1C:
        return(EMANELTE::MHAL::DCI_FORMAT_1C);

      case SRSRAN_DCI_FORMAT1B:
        return(EMANELTE::MHAL::DCI_FORMAT_1B);

      case SRSRAN_DCI_FORMAT1D:
        return(EMANELTE::MHAL::DCI_FORMAT_1D);

      case SRSRAN_DCI_FORMAT2:
        return(EMANELTE::MHAL::DCI_FORMAT_2);

      case SRSRAN_DCI_FORMAT2A:
        return(EMANELTE::MHAL::DCI_FORMAT_2A);

      case SRSRAN_DCI_FORMAT2B:
        return(EMANELTE::MHAL::DCI_FORMAT_2B);

      default:
       throw("MHAL:convert: invalid dci format");

      return (EMANELTE::MHAL::DCI_FORMAT_ERR);
   }
}

static inline EMANELTE::MHAL::MOD_TYPE convert(srsran_mod_t type)
{
   switch(type)
    {
       case SRSRAN_MOD_BPSK: 
         return (EMANELTE::MHAL::MOD_BPSK);

       case SRSRAN_MOD_QPSK:
         return (EMANELTE::MHAL::MOD_QPSK);

       case SRSRAN_MOD_16QAM:
         return (EMANELTE::MHAL::MOD_16QAM);

       case SRSRAN_MOD_64QAM:
         return (EMANELTE::MHAL::MOD_64QAM);

       default:
         throw("MHAL:convert: invalid mod type");

       return (EMANELTE::MHAL::MOD_ERR);
    }
}

// lookup carrier that matches the frequencies associated with the cc_idx
CarrierResult
findCarrierByIndex(const EMANELTE::MHAL::UE_UL_Message & ue_ul_msg, uint32_t cc_idx)
 {
   const auto iter = carrierIndexFrequencyTable_.find(cc_idx);

   if(iter != carrierIndexFrequencyTable_.end())
    {
      const auto my_cell_id = carrierTable_.at(cc_idx);

      for(auto & carrier : ue_ul_msg.carriers())
       {
         // match our rx freq to the msg carrier center freq
         if(iter->second.first == carrier.frequency_hz())
          {
            if(carrierTable_.count(cc_idx))
             {
               // XXX_CC TODO check this 
               if(my_cell_id != carrier.phy_cell_id())
                {
                  Info("%s: cc %u, found, but my_cell_id %u != carrier cell id %u\n",
                        __func__, cc_idx, my_cell_id, carrier.phy_cell_id());

                }

               return CarrierResult(true, carrier);
             }
          }
       }      
    }
 
  Info("%s: cc %u, NOT found\n", __func__, cc_idx);
 
  return CarrierResult(false, EMANELTE::MHAL::UE_UL_Message_CarrierMessage{});
 }


// lookup tx freq that matches the frequencies associated with the cc_idx
static inline uint64_t getTxFrequency(uint32_t cc_idx)
{
   const auto iter = carrierIndexFrequencyTable_.find(cc_idx);

   if(iter != carrierIndexFrequencyTable_.end())
    {
      return iter->second.second; // tx
    }

  return 0;
}

// lookup rx freq that matches the frequencies associated with the cc_idx
static inline uint64_t getRxFrequency(uint32_t cc_idx)
{
   const auto iter = carrierIndexFrequencyTable_.find(cc_idx);

   if(iter != carrierIndexFrequencyTable_.end())
    {
      return iter->second.first; // rx
    }

  return 0;
}



/*
typedef struct SRSRAN_API {
  srsran_cell_t      cell;
  srsran_dl_sf_cfg_t dl_sf;
  srsran_pbch_t      pbch;
  srsran_pcfich_t    pcfich;
  srsran_regs_t      regs;
  srsran_pdcch_t     pdcch;
  srsran_pdsch_t     pdsch;
  srsran_pmch_t      pmch;
  srsran_phich_t     phich;
} srsran_enb_dl_t;

typedef struct SRSRAN_API {
  uint32_t k[4];
  uint32_t k0;
  uint32_t l;
  bool     assigned;
}srsran_regs_reg_t;

typedef struct SRSRAN_API {
  uint32_t          nof_regs;
  srsran_regs_reg_t **regs;
}srsran_regs_ch_t;

typedef struct SRSRAN_API {
  srsran_cell_t cell;
  uint32_t      max_ctrl_symbols;
  uint32_t      ngroups_phich;
  uint32_t      ngroups_phich_m1;

  srsran_phich_r_t      phich_res;
  srsran_phich_length_t phich_len;
  
  srsran_regs_ch_t  pcfich;
  srsran_regs_ch_t* phich; 
  srsran_regs_ch_t  pdcch[3];

  uint32_t           phich_mi;
  uint32_t           nof_regs;
  srsran_regs_reg_t* regs;
}srsran_regs_t;

typedef struct SRSRAN_API {
  srsran_cell_t  cell;
  uint32_t       nof_regs[3];
  uint32_t       nof_cce[3];
  uint32_t       max_bits;
  uint32_t       nof_rx_antennas;
  bool           is_ue;
  srsran_regs_t* regs;
  float          rm_f[3*(SRSRAN_DCI_MAX_BITS + 16)];
  float*         llr;
  srsran_modem_table_t mod;
  srsran_sequence_t    seq[SRSRAN_NOF_SF_X_FRAME];
  srsran_viterbi_t     decoder;
  srsran_crc_t         crc;
} srsran_pdcch_t;

typedef struct SRSRAN_API {
  uint8_t               payload[SRSRAN_DCI_MAX_BITS];
  uint32_t              nof_bits;
  srsran_dci_location_t location;
  srsran_dci_format_t   format;
  uint16_t              rnti;
} srsran_dci_msg_t;

typedef struct SRSRAN_API {
  uint32_t L;    // Aggregation level
  uint32_t ncce; // Position of first CCE of the dci
} srsran_dci_location_t;

typedef struct SRSRAN_API {
  uint32_t mcs_idx;
  int      rv;
  bool     ndi;
  uint32_t cw_idx;
} srsran_dci_tb_t; */

// see lib/src/phy/phch/pdcch.c
#define PDCCH_NOF_FORMATS               4
#define PDCCH_FORMAT_NOF_CCE(i)          (1<<i)
#define PDCCH_FORMAT_NOF_REGS(i)        ((1<<i)*9)
#define PDCCH_FORMAT_NOF_BITS(i)        ((1<<i)*72)

#define NOF_CCE(cfi)  ((cfi>0&&cfi<4)?q->pdcch.nof_cce [cfi-1]:0)
#define NOF_REGS(cfi) ((cfi>0&&cfi<4)?q->pdcch.nof_regs[cfi-1]:0)

// srsran_pdcch_encode(&q->pdcch, &q->dl_sf, &dci_msg, q->sf_symbols)
static int enb_dl_put_dl_pdcch_i(const srsran_enb_dl_t * q,
                                 const srsran_dci_msg_t * dci_msg,
                                 uint32_t ref,
                                 int type,  // 0 for DL, 1 for UL
                                 uint32_t cc_idx)
 {
   const auto rnti = dci_msg->rnti;

   // see lib/src/phy/phch/regs.c int srsran_regs_pdcch_put_offset(srsran_regs_t *h, 
   //                                                              uint32_t cfi, 
   //                                                              uint32_t start_reg,
   //                                                              uint32_t nof_regs)
   const uint32_t nof_regs = PDCCH_FORMAT_NOF_REGS(dci_msg->location.L);
   uint32_t start_reg      = dci_msg->location.ncce * 9;

   // see lib/src/phy/phch/pdcch.c srsran_pdcch_encode(srsran_pdcch_t*     q,
   //                                                  srsran_dl_sf_cfg_t* sf,
   //                                                  srsran_dci_msg_t*   msg,
   if(!((dci_msg->location.ncce + PDCCH_FORMAT_NOF_CCE(dci_msg->location.L) <= NOF_CCE(q->dl_sf.cfi)) &&
        (dci_msg->nof_bits < (SRSRAN_DCI_MAX_BITS - 16)))) 
    {
      Warning("PDCCH:%s cc %u, type %s, rnti 0x%hx, cfi %d, illegal dci msg, ncce %d, format_ncce %d, cfi_ncce %d, nof_bits %d, max_bits %d\n", 
            __func__,
            cc_idx,
            type ? "UL" : "DL",
            rnti,
            q->dl_sf.cfi,
            dci_msg->location.ncce, 
            PDCCH_FORMAT_NOF_CCE(dci_msg->location.L),
            NOF_CCE(q->dl_sf.cfi),
            dci_msg->nof_bits,
            (SRSRAN_DCI_MAX_BITS - 16));

      // ALINK_XXX TODO
      // srsran p/r #299 amd issue #347 temp fix set start_reg to 0
      start_reg = 0;
    }

   const uint32_t regs_len = start_reg + nof_regs;

   if(regs_len > NOF_REGS(q->dl_sf.cfi))
    {
      Warning("PDCCH:%s cc %u, type %s, rnti 0x%hx, cfi %d, pdccd->nof_regs %d, regs_len %u, ncce %d -> start_reg %d, L %d -> nof_regs %d\n", 
              __func__,
              cc_idx,
              type ? "UL" : "DL",
              rnti,
              q->dl_sf.cfi,
              NOF_REGS(q->dl_sf.cfi),
              regs_len,
              dci_msg->location.ncce, 
              start_reg,
              dci_msg->location.L,
              nof_regs);

      return SRSRAN_ERROR;
   }

  const auto frequencyHz = getTxFrequency(cc_idx);

  auto carrier = getCarrierByFrequency<EMANELTE::MHAL::ENB_DL_Message_CarrierMessage,
                                       EMANELTE::MHAL::ENB_DL_Message>(enb_dl_msg_, frequencyHz);


  auto pdcch_message = carrier->add_pdcch();

  auto controlCarrier = getCarrierByFrequency<EMANELTE::MHAL::TxControlCarrierMessage, 
                                              EMANELTE::MHAL::TxControlMessage>(txControl_, frequencyHz);


  auto channelMessage = controlCarrier->mutable_downlink()->add_pdcch();

  initDownlinkChannelMessage(channelMessage,
                             EMANELTE::MHAL::CHAN_PDCCH,
                             EMANELTE::MHAL::MOD_QPSK,
                             rnti,
                             dci_msg->nof_bits);


  for(uint32_t i = start_reg; i < regs_len; ++i)
   {
    const auto reg = q->pdcch.regs->pdcch[q->dl_sf.cfi-1].regs[i];

    if(reg)
     {
       const uint32_t  k0 = reg->k0;
       const uint32_t  l  = reg->l;
       const uint32_t* k  = &reg->k[0];

       const uint32_t rb = k0 / 12;

       Debug("PDCCH DCI group sf_idx=%d, reg=%d, rnti=%d placement: "
             "(l=%u, "
             "k0=%u, "
             "k[0]=%u "
             "k[1]=%u "
             "k[2]=%u "
             "k[3]=%u) in rb=%u\n", tti_tx_ % 10, i, rnti, l, k0, k[0], k[1], k[2], k[3], rb);

       channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, frequencyHz));
     }
   }

  if(type == 0)
   {
     // dl dci
     auto dl_dci_message = pdcch_message->mutable_dl_dci();

     dl_dci_message->set_rnti(dci_msg->rnti);
     dl_dci_message->set_refid(pdcch_ref_++);

     // dci msg
     auto dl_dci_msg = dl_dci_message->mutable_dci_msg();

     dl_dci_msg->set_num_bits(dci_msg->nof_bits);
     dl_dci_msg->set_l_level(dci_msg->location.L);
     dl_dci_msg->set_l_ncce(dci_msg->location.ncce);
     dl_dci_msg->set_data(dci_msg->payload, dci_msg->nof_bits);
     dl_dci_msg->set_format(convert(dci_msg->format));

#ifdef DEBUG_HEX
     InfoHex(dci_msg->payload, dci_msg->nof_bits,
             "PDCCH_DL:%s cc %u, rnti=0x%hx, refid %d, nof_bits %d\n",
             __func__, cc_idx, rnti, pdcch_ref_ - 1, dci_msg->nof_bits);
#endif
   }
  else
   {
     // ul dci
     auto dl_dci_message = pdcch_message->mutable_ul_dci();

     dl_dci_message->set_rnti(dci_msg->rnti);

     // dci msg
     auto ul_dci_msg = dl_dci_message->mutable_dci_msg();

     ul_dci_msg->set_num_bits(dci_msg->nof_bits);
     ul_dci_msg->set_l_level(dci_msg->location.L);
     ul_dci_msg->set_l_ncce(dci_msg->location.ncce);
     ul_dci_msg->set_data(dci_msg->payload, dci_msg->nof_bits);
     ul_dci_msg->set_format(convert(dci_msg->format));

#ifdef DEBUG_HEX
     InfoHex(dci_msg->payload, dci_msg->nof_bits,
             "PDCCH_UL:%s cc %u, rnti=0x%hx, nof_bits %d\n",
             __func__, cc_idx, rnti, dci_msg->nof_bits);
#endif
   }

  return SRSRAN_SUCCESS;
}

// lib/src/phy/phch/pdsch.c
// srsran_pdsch_encode(srsran_pdsch_t* q, 
//                     srsran_dl_sf_cfg_t* sf, 
//                     srsran_pdsch_cfg_t* cfg, 
//                     uint8_t*data[SRSRAN_MAX_CODEWORDS] ...);

/*
typedef struct SRSRAN_API {
  srsran_cell_t      cell;
  srsran_dl_sf_cfg_t dl_sf;
  srsran_pbch_t      pbch;
  srsran_pcfich_t    pcfich;
  srsran_regs_t      regs;
  srsran_pdcch_t     pdcch;
  srsran_pdsch_t     pdsch;
  srsran_pmch_t      pmch;
  srsran_phich_t     phich;
} srsran_enb_dl_t;

typedef struct SRSRAN_API {
  srsran_tdd_config_t tdd_config;
  uint32_t            tti;
  uint32_t            cfi;
  srsran_sf_t         sf_type;
  uint32_t            non_mbsfn_region;
} srsran_dl_sf_cfg_t;

typedef struct SRSRAN_API {
  srsran_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srsran_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
  union {
    srsran_softbuffer_tx_t* tx[SRSRAN_MAX_CODEWORDS];
    srsran_softbuffer_rx_t* rx[SRSRAN_MAX_CODEWORDS];
  } softbuffers;
  bool     meas_time_en;
  uint32_t meas_time_value;
} srsran_pdsch_cfg_t;

typedef struct SRSRAN_API {
  srsran_tx_scheme_t tx_scheme;
  uint32_t           pmi;
  bool               prb_idx[2][SRSRAN_MAX_PRB];
  uint32_t           nof_prb;
  uint32_t           nof_re;
  uint32_t           nof_symb_slot[2];
  srsran_ra_tb_t     tb[SRSRAN_MAX_CODEWORDS];
  int                last_tbs[SRSRAN_MAX_CODEWORDS];
  uint32_t           nof_tb;
  uint32_t           nof_layers;
} srsran_pdsch_grant_t;

typedef struct SRSRAN_API {
  srsran_mod_t mod;
  int          tbs;
  int          rv;
  uint32_t     nof_bits;
  uint32_t     cw_idx;
  bool         enabled;
  // this is for debugging and metrics purposes
  uint32_t mcs_idx;
} srsran_ra_tb_t; */

// set pdsch dl
static int enb_dl_put_dl_pdsch_i(const srsran_enb_dl_t * q,
                                 srsran_pdsch_cfg_t* pdsch, 
                                 uint8_t* data,
                                 uint32_t ref,
                                 uint32_t tb,
                                 uint32_t cc_idx)
 {
   const auto grant = pdsch->grant;
   const auto rnti  = pdsch->rnti;

   const uint32_t sf_idx = (tti_tx_ % 10);

   float rho_a_db = 0.0;

   const auto riter = rho_a_db_map_[sf_idx].find(rnti);

   if(riter != rho_a_db_map_[sf_idx].end())
    {
      rho_a_db = riter->second;
    }

   const auto frequencyHz = getTxFrequency(cc_idx);

   auto controlCarrier = getCarrierByFrequency<EMANELTE::MHAL::TxControlCarrierMessage, 
                                               EMANELTE::MHAL::TxControlMessage>(txControl_, frequencyHz);

   auto channelMessage = controlCarrier->mutable_downlink()->add_pdsch();

   initDownlinkChannelMessage(channelMessage,
                              EMANELTE::MHAL::CHAN_PDSCH,
                              convert(grant.tb[tb].mod),
                              rnti,
                              grant.tb[tb].tbs,
                              rho_a_db);

   // Add resource block assignment from the phy_grant
   for(uint32_t rb = 0; rb < q->cell.nof_prb; ++rb)
    {
      if(grant.prb_idx[0][rb])
       {
         channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, frequencyHz));
       }

      if(grant.prb_idx[1][rb])
       {
         channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, frequencyHz));
       }
    }

   auto carrier = getCarrierByFrequency<EMANELTE::MHAL::ENB_DL_Message_CarrierMessage,
                                        EMANELTE::MHAL::ENB_DL_Message>(enb_dl_msg_, frequencyHz);

   auto pdsch_message = carrier->mutable_pdsch();

   // pdsch data
   auto pdsch_data = pdsch_message->add_data();

   pdsch_data->set_refid(pdsch_ref_++);
   pdsch_data->set_tb(tb);
   pdsch_data->set_tbs(grant.tb[tb].tbs);
   pdsch_data->set_data(data, bits_to_bytes(grant.tb[tb].tbs));
   
   ENBSTATS::putDLGrant(rnti);

#ifdef DEBUG_HEX
   InfoHex(data, bits_to_bytes(grant.tb[tb].tbs),
           "PDSCH:%s cc %u, rnti 0x%hx, refid %d, tbs %d\n",
           __func__, cc_idx, rnti, pdsch_ref_ - 1, grant.tb[tb].tbs);
#endif

   return SRSRAN_SUCCESS;
}


/*
typedef struct SRSRAN_API {
  srsran_cell_t      cell;
  srsran_dl_sf_cfg_t dl_sf;
  srsran_pbch_t      pbch;
  srsran_pcfich_t    pcfich;
  srsran_regs_t      regs;
  srsran_pdcch_t     pdcch;
  srsran_pdsch_t     pdsch;
  srsran_pmch_t      pmch;
  srsran_phich_t     phich;
} srsran_enb_dl_t;

typedef struct SRSRAN_API {
  srsran_pdsch_cfg_t pdsch_cfg;
  uint16_t           area_id;
} srsran_pmch_cfg_t;

typedef struct SRSRAN_API {
  srsran_tdd_config_t tdd_config;
  uint32_t            tti;
  uint32_t            cfi;
  srsran_sf_t         sf_type;
  uint32_t            non_mbsfn_region;
} srsran_dl_sf_cfg_t;

typedef struct SRSRAN_API {
  srsran_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srsran_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
  union {
    srsran_softbuffer_tx_t* tx[SRSRAN_MAX_CODEWORDS];
    srsran_softbuffer_rx_t* rx[SRSRAN_MAX_CODEWORDS];
  } softbuffers;
  bool     meas_time_en;
  uint32_t meas_time_value;
} srsran_pdsch_cfg_t; */

static int enb_dl_put_pmch_i(const srsran_enb_dl_t * q,
                            srsran_pmch_cfg_t* pmch_cfg,
                            uint8_t* data,
                            uint16_t rnti,
                            uint32_t cc_idx)
 {
   const auto grant = pmch_cfg->pdsch_cfg.grant;

   if(grant.nof_tb != 1)
    {
      Error("PMCH:%s cc %u, rnti 0x%hx, nof_tb %u, expected 1\n", 
            __func__, cc_idx, rnti, grant.nof_tb);

      return SRSRAN_ERROR;
    }

   const uint32_t tb = 0;

   const auto frequencyHz = getTxFrequency(cc_idx);

   // pmch
   auto carrier = getCarrierByFrequency<EMANELTE::MHAL::ENB_DL_Message_CarrierMessage,
                                        EMANELTE::MHAL::ENB_DL_Message>(enb_dl_msg_, frequencyHz);

   auto pmch_message = carrier->mutable_pmch();

   pmch_message->set_area_id(pmch_cfg->area_id);
   pmch_message->set_tbs(grant.tb[tb].tbs);
   pmch_message->set_rnti(rnti);
   pmch_message->set_data(data ? data : zeros_, grant.tb[tb].tbs);

   auto controlCarrier = getCarrierByFrequency<EMANELTE::MHAL::TxControlCarrierMessage, 
                                               EMANELTE::MHAL::TxControlMessage>(txControl_, frequencyHz);

   auto channelMessage = controlCarrier->mutable_downlink()->mutable_pmch();

   initDownlinkChannelMessage(channelMessage,
                              EMANELTE::MHAL::CHAN_PMCH,
                              convert(grant.tb[tb].mod),
                              rnti,
                              grant.tb[tb].tbs);

   // channelMessage.add_resource_blocks();
   for(uint32_t rb = 0; rb < q->cell.nof_prb; ++rb)
     {
       channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, frequencyHz));
       channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, frequencyHz));
     }

#ifdef DEBUG_HEX
   InfoHex(data, grant.tb[tb].tbs,
           "PMCH:%s cc %u, rnti=0x%hx, area_id %d, tbs %d\n",
           __func__, cc_idx, rnti, pmch_cfg->area_id, grant.tb[tb].tbs);
#endif

   return SRSRAN_SUCCESS;
}


void enb_init_i(uint32_t idx,
                uint32_t sf_interval_msec, 
                uint32_t physical_cell_id, 
                srsran_cp_t cp,
                double ul_freq_hz, // rx
                double dl_freq_hz, // tx
                int n_prb, 
                EMANELTE::MHAL::mhal_config_t & mhal_config,
                rrc_cfg_t * rrc_cfg)
{
  pdsch_rs_power_milliwatt_ = pow(10.0, static_cast<float>(rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.ref_sig_pwr) / 10.0);

  cell_cp_ = cp;

  Info("INIT:%s idx=%u, PCI=%u\n"
       "\tsf_interval=%u msec\n"
       "\trx_freq=%6.4f MHz\n"
       "\ttx_freq=%6.4f MHz\n"
       "\tn_prb=%d\n"
       "\trs_power=%d\n"
       "\tpdsch_rs_power_milliwatt=%0.2f\n"
       "\tp_b=%d\n"
       "\tpdsch_rho_b_over_rho_a=%.02f\n",
       __func__,
       idx,
       physical_cell_id,
       sf_interval_msec,
       ul_freq_hz/1e6,
       dl_freq_hz/1e6,
       n_prb,
       rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.ref_sig_pwr,
       pdsch_rs_power_milliwatt_,
       rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.p_b,
       pdsch_rho_b_over_rho_a_);

  EMANELTE::MHAL::ENB::initialize(idx,
     mhal_config,
     EMANELTE::MHAL::ENB::mhal_enb_config_t(physical_cell_id,
                                            sf_interval_msec,
                                            cp == SRSRAN_CP_NORM ? SRSRAN_CP_NORM_NSYMB : SRSRAN_CP_EXT_NSYMB,
                                            ul_freq_hz, // rx
                                            dl_freq_hz, // tx
                                            n_prb,
                                            pdsch_rs_power_milliwatt_,
                                            pdsch_rho_b_over_rho_a_));
}



// BEGIN phy_adapter enb api
void enb_initialize(uint32_t sf_interval_msec, 
                    phy_cell_cfg_list_t cfg_list,
                    EMANELTE::MHAL::mhal_config_t & mhal_config,
                    rrc_cfg_t * rrc_cfg)
{
   logger_phy = &srslog::fetch_basic_logger("PHY");

   carrierIndexFrequencyTable_.clear();

   uint32_t idx = 0;
   for(auto & cell_cfg : cfg_list)
    {
       enb_init_i(idx++,
                  sf_interval_msec, 
                  cell_cfg.cell.id, 
                  cell_cfg.cell.cp, 
                  cell_cfg.ul_freq_hz, 
                  cell_cfg.dl_freq_hz, 
                  cell_cfg.cell.nof_prb, 
                  mhal_config,
                  rrc_cfg);
    }
}


void enb_set_frequency(uint32_t cc_idx,
                       double rx_freq_hz,
                       double tx_freq_hz)
{
   carrierIndexFrequencyTable_[cc_idx] = FrequencyPair{llround(rx_freq_hz), llround(tx_freq_hz)};

   Warning("%s cc_idx %u, rx_freq %6.4f MHz, tx_freq %6.4f MHz\n",
       __func__,
       cc_idx,
       rx_freq_hz/1e6,
       tx_freq_hz/1e6);
}



void enb_start()
{
  Info("INIT:%s\n", __func__);

  pthread_mutexattr_t mattr;

  if(pthread_mutexattr_init(&mattr) < 0)
   {
     Error("INIT:%s pthread_mutexattr_init error %s, exit\n", __func__, strerror(errno));

     exit(1);
   }
  else
   {
     if(pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT) < 0)
       {
         Error("INIT:%s pthread_mutexattr_setprotocol error %s, exit\n", __func__, strerror(errno));
         exit(1);
       }

     pthread_mutexattr_destroy(&mattr);
  }

  enb_dl_msg_.Clear();

  txControl_.Clear();

  EMANELTE::MHAL::ENB::start();
}


void enb_stop()
{
  Info("STOP:%s\n", __func__);

  EMANELTE::MHAL::ENB::stop();
}


void enb_dl_cc_tx_init(const srsran_enb_dl_t *q,
                       uint32_t tti_tx,
                       uint32_t cfi,
                       uint32_t cc_idx)
{
  // cc workers called in sequence 0 -> n
  if(cc_idx == 0)
   {
     // lock here, unlocked after tx_end to prevent any worker thread(s)
     // from attempting to start a new tx sequence before the current tx sequence
     // is finished
     dl_mutex_.lock();

     enb_dl_msg_.Clear();

     txControl_.Clear();
   }

  // subframe index
  const uint32_t sf_idx = (tti_tx % 10);

  rho_a_db_map_[sf_idx].clear();

  enb_dl_msg_.set_tti(tti_tx);

  const auto frequencyHz = getTxFrequency(cc_idx);

  // note - cfi should be nof_ctrl_symbols on regular frames and
  //        non_mbsfn_region_length (from sib13) on mbsfn frames
  auto carrier = getCarrierByFrequency<EMANELTE::MHAL::ENB_DL_Message_CarrierMessage,
                                       EMANELTE::MHAL::ENB_DL_Message>(enb_dl_msg_, frequencyHz);

  auto controlCarrier = getCarrierByFrequency<EMANELTE::MHAL::TxControlCarrierMessage, 
                                              EMANELTE::MHAL::TxControlMessage>(txControl_, frequencyHz);

  auto downlink = controlCarrier->mutable_downlink();

  carrier->set_cfi(cfi);
  carrier->set_phy_cell_id(q->cell.id);

  downlink->set_num_resource_blocks(q->cell.nof_prb);
  downlink->set_cfi(carrier->cfi());

  controlCarrier->set_phy_cell_id(q->cell.id);

  // save pci/cc_idx
  pciTable_[q->cell.id] = cc_idx;

  carrierTable_[cc_idx] = q->cell.id;

  // save the tti_tx
  tti_tx_ = tti_tx;

  // PCFICH encoding
  auto channelMessage = downlink->mutable_pcfich();

  initDownlinkChannelMessage(channelMessage,
                             EMANELTE::MHAL::CHAN_PCFICH,
                             EMANELTE::MHAL::MOD_QPSK,
                             0,
                             2);  // 2 bit to encode dfi

  for(int i=0; i<3; ++i)
    {
      const srsran_pcfich_t*   p1  = &q->pcfich;
      const srsran_regs_t*     p2  = p1->regs;
      const srsran_regs_ch_t*  rch = &p2->pcfich;
      const srsran_regs_reg_t* reg = rch->regs[i];

      uint32_t k0 = reg->k0;
      uint32_t l  = reg->l;
      const uint32_t * k = &reg->k[0];

      //srsran_regs_ch_t * pcfich = &((q->pcfich.regs)->pcfich);
      uint32_t rb = k0 / 12;
      Debug("TX:%s PCFICH cc=%u group i=%d on this subframe placed at resource starting at "
            "(l=%u, "
            "k0=%u, "
            "k[0]=%u "
            "k[1]=%u "
            "k[2]=%u "
            "k[3]=%u) in resource block=%u\n", __func__, cc_idx, i, l, k0, k[0], k[1], k[2], k[3], rb);

      channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, frequencyHz));
    }

  // Set side chain PSS, SSS and MIB information on appropriate subframes
  if(sf_idx == 0 || sf_idx == 5) 
    {
      // physical cell group and id derived from pci

      // set cyclical prefix mode
      carrier->mutable_pss_sss()->set_cp_mode(q->cell.cp == SRSRAN_CP_NORM ? 
                                              EMANELTE::MHAL::CP_NORM : 
                                              EMANELTE::MHAL::CP_EXTD);

      // MIB on first subframe
      if(sf_idx == 0)
       {
         auto pbch = carrier->mutable_pbch();

         auto channelMessage = downlink->mutable_pbch();

         initDownlinkChannelMessage(channelMessage,
                                    EMANELTE::MHAL::CHAN_PBCH,
                                    EMANELTE::MHAL::MOD_QPSK,
                                    0,
                                    40);  // MIB + 16 bit CRC

         // MIB occupies the middle 72 resource elements of the second slot of subframe 0, which
         // is the middle 6 or 7 resource blocks depending on nof_prb being even or odd.
         // Approximate this by sending a segment for each fullly occupied resource block,
         // So 5 blocks when num_prb is odd.
         int first_prb = q->cell.nof_prb / 2 - 3 + (q->cell.nof_prb % 2);

         int num_prb = q->cell.nof_prb % 2 ? 5 : 6;

         for(int i=0; i<num_prb; ++i)
           {
             channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(first_prb + i, frequencyHz));
           }

         switch(q->cell.phich_resources) 
          {
            case SRSRAN_PHICH_R_1_6:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE_SIXTH);
            break;

            case SRSRAN_PHICH_R_1_2:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE_HALF);
            break;

            case SRSRAN_PHICH_R_1:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE);
            break;

            case SRSRAN_PHICH_R_2:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_TWO);
            break;

            default:
             throw("MHAL:enb_dl_put_base: unhandled cell phich_resources type");
          }

         switch(q->cell.phich_length) 
          {
            case SRSRAN_PHICH_NORM:
               pbch->set_phich_length(EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_NORM);
            break;

            case SRSRAN_PHICH_EXT:
               pbch->set_phich_length(EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_EXTD);
            break;

            default:
             throw("MHAL:enb_dl_put_base: unhandled cell phich_length type");
          }

         pbch->set_num_prb(q->cell.nof_prb);

         pbch->set_num_antennas(q->cell.nof_ports);
      }
   }
}


// send msg to mhal
bool enb_dl_send_signal(time_t sot_sec, float frac_sec)
{
  bool result = false;

  EMANELTE::MHAL::Data data;

  if(enb_dl_msg_.SerializeToString(&data))
   {
     txControl_.set_reference_signal_power_milliwatt(pdsch_rs_power_milliwatt_);

     // align sot to sf time
     const timeval tv_sf_time = {sot_sec, (time_t)(round(frac_sec * 1e3)*1e3)};
     
     auto ts = txControl_.mutable_sf_time();
     ts->set_ts_sec(tv_sf_time.tv_sec);
     ts->set_ts_usec(tv_sf_time.tv_usec);

     txControl_.set_message_type(EMANELTE::MHAL::DOWNLINK);
     txControl_.set_tx_seqnum(tx_seqnum_++);
     txControl_.set_tti_tx(tti_tx_);

#if 0
     Info("TX:%s msg: %s\n", __func__, enb_dl_msg_.DebugString().c_str());
#endif

#if 0
     Info("TX:%s ctrl:%s\n", __func__, txControl_.DebugString().c_str());
#endif

     EMANELTE::MHAL::ENB::send_msg(data, txControl_);
   }
  else
   {
     Error("TX:%s SerializeToString ERROR len %zu\n", __func__, data.length());
   }

  dl_mutex_.unlock();

  return result;
}


// XXX TODO this needs review
// set the power scaling on a per rnti basis
void enb_dl_set_power_allocation(uint32_t tti, uint16_t rnti, float rho_a_db, float rho_b_db)
{
  const uint32_t sf_idx = (tti % 10);

  rho_a_db_map_[sf_idx].emplace(rnti, rho_a_db);

  Debug("MHAL:%s "
        "sf_idx %d, "
        "rnti 0x%hx, "
        "rho_a_db %0.2f\n",
        __func__,
        sf_idx,
        rnti,
        rho_a_db);
}

   

/* typedef struct SRSRAN_API {
  srsran_cell_t      cell;
  srsran_dl_sf_cfg_t dl_sf;
  srsran_pbch_t      pbch;
  srsran_pcfich_t    pcfich;
  srsran_regs_t      regs;
  srsran_pdcch_t     pdcch;
  srsran_pdsch_t     pdsch;
  srsran_pmch_t      pmch;
  srsran_phich_t     phich;
} srsran_enb_dl_t; 

typedef struct SRSRAN_API {
  uint16_t              rnti;
  srsran_dci_format_t   format;
  srsran_dci_location_t location;

  // Resource Allocation
  srsran_ra_type_t alloc_type;
  union {
    srsran_ra_type0_t type0_alloc;
    srsran_ra_type1_t type1_alloc;
    srsran_ra_type2_t type2_alloc;
  };

  // Codeword information
  srsran_dci_tb_t tb[SRSRAN_MAX_CODEWORDS];
  bool            tb_cw_swap;
  uint32_t        pinfo;

  // Power control
  bool    pconf;
  bool    power_offset;
  uint8_t tpc_pucch;

  // RA order
  bool     is_ra_order;
  uint32_t ra_preamble;
  uint32_t ra_mask_idx;

  // Release 10
  uint32_t cif;
  bool     cif_present;
  bool     srs_request;
  bool     srs_request_present;

  // Other parameters
  uint32_t pid;
  uint32_t dai;
  bool     is_tdd;
  bool     is_dwpts;
  bool     sram_id;
} srsran_dci_dl_t; */

// see lib/src/phy/enb/enb_dl.c 
// int srsran_enb_dl_put_pdcch_dl(srsran_enb_dl_t* q, srsran_dci_cfg_t* dci_cfg, srsran_dci_dl_t* dci_dl)
int enb_dl_put_pdcch_dl_i(srsran_enb_dl_t* q, 
                          srsran_dci_cfg_t* dci_cfg,
                          srsran_dci_dl_t* dci_dl, 
                          uint32_t ref,
                          uint32_t cc_idx)
{
  srsran_dci_msg_t dci_msg;
  bzero(&dci_msg, sizeof(dci_msg));

  if(srsran_dci_msg_pack_pdsch(&q->cell, &q->dl_sf, dci_cfg, dci_dl, &dci_msg) == SRSRAN_SUCCESS)
    {
      return enb_dl_put_dl_pdcch_i(q, &dci_msg, ref, 0, cc_idx); // DL
    }
  else
    {
      Error("PDCCH:%s error calling srsran_dci_msg_pack_pdsch(), ref %u\n", __func__, ref);

      return SRSRAN_ERROR;
    }
}

/*typedef struct SRSRAN_API {
  srsran_cell_t      cell;
  srsran_dl_sf_cfg_t dl_sf;
  srsran_pbch_t      pbch;
  srsran_pcfich_t    pcfich;
  srsran_regs_t      regs;
  srsran_pdcch_t     pdcch;
  srsran_pdsch_t     pdsch;
  srsran_pmch_t      pmch;
  srsran_phich_t     phich;
} srsran_enb_dl_t; 

 typedef struct SRSRAN_API {
  srsran_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srsran_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
  union {
    srsran_softbuffer_tx_t* tx[SRSRAN_MAX_CODEWORDS];
    srsran_softbuffer_rx_t* rx[SRSRAN_MAX_CODEWORDS];
  } softbuffers;
  bool     meas_time_en;
  uint32_t meas_time_value;
} srsran_pdsch_cfg_t;

 typedef struct {
   srsran_dci_dl_t         dci;
   uint8_t*                data[SRSRAN_MAX_TB];
   srsran_softbuffer_tx_t* softbuffer_tx[SRSRAN_MAX_TB];
 } dl_sched_grant_t; */
int enb_dl_cc_put_pdcch_dl(srsran_enb_dl_t* q, 
                           srsran_dci_cfg_t* dci_cfg,
                           mac_interface_phy_lte::dl_sched_grant_t* grant,
                           uint32_t ref,
                           uint32_t cc_idx)
{
  for(uint32_t tb = 0; tb < SRSRAN_MAX_TB; ++tb)
    {
      // check if data is ready
      if(grant->data[tb])
       {
         if(enb_dl_put_pdcch_dl_i(q, dci_cfg, &grant->dci, ref, cc_idx))
          {
             Error("PDCCH:%s cc %u, Error ref %u, tb %u, rnti 0x%hx\n", 
                   __func__, cc_idx, ref, tb, grant->dci.rnti);
          }
       }
   }

   return SRSRAN_SUCCESS;
}


/*typedef struct SRSRAN_API {
  srsran_cell_t      cell;
  srsran_dl_sf_cfg_t dl_sf;
  srsran_pbch_t      pbch;
  srsran_pcfich_t    pcfich;
  srsran_regs_t      regs;
  srsran_pdcch_t     pdcch;
  srsran_pdsch_t     pdsch;
  srsran_pmch_t      pmch;
  srsran_phich_t     phich;
} srsran_enb_dl_t; 

 typedef struct SRSRAN_API {
  srsran_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srsran_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
  union {
    srsran_softbuffer_tx_t* tx[SRSRAN_MAX_CODEWORDS];
    srsran_softbuffer_rx_t* rx[SRSRAN_MAX_CODEWORDS];
  } softbuffers;
  bool     meas_time_en;
  uint32_t meas_time_value;
} srsran_pdsch_cfg_t;

 typedef struct {
   srsran_dci_dl_t         dci;
   uint8_t*                data[SRSRAN_MAX_TB];
   srsran_softbuffer_tx_t* softbuffer_tx[SRSRAN_MAX_TB];
 } dl_sched_grant_t; */

int enb_dl_cc_put_pdsch_dl(srsran_enb_dl_t* q, 
                        srsran_pdsch_cfg_t* pdsch, 
                        mac_interface_phy_lte::dl_sched_grant_t* grant,
                        uint32_t ref,
                        uint32_t cc_idx)
{
  for(uint32_t tb = 0; tb < SRSRAN_MAX_TB; ++tb)
    {
      // check if data is ready
      if(grant->data[tb])
       {
         if(enb_dl_put_dl_pdsch_i(q, pdsch, grant->data[tb], ref, tb, cc_idx) != SRSRAN_SUCCESS)
           {
             Error("PDSCH:%s cc %u, Error ref %u, tb %u, rnti 0x%hx\n", 
                   __func__, cc_idx, ref, tb, grant->dci.rnti);
          }
      }
   }

   return SRSRAN_SUCCESS;
}



// see lib/src/phy/enb/enb_dl.c
// int srsran_enb_dl_put_pmch(srsran_enb_dl_t* q, srsran_pmch_cfg_t* pmch_cfg, uint8_t* data)
int enb_dl_cc_put_pmch(srsran_enb_dl_t* q, 
                       srsran_pmch_cfg_t* pmch_cfg, 
                       mac_interface_phy_lte::dl_sched_grant_t* dl_sched_grant,
                       uint32_t cc_idx)
{
  uint16_t rnti = pmch_cfg->pdsch_cfg.rnti;

  if(rnti == 0)
   {
     Warning("PMCH:%s cc %u, rnti %hu, set to 0xfffd\n", __func__, cc_idx, rnti);

     rnti = 0xfffd;
   }

  return enb_dl_put_pmch_i(q, pmch_cfg, dl_sched_grant->data[0], rnti, cc_idx);
}

// see lib/src/phy/enb/enb_dl.c
// int srsran_enb_dl_put_pdcch_ul(srsran_enb_dl_t* q, srsran_dci_cfg_t* dci_cfg, srsran_dci_ul_t* dci_ul)
int enb_dl_cc_put_pdcch_ul(srsran_enb_dl_t* q, 
                           srsran_dci_cfg_t* dci_cfg,
                           srsran_dci_ul_t* dci_ul,
                           uint32_t ref,
                           uint32_t cc_idx)
{
  srsran_dci_msg_t dci_msg;
  bzero(&dci_msg, sizeof(dci_msg));

  if(srsran_dci_msg_pack_pusch(&q->cell, &q->dl_sf, dci_cfg, dci_ul, &dci_msg) == SRSRAN_SUCCESS)
    {
      return enb_dl_put_dl_pdcch_i(q, &dci_msg, ref, 1, cc_idx); // UL
    }
  else
    {
      Error("PDCCH:%s cc %u, error calling srsran_dci_msg_pack_pdcch(), ref %u\n", __func__, cc_idx, ref);

      return SRSRAN_ERROR;
    }
}


/* typedef struct SRSRAN_API {
  srsran_cell_t      cell;
  srsran_dl_sf_cfg_t dl_sf;
  srsran_pbch_t      pbch;
  srsran_pcfich_t    pcfich;
  srsran_regs_t      regs;
  srsran_pdcch_t     pdcch;
  srsran_pdsch_t     pdsch;
  srsran_pmch_t      pmch;
  srsran_phich_t     phich;
} srsran_enb_dl_t; 

typedef struct SRSRAN_API {
  uint32_t k[4];
  uint32_t k0;
  uint32_t l;
  bool     assigned;
}srsran_regs_reg_t;

typedef struct SRSRAN_API {
  uint32_t          nof_regs;
  srsran_regs_reg_t **regs;
}srsran_regs_ch_t;

typedef struct SRSRAN_API {
  srsran_cell_t cell;
  uint32_t      max_ctrl_symbols;
  uint32_t      ngroups_phich;
  uint32_t      ngroups_phich_m1;

  srsran_phich_r_t      phich_res;
  srsran_phich_length_t phich_len;
  
  srsran_regs_ch_t pcfich;
  srsran_regs_ch_t *phich;   // there are several phich
  srsran_regs_ch_t pdcch[3]; // PDCCH indexing, permutation and interleaving is computed for
                             // the three possible CFI value

  uint32_t           phich_mi;
  uint32_t           nof_regs;
  srsran_regs_reg_t *regs;
}srsran_regs_t;

typedef struct SRSRAN_API {
  srsran_cell_t  cell;
  uint32_t       nof_rx_antennas;
  srsran_regs_t* regs;

  // bit message 
  uint8_t data[SRSRAN_PHICH_NBITS];
  float data_rx[SRSRAN_PHICH_NBITS];

  // tx & rx objects
  srsran_modem_table_t mod;
  srsran_sequence_t    seq[SRSRAN_NOF_SF_X_FRAME];
} srsran_phich_t;

typedef struct SRSRAN_API {
  uint32_t ngroup;
  uint32_t nseq;
} srsran_phich_resource_t;

typedef struct SRSRAN_API {
  uint32_t n_prb_lowest;
  uint32_t n_dmrs;
  uint32_t I_phich;
} srsran_phich_grant_t;

 typedef struct {
    uint16_t rnti;
    bool     ack;
  } ul_sched_ack_t; */

// see lib/src/phy/enb/enb_dl.c
int enb_dl_cc_put_phich(srsran_enb_dl_t* q,
                        srsran_phich_grant_t* grant,
                        mac_interface_phy_lte::ul_sched_ack_t * ack,
                        uint32_t cc_idx)
{
  srsran_phich_resource_t resource;
  bzero(&resource, sizeof(resource));

  srsran_phich_calc(&q->phich, grant, &resource);

  const auto frequencyHz = getTxFrequency(cc_idx);

  auto carrier = getCarrierByFrequency<EMANELTE::MHAL::ENB_DL_Message_CarrierMessage,
                                       EMANELTE::MHAL::ENB_DL_Message>(enb_dl_msg_, frequencyHz);

  auto phich = carrier->mutable_phich();

  phich->set_rnti(ack->rnti);
  phich->set_ack(ack->ack);
  phich->set_num_prb_low(grant->n_prb_lowest);
  phich->set_num_dmrs(grant->n_dmrs);

  auto controlCarrier = getCarrierByFrequency<EMANELTE::MHAL::TxControlCarrierMessage, 
                                              EMANELTE::MHAL::TxControlMessage>(txControl_, frequencyHz);

  auto channelMessage = controlCarrier->mutable_downlink()->add_phich();

  initDownlinkChannelMessage(channelMessage,
                             EMANELTE::MHAL::CHAN_PHICH,
                             EMANELTE::MHAL::MOD_BPSK,
                             ack->rnti,
                             3);  // phich is 000 for nak, 
                                  // 111 for ack. each bit is BPSK modulated 
                                  // to a symbol, and each symbol spread to 4 REs (12 REs total)

   const auto regs = q->phich.regs;

   if (SRSRAN_CP_ISEXT(regs->cell.cp)) {
     resource.ngroup /= 2;
   }

   const auto & rch = regs->phich[resource.ngroup];

   // nof_regs is 3 for phich groups (12 REs total per group).
   // l should always be 0 for Normal PHICH duration and [0,2] for Extended
   for (uint32_t i = 0; i < rch.nof_regs; i++) {
     uint32_t rb = rch.regs[i]->k0 / 12;

     channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, frequencyHz));
   }

   Debug("PHICH:%s cc %u, rnti 0x%hx, ack %d, n_prb_L %d, n_dmrs %d\n", 
        __func__,
        cc_idx,
        ack->rnti,
        ack->ack,
        grant->n_prb_lowest,
        grant->n_dmrs);

   return SRSRAN_SUCCESS;
}


bool enb_ul_get_signal(uint32_t tti, srsran_timestamp_t * ts)
{
  std::lock_guard<std::mutex> lock(ul_mutex_);

  curr_tti_ = tti;

  EMANELTE::MHAL::ENB::set_tti(tti);

  // clear any old testers
  for(auto & ulMessage : ulMessages_)
   {
     UL_Message_SINRTester(ulMessage).release();
   }

  ulMessages_.clear();

  EMANELTE::MHAL::RxMessages rxMessages;

  struct timeval tv_tti;

  EMANELTE::MHAL::ENB::get_messages(rxMessages, tv_tti);

  // set rx time 
  ts->full_secs = tv_tti.tv_sec;
  ts->frac_secs = tv_tti.tv_usec/1e6;

  // for each rx msg
  for(const auto & rxMessage : rxMessages)
   {
     EMANELTE::MHAL::UE_UL_Message ue_ul_msg;

     if(ue_ul_msg.ParseFromString(RxMessage_Data(rxMessage)))
      {
        const auto & rxControl = RxMessage_RxControl(rxMessage);

        EMANELTE::MHAL::SINRTester sinrTester{RxMessage_SINRTesters(rxMessage)};

        bool bPciMatch = false;

        for(auto & carrier : ue_ul_msg.carriers())
         {
           if(pciTable_.count(carrier.phy_cell_id()))
            {
              bPciMatch = true;

              break; // need min of 1 match
            }
         }

        if(bPciMatch)
         {
           Debug("RX:%s sf_time %ld:%06ld, seqnum %lu, rnti 0x%hx, tti %u, cariers %d\n",
                  __func__,
                  rxControl.sf_time_.tv_sec,
                  rxControl.sf_time_.tv_usec,
                  rxControl.rx_seqnum_,
                  ue_ul_msg.crnti(),
                  ue_ul_msg.tti(),
                  ue_ul_msg.carriers().size());

           ulMessages_.emplace_back(ue_ul_msg, rxControl, sinrTester);
         }
        else
         {
           Warning("RX:%s no matching pci's, in %d carriers, drop\n",
                  __func__,
                  ue_ul_msg.carriers().size());

           sinrTester.release();
         }
      }
    else
      {
        Error("RX:%s ParseFromString ERROR\n", __func__);
      }
   }

  return (! ulMessages_.empty());
}


int enb_ul_cc_get_prach(uint32_t * indices, 
                        float * offsets, 
                        float * p2avg,
                        uint32_t max_entries,
                        uint32_t & num_entries,
                        uint32_t cc_idx)
{
  std::lock_guard<std::mutex> lock(ul_mutex_);

  int result = SRSRAN_SUCCESS;

  num_entries = 0;

  std::set<uint32_t> unique;

  for(const auto & ulMessage : ulMessages_)
    {
      if(num_entries >= max_entries)
       {
         break;
       }

      const auto carrierResult = findCarrierByIndex(UL_Message_Message(ulMessage), cc_idx);

      if(CarrierResult_Found(carrierResult))
       {
         const auto & carrier = CarrierResult_Carrier(carrierResult);

         if(carrier.has_prach())
          {
            const auto sinrResult = 
              UL_Message_SINRTester(ulMessage).sinrCheck2(EMANELTE::MHAL::CHAN_PRACH,
                                                          carrier.frequency_hz());

            if(! sinrResult.bPassed_)
             {
               Warning("PRACH:%s: cc %u, fail snr, %f\n", __func__, cc_idx, sinrResult.sinr_dB_);
               continue;
             }

            const auto & prach    = carrier.prach();
            const auto & preamble = prach.preamble();

            // check for unique
            if(unique.count(preamble.index()) == 0)
             {
              unique.insert(preamble.index());

              indices[num_entries] = preamble.index();

              // timing offset estimation not currently implemented
              offsets[num_entries] = 0.0;
              p2avg[num_entries]   = 0.0;

              ++num_entries;

              Warning("PRACH:%s cc %u, entry[%u], accept index %d\n",
                      __func__, cc_idx, num_entries, preamble.index());
            }
          else
           {
             Info("PRACH:%s entry[%u], ignore duplicate index %d\n",
                  __func__, num_entries, preamble.index());
          }
        }
       else
        {
          Debug("PRACH:%s no preambles\n", __func__);
        }
      }
    }

  return result;
}


/*
typedef struct SRSRAN_API {
  srsran_cell_t         cell;
  cf_t*                 sf_symbols;
  srsran_chest_ul_res_t chest_res;
  srsran_ofdm_t         fft;
  srsran_chest_ul_t     chest;
  srsran_pusch_t        pusch;
  srsran_pucch_t        pucch;
} srsran_enb_ul_t;

typedef struct SRSRAN_API {
  srsran_cell_t          cell;
  srsran_modem_table_t   mod;
  srsran_uci_cqi_pucch_t cqi;
  srsran_pucch_user_t**  users;
  srsran_sequence_t      tmp_seq;
  uint16_t               ue_rnti;
  bool                   is_ue;

  uint8_t  bits_scram[SRSRAN_PUCCH_MAX_BITS];
  cf_t     d[SRSRAN_PUCCH_MAX_BITS / 2];
  uint32_t n_cs_cell[SRSRAN_NSLOTS_X_FRAME][SRSRAN_CP_NORM_NSYMB];
  uint32_t f_gh[SRSRAN_NSLOTS_X_FRAME];
  float    tmp_arg[SRSRAN_PUCCH_N_SEQ];
} srsran_pucch_t;

typedef struct SRSRAN_API {
  srsran_tdd_config_t tdd_config;
  uint32_t            tti;
  bool                shortened;
} srsran_ul_sf_cfg_t;

typedef struct SRSRAN_API {
  // Input configuration for this subframe
  uint16_t rnti;

  // UCI configuration
  srsran_uci_cfg_t uci_cfg;

  // Common configuration
  uint32_t delta_pucch_shift;
  uint32_t n_rb_2;
  uint32_t N_cs;
  uint32_t N_pucch_1;
  bool     group_hopping_en; // common pusch config

  // Dedicated PUCCH configuration
  uint32_t I_sr;
  bool     sr_configured;
  uint32_t n_pucch_1[4]; // 4 n_pucch resources specified by RRC
  uint32_t n_pucch_2;
  uint32_t n_pucch_sr;
  bool     simul_cqi_ack;
  bool     tdd_ack_bundle; // if false, multiplex
  bool     sps_enabled;
  uint32_t tpc_for_pucch;

  // Release 10 CA specific
  srsran_ack_nack_feedback_mode_t ack_nack_feedback_mode;
  uint32_t                        n1_pucch_an_cs[SRSRAN_PUCCH_SIZE_AN_CS][SRSRAN_PUCCH_NOF_AN_CS];
  uint32_t                        n3_pucch_an_list[SRSRAN_PUCCH_SIZE_AN_CS];

  // Other configuration
  float threshold_format1;
  float threshold_data_valid_format1a;
  float threshold_data_valid_format2;

  // PUCCH configuration generated during a call to encode/decode
  srsran_pucch_format_t format;
  uint32_t              n_pucch;
  uint8_t               pucch2_drs_bits[SRSRAN_PUCCH2_MAX_DMRS_BITS];
} srsran_pucch_cfg_t;

typedef struct SRSRAN_API {
  uint8_t ack_value[SRSRAN_UCI_MAX_ACK_BITS];
  bool    valid;
} srsran_uci_value_ack_t;

 typedef struct SRSRAN_API {
  bool     pending_tb[SRSRAN_MAX_CODEWORDS]; //< Indicates whether there was a grant that requires an ACK/NACK
  uint32_t nof_acks;                         //< Number of transport blocks, deduced from transmission mode
  uint32_t ncce[SRSRAN_UCI_MAX_M];
  uint32_t N_bundle;
  uint32_t tdd_ack_M;
  uint32_t tdd_ack_m;
  bool     tdd_is_multiplex;
  uint32_t tpc_for_pucch;
  uint32_t grant_cc_idx;
} srsran_uci_cfg_ack_t;
   
typedef struct SRSRAN_API {
  srsran_uci_cfg_ack_t ack;
  srsran_cqi_cfg_t     cqi;
  bool                 is_scheduling_request_tti;
} srsran_uci_cfg_t;

typedef struct SRSRAN_API {
  bool                   scheduling_request;
  srsran_cqi_value_t     cqi;
  srsran_uci_value_ack_t ack;
  uint8_t                ri; // Only 1-bit supported for RI
} srsran_uci_value_t;
    
typedef struct SRSRAN_API {
  srsran_uci_value_t uci_data;
  float              dmrs_correlation;
  float              correlation;
  bool               detected;
} srsran_pucch_res_t; */

// see lib/src/phy/enb/enb_ul.c
/* int srsran_enb_ul_get_pucch(srsran_enb_ul_t*    q,
                               srsran_ul_sf_cfg_t* ul_sf,
                               srsran_pucch_cfg_t* cfg,
                               srsran_pucch_res_t* res)
*/

int enb_ul_cc_get_pucch(srsran_enb_ul_t*    q,
                        srsran_ul_sf_cfg_t* ul_sf,
                        srsran_pucch_cfg_t* cfg,
                        srsran_pucch_res_t* res,
                        uint32_t cc_idx)
{
  std::lock_guard<std::mutex> lock(ul_mutex_);

  // see lib/src/phy/enb/enb_ul.c get_pucch()
  if (!srsran_pucch_cfg_isvalid(cfg, q->cell.nof_prb)) {
    Error("PUCCH %s, Invalid PUCCH configuration\n", __func__);
    return -1;
  }

  int                ret                               = SRSRAN_SUCCESS;
  uint32_t           n_pucch_i[SRSRAN_PUCCH_MAX_ALLOC] = {};
  srsran_pucch_res_t pucch_res                         = {};
  uint32_t           uci_cfg_total_ack                 = srsran_uci_cfg_total_ack(&cfg->uci_cfg);

  // Drop CQI if there is collision with ACK
  if (!cfg->simul_cqi_ack && uci_cfg_total_ack > 0 && cfg->uci_cfg.cqi.data_enable) {
    cfg->uci_cfg.cqi.data_enable = false;
  }

  // Select format
  cfg->format = srsran_pucch_proc_select_format(&q->cell, cfg, &cfg->uci_cfg, NULL);
  if (cfg->format == SRSRAN_PUCCH_FORMAT_ERROR) {
    ERROR("Returned Error while selecting PUCCH format\n");
    return SRSRAN_ERROR;
  }

  // Get possible resources
  int nof_resources = srsran_pucch_proc_get_resources(&q->cell, cfg, &cfg->uci_cfg, NULL, n_pucch_i);
  if (nof_resources < 1 || nof_resources > SRSRAN_PUCCH_CS_MAX_ACK) {
    ERROR("No PUCCH resource could be calculated (%d)\n", nof_resources);
    return SRSRAN_ERROR;
  }

  // see lib/src/phy/enb/enb_ul.c get_pucch()
  // and lib/src/phy/ue/test/pucch_resource_test.c
  // this is needed to set cfg->format
  srsran_uci_value_t uci_data;
  ZERO_OBJECT(uci_data);

  // see lib/src/phy/phch/pucch_proc.c <<srsran_pucch_proc_get_npucch>>
  uint8_t b[SRSRAN_UCI_MAX_ACK_BITS] = {};

#if 0
  // Prepare configuration
  srsran_ue_ul_pucch_resource_selection(&q->cell, cfg, &cfg->uci_cfg, &uci_data, b);
#endif

  const auto rnti = cfg->rnti;

  res->dmrs_correlation = 1.0;
  res->correlation      = 1.0;
  res->detected         = false;

  // for each ue uplink message
  for(const auto & ulMessage : ulMessages_)
   {
     if(res->detected)
      {
        break;
      } 

     const auto carrierResult = findCarrierByIndex(UL_Message_Message(ulMessage), cc_idx);

     if(CarrierResult_Found(carrierResult))
      {
        const auto & carrier = CarrierResult_Carrier(carrierResult);

        if(carrier.has_pucch())
         {
           const auto & pucch_message = carrier.pucch();

           // for each grant
           for(const auto & grant : pucch_message.grant())
            {
              Debug("PUCCH:%s cc %u, sr %d, acks %d, ul_rnti 0x%hx vs rnti 0x%hx, %d grants\n", 
                   __func__,
                   cc_idx,
                   cfg->uci_cfg.is_scheduling_request_tti,
                   srsran_uci_cfg_total_ack(&cfg->uci_cfg),
                   grant.rnti(), rnti, pucch_message.grant_size());

              std::string format;

              if(grant.rnti() == rnti)
               {
                 const auto sinrResult = 
                   UL_Message_SINRTester(ulMessage).sinrCheck2(EMANELTE::MHAL::CHAN_PUCCH, 
                                                               rnti, 
                                                               carrier.frequency_hz());

                 if(sinrResult.bPassed_)
                  {
                    const auto & uci_message = grant.uci();

                    memcpy(&res->uci_data, uci_message.data(), uci_message.length());

                    res->detected = true;

                    q->chest_res.snr_db             = sinrResult.sinr_dB_;
                    q->chest_res.noise_estimate_dbm = sinrResult.noiseFloor_dBm_;

                    // from lib/src/phy/phch/pucch.c srsran_pucch_decode()
                    switch (cfg->format) {
                     case SRSRAN_PUCCH_FORMAT_1A:
                     case SRSRAN_PUCCH_FORMAT_1B:
                       res->uci_data.ack.valid = true;
                       format = "1A/1B";
                     break;

                     case SRSRAN_PUCCH_FORMAT_2:
                     case SRSRAN_PUCCH_FORMAT_2A:
                     case SRSRAN_PUCCH_FORMAT_2B:
                       res->uci_data.ack.valid    = true;
                       res->uci_data.cqi.data_crc = true;
                       format = "2";
                     break;

                     case SRSRAN_PUCCH_FORMAT_1:
                     case SRSRAN_PUCCH_FORMAT_3:
                       format = "1/3";
                     break;

                     default:
                       format = "default";
                    }

#ifdef DEBUG_HEX
                    InfoHex(uci_message.data(), uci_message.length(),
                            "PUCCH:%s cc %u, found pucch format %s, rnti %hx, corr %f\n",
                            __func__, cc_idx, format.c_str(), rnti, res->correlation);
#endif

                    // pass
                    ENBSTATS::getPUCCH(rnti, true);
                  }
                 else
                  {
                    Warning("PUCCH:%s: cc %u, fail snr, rnti %hu, %f\n", 
                            __func__, cc_idx, rnti, sinrResult.sinr_dB_);

                    q->chest_res.snr_db             = sinrResult.sinr_dB_;
                    q->chest_res.noise_estimate_dbm = sinrResult.noiseFloor_dBm_;

                    // PUCCH failed snr, ignore
                    ENBSTATS::getPUCCH(rnti, false);
                  }

                 // done with this rnti
                 break;
                }
             }
          }
       }
    }

  return SRSRAN_SUCCESS;
}

/*

typedef struct SRSRAN_API {
  srsran_cell_t         cell;
  cf_t*                 sf_symbols;
  srsran_chest_ul_res_t chest_res;
  srsran_ofdm_t         fft;
  srsran_chest_ul_t     chest;
  srsran_pusch_t        pusch;
  srsran_pucch_t        pucch;
} srsran_enb_ul_t;

typedef struct SRSRAN_API {
  cf_t*    ce;
  uint32_t nof_re;
  float    noise_estimate;
  float    noise_estimate_dbm;
  float    snr;
  float    snr_db;
  float    cfo;
} srsran_chest_ul_res_t;

typedef struct SRSRAN_API {
  srsran_tdd_config_t tdd_config;
  uint32_t            tti;
  bool                shortened;
} srsran_ul_sf_cfg_t

typedef struct SRSRAN_API {
  uint16_t                rnti;
  srsran_uci_cfg_t        uci_cfg;
  srsran_uci_offset_cfg_t uci_offset;
  srsran_pusch_grant_t    grant;
  uint32_t                max_nof_iterations;
  uint32_t                last_O_cqi;
  uint32_t                K_segm;
  uint32_t                current_tx_nb;
  bool                    csi_enable;
  bool                    enable_64qam;
  union {
    srsran_softbuffer_tx_t* tx;
    srsran_softbuffer_rx_t* rx;
  } softbuffers;
  bool     meas_time_en;
  uint32_t meas_time_value;
} srsran_pusch_cfg_t;

typedef struct SRSRAN_API {
  bool                   scheduling_request;
  srsran_cqi_value_t     cqi;
  srsran_uci_value_ack_t ack;
  uint8_t                ri; // Only 1-bit supported for RI
} srsran_uci_value_t;
 
typedef struct SRSRAN_API {
  uint8_t*           data;
  srsran_uci_value_t uci;
  bool               crc;
  float              avg_iterations_block;
} srsran_pusch_res_t;
*/

int enb_ul_cc_get_pusch(srsran_enb_ul_t*    q,
                        srsran_ul_sf_cfg_t* ul_sf,
                        srsran_pusch_cfg_t* cfg,
                        srsran_pusch_res_t* res,
                        uint16_t rnti,
                        uint32_t cc_idx)
{
  std::lock_guard<std::mutex> lock(ul_mutex_);

  int result = SRSRAN_SUCCESS;

  res->crc            = false;
  res->uci.ack.valid  = false;

  // for each uplink message
  for(const auto & ulMessage : ulMessages_)
   {
     if(res->crc)
      {
        break;
      }

     const auto carrierResult = findCarrierByIndex(UL_Message_Message(ulMessage), cc_idx);
 
     if(CarrierResult_Found(carrierResult))
      {
        const auto & carrier = CarrierResult_Carrier(carrierResult);

        if(carrier.has_pusch())
         {
           const auto & pusch_message = carrier.pusch();

           // for each grant
           for(const auto & grant : pusch_message.grant())
            {
              Debug("PUSCH:%s cc %u, check ul_rnti 0x%hx vs rnti 0x%hx, %d grants\n",
                   __func__, cc_idx, grant.rnti(), rnti, pusch_message.grant_size());

              if(grant.rnti() == rnti)
               {
                 const auto sinrResult = 
                   UL_Message_SINRTester(ulMessage).sinrCheck2(EMANELTE::MHAL::CHAN_PUSCH,
                                                                  rnti,
                                                                  carrier.frequency_hz());
                 if(sinrResult.bPassed_)
                  {
                    const auto & ul_grant_message = grant.ul_grant();
                    const auto & uci_message      = grant.uci();
                    const auto & payload          = grant.payload();

                    // srsran_pusch_grant_t  
                    memcpy(&cfg->grant, ul_grant_message.data(), ul_grant_message.length());

                    // srsran_uci_value_t
                    memcpy(&res->uci, uci_message.data(), uci_message.length());

                    // payload
                    memcpy(res->data, payload.data(), payload.length());

                    // see lib/src/phy/phch/pusch.c srsran_pusch_decode()
                    res->avg_iterations_block = 1;
                    res->crc                  = true;
                    res->uci.ack.valid        = true;

                    q->chest_res.snr_db             = sinrResult.sinr_dB_;
                    q->chest_res.noise_estimate_dbm = sinrResult.noiseFloor_dBm_;
 
#ifdef DEBUG_HEX
                    InfoHex(payload.data(), payload.length(),
                            "PUSCH:%s cc %u, rnti %hx, snr_db %f\n",
                            __func__, cc_idx, rnti, q->chest_res.snr_db);
#endif

                    // pass
                    ENBSTATS::getPUSCH(rnti, true);
                  }
                else
                  {
                    Warning("PUSCH:%s: cc %u, fail snr, rnti %hu, %f\n", 
                            __func__, cc_idx, rnti, sinrResult.sinr_dB_);

                    q->chest_res.snr_db             = sinrResult.sinr_dB_;
                    q->chest_res.noise_estimate_dbm = sinrResult.noiseFloor_dBm_;

                    // PUSCH failed snr, ignore
                    ENBSTATS::getPUSCH(rnti, false);
                  }

                // done with this rnti
                break;
              }
            }
         }
      }
   }

  return result;
}


} // end namespace phy_adapter
} // end namespace srsenb
