/**
 *
 * \section COPYRIGHT
 *
 * Copyright (c) 2019 - Adjacent Link LLC, Bridgewater, New Jersey
 *
 * \section LICENSE
 *
 * This file is part of srsLTE.
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

#include <stdint.h> // radio.h needs this

#include "srsran/config.h"
#include "srsran/radio/radio.h"
#include "srsran/srslog/srslog.h"
#include "srsran/phy/adapter/phy_adapter_common.h"

#include "srsue/hdr/phy/phy_adapter.h"
#include "srsue/hdr/phy/sync.h"

#include "libemanelte/enbotamessage.pb.h"
#include "libemanelte/ueotamessage.pb.h"
#include "libemanelte/mhalue.h"
#include "libemanelte/uestatisticmanager.h"
#include "libemanelte/sinrtester.h"

#include <mutex>

// private namespace for misc helpers and state for PHY_ADAPTER
namespace {
 // uplink
 EMANELTE::MHAL::UE_UL_Message     ue_ul_msg_;
 EMANELTE::MHAL::TxControlMessage  txControl_;

 // enb dl msg, rx control info and sinr tester impls
 using DL_Message = std::tuple<EMANELTE::MHAL::ENB_DL_Message, 
                               EMANELTE::MHAL::RxControl,
                               EMANELTE::MHAL::SINRTester>;
 // helpers
 #define DL_Message_Message(x)     std::get<0>((x))
 #define DL_Message_RxControl(x)   std::get<1>((x))
 #define DL_Message_SINRTester(x)  std::get<2>((x))

 // vector of dl signals from each enb
 using DL_Messages = std::vector<DL_Message>;

 // time stamp and mhal rx messages for a frame
 using FrameSignals = std::tuple<bool, struct timeval, EMANELTE::MHAL::RxMessages>;
 // helpers
 #define FrameMessage_isSet(x)       std::get<0>((x))
 #define FrameMessage_timestamp(x)   std::get<1>((x))
 #define FrameMessage_rxMessages(x)  std::get<2>((x))

 // search for carrier result
 using CarrierResult = std::pair<bool, const EMANELTE::MHAL::ENB_DL_Message_CarrierMessage &>;
 // helpers
#define CarrierResult_Found(x)   std::get<0>((x))
#define CarrierResult_Carrier(x) std::get<1>((x))

 // track carrierIndex to rx/tx carrier center frequency
 CarrierIndexFrequencyTable carrierIndexFrequencyTable_;

 FrequencyToCarrierIndex rxFrequencyToCarrierIndex_;

 // for use into srsran lib calls
 srsran::rf_buffer_t buffer_(1);

 // all rx messages for this frame
 FrameSignals frameSignals_{false, {0,0}, {}};

 // dl message for this frame
 DL_Message dlMessageThisFrame_{{},{},{{}}};

 uint32_t my_cell_id_ = 0;

 srslog::basic_logger * logger_phy = nullptr;

 struct SignalQuality {
  double sinr_dB_;
  double noiseFloor_dBm_;

  SignalQuality(double sinr, double noiseFloor) :
   sinr_dB_(sinr),
   noiseFloor_dBm_(noiseFloor)
  { }
 };


 class SINRManager {
  public:
    // 1000 frames or 1 sec
    SINRManager(size_t maxEntries = 1000) :
     maxEntries_(maxEntries)
    { 
      sum_ = 0;
    }
 
    void clear()
     {
       std::lock_guard<std::mutex> lock(mutex_);

       entries_.clear();
       sum_ = 0;
     }

    void update(double x)
     {
       std::lock_guard<std::mutex> lock(mutex_);

       sum_ += x;

       entries_.push_front(x);

       if(entries_.size() > maxEntries_)
        {
          sum_ -= entries_.back();

          entries_.pop_back();
        }
     }

    double get()
     {
       std::lock_guard<std::mutex> lock(mutex_);

       if(entries_.empty())
        {
          return -150;
        }
       else
        {
          return sum_/entries_.size();
        }
     }

   private:
    const size_t maxEntries_;

    std::deque<double> entries_;

    double sum_;

    std::mutex mutex_;
 };
   

 SINRManager sinrManager_[MAX_NUM_CARRIERS];
  

 // pdsch rnti/messages with signal quality
 using ENB_DL_Message_PDSCH_Entry = std::pair<EMANELTE::MHAL::ENB_DL_Message_PDSCH_Data, SignalQuality>;

 // rnti, entry
 using ENB_DL_PDSCH_MESSAGES = std::map<uint16_t, ENB_DL_Message_PDSCH_Entry>;

 ENB_DL_PDSCH_MESSAGES enb_dl_pdsch_messages_;

 uint64_t tx_seqnum_         = 0;
 uint16_t crnti_             = 0;
 uint32_t earfcn_            = 0;
 uint32_t tti_tx_            = 0;
 uint32_t prach_freq_offset_ = 0;
 srsue::sync * sync_         = nullptr;

 std::mutex ul_mutex_;

 static inline bool is_valid_n_id_2(int n_id_2)
  {
    return(n_id_2 >= 0 && n_id_2 < 3);
  }

 srsran_dci_format_t get_msg_format(EMANELTE::MHAL::DCI_FORMAT format)
  {
   switch(format)
    {
     case EMANELTE::MHAL::DCI_FORMAT_0:
       return  SRSRAN_DCI_FORMAT0;

     case EMANELTE::MHAL::DCI_FORMAT_1:
       return  SRSRAN_DCI_FORMAT1;
 
     case EMANELTE::MHAL::DCI_FORMAT_1A:
       return  SRSRAN_DCI_FORMAT1A;
 
     case EMANELTE::MHAL::DCI_FORMAT_1B:
       return  SRSRAN_DCI_FORMAT1B;

     case EMANELTE::MHAL::DCI_FORMAT_1C:
       return  SRSRAN_DCI_FORMAT1C;

     case EMANELTE::MHAL::DCI_FORMAT_1D:
       return  SRSRAN_DCI_FORMAT1D;

     case EMANELTE::MHAL::DCI_FORMAT_2:
       return  SRSRAN_DCI_FORMAT2;

     case EMANELTE::MHAL::DCI_FORMAT_2A:
       return  SRSRAN_DCI_FORMAT2A;

     case EMANELTE::MHAL::DCI_FORMAT_2B:
       return  SRSRAN_DCI_FORMAT2B;

     default:
       return SRSRAN_DCI_NOF_FORMATS;
    }
  }


  void initUplinkChannelMessage(EMANELTE::MHAL::ChannelMessage * channelMessage,
                                EMANELTE::MHAL::CHANNEL_TYPE ctype,
                                EMANELTE::MHAL::MOD_TYPE modType,
                                uint32_t infoBits,
                                float txPowerScaledB=0.0)
  {
    channelMessage->set_channel_type(ctype);
    channelMessage->set_modulation_type(modType);
    channelMessage->set_number_of_bits(infoBits);
    channelMessage->set_tx_power_scale_db(txPowerScaledB);
  }

  inline int bits_to_bytes(int bits) { return bits/8; }

}

namespace srsue {
namespace phy_adapter {

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


typedef std::vector<EMANELTE::MHAL::ENB_DL_Message_PDCCH_DL_DCI> DL_DCI_Results;

// message, sinr
typedef std::pair<EMANELTE::MHAL::ENB_DL_Message_PDCCH_UL_DCI, SignalQuality> UL_DCI_Result;

typedef std::vector<UL_DCI_Result> UL_DCI_Results;


// message, sinr
typedef std::pair<EMANELTE::MHAL::ENB_DL_Message_PDSCH_Data, SignalQuality> PDSCH_Result;

typedef std::vector<PDSCH_Result> PDSCH_Results;


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

// lookup carrier that matches the frequency associated with the cc_idx
static CarrierResult
findCarrierByIndex(const EMANELTE::MHAL::ENB_DL_Message & enb_dl_msg, uint32_t cc_idx)
 {
  const auto iter = carrierIndexFrequencyTable_.find(cc_idx);

   if(iter != carrierIndexFrequencyTable_.end())
    {
      for(const auto & carrier : enb_dl_msg.carriers())
       {
         // match our rx freq to the msg carrier tx center freq
         if(iter->second.first == carrier.frequency_hz())
          {
            // XXX_CC TODO check this
            if(my_cell_id_ != carrier.phy_cell_id())
             {
               Info("%s: cc %u, found, but my_cell_id %u != carrier cell id %u\n",
                     __func__, cc_idx, my_cell_id_, carrier.phy_cell_id());
             }

            return CarrierResult{true, carrier};
          }
       }
    }
  
  return CarrierResult{false, EMANELTE::MHAL::ENB_DL_Message_CarrierMessage{}};
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


// see sf_worker::update_measurements() -> cc_worker::update_measurements()
static void ue_dl_update_chest_i(srsran_chest_dl_res_t * chest_res, float snr_db, float noise_db)
{
    //  from faux_rf
    //  cc_idx avg_noise 0.000, acg_rsrp_dbm -20.313, avg_rsrq_db -3.519, avg_rssi_dbm 2.536, pathloss 20.313, avg_snr_db_cqi  141.119
    chest_res->sync_error         = 0;
    chest_res->snr_db             = snr_db;
    chest_res->noise_estimate_dbm = noise_db;
    chest_res->noise_estimate     = noise_db;
}

// get all ota messages (all enb dl messages and rx control info)
static DL_Messages ue_dl_get_signals_i(srsran_timestamp_t * ts)
{
  if(! FrameMessage_isSet(frameSignals_))
   {
      Error("No Messages:%s: \n", __func__);

      return DL_Messages{};
   }

  if(ts)
   {
     ts->full_secs = FrameMessage_timestamp(frameSignals_).tv_sec;
     ts->frac_secs = FrameMessage_timestamp(frameSignals_).tv_usec / 1e6;
   }

  // all signals from all enb(s)
  DL_Messages dlMessages;

  // check for unique pci, we can handle only unique pci's
  std::set<uint32_t> unique;

  // for each rx message
  for(const auto & rxMessage : FrameMessage_rxMessages(frameSignals_))
   {
     bool bEnbIsUnique = true;

     EMANELTE::MHAL::ENB_DL_Message enb_dl_msg;

     if(enb_dl_msg.ParseFromString(RxMessage_Data(rxMessage)))
      {
       const auto & rxControl = RxMessage_RxControl(rxMessage);

       EMANELTE::MHAL::SINRTester sinrTester{RxMessage_SINRTesters(rxMessage)};

       // check each carrier for this enb
       for(const auto & carrier : enb_dl_msg.carriers())
        {
          const uint32_t & pci = carrier.phy_cell_id();

          if(! unique.insert(pci).second)
           {
             Info("RX:%s carrier %lu Hz, rx_seq %lu, duplicate pci %u, drop\n",
                   __func__,
                   carrier.frequency_hz(),
                   rxControl.rx_seqnum_,
                   pci);

             bEnbIsUnique = false;

             sinrTester.release();

             break; // done with this enb
           }
        }

       // unique enb, first come first served
       if(bEnbIsUnique)
        {
          // update signal quality for this enb
          // XXX TODO track ref signals only or avg snr?
          for(const auto & carrier : enb_dl_msg.carriers())
           {
             const auto iter = rxFrequencyToCarrierIndex_.find(carrier.frequency_hz());

             if(iter != rxFrequencyToCarrierIndex_.end())
              {
                sinrManager_[iter->second].update(rxControl.avg_snr_[iter->second]);
              }
           }

          // save msg compenents
          dlMessages.emplace_back(enb_dl_msg, rxControl, sinrTester);
        }
      }
     else
      {
        Error("MHAL:%s ParseFromString ERROR\n", __func__);
      }
   }

  return (dlMessages);
}


// return message for a specific pci (enb)
static DL_Messages ue_dl_enb_subframe_get_pci_i(srsran_ue_sync_t * ue_sync, const uint32_t * tti)
{
   const auto dlMessages = ue_dl_get_signals_i(&ue_sync->last_timestamp);

   for(auto & dlMessage : dlMessages)
    {
      const auto & enb_dl_msg = DL_Message_Message(dlMessage);

      for(const auto & carrier : enb_dl_msg.carriers())
       {
         const auto pci = carrier.phy_cell_id();

         if(pci == ue_sync->cell.id)
          {
            const uint32_t sf_idx = tti ? (*tti) % 10 : 0;

            ue_sync->sf_idx        = sf_idx;
            ue_sync->strack.sf_idx = sf_idx;
            ue_sync->sfind.sf_idx  = sf_idx;

            ++ue_sync->nof_recv_sf;

            if(sf_idx == 0)
             {
               ++ue_sync->frame_find_cnt;
               ++ue_sync->frame_ok_cnt;
               ++ue_sync->frame_total_cnt;
             }

            return DL_Messages{dlMessage};
          }
         else
          {
            Info("RX:%s carrier msg pci %u != my_pci %u\n", __func__, pci, ue_sync->cell.id);
          }
       }
    }

  return DL_Messages{};
}


static UL_DCI_Results get_ul_dci_list_i(uint16_t rnti, uint32_t cc_idx)
{
  UL_DCI_Results ul_dci_results;

  const auto carrierResult = findCarrierByIndex(DL_Message_Message(dlMessageThisFrame_), cc_idx);

  if(CarrierResult_Found(carrierResult))
   {
     const auto & carrier = CarrierResult_Carrier(carrierResult);

     for(const auto & pdcch : carrier.pdcch())
      {
        if(pdcch.has_ul_dci())
         {
           const auto & ul_dci_message = pdcch.ul_dci();

           if(ul_dci_message.rnti() == rnti)
            {
              const auto sinrResult = 
                DL_Message_SINRTester(dlMessageThisFrame_).sinrCheck2(EMANELTE::MHAL::CHAN_PDCCH,
                                                                      rnti, 
                                                                      carrier.frequency_hz());

              if(sinrResult.bPassed_)
               {
                 Info("PUCCH:%s: found cc %u, dci rnti 0x%hx\n", __func__, cc_idx, rnti);

                 // message, sinr
                 ul_dci_results.emplace_back(ul_dci_message, 
                                             SignalQuality{sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_});

                 break; // found, done
               }
              else
               {
                 Warning("PUCCH:%s: fail cc %u, snr rnti 0x%hx\n", __func__, cc_idx, rnti);
               }
            }
           else
            {
              Debug("PUCCH:%s: cc %u, rnti 0x%hx != ul_dci_rnti 0x%hx, skip\n", 
                     __func__, cc_idx, rnti, ul_dci_message.rnti());
            }
         }
      }
   }

  return ul_dci_results;
}


static DL_DCI_Results get_dl_dci_list_i(uint16_t rnti, uint32_t cc_idx)
{
  DL_DCI_Results dl_dci_results;

  const auto carrierResult = findCarrierByIndex(DL_Message_Message(dlMessageThisFrame_), cc_idx);

  if(CarrierResult_Found(carrierResult))
   {
     const auto & carrier = CarrierResult_Carrier(carrierResult);

     for(const auto & pdcch : carrier.pdcch())
      {
        if(pdcch.has_dl_dci())
         {
           const auto & dl_dci_message = pdcch.dl_dci();

           if(dl_dci_message.rnti() == rnti)
            {
              if(DL_Message_SINRTester(dlMessageThisFrame_).sinrCheck2(EMANELTE::MHAL::CHAN_PDCCH,
                                                                       rnti,
                                                                       carrier.frequency_hz()).bPassed_)
               {
                 Info("PDSCH:%s: found cc %u, dci rnti 0x%hx, refid %u\n", 
                        __func__, cc_idx, rnti, dl_dci_message.refid());

                 dl_dci_results.emplace_back(dl_dci_message);

                 break; // found, done
               }
              else
               {
                 Warning("PDSCH:%s: fail cc %u, snr rnti 0x%hx\n", __func__, cc_idx, rnti);
               }
            }
          else
            {
              Debug("PDSCH:%s: cc %u, rnti 0x%hx != dl_dci_rnti 0x%hx, refid %u, skip\n", 
                     __func__, cc_idx, rnti, dl_dci_message.rnti(), dl_dci_message.refid());
            }
         }
      }
   }

  return dl_dci_results;
}



static PDSCH_Results ue_dl_get_pdsch_data_list_i(uint32_t refid, uint16_t rnti, uint32_t cc_idx)
{
  PDSCH_Results pdsch_results;

  const auto carrierResult = findCarrierByIndex(DL_Message_Message(dlMessageThisFrame_), cc_idx);

  if(CarrierResult_Found(carrierResult))
   {
     const auto & carrier = CarrierResult_Carrier(carrierResult);

     if(carrier.has_pdsch())
      {
        const auto sinrResult = 
          DL_Message_SINRTester(dlMessageThisFrame_).sinrCheck2(EMANELTE::MHAL::CHAN_PDSCH,
                                                                rnti,
                                                                carrier.frequency_hz());

        if(sinrResult.bPassed_)
         {
           const auto & pdsch_message = carrier.pdsch();

           for(const auto & data : pdsch_message.data())
            {
              Info("PDSCH:%s: cc %u, refid %u, tb %u\n", __func__, cc_idx, data.refid(), data.tb());

              if(data.refid() == refid)
               {
                 pdsch_results.emplace_back(data, SignalQuality(sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_));
               }
            }
         }
        else
         {
           Warning("PDSCH:%s: fail cc %u, snr rnti 0x%hx\n", __func__, cc_idx, rnti);
         }
      }
   }

  return pdsch_results;
}

static void ue_set_crnti_i(uint16_t crnti)
{
  Info("MHAL:%s from 0x%hx to 0x%hx\n", __func__, crnti_, crnti);

  crnti_ = crnti;

  UESTATS::setCrnti(crnti);
}


void ue_initialize(uint32_t sf_interval_msec, EMANELTE::MHAL::mhal_config_t & mhal_config)
{
  logger_phy = &srslog::fetch_basic_logger("PHY");

  carrierIndexFrequencyTable_.clear();

  rxFrequencyToCarrierIndex_.clear();

  Info("INIT:%s sf_interval %u msec\n", __func__, sf_interval_msec);

  EMANELTE::MHAL::UE::initialize(sf_interval_msec, mhal_config);
}


void ue_set_earfcn(double rx_freq_hz, double tx_freq_hz, uint32_t earfcn)
{
  Info("INIT:%s rx_freq %6.4f MHz, tx_freq %6.4f MHz, earfcn %u -> %u\n",
       __func__,
       rx_freq_hz/1e6,
       tx_freq_hz/1e6,
       earfcn_,
       earfcn);

  earfcn_ = earfcn;
}

void ue_set_frequency(uint32_t cc_idx,
                      double rx_freq_hz,
                      double tx_freq_hz)
{
   carrierIndexFrequencyTable_[cc_idx] = FrequencyPair{llround(rx_freq_hz), llround(tx_freq_hz)}; // rx/tx

   rxFrequencyToCarrierIndex_[llround(rx_freq_hz)] = cc_idx;

   Warning("%s cc_idx %u, rx_freq %6.4f MHz, tx_freq %6.4f MHz\n",
           __func__,
           cc_idx,
           rx_freq_hz/1e6,
           tx_freq_hz/1e6);

   EMANELTE::MHAL::UE::set_frequencies(cc_idx, rx_freq_hz, tx_freq_hz);
}


void ue_set_sync(srsue::sync * sync)
{
  sync_ = sync;
}


void ue_set_cell(const srsue::phy_cell_t* cell)
{
  Warning("INIT:%s pci %u -> %u\n", __func__, my_cell_id_, cell->pci);

  my_cell_id_ = cell->pci;
}


void ue_set_bandwidth(int n_prb)
{
  Info("INIT:%s n_prb %d\n", __func__, n_prb);

  EMANELTE::MHAL::UE::set_num_resource_blocks(n_prb);
}


void ue_start()
{
  Info("START:%s\n", __func__);

  pthread_mutexattr_t mattr;

  if(pthread_mutexattr_init(&mattr) < 0)
   {
     Error("START:%s pthread_mutexattr_init error %s, exit\n", __func__, strerror(errno));
     exit(1);
   }
  else
   {
     if(pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT) < 0)
       {
         Error("START:%s pthread_mutexattr_setprotocol error %s, exit\n", __func__, strerror(errno));
         exit(1);
       }

     pthread_mutexattr_destroy(&mattr);
  }

  ue_ul_msg_.Clear();

  txControl_.Clear();

  EMANELTE::MHAL::UE::start();
}


void ue_stop()
{
  Info("STOP:%s\n", __func__);

  EMANELTE::MHAL::UE::stop();
}


void ue_set_prach_freq_offset(uint32_t freq_offset)
{
  Info("MHAL:%s %u\n", __func__, freq_offset);

  prach_freq_offset_ = freq_offset;
}


// read frame for this tti common to all states
int ue_dl_read_frame(srsran_timestamp_t* rx_time)
{
  auto & tv_tti = FrameMessage_timestamp(frameSignals_);

  EMANELTE::MHAL::UE::get_messages(FrameMessage_rxMessages(frameSignals_), tv_tti);

  FrameMessage_isSet(frameSignals_) = true;

  if(rx_time)
   {
     rx_time->full_secs = tv_tti.tv_sec; 
     rx_time->frac_secs = tv_tti.tv_usec / 1e6;
   }

  return 1;
}


/*typedef struct SRSRAN_API {
  uint32_t            cell_id;
  srsran_cp_t         cp;
  srsran_frame_type_t frame_type;
  float               peak; 
  float               mode; 
  float               psr;
  float               cfo; 
} srsran_ue_cellsearch_result_t;

typedef enum SRSRAN_API { SRSRAN_FDD = 0, SRSRAN_TDD = 1 } srsran_frame_type_t;

typedef struct SRSRAN_API {
  srsran_ue_sync_t ue_sync;
  cf_t             *sf_buffer[SRSRAN_MAX_PORTS];
  uint32_t         nof_rx_antennas;
  uint32_t         max_frames;
  uint32_t         nof_valid_frames;  // number of 5 ms frames to scan 
  uint32_t         *mode_ntimes;
  uint8_t          *mode_counted; 
  srsran_ue_cellsearch_result_t *candidates; 
} srsran_ue_cellsearch_t; */

// 1 initial state cell search
int ue_dl_cellsearch_scan(srsran_ue_cellsearch_t * cs,
                          srsran_ue_cellsearch_result_t * res,
                          int force_nid_2,
                          uint32_t *max_peak)
{
  const auto cc_idx = 0; // always cc_idx 0 on cell search

  // cell search seems to be done in blocks of 5 sf's
  const uint32_t max_tries = cs->max_frames * 5; // 40 sf

  // n_id_2's
  std::set<uint32_t> n_id2s;

  UESTATS::Cells cells;

  uint32_t num_pss_sss_found = 0;
  uint32_t try_num           = 0;

  // notify in cell search
  EMANELTE::MHAL::UE::begin_cell_search();

  while(++try_num <= max_tries)
   {
     // radio recv is done here during search
     sync_->radio_recv_fnc(buffer_, 0);

     const auto dlMessages = ue_dl_get_signals_i(&cs->ue_sync.last_timestamp);

     // for each enb msg (if any)
     for(const auto & dlMessage : dlMessages)
      {
        const auto & enb_dl_msg = DL_Message_Message(dlMessage);

        // locate carrier 0
        const auto carrierResult = findCarrierByIndex(enb_dl_msg, cc_idx);

        if(CarrierResult_Found(carrierResult))
         {
           const auto & carrier = CarrierResult_Carrier(carrierResult);
#if 0
           Info("RX:%s: carrier %s\n", __func__, carrier.DebugString().c_str());
#endif
           const uint32_t pci    = carrier.phy_cell_id();
           const uint32_t n_id_1 = pci / 3;
           const uint32_t n_id_2 = pci % 3;

           // force is enabled, but this cell id group does not match
           if(is_valid_n_id_2(force_nid_2) && n_id_2 != (uint32_t)force_nid_2)
            {
              Info("RX:%s: n_id_1 %u, n_id_2 %u != %d, ignore\n",
                   __func__, n_id_1, n_id_2, force_nid_2);
 
              continue;
            }

           float peak_sum       = 0.0;
           srsran_cp_t cp       = SRSRAN_CP_NORM;
           uint32_t num_samples = 0;

           // search for pss/sss
           if(carrier.has_pss_sss())
            {
              const auto & pss_sss = carrier.pss_sss();

              // should all be the same
              cp = pss_sss.cp_mode() == EMANELTE::MHAL::CP_NORM ? SRSRAN_CP_NORM : SRSRAN_CP_EXT;

              const auto & rxControl = DL_Message_RxControl(dlMessage);

              peak_sum = rxControl.peak_sum_[cc_idx];

              num_samples = rxControl.num_samples_[cc_idx];

              ++num_pss_sss_found;

              Info("RX:%s: PCI %u, peak_sum %0.1f, num_samples %u\n",
                   __func__, pci, peak_sum, num_samples);
            }

           if(num_samples > 0)
            {
              const float peak_avg = peak_sum / num_samples;

              // save cell info
              cells[pci] = peak_avg;

              // cell id [0,1,2]
              if(n_id2s.insert(n_id_2).second == true)
                {
                  res[n_id_2].cell_id     = pci;
                  res[n_id_2].cp          = cp;
                  res[n_id_2].peak        = peak_avg;
                  res[n_id_2].mode        = 1.0;
                  res[n_id_2].psr         = 0.0;
                  res[n_id_2].cfo         = 0.0;
                  res[n_id_2].frame_type  = SRSRAN_FDD;

                  Info("RX:%s: new PCI %u, n_id_1 %u, n_id_2 %u, peak_avg %f\n",
                       __func__,
                       pci,
                       n_id_1,
                       n_id_2,
                       peak_avg);
                }
               else
                {
                  // tie goes to the first entry (numeric lowest id)
                  if(peak_avg > res[n_id_2].peak)
                   {
                     Info("RX:%s: replace PCI %u, n_id_1 %u, n_id_2 %u, peak_avg %f\n",
                           __func__,
                           pci,
                           n_id_1,
                           n_id_2,
                           peak_avg);
 
                       res[n_id_2].cell_id = pci;
                       res[n_id_2].cp      = cp;
                       res[n_id_2].peak    = peak_avg;
                  } 
               }
            }
         }
      } // end for each enb msg
   } // end while

  cs->ue_sync.pss_stable_cnt = num_pss_sss_found;
  cs->ue_sync.pss_is_stable  = num_pss_sss_found > 0 ? true : false;

  float max_avg = 0.0f;

  // now find the best
  for(const auto & id : n_id2s)
    {
      if(res[id].peak > max_avg)
        {
          *max_peak = id;

          max_avg = res[id].peak;
        }
    }

  Info("RX:%s: sf_idx %u, DONE, num_cells %zu, max_peak id %u, max_avg %f\n",
          __func__,
          cs->ue_sync.sf_idx,
          n_id2s.size(),
          *max_peak,
          max_avg);

  UESTATS::enterCellSearch(cells, earfcn_);

  return n_id2s.size();
}

/* typedef struct SRSRAN_API {
  srsran_ue_sync_t              ue_sync;
  cf_t                          *sf_buffer[SRSRAN_MAX_PORTS];
  uint32_t                      nof_rx_antennas;
  uint32_t                      max_frames;
  uint32_t                      nof_valid_frames;  // number of 5 ms frames to scan 
  uint32_t                      *mode_ntimes;
  uint8_t                       *mode_counted; 
  srsran_ue_cellsearch_result_t *candidates; 
} srsran_ue_cellsearch_t;

typedef struct {
  srsran_ue_mib_t  ue_mib; 
  srsran_ue_sync_t ue_sync; 
  cf_t             *sf_buffer[SRSRAN_MAX_PORTS];
  uint32_t         nof_rx_antennas;
} srsran_ue_mib_sync_t;

typedef struct SRSRAN_API {
  srsran_sync_t         sfind;
  cf_t*                 sf_symbols[SRSRAN_MAX_PORTS];
  srsran_ofdm_t         fft;
  srsran_pbch_t         pbch;
  srsran_chest_dl_t     chest;
  srsran_chest_dl_res_t chest_res;
  uint8_t               bch_payload[SRSRAN_BCH_PAYLOAD_LEN];
  uint32_t              nof_tx_ports; 
  uint32_t              sfn_offset; 
  uint32_t              frame_cnt; 
} srsran_ue_mib_t;

typedef struct SRSRAN_API {
  cf_t*    ce[SRSRAN_MAX_PORTS][SRSRAN_MAX_PORTS];
  uint32_t nof_re;
  float    noise_estimate;
  float    noise_estimate_dbm;
  float    snr_db;
  float    snr_ant_port_db[SRSRAN_MAX_PORTS][SRSRAN_MAX_PORTS];
  float    rsrp;
  float    rsrp_dbm;
  float    rsrp_neigh_dbm;
  float    rsrp_port_dbm[SRSRAN_MAX_PORTS];
  float    rsrp_ant_port_dbm[SRSRAN_MAX_PORTS][SRSRAN_MAX_PORTS];
  float    rsrq;
  float    rsrq_db;
  float    rsrq_ant_port_db[SRSRAN_MAX_PORTS][SRSRAN_MAX_PORTS];
  float    rssi_dbm;
  float    cfo;
  float    sync_error;
} srsran_chest_dl_res_t; */

// 2 mib search
int ue_dl_mib_search(const srsran_ue_cellsearch_t * cs,
                     srsran_ue_mib_sync_t * ue_mib_sync,
                     srsran_cell_t * cell)
{
  // 40 sf
  const uint32_t max_tries = cs->max_frames * 5;
  uint32_t try_num         = 0;

  while(++try_num <= max_tries)
   {
     // radio recv called here
     sync_->radio_recv_fnc(buffer_, 0);

     const auto dlMessages = ue_dl_enb_subframe_get_pci_i(&ue_mib_sync->ue_sync, NULL);

     Info("RX:ue_dl_mib_search: pci %hu, try %d/%u, %zu signals\n", 
           ue_mib_sync->ue_sync.cell.id, try_num, max_tries, dlMessages.size());

     // expect 1 and only 1
     if(! dlMessages.empty())
      {
        const auto & dlMessage = dlMessages[0];

        const auto & enb_dl_msg = DL_Message_Message(dlMessage);

        const auto carrierResult = findCarrierByIndex(enb_dl_msg, 0); // cc_id 0

        if(CarrierResult_Found(carrierResult))
         {
           const auto & carrier = CarrierResult_Carrier(carrierResult);
#if 0
           Info("RX:%s: carrier %s\n", __func__, carrier.DebugString().c_str());
#endif
           if(carrier.has_pbch())
            {
              if(DL_Message_SINRTester(dlMessage).sinrCheck2(EMANELTE::MHAL::CHAN_PBCH,
                                                             carrier.frequency_hz()).bPassed_)
               {
                 if(carrier.has_pss_sss())
                  {
                    const auto & pss_sss = carrier.pss_sss();

                    const auto & pbch = carrier.pbch();

                    Info("RX:ue_dl_mib_search: found PBCH\n");

                    cell->nof_prb   = pbch.num_prb();
                    cell->nof_ports = pbch.num_antennas();

                    ue_mib_sync->ue_sync.state          = SF_TRACK;
                    ue_mib_sync->ue_sync.pss_stable_cnt = 1;
                    ue_mib_sync->ue_sync.pss_is_stable  = true;

                    switch(pbch.phich_resources())
                     {
                       case EMANELTE::MHAL::PR_ONE_SIXTH:
                        cell->phich_resources = SRSRAN_PHICH_R_1_6;
                       break;

                       case EMANELTE::MHAL::PR_ONE_HALF:
                        cell->phich_resources = SRSRAN_PHICH_R_1_2;
                       break;

                       case EMANELTE::MHAL::PR_ONE:
                        cell->phich_resources = SRSRAN_PHICH_R_1;
                       break;

                       case EMANELTE::MHAL::PR_TWO:
                        cell->phich_resources = SRSRAN_PHICH_R_2;
                       break;
                     }

                    switch(pbch.phich_length())
                     {
                       case EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_NORM:
                        cell->phich_length = SRSRAN_PHICH_NORM;
                       break;

                       case EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_EXTD:
                        cell->phich_length = SRSRAN_PHICH_EXT;
                       break;
                     }

                    UESTATS::enterMibSearch(true);

                    return 1;
                  }
                }
               else
                {
                  Info("MIB:%s: fail snr\n", __func__);
                }
             }
          }
       }
    }

  UESTATS::enterMibSearch(false);

  return 0;
}


/*typedef struct SRSRAN_API {
  srsran_sync_t s        find;
  srsran_sync_t          strack;
  uint32_t               max_prb;
  srsran_agc_t           agc; 
  bool                   do_agc; 
  uint32_t               agc_period; 
  int                    decimate;
  srsran_timestamp_t     last_timestamp;
  uint32_t               nof_rx_antennas; 
  srsran_ue_sync_state_t state;
  uint32_t               frame_len; 
  uint32_t               fft_size;
  uint32_t               nof_recv_sf;
  uint32_t               nof_avg_find_frames;
  uint32_t               frame_find_cnt;
  uint32_t               sf_len;
  uint64_t               frame_ok_cnt;
  uint32_t               frame_no_cnt; 
  uint32_t               frame_total_cnt; 
  uint32_t               frame_number; // not used
  srsran_cell_t          cell; 
  uint32_t               sf_idx;
  bool                   cfo_is_copied;
  bool                   cfo_correct_enable_track;
  bool                   cfo_correct_enable_find;
  float                  cfo_current_value;
  float                  cfo_loop_bw_pss;
  float                  cfo_loop_bw_ref;
  float                  cfo_pss_min;
  float                  cfo_ref_min;
  float                  cfo_ref_max;
  uint32_t               pss_stable_cnt;
  uint32_t               pss_stable_timeout;
  bool                   pss_is_stable;
  uint32_t               peak_idx;
  int                    next_rf_sample_offset;
  int                    last_sample_offset; 
  float                  mean_sample_offset; 
  uint32_t               sample_offset_correct_period;
  float                  sfo_ema; 
} srsran_ue_sync_t; */

// 3 system frame search
int ue_dl_system_frame_search(srsran_ue_sync_t * ue_sync, uint32_t * sfn)
{
  const uint32_t max_tries = 1;

  uint32_t try_num = 0;

  while(++try_num <= max_tries)
   {
     // radio recv called here
     sync_->radio_recv_fnc(buffer_, 0);

     const auto dlMessages = ue_dl_enb_subframe_get_pci_i(ue_sync, NULL);

     // expect 1 and only 1
     if(! dlMessages.empty())
      {
        const auto & dlMessage = dlMessages[0];

        const auto & enb_dl_msg = DL_Message_Message(dlMessage);

        const auto carrierResult = findCarrierByIndex(enb_dl_msg, 0); // cc_id 0

        if(CarrierResult_Found(carrierResult))
         {
           const auto & carrier = CarrierResult_Carrier(carrierResult);

           if(carrier.has_pbch())
            {
#if 0
             Info("RX:%s: carrier %s\n", __func__, carrier.DebugString().c_str());
#endif
              // check for PSS SSS if PBCH is good
              if(DL_Message_SINRTester(dlMessage).sinrCheck2(EMANELTE::MHAL::CHAN_PBCH,
                                                             carrier.frequency_hz()).bPassed_)
               {
                 if(carrier.has_pss_sss())
                  {
                    const auto & pss_sss = carrier.pss_sss();

                    const auto & pbch = carrier.pbch();

                    Info("RX:%s: found PBCH, try %u/%u\n",
                         __func__,
                          try_num,
                          max_tries);

                      ue_sync->state           = SF_TRACK;
                      ue_sync->pss_stable_cnt  = 1;
                      ue_sync->pss_is_stable   = true;

                      // set system frame number
                      *sfn = enb_dl_msg.tti();
                
                      UESTATS::enterSysFrameSearch(true);

                      return 1;
                  }
               }
              else
               {
                 Warning("PBCH:%s: fail snr\n", __func__);
               }
            }
         }
      }
   }

  UESTATS::enterSysFrameSearch(false);

  return 0;
}


// 4 this is the main rx handler
int ue_dl_sync_search(srsran_ue_sync_t * ue_sync, uint32_t tti)
{
   // set next tx tti
   tti_tx_ = (tti+4)%10240;

   EMANELTE::MHAL::UE::set_tti(tti);

   // lower level radio recv called here
   sync_->radio_recv_fnc(buffer_, 0);

   DL_Message_SINRTester(dlMessageThisFrame_).release();

   enb_dl_pdsch_messages_.clear();

   const auto dlMessages = ue_dl_enb_subframe_get_pci_i(ue_sync, &tti);

   // expect 1 and only 1 for single antenna mode
   if(! dlMessages.empty())
    {
      dlMessageThisFrame_ = dlMessages[0];

      UESTATS::enterSyncSearch(true);
    }

   return dlMessages.size();
}


float ue_dl_get_rssi(uint32_t cell_id, uint32_t cc_idx)
{
   // XXX TODO sinr to rssi/rsrq see phy.snr_ema_coeff defualt 0.1
   const float rssi = sinrManager_[cc_idx].get() / 10.0f;

   Debug("Rx:%s: cell %u, cc_idx %u, rssi %f\n", __func__, cell_id, cc_idx, rssi);

   return rssi;
}

/*
typedef struct SRSRAN_API {
  uint32_t L;    // Aggregation level
  uint32_t ncce; // Position of first CCE of the dci
} srsran_dci_location_t;

typedef struct SRSRAN_API {
  uint8_t               payload[SRSRAN_DCI_MAX_BITS];
  uint32_t              nof_bits;
  srsran_dci_location_t location;
  srsran_dci_format_t   format;
  uint16_t              rnti;
} srsran_dci_msg_t;

typedef struct SRSRAN_API {
  srsran_dl_cfg_t       cfg;
  srsran_chest_dl_cfg_t chest_cfg;
  uint32_t              last_ri;
  float                 snr_to_cqi_offset;
} srsran_ue_dl_cfg_t; */

// see ue_dl_find_dl_dc
// int srsran_ue_dl_find_dl_dci(srsran_ue_dl_t*     q,
//                              srsran_dl_sf_cfg_t* sf,
//                              srsran_ue_dl_cfg_t* dl_cfg,
//                              uint16_t            rnti,
//                              srsran_dci_dl_t     dci_dl[SRSRAN_MAX_DCI_MSG])
int ue_dl_cc_find_dl_dci(srsran_ue_dl_t*     q,
                         srsran_dl_sf_cfg_t* sf,
                         srsran_ue_dl_cfg_t* dl_cfg,
                         uint16_t            rnti,
                         srsran_dci_dl_t     dci_dl[SRSRAN_MAX_DCI_MSG],
                         uint32_t            cc_idx)

{
  srsran_dci_msg_t dci_msg[SRSRAN_MAX_DCI_MSG] = {{}};

  int nof_msg = 0;

  const auto dl_dci_results = get_dl_dci_list_i(rnti, cc_idx);

  // expecting 1 dci/rnti
  if(dl_dci_results.size() == 1)
    {
      const auto & dci_message = dl_dci_results[0];

      const auto pdsch_results = ue_dl_get_pdsch_data_list_i(dci_message.refid(), rnti, cc_idx);

      // XXX TODO pass/fail
      UESTATS::getPDCCH(rnti, true);

      // expecting 1 pdsch/dci
      if(pdsch_results.size() == 1)
        {
          const auto & pdsch_result = pdsch_results.front();

          // XXX TODO pass/fail
          UESTATS::getPDSCH(rnti, true);

          // save the grant for pdsch_decode
          enb_dl_pdsch_messages_.emplace(rnti, 
                                         ENB_DL_Message_PDSCH_Entry{pdsch_result.first, 
                                                                    pdsch_result.second});

          const auto & dl_dci_message      = dci_message.dci_msg();
          const auto & dl_dci_message_data = dl_dci_message.data();

          auto & dci_entry = dci_msg[0];

          dci_entry.nof_bits      = dl_dci_message.num_bits();
          dci_entry.rnti          = rnti;
          dci_entry.format        = get_msg_format(dl_dci_message.format());
          dci_entry.location.L    = dl_dci_message.l_level();
          dci_entry.location.ncce = dl_dci_message.l_ncce();

          memcpy(dci_entry.payload, dl_dci_message_data.data(), dl_dci_message_data.size());
          ++nof_msg;

          ue_dl_update_chest_i(&q->chest_res, pdsch_result.second.sinr_dB_, pdsch_result.second.noiseFloor_dBm_);

#ifdef DEBUG_HEX
          InfoHex(dl_dci_message_data.data(), dl_dci_message_data.size(),
                  "PDCCH:%s dl_dci ref id %u, rnti 0x%hx, dci_len %zu\n", 
                  __func__, dci_message.refid(), rnti, dl_dci_message_data.size());
#else
          Info("PDCCH:%s dl_dci ref id %u, rnti 0x%hx, dci_len %zu\n", 
                __func__, dci_message.refid(), rnti, dl_dci_message_data.size());
#endif

          // Unpack DCI messages see lib/src/phy/phch/dci.c
          for (int i = 0; i < nof_msg; i++) {
            if (srsran_dci_msg_unpack_pdsch(&q->cell, sf, &dl_cfg->cfg.dci, &dci_msg[i], &dci_dl[i])) {
               Error("PDCCH:%s Unpacking DL DCI\n", __func__);
               return SRSRAN_ERROR;
            }
          }
       }
      else
       {
         if(pdsch_results.size() > 1)
          {
            Info("PDCCH:%s found %zu dl_dci for rnti 0x%hx\n", 
                    __func__, pdsch_results.size(), rnti);
          }
         else
          {
            Error("PDCCH:%s no pdsch data for rnti 0x%hx, refid %u\n",
                  __func__, rnti, dci_message.refid());
          }
       }
    }
   else
    {
      if(dl_dci_results.size() > 1)
       {
         Warning("PDCCH:%s found %zu dl_dci for rnti 0x%hx\n", 
                 __func__, dl_dci_results.size(), rnti);
       }
    }

  return nof_msg;
}

/*
typedef struct SRSRAN_API {
  uint32_t L;    // Aggregation level
  uint32_t ncce; // Position of first CCE of the dci
} srsran_dci_location_t;

typedef struct SRSRAN_API {
  uint8_t               payload[SRSRAN_DCI_MAX_BITS];
  uint32_t              nof_bits;
  srsran_dci_location_t location;
  srsran_dci_format_t   format;
  uint16_t              rnti;
} srsran_dci_msg_t; */

// see lib/src/phy/ue/ue_dl.c
int ue_dl_cc_find_ul_dci(srsran_ue_dl_t*     q,
                         srsran_dl_sf_cfg_t* sf,
                         srsran_ue_dl_cfg_t* dl_cfg,
                         uint16_t            rnti,
                         srsran_dci_ul_t     dci_ul[SRSRAN_MAX_DCI_MSG],
                         uint32_t cc_idx)
{
  srsran_dci_msg_t dci_msg[SRSRAN_MAX_DCI_MSG] = {{}};

  int nof_msg = 0;

  if(rnti) 
   {
     const auto ul_dci_results = get_ul_dci_list_i(rnti, cc_idx);

     // expecting 1 dci/rnti
     if(ul_dci_results.size() == 1)
      {
        const auto & dci_message         = ul_dci_results[0].first;
        const auto & ul_dci_message      = dci_message.dci_msg();
        const auto & ul_dci_message_data = ul_dci_message.data();
 
        ue_dl_update_chest_i(&q->chest_res, ul_dci_results[0].second.sinr_dB_, ul_dci_results[0].second.noiseFloor_dBm_);

        auto & dci_entry = dci_msg[0];

        dci_entry.nof_bits      = ul_dci_message.num_bits();
        dci_entry.rnti          = rnti;
        dci_entry.format        = get_msg_format(ul_dci_message.format());
        dci_entry.location.L    = ul_dci_message.l_level();
        dci_entry.location.ncce = ul_dci_message.l_ncce();

        memcpy(dci_entry.payload, ul_dci_message_data.data(), ul_dci_message_data.size());

        Info("PUCCH:%s found ul_dci rnti 0x%hx\n", __func__, rnti);

        ++nof_msg;

        // Unpack DCI messages
        for (int i = 0; i < nof_msg; i++) {
          if (srsran_dci_msg_unpack_pusch(&q->cell, sf, &dl_cfg->cfg.dci, &dci_msg[i], &dci_ul[i])) {
            Error("PUCCH:%s Unpacking UL DCI\n", __func__);
            return SRSRAN_ERROR;
          }
        }

      }
   else
    {
      if(ul_dci_results.size() > 1)
       {
         Warning("PUCCH:%s found %zu ul_dci for rnti 0x%hx\n", 
                  __func__, ul_dci_results.size(), rnti);
       }
    }
  }
 
  return nof_msg;
}

/* typedef struct SRSRAN_API {
  srsran_cell_t cell;
  uint32_t      nof_rx_antennas;
  uint16_t      current_mbsfn_area_id;
  uint16_t      pregen_rnti;
   
  srsran_pcfich_t pcfich;
  srsran_pdcch_t  pdcch;
  srsran_pdsch_t  pdsch;
  srsran_pmch_t   pmch;
  srsran_phich_t  phich;

  srsran_regs_t regs[MI_MAX_REGS];
  uint32_t      mi_manual_index; 
  bool          mi_auto;

  srsran_chest_dl_t     chest;
  srsran_chest_dl_res_t chest_res;
  srsran_ofdm_t         fft[SRSRAN_MAX_PORTS];
  srsran_ofdm_t         fft_mbsfn;

  // Variables for blind DCI search
  dci_blind_search_t current_ss_ue[MI_MAX_REGS][3][10];
  dci_blind_search_t current_ss_common[MI_MAX_REGS][3]; 
  srsran_dci_msg_t   pending_ul_dci_msg[SRSRAN_MAX_DCI_MSG];
  uint32_t           pending_ul_dci_count;
} srsran_ue_dl_t;

typedef struct SRSRAN_API {
  srsran_tdd_config_t tdd_config;
  uint32_t            tti;
  uint32_t            cfi;
  srsran_sf_t         sf_type;
  uint32_t            non_mbsfn_region;
} srsran_dl_sf_cfg_t;

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
  srsran_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srsran_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
} srsran_pdsch_cfg_t;

typedef struct {
  uint8_t* payload;
  bool     crc;
  float    avg_iterations_block;
} srsran_pdsch_res_t; */

// see lib/src/phy/phch/pdsch.c srsran_pdsch_decode()
int ue_dl_cc_decode_pdsch(srsran_ue_dl_t*     q,
                          srsran_dl_sf_cfg_t* sf,
                          srsran_pdsch_cfg_t* cfg,
                          srsran_pdsch_res_t  data[SRSRAN_MAX_CODEWORDS],
                          uint32_t cc_idx)
{
   const auto rnti = cfg->rnti;

   for(uint32_t tb = 0; tb < SRSRAN_MAX_CODEWORDS; ++tb)
    {
     if(cfg->grant.tb[tb].enabled)
       {
         const auto iter = enb_dl_pdsch_messages_.find(rnti);

         if(iter != enb_dl_pdsch_messages_.end())
           {
             const auto & pdsch_result  = iter->second;
             const auto & pdsch_message = pdsch_result.first;
             const auto & pdsch_data    = pdsch_message.data();

             memcpy(data[tb].payload, pdsch_data.data(), pdsch_data.size());

             data[tb].avg_iterations_block = 1;
             data[tb].crc = true;

             ue_dl_update_chest_i(&q->chest_res, pdsch_result.second.sinr_dB_, pdsch_result.second.noiseFloor_dBm_);

#ifdef DEBUG_HEX
             InfoHex(pdsch_data.data(), pdsch_data.size(),
                     "PDSCH:%s: rnti 0x%hx, refid %d, tb[%d], payload %zu bytes, snr %f\n",
                     __func__, rnti, pdsch_message.refid(), tb, pdsch_data.size(), q->chest_res.snr_db);
#else
             Info("PDSCH:%s: rnti 0x%hx, refid %d, tb[%d], payload %zu bytes, snr %f\n",
                   __func__, rnti, pdsch_message.refid(), tb, pdsch_data.size(), q->chest_res.snr_db);
#endif
           }
         else
           {
             Error("PDSCH:%s: rnti %0xhx, no pdsch data\n", __func__, rnti);
           }
       }
    }

  return SRSRAN_SUCCESS;
}


/* typedef struct SRSRAN_API {
  uint32_t n_prb_lowest;
  uint32_t n_dmrs;
  uint32_t I_phich;
} srsran_phich_grant_t;

typedef struct SRSRAN_API {
  uint32_t ngroup;
  uint32_t nseq;
} srsran_phich_resource_t;

typedef struct SRSRAN_API {
  bool  ack_value;
  float distance;
} srsran_phich_res_t;

typedef struct SRSRAN_API {
  srsran_dl_cfg_t       cfg;
  srsran_chest_dl_cfg_t chest_cfg;
  uint32_t              last_ri;
  float                 snr_to_cqi_offset;
} srsran_ue_dl_cfg_t; */

// see lib/src/phy/ue/ue_dl.c
int ue_dl_cc_decode_phich(srsran_ue_dl_t*       q,
                          srsran_dl_sf_cfg_t*   sf,
                          srsran_ue_dl_cfg_t*   cfg,
                          srsran_phich_grant_t* grant,
                          srsran_phich_res_t*   result,
                          uint16_t rnti,
                          uint32_t cc_idx)
{
  srsran_phich_resource_t n_phich;

  srsran_phich_calc(&q->phich, grant, &n_phich);

  const auto carrierResult = findCarrierByIndex(DL_Message_Message(dlMessageThisFrame_), cc_idx);

  if(CarrierResult_Found(carrierResult))
   {
     const auto & carrier = CarrierResult_Carrier(carrierResult);

     if(carrier.has_phich())
      {
        const auto sinrResult = 
          DL_Message_SINRTester(dlMessageThisFrame_).sinrCheck2(EMANELTE::MHAL::CHAN_PHICH,
                                                                rnti,
                                                                carrier.frequency_hz());

        ue_dl_update_chest_i(&q->chest_res, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

        if(sinrResult.bPassed_)
         {
           const auto & phich_message = carrier.phich();

           if(rnti                == phich_message.rnti()        && 
              grant->n_prb_lowest == phich_message.num_prb_low() &&
              grant->n_dmrs       == phich_message.num_dmrs())
            {
              result->ack_value = phich_message.ack();
              result->distance  = 1.0;
            }

           Info("PHICH:%s cc=%u, sf_idx=%d, n_prb_l=%d, n_dmrs=%d, I_phich=%d, rnti 0x%hx, ack %d, dist %f\n",
                __func__,
                cc_idx,
                sf->tti % 10,
                grant->n_prb_lowest,
                grant->n_dmrs,
                grant->I_phich,
                rnti,
                result->ack_value,
                result->distance);
          }
        else
         {
           Warning("PHICH:%s: fail snr\n", __func__);
         }
      }
   }

   return SRSRAN_SUCCESS;
}


/* typedef struct SRSRAN_API {
  srsran_cell_t cell;
  uint32_t      nof_rx_antennas;
  uint16_t      current_mbsfn_area_id;
  uint16_t      pregen_rnti;
   
  srsran_pcfich_t pcfich;
  srsran_pdcch_t  pdcch;
  srsran_pdsch_t  pdsch;
  srsran_pmch_t   pmch;
  srsran_phich_t  phich;

  srsran_regs_t regs[MI_MAX_REGS];
  uint32_t      mi_manual_index; 
  bool          mi_auto;

  srsran_chest_dl_t     chest;
  srsran_chest_dl_res_t chest_res;
  srsran_ofdm_t         fft[SRSRAN_MAX_PORTS];
  srsran_ofdm_t         fft_mbsfn;

  // Variables for blind DCI search
  dci_blind_search_t current_ss_ue[MI_MAX_REGS][3][10];
  dci_blind_search_t current_ss_common[MI_MAX_REGS][3]; 
  srsran_dci_msg_t   pending_ul_dci_msg[SRSRAN_MAX_DCI_MSG];
  uint32_t           pending_ul_dci_count;
} srsran_ue_dl_t;

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
  srsran_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srsran_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
} srsran_pdsch_cfg_t;

typedef struct SRSRAN_API {
  srsran_pdsch_cfg_t pdsch_cfg;
  uint16_t           area_id;
} srsran_pmch_cfg_t;

typedef struct {
  uint8_t* payload;
  bool     crc;
  float    avg_iterations_block;
} srsran_pdsch_res_t; */

int ue_dl_cc_decode_pmch(srsran_ue_dl_t*     q,
                         srsran_dl_sf_cfg_t* sf,
                         srsran_pmch_cfg_t*  cfg,
                         srsran_pdsch_res_t  data[SRSRAN_MAX_CODEWORDS],
                         uint32_t cc_idx)
{
   const auto area_id = cfg->area_id;

   for(uint32_t tb = 0; tb < SRSRAN_MAX_CODEWORDS; ++tb)
    {
      if(cfg->pdsch_cfg.grant.tb[tb].enabled)
       {
         const auto carrierResult = findCarrierByIndex(DL_Message_Message(dlMessageThisFrame_), cc_idx);

         if(CarrierResult_Found(carrierResult))
          {
            const auto & carrier = CarrierResult_Carrier(carrierResult);

            if(carrier.has_pmch())
             {
               const auto & pmch = carrier.pmch();

               if(area_id == pmch.area_id())
                {
                  const auto sinrResult = 
                    DL_Message_SINRTester(dlMessageThisFrame_).sinrCheck2(EMANELTE::MHAL::CHAN_PMCH,
                                                                          carrier.frequency_hz());

                  ue_dl_update_chest_i(&q->chest_res, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

                  if(sinrResult.bPassed_)
                   {
                     memcpy(data[tb].payload, pmch.data().data(), pmch.data().length());
 
                     data[tb].avg_iterations_block = 1;

                     data[tb].crc = true;
#ifdef DEBUG_HEX
                     InfoHex(pmch.data().data(), pmch.data().size(),
                             "PMCH:%s: cc %u, areaid %d, tb[%d], payload %zu bytes, snr %f\n",
                             __func__, cc_idx, area_id, tb, pmch.data().size(), q->chest_res.snr_db);
#else
                     Info("PMCH:%s: cc %u, areaid %d, tb[%d], payload %zu bytes, snr %f\n",
                          __func__, cc_idx, area_id, tb, pmch.data().size(), q->chest_res.snr_db);
#endif
                   }
                  else
                   {
                     Warning("PMCH:%s: cc %u, area_id %d, fail snr\n", __func__, cc_idx, area_id);
                   }
                }
              else
                {
                  Info("MHAL:%s: cc %u, dl_area_id %u != area_id %hu, skip\n", __func__, cc_idx, pmch.area_id(), area_id);
                }
             }
          }
       }
    }

  return SRSRAN_SUCCESS;
}


void ue_ul_tx_init()
{
  Debug("TX:%s: \n", __func__);
}

// send to mhal
void ue_ul_send_signal(time_t sot_sec, float frac_sec, const srsran_cell_t & cell)
{
  // end of tx sequence, tx_end will release lock
  std::lock_guard<std::mutex> lock(ul_mutex_);

  ue_ul_msg_.set_crnti(crnti_);
  ue_ul_msg_.set_tti(tti_tx_);

  // finalize  ul_msg/txControl
  for(int idx = 0; idx < ue_ul_msg_.carriers().size(); ++idx)
   {
     ue_ul_msg_.mutable_carriers(idx)->set_phy_cell_id(cell.id);
   }

  for(int idx = 0; idx < txControl_.carriers().size(); ++idx)
   {
     txControl_.mutable_carriers(idx)->set_phy_cell_id(cell.id);
   }

  EMANELTE::MHAL::Data data;

  if(ue_ul_msg_.SerializeToString(&data))
   {
     // align sot to sf time
     const timeval tv_sf_time = {sot_sec, (time_t)(round(frac_sec * 1e3)*1e3)};
     
     auto ts = txControl_.mutable_sf_time();
     ts->set_ts_sec(tv_sf_time.tv_sec);
     ts->set_ts_usec(tv_sf_time.tv_usec);

     txControl_.set_message_type(EMANELTE::MHAL::UPLINK);
     txControl_.set_tx_seqnum(tx_seqnum_++);
     txControl_.set_tti_tx(tti_tx_);

     EMANELTE::MHAL::UE::send_msg(data, txControl_);
   }
 else
   {
     Error("TX:%s: SerializeToString ERROR len %zu\n", __func__, data.length());
   }

  // msg sent clear old data
  ue_ul_msg_.Clear();

  txControl_.Clear();
}


void ue_ul_put_prach(int index)
{
  std::lock_guard<std::mutex> lock(ul_mutex_);

  // use carrier 0 for prach
  const std::uint32_t cc_idx = 0;

  // tx frequency for carrier idx
  const auto frequencyHz = getTxFrequency(cc_idx);

  auto controlCarrier = getCarrierByFrequency<EMANELTE::MHAL::TxControlCarrierMessage, 
                                              EMANELTE::MHAL::TxControlMessage>(txControl_, frequencyHz);

  auto channelMessage = controlCarrier->mutable_uplink()->mutable_prach();

  initUplinkChannelMessage(channelMessage,
                           EMANELTE::MHAL::CHAN_PRACH,
                           EMANELTE::MHAL::MOD_BPSK,   // modtype
                           839);                       // PRACH sequence is 839 for formats 0-3 (all allowed by FDD) 

  // The upstream PRACH message is not really a slotted message
  // and can span 2 or 3 subframes. Set slot1 and slot2 resource blocks the same.
  // prach spans the 6 resource blocks starting from prach_freq_offset
  for(int i = 0; i < 6; ++i)
   {
     channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(prach_freq_offset_ + i, frequencyHz));
     channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(prach_freq_offset_ + i, frequencyHz));
   }

  auto carrier = getCarrierByFrequency<EMANELTE::MHAL::UE_UL_Message_CarrierMessage,
                                       EMANELTE::MHAL::UE_UL_Message>(ue_ul_msg_, frequencyHz);

  auto prach    = carrier->mutable_prach();
  auto preamble = prach->mutable_preamble();

  preamble->set_index(index);

  Info("PRACH:%s: index %d\n", __func__, index);
}


/*typedef struct SRSRAN_API {
  srsran_cell_t    cell;
  uint16_t         current_rnti;
  bool             signals_pregenerated;
  srsran_pusch_t   pusch;
  srsran_pucch_t   pucch;
  srsran_ra_ul_pusch_hopping_t hopping;
} srsran_ue_ul_t;

 typedef struct SRSRAN_API {
  uint16_t         rnti;
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
  // Uplink config (includes common and dedicated variables)
  srsran_pucch_cfg_t                pucch;
  srsran_pusch_cfg_t                pusch;
  srsran_pusch_hopping_cfg_t        hopping;
  srsran_ue_ul_powerctrl_t          power_ctrl;
  srsran_refsignal_dmrs_pusch_cfg_t dmrs;
  srsran_refsignal_srs_cfg_t        srs;
} srsran_ul_cfg_t;

typedef struct SRSRAN_API {
  srsran_ul_cfg_t ul_cfg;
  bool            grant_available;
  uint32_t        cc_idx;
  bool            normalize_en;
  bool            cfo_en;
  float           cfo_tol;
  float           cfo_value;
} srsran_ue_ul_cfg_t; */

int ue_ul_put_pucch_i(srsran_ue_ul_t* q, 
                      srsran_ul_sf_cfg_t* sf,
                      srsran_ue_ul_cfg_t* cfg,
                      srsran_uci_value_t* uci_data,
                      uint32_t cc_idx)
{
   std::lock_guard<std::mutex> lock(ul_mutex_);

   const auto frequencyHz = getTxFrequency(cc_idx);

   auto carrier = getCarrierByFrequency<EMANELTE::MHAL::UE_UL_Message_CarrierMessage,
                                        EMANELTE::MHAL::UE_UL_Message>(ue_ul_msg_, frequencyHz);


   auto pucch_message = carrier->mutable_pucch();
   auto grant_message = pucch_message->add_grant();
   auto pucch_cfg     = cfg->ul_cfg.pucch;
   const auto rnti    = pucch_cfg.rnti;

   srsran_uci_value_t uci_value2 = *uci_data;

   // see lib/src/phy/ue/ue_ul.c
   srsran_ue_ul_pucch_resource_selection(&q->cell, &cfg->ul_cfg.pucch, &cfg->ul_cfg.pucch.uci_cfg, uci_data, uci_value2.ack.ack_value);

   // default: SRSRAN_PUCCH_FORMAT_1
   EMANELTE::MHAL::MOD_TYPE modType = EMANELTE::MHAL::MOD_BPSK;

   uint32_t bits = 0;

   switch(pucch_cfg.format)
     {
     case SRSRAN_PUCCH_FORMAT_1:   // 1 HARQ ACK
       bits = 0;
       modType = EMANELTE::MHAL::MOD_BPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_1A:  // 1 HARQ ACK
       bits = 1;
       modType = EMANELTE::MHAL::MOD_BPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_1B:  // 2 HARQ ACK
       bits = 2;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_2:   // CSI
       bits = 20;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_2A:  // CSI + 1 HARQ ACK
       bits = 21;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_2B:  // CSI + 2 HARQ ACK
       bits = 22;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_ERROR:
     default:
       Error("PUCCH:ue_ul_put_pucch: unknown pucch format: %d\n",
             pucch_cfg.format);
     }

   auto controlCarrier = getCarrierByFrequency<EMANELTE::MHAL::TxControlCarrierMessage, 
                                               EMANELTE::MHAL::TxControlMessage>(txControl_, frequencyHz);

   auto channelMessage = controlCarrier->mutable_uplink()->add_pucch();

   initUplinkChannelMessage(channelMessage,
                            EMANELTE::MHAL::CHAN_PUCCH,
                            modType,
                            bits);

   channelMessage->set_rnti(pucch_cfg.rnti);

   // see lib/src/phy/phch/pucch.c 
   // pucch_cp(srsran_pucch_t* q, 
   //          srsran_ul_sf_cfg_t* sf, 
   //          srsran_pucch_cfg_t* cfg, 
   //          cf_t* source, cf_t* dest,
   //          bool source_is_grid)
 
   // Determine n_prb
   uint16_t n_prb[2] = {0};

   for(int ns = 0; ns < 2; ++ns)
     {
       if(! ((n_prb[ns] = srsran_pucch_n_prb(&q->cell, &pucch_cfg, ns)) < q->cell.nof_prb))
         {
           Error("PUCCH:%s ns %d, n_prb=%d > cell_nof_prb %d\n", 
                 __func__, ns, n_prb[ns], q->cell.nof_prb);

           return SRSRAN_ERROR;
         }
     }

   // flag when resource blocks are different on slot 1 and 2 of the subframe
   channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(n_prb[0], frequencyHz));
   channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(n_prb[1], frequencyHz));

   grant_message->set_num_prb(n_prb[1]);
   grant_message->set_num_pucch(pucch_cfg.n_pucch);
   grant_message->set_rnti(rnti);
   grant_message->set_uci(&uci_value2, sizeof(srsran_uci_value_t));

#ifdef DEBUG_HEX
   InfoHex(&uci_value2, sizeof(srsran_uci_value_t), 
           "PUCCH:%s: rnti 0x%hx\n", __func__, rnti);
#else
   Debug("PUCCH:%s: rnti 0x%hx\n", __func__, rnti);
#endif

   // signal ready
   return 1;
}



/* typedef struct SRSRAN_API {
  srsran_mod_t mod;
  int          tbs;
  int          rv;
  uint32_t     nof_bits;
  uint32_t     cw_idx;
  bool         enabled;
} srsran_ra_tb_t;

 typedef struct SRSRAN_API {
  bool           is_from_rar;
  uint32_t       L_prb;
  uint32_t       n_prb[2];       // rb_start after frequency hopping
  uint32_t       n_prb_tilde[2]; // rb_start after frequency hopping per retx
  uint32_t       freq_hopping;
  uint32_t       nof_re;
  uint32_t       nof_symb;
  srsran_ra_tb_t tb;
  srsran_ra_tb_t last_tb;
  uint32_t       n_dmrs;
} srsran_pusch_grant_t;

typedef struct SRSRAN_API {
  srsran_uci_cfg_ack_t ack;
  srsran_cqi_cfg_t     cqi;
  bool                 is_scheduling_request_tti;
} srsran_uci_cfg_t;

typedef struct SRSRAN_API {
  uint16_t                rnti;
  srsran_uci_cfg_t        uci_cfg;
  srsran_uci_offset_cfg_t uci_offset;
  srsran_pusch_grant_t    grant;

  uint32_t max_nof_iterations;
  uint32_t last_O_cqi;
  uint32_t K_segm;
  uint32_t current_tx_nb;
  bool     csi_enable;
  bool     enable_64qam;

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
  uint8_t*           ptr;
  srsran_uci_value_t uci;
} srsran_pusch_data_t; */
static int ue_ul_put_pusch_i(srsran_pusch_cfg_t* cfg, srsran_pusch_data_t* data, uint32_t cc_idx)
{
   std::lock_guard<std::mutex> lock(ul_mutex_);

   const auto frequencyHz = getTxFrequency(cc_idx);

   auto controlCarrier = getCarrierByFrequency<EMANELTE::MHAL::TxControlCarrierMessage, 
                                              EMANELTE::MHAL::TxControlMessage>(txControl_, frequencyHz);

   auto channelMessage = controlCarrier->mutable_uplink()->add_pucch();

   const auto grant = &cfg->grant;
   const auto rnti  = cfg->rnti;

   initUplinkChannelMessage(channelMessage,
                            EMANELTE::MHAL::CHAN_PUSCH,
                            convert(grant->tb.mod),
                            grant->tb.tbs);

   channelMessage->set_rnti(rnti);

   for(size_t i = 0; i < grant->L_prb; ++i)
    {
      channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(grant->n_prb[0] + i, frequencyHz));
      channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(grant->n_prb[1] + i, frequencyHz));
    }

   auto carrier = getCarrierByFrequency<EMANELTE::MHAL::UE_UL_Message_CarrierMessage,
                                       EMANELTE::MHAL::UE_UL_Message>(ue_ul_msg_, frequencyHz);

   auto pusch_message = carrier->mutable_pusch();
   auto grant_message = pusch_message->add_grant();

   grant_message->set_rnti(rnti);

   // srsran_pusch_grant_t
   grant_message->set_ul_grant(grant, sizeof(srsran_pusch_grant_t));

   // srsran_uci_value_t
   grant_message->set_uci(&data->uci, sizeof(srsran_uci_value_t));

   // payload
   grant_message->set_payload(data->ptr, bits_to_bytes(grant->tb.tbs));

#ifdef DEBUG_HEX
   InfoHex(data->ptr, bits_to_bytes(grant->tb.tbs), 
           "PUSCH:%s: rnti 0x%hx\n", __func__, rnti);
#else
   Info("PUSCH:%s: rnti 0x%hx\n", __func__, rnti);
#endif

   UESTATS::putULGrant(rnti);

   // signal ready
   return 1;
}


/* typedef struct SRSRAN_API {
  srsran_cell_t    cell;
  uint16_t         current_rnti;
  bool             signals_pregenerated;
  srsran_pusch_t   pusch;
  srsran_pucch_t   pucch;
  srsran_ra_ul_pusch_hopping_t hopping;
} srsran_ue_ul_t;

typedef struct SRSRAN_API {
  srsran_tdd_config_t tdd_config;
  uint32_t            tti;
  bool                shortened;
} srsran_ul_sf_cfg_t;

typedef struct SRSRAN_API {
  // Uplink config (includes common and dedicated variables)
  srsran_pucch_cfg_t                pucch;
  srsran_pusch_cfg_t                pusch;
  srsran_pusch_hopping_cfg_t        hopping;
  srsran_ue_ul_powerctrl_t          power_ctrl;
  srsran_refsignal_dmrs_pusch_cfg_t dmrs;
  srsran_refsignal_srs_cfg_t        srs;
} srsran_ul_cfg_t;

typedef struct SRSRAN_API {
  srsran_ul_cfg_t ul_cfg;
  bool            grant_available;
  uint32_t        cc_idx;
  bool            normalize_en;
  bool            cfo_en;
  float           cfo_tol;
  float           cfo_value;
} srsran_ue_ul_cfg_t;

typedef struct SRSRAN_API {
  bool                   scheduling_request;
  srsran_cqi_value_t     cqi;
  srsran_uci_value_ack_t ack;
  uint8_t                ri; // Only 1-bit supported for RI
} srsran_uci_value_t;

typedef struct SRSRAN_API {
  uint8_t*           ptr;
  srsran_uci_value_t uci;
} srsran_pusch_data_t; */

// see lib/src/phy/ue/ue_ul.c
// srsran_ue_ul_encode(srsran_ue_ul_t* q, 
//                     srsran_ul_sf_cfg_t* sf,
//                     srsran_ue_ul_cfg_t* cfg,
//                     srsran_pusch_data_t* data);
//
int ue_ul_encode(srsran_ue_ul_t* q, srsran_ul_sf_cfg_t* sf, srsran_ue_ul_cfg_t* cfg, srsran_pusch_data_t* data, uint32_t cc_idx)
{
  /* Convert DTX to NACK in channel-selection mode (Release 10 only)*/
  if(cfg->ul_cfg.pucch.ack_nack_feedback_mode != SRSRAN_PUCCH_ACK_NACK_FEEDBACK_MODE_NORMAL) {
    uint32_t dtx_count = 0;
    for(uint32_t a = 0; a < srsran_uci_cfg_total_ack(&cfg->ul_cfg.pusch.uci_cfg); a++) {
      if(data->uci.ack.ack_value[a] == 2) {
        data->uci.ack.ack_value[a] = 0;
        dtx_count++;
      }
    }

    /* If all bits are DTX, do not transmit HARQ */
    if(dtx_count == srsran_uci_cfg_total_ack(&cfg->ul_cfg.pusch.uci_cfg)) {
      for (int i = 0; i < 2; i++) { // Format 1b-CS only supports 2 CC
       cfg->ul_cfg.pusch.uci_cfg.ack[i].nof_acks = 0;
      }
    }
  }

  ue_set_crnti_i(cfg->ul_cfg.pucch.rnti);

   // see lib/src/phy/ue/ue_ul.c
#define uci_pending(cfg) (srsran_uci_cfg_total_ack(&cfg) > 0 || cfg.cqi.data_enable || cfg.cqi.ri_len > 0)
   if(cfg->grant_available) 
    {
      return ue_ul_put_pusch_i(&cfg->ul_cfg.pusch, data, cc_idx);
    } 
   else if((uci_pending(cfg->ul_cfg.pucch.uci_cfg) || 
            data->uci.scheduling_request) && cfg->cc_idx == 0)
    // Send PUCCH over PCell only
    {
      return ue_ul_put_pucch_i(q, sf, cfg, &data->uci, cc_idx);
    }
   else
    {
      return 0;
    }
}

} // end namespace phy_adapter
} // end namepsace srsue
