/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsLTE library.
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

// J Giovatto May 06, 2021
// The approach is simple, the enb and ue transmit raw IQ data which
// would have been sent to the radio device driver (UHD) etc. Samples
// are sent and received verbatim except in the case where the ue is in
// cell search where downsampling is applied. At some point some artifical
// "jammming" of the data stream my be desireable by substiting some random values.
// Shared memory was chosen over unix/inet sockets to allow for the fastest
// data transfer and possible combining IQ data in a single buffer. Each ul and dl subframe
// worth of iqdata and metadata is held in a "sf_bin" where tx sf_bin index is 4 sf 
// ahead of the rx sf_bin index. 
//
// The enb creates a shared memory segment for each channel or port, to keep things simple
// we simple use the earfcn id. So far a 2 carrier enb we set 2 shared memory segments or ports.
// A shared semaphore syncronization object is created for each subframe within a port.
//
// Each ue can then attach to a particular shared memory segment again specified by the port id.
//
// see /dev/shm for shared memory segemnts, these may be orphaned if close is not called on termination.
//
// see the semaphores and shmem objects list in /dev/shm
//
// changes to ue and enb conf resp:
// device_name = shmem
// device_args = type=enb,port0=0,port1=1
// device_args = type=ue,port0=0
//
// 1) sudo ./srsepc ./epc.conf
// 2) sudo ./srsenb ./enb.conf.fauxrf
// 3) sudo ./srsue  ./ue.conf.fauxrf
//
// see https://github.com/pgorczak/srslte-docker-emulated for running on docker
//
// see top level demo dir for running with lxc containers.

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <fcntl.h>    /* For O_* constants */

#include <sys/time.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/select.h>
#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */

#include "srsran/srsran.h"
#include "rf_shmem_imp.h"
#include "rf_helper.h"
#include "srsran/phy/rf/rf.h"
#include "srsran/phy/resampling/resample_arb.h"

// define to allow debug
#undef RF_SHMEM_DEBUG_MODE

#ifdef RF_SHMEM_DEBUG_MODE
static bool rf_shmem_log_dbug = true;
static bool rf_shmem_log_info = true;
static bool rf_shmem_log_warn = true;
#else
static bool rf_shmem_log_dbug = false;
static bool rf_shmem_log_info = true;
static bool rf_shmem_log_warn = true;
#endif

static char rf_shmem_node_type = ' ';

#define RF_SHMEM_LOG_FMT "%02d:%02d:%02d.%06ld [SRF.%c] [%c] %s,  "

#define RF_SHMEM_LOG(_lvl, output, _C, _fmt, ...) do {                                  \
                                 if(_lvl) {                                             \
                                   struct timeval _tv_now;                              \
                                   struct tm _tm[1];                                    \
                                   gettimeofday(&_tv_now, NULL);                        \
                                   localtime_r(&_tv_now.tv_sec, &_tm[0]);               \
                                   fprintf(output, RF_SHMEM_LOG_FMT _fmt "\n",          \
                                           _tm[0].tm_hour,                              \
                                           _tm[0].tm_min,                               \
                                           _tm[0].tm_sec,                               \
                                           _tv_now.tv_usec,                             \
                                           rf_shmem_node_type,                          \
                                           _C,                                          \
                                           __func__,                                    \
                                           ##__VA_ARGS__);                              \
                                     }                                                  \
                                 } while(0);


#define RF_SHMEM_WARN(_fmt, ...) RF_SHMEM_LOG(rf_shmem_log_warn, stderr, 'W', _fmt, ##__VA_ARGS__)
#define RF_SHMEM_DBUG(_fmt, ...) RF_SHMEM_LOG(rf_shmem_log_dbug, stdout, 'D', _fmt, ##__VA_ARGS__)
#define RF_SHMEM_INFO(_fmt, ...) RF_SHMEM_LOG(rf_shmem_log_info, stdout, 'I', _fmt, ##__VA_ARGS__)

// bytes per sample
#define RF_SHMEM_BYTES_X_SAMPLE(x) ((x)*sizeof(cf_t))

// samples per byte
#define RF_SHMEM_SAMPLES_X_BYTE(x) ((x)/sizeof(cf_t))

#define RF_SHMEM_NTYPE_NONE  (0)  
#define RF_SHMEM_NTYPE_UE    (1)  
#define RF_SHMEM_NTYPE_ENB   (2)

#define RF_SHMEM_NUM_SF_X_FRAME 10

static const struct timeval tv_zero = {0,0};
static const struct timeval tv_sf   = {0,1000}; // 1 sf
static const struct timeval tv_4sf  = {0,4000}; // 4 sf

// msg element meta data
typedef struct {
  uint64_t       seqnum;                      // seq num
  uint32_t       nof_bytes;                   // num bytes
  uint32_t       nof_sf;                      // num subframes
  float          tx_srate;                    // tx sample rate
  struct timeval tv_tx_tti;                   // tti time (tti + 4)
  struct timeval tv_tx_time;                  // actual tx time
  int            is_sob;                      // is start of burst
  int            is_eob;                      // is end of burst
} rf_shmem_element_meta_t;


// sf len at nprb=100 is 184320 bytes or 23040 samples
// subtract the other fields of the struct to align mem size to 256k per sf_bin
// to avoid shmget failure
#define RF_SHMEM_MAX_CF_LEN RF_SHMEM_SAMPLES_X_BYTE((256000 - sizeof(rf_shmem_element_meta_t)))

// msg element (not for stack allocation)
typedef struct {
  rf_shmem_element_meta_t meta;                        // meta data
  cf_t                    iqdata[RF_SHMEM_MAX_CF_LEN]; // data
} rf_shmem_element_t;

sem_t * sem[SRSRAN_MAX_CHANNELS][RF_SHMEM_NUM_SF_X_FRAME] = {{0}};  // element r/w sf_bin locks

// msg element bins 1 for each sf (tti)
typedef struct {
  rf_shmem_element_t elements[RF_SHMEM_NUM_SF_X_FRAME];
} rf_shmem_segment_t;

#define RF_SHMEM_DATA_SEGMENT_SIZE sizeof(rf_shmem_segment_t)

const char * printMsg(const rf_shmem_element_t * element, char * buff, int buff_len)
 {
   snprintf(buff, buff_len, "seqnum %lu, nof_bytes %u, nof_sf %u, srate %6.4f MHz, tti_tx %ld:%06ld, sob %d, eob %d",
            element->meta.seqnum,
            element->meta.nof_bytes,
            element->meta.nof_sf,
            element->meta.tx_srate/1e6,
            element->meta.tv_tx_tti.tv_sec,
            element->meta.tv_tx_tti.tv_usec,
            element->meta.is_sob,
            element->meta.is_eob);
    
    return buff;
 }

// rf dev info
typedef struct {
   const char *              dev_name;
   int                       nodetype;
   double                    rx_gain;
   double                    tx_gain;
   double                    rx_srate;
   double                    tx_srate;
   double                    rx_freq[SRSRAN_MAX_CHANNELS];
   double                    tx_freq[SRSRAN_MAX_CHANNELS];
   double                    clock_rate;
   srsran_rf_error_handler_t error_handler;
   void *                    error_arg;
   bool                      rx_stream;
   uint64_t                  tx_seqnum;
   pthread_mutex_t           state_lock;
   struct timeval            tv_sos;      // start of stream
   struct timeval            tv_this_tti;
   struct timeval            tv_next_tti;
   size_t                    tx_nof_late;
   size_t                    tx_nof_drop;
   srsran_rf_info_t          rf_info;
   int                       shm_dl_fd[SRSRAN_MAX_CHANNELS];
   int                       shm_ul_fd[SRSRAN_MAX_CHANNELS];
   void *                    shm_dl[SRSRAN_MAX_CHANNELS];       // dl shared mem
   void *                    shm_ul[SRSRAN_MAX_CHANNELS];       // ul shared mem
   rf_shmem_segment_t *      rx_segment[SRSRAN_MAX_CHANNELS];   // rx bins
   rf_shmem_segment_t *      tx_segment[SRSRAN_MAX_CHANNELS];   // tx bins
   uint32_t                  tx_loss;                           // random loss 0=none, 100=all
   uint32_t                  nof_channels;                      // num carriers/channels ???
   char                      ports[SRSRAN_MAX_CHANNELS][256];   // shmem port ids
} rf_shmem_state_t;



void rf_shmem_suppress_stdout(void *h)
 {
#ifndef RF_SHMEM_DEBUG_MODE
    rf_shmem_log_dbug = false;
#endif
 }


static inline time_t tv_to_usec(const struct timeval * tv)
 {
   return (tv->tv_sec * 1000000) + tv->tv_usec;
 }


static inline uint32_t get_sf_bin(const struct timeval * tv)
 {
   return (tv_to_usec(tv) / tv_to_usec(&tv_sf)) % RF_SHMEM_NUM_SF_X_FRAME;
 }


static void rf_shmem_handle_error(void * arg, srsran_rf_error_t error)
{
  // XXX TODO make use of this handler
  printf("type %s", 
          error.type == SRSRAN_RF_ERROR_LATE      ? "late"      :
          error.type == SRSRAN_RF_ERROR_UNDERFLOW ? "underflow" :
          error.type == SRSRAN_RF_ERROR_OVERFLOW  ? "overflow"  :
          error.type == SRSRAN_RF_ERROR_OTHER     ? "other"     :
          "unknown error");
}

static rf_shmem_state_t rf_shmem_state = { .dev_name      = "shmem",
                                           .nodetype      = RF_SHMEM_NTYPE_NONE,
                                           .rx_gain       = 0.0,
                                           .tx_gain       = 0.0,
                                           .rx_srate      = SRSRAN_CS_SAMP_FREQ,
                                           .tx_srate      = SRSRAN_CS_SAMP_FREQ,
                                           .rx_freq       = {0.0},
                                           .tx_freq       = {0.0},
                                           .clock_rate    = 0.0,
                                           .error_handler = rf_shmem_handle_error,
                                           .error_arg     = NULL,
                                           .rx_stream     = false,
                                           .tx_seqnum     = 0,
                                           .state_lock    = PTHREAD_MUTEX_INITIALIZER,
                                           .tv_sos        = {},
                                           .tv_this_tti   = {},
                                           .tv_next_tti   = {},
                                           .tx_nof_late   = 0,
                                           .tx_nof_drop   = 0,
                                           .rf_info       = {},
                                           .shm_dl_fd     = {-1},
                                           .shm_ul_fd     = {-1},
                                           .shm_dl        = {NULL},
                                           .shm_ul        = {NULL},
                                           .rx_segment    = {NULL},
                                           .tx_segment    = {NULL},
                                           .tx_loss       = 0,
                                           .nof_channels  = 0,
                                         };

#define RF_SHMEM_GET_STATE(h)  if(!h) printf("NULL handle in call to %s !!!\n", __func__); \
                                 rf_shmem_state_t *_state = (rf_shmem_state_t *)(h)


#define  RF_SHMEM_DL_FMT  "/shmemrf_shmem_dl_port_%s"
#define  RF_SHMEM_UL_FMT  "/shmemrf_shmem_ul_port_%s"
#define  RF_SHMEM_SEM_FMT "/shmemrf_sem_port_%s_sf_%02d"

static inline bool rf_shmem_is_enb(rf_shmem_state_t * state)
{
  return (state->nodetype == RF_SHMEM_NTYPE_ENB);
}

// timeval to full and frac seconds
static inline void rf_shmem_tv_to_fs(const struct timeval *tv, time_t *full_secs, double *frac_secs)
{
  if(full_secs && frac_secs)
    {
      *full_secs = tv->tv_sec; 
      *frac_secs = tv->tv_usec / 1.0e6;
    }
}

// get fractional seconds from a timeval
static inline double rf_shmem_get_fs(const struct timeval *tv)
{
  return tv->tv_sec + tv->tv_usec / 1.0e6;
}


// XXX this could be a place where we introduce noise/jamming effects
static int rf_shmem_resample(double srate_in, 
                             double srate_out, 
                             void * data_in, 
                             void * data_out,
                             int nbytes)
{
  // downsample needed during initial sync since ue is at lowest sample rate
  if(srate_in && srate_out && (srate_in != srate_out))
   {
     const double sratio = srate_out / srate_in;

     // 'upsample' unable to sync
     if(sratio > 1.0)
      { 
        RF_SHMEM_WARN("srate %4.2lf/%4.2lf MHz, sratio %3.3lf, upsample may not decode",
                     srate_in/1e6,
                     srate_out/1e6,
                     sratio);
      }
     else
      {
        RF_SHMEM_DBUG("srate %4.2lf/%4.2lf MHz, sratio %3.3lf",
                     srate_in/1e6,
                     srate_out/1e6,
                     sratio);
      }

     srsran_resample_arb_t r;
     srsran_resample_arb_init(&r, sratio, 0);

     return RF_SHMEM_BYTES_X_SAMPLE(srsran_resample_arb_compute(&r, 
                                                                (cf_t*)data_in, 
                                                                (cf_t*)data_out, 
                                                                RF_SHMEM_SAMPLES_X_BYTE(nbytes)));
   }
  else
   {
     // no resampling needed
     memcpy(data_out, data_in, nbytes);

     return nbytes;
   }
}



static int rf_shmem_open_ipc(rf_shmem_state_t * state, uint32_t channel)
{
  int dl_shm_flags = 0, ul_shm_flags = 0;

  mode_t mode = S_IRWXU;// | S_IRWXG | S_IRWXO;

  if(rf_shmem_is_enb(state))
   {
     rf_shmem_node_type = 'E';

     // enb create shmem segments
     dl_shm_flags = O_CREAT | O_TRUNC | O_RDWR;
     ul_shm_flags = O_CREAT | O_TRUNC | O_RDWR;
   }
  else
   {
     rf_shmem_node_type = 'U';

     dl_shm_flags = O_RDWR;
     ul_shm_flags = O_RDWR;
   }

  // dl shm key
  char shmem_name[256];

  snprintf(shmem_name, sizeof(shmem_name), RF_SHMEM_DL_FMT, state->ports[channel]);

  if((state->shm_dl_fd[channel] = shm_open(shmem_name, dl_shm_flags, mode)) < 0)
   {
     RF_SHMEM_WARN("failed to get shm_dl_fd for %s, %s, skipping", shmem_name, strerror(errno));

     return -1;
   }
  else
   {
     if(rf_shmem_is_enb(state))
      {
        ftruncate(state->shm_dl_fd[channel], RF_SHMEM_DATA_SEGMENT_SIZE);
      }
     RF_SHMEM_INFO("got shm_dl_fd for %s", shmem_name);
    }
   

  snprintf(shmem_name, sizeof(shmem_name), RF_SHMEM_UL_FMT, state->ports[channel]);

  if((state->shm_ul_fd[channel] = shm_open(shmem_name, ul_shm_flags, mode)) < 0)
   {
     RF_SHMEM_WARN("failed to get shm_ul_fd for %s, %s, skipping", shmem_name, strerror(errno));

     return -1;
   }
  else
   {
    if(rf_shmem_is_enb(state))
      {
        ftruncate(state->shm_ul_fd[channel], RF_SHMEM_DATA_SEGMENT_SIZE);
      }
     RF_SHMEM_INFO("got shm_ul_fd for %s", shmem_name);
   }

  // dl shm addr
  if((state->shm_dl[channel] = 
             mmap(0, 
                  RF_SHMEM_DATA_SEGMENT_SIZE, 
                  PROT_READ | PROT_WRITE, MAP_SHARED, 
                  state->shm_dl_fd[channel], 0)) == MAP_FAILED)
   {
     RF_SHMEM_WARN("failed to map shm_dl %s", strerror(errno));

     rf_shmem_close(state);

     return -1;
   }

  // ul shm addr
  if((state->shm_ul[channel] = 
             mmap(0, 
             RF_SHMEM_DATA_SEGMENT_SIZE,
             PROT_READ | PROT_WRITE, MAP_SHARED,
             state->shm_ul_fd[channel], 0)) == MAP_FAILED)
   {
     RF_SHMEM_WARN("failed to map shm_ul %s", strerror(errno));

     rf_shmem_close(state);

     return -1;
   }

  // set ul/dl bins
  if(rf_shmem_is_enb(state))
   {
     state->tx_segment[channel] = (rf_shmem_segment_t *) state->shm_dl[channel];
     state->rx_segment[channel] = (rf_shmem_segment_t *) state->shm_ul[channel];
   }
  else
   {
     state->tx_segment[channel] = (rf_shmem_segment_t *) state->shm_ul[channel];
     state->rx_segment[channel] = (rf_shmem_segment_t *) state->shm_dl[channel];
   }


  // shared sems, 1 for each sf_bin
  for(int sf = 0; sf < RF_SHMEM_NUM_SF_X_FRAME; ++sf)
   {
     snprintf(shmem_name, sizeof(shmem_name), RF_SHMEM_SEM_FMT, state->ports[channel], sf);

     if(rf_shmem_is_enb(state))
      {
        // cleanup any orphans
        sem_unlink(shmem_name);
 
        // initial value 1
        if((sem[channel][sf] = sem_open(shmem_name, O_CREAT, 0600, 1)) == NULL)
         {
           RF_SHMEM_WARN("failed to create sem %s, %s", shmem_name, strerror(errno));

           rf_shmem_close(state);

           return -1;
         }
        else
         {
           RF_SHMEM_INFO("created sem %s", shmem_name);
         }
      }
     else
      {
        if((sem[channel][sf] = sem_open(shmem_name, 0)) == NULL)
         {
           RF_SHMEM_WARN("failed to open sem %s, %s", shmem_name, strerror(errno));

           rf_shmem_close(state);

           return -1;
         }
        else
         {
           RF_SHMEM_INFO("opened sem %s", shmem_name);
         }
      }
   }

  // clear data segments
  memset(state->shm_ul[channel], 0x0, RF_SHMEM_DATA_SEGMENT_SIZE);
  memset(state->shm_dl[channel], 0x0, RF_SHMEM_DATA_SEGMENT_SIZE);

  return 0;
}



static void rf_shmem_wait_next_tti(void *h, struct timeval * tv_ref)
{
   RF_SHMEM_GET_STATE(h);

   struct timeval tv_diff = {0,0};

   // this is where we set the pace for the system TTI
   timersub(&_state->tv_next_tti, tv_ref, &tv_diff);

   if(timercmp(&tv_diff, &tv_zero, >))
    {
      RF_SHMEM_DBUG("wait %6.6lf for next tti", rf_shmem_get_fs(&tv_diff));
      select(0, NULL, NULL, NULL, &tv_diff);
    }
   else
    {
      RF_SHMEM_DBUG("late %6.6lf for this tti", rf_shmem_get_fs(&tv_diff));
    }

   _state->tv_this_tti = _state->tv_next_tti;

   timeradd(&_state->tv_next_tti, &tv_sf, &_state->tv_next_tti);

   gettimeofday(tv_ref, NULL);
}



// ************ begin RF API ************

const char* rf_shmem_devname(void *h)
 {
   RF_SHMEM_GET_STATE(h);

   return _state->dev_name;
 }


int rf_shmem_start_rx_stream(void *h, bool now)
 {
   RF_SHMEM_GET_STATE(h);
   
   pthread_mutex_lock(&_state->state_lock);

   gettimeofday(&_state->tv_sos, NULL);

   // aligin time on the next full second
   if(_state->tv_sos.tv_usec > 0)
    {
      usleep(1000000 - _state->tv_sos.tv_usec);
   
      _state->tv_sos.tv_sec += 1;
      _state->tv_sos.tv_usec = 0;
    }

   // initial tti and next
   _state->tv_this_tti = _state->tv_sos;
   timeradd(&_state->tv_sos, &tv_sf, &_state->tv_next_tti);

   RF_SHMEM_INFO("start rx stream, time_0 %ld:%06ld, next_tti %ld:%06ld", 
                 _state->tv_sos.tv_sec, 
                 _state->tv_sos.tv_usec,
                 _state->tv_next_tti.tv_sec, 
                 _state->tv_next_tti.tv_usec);

   _state->rx_stream = true;

   pthread_mutex_unlock(&_state->state_lock);

   return 0;
 }


int rf_shmem_stop_rx_stream(void *h)
 {
   RF_SHMEM_GET_STATE(h);

   pthread_mutex_lock(&_state->state_lock);

   RF_SHMEM_INFO("end rx stream");

   _state->rx_stream = false;

   pthread_mutex_unlock(&_state->state_lock);

   return 0;
 }


void rf_shmem_flush_buffer(void *h)
 {
   RF_SHMEM_INFO("todo");
 }


bool rf_shmem_has_rssi(void *h)
 {
   return false;
 }


float rf_shmem_get_rssi(void *h)
 {
   const float rssi = 1.0;

   RF_SHMEM_INFO("rssi %4.3f", rssi);

   return rssi;
 }


void rf_shmem_register_error_handler(void *h, srsran_rf_error_handler_t error_handler, void * arg)
 {
   RF_SHMEM_GET_STATE(h);

   _state->error_handler = error_handler;
   _state->error_arg     = arg;
 }


int rf_shmem_open(char *args, void **h)
 {
   return rf_shmem_open_multi(args, h, 1);
 }


int rf_shmem_open_multi(char *args, void **h, uint32_t nof_channels)
 {
   RF_SHMEM_INFO("channels %u, args [%s]", nof_channels, args ? args : "none");

   *h = NULL;

   if(! (nof_channels < SRSRAN_MAX_CHANNELS))
    {
      RF_SHMEM_INFO("channels %u !< MAX %u", nof_channels, SRSRAN_MAX_CHANNELS);

      return -1;
    }

   rf_shmem_state_t * state = &rf_shmem_state;

   state->nof_channels = nof_channels;

   if(args && strlen(args))
    {
      char tmp_str[256] = {0};
 
      parse_string(args, "type", -1, tmp_str);

      // get node type
      if(! strncmp(tmp_str, "enb", strlen("enb")))
       {
         state->nodetype = RF_SHMEM_NTYPE_ENB;
       }
      else if(! strncmp(tmp_str, "ue", strlen("ue")))
       {
         state->nodetype = RF_SHMEM_NTYPE_UE;
       }
      else
       {
         RF_SHMEM_WARN("unexpected node type %s\n", args);
       }

      if(state->nodetype == RF_SHMEM_NTYPE_NONE)
       {
          RF_SHMEM_WARN("expected type ue or end\n");

          return -1;
       }

      // get optional loss
      parse_uint32(args, "loss", -1, &state->tx_loss);
    }

   int num_opened_channels = 0;

   for(uint32_t channel = 0; channel < nof_channels; ++channel)
    {
      // get the port id(s)
      parse_string(args, "port", channel, state->ports[channel]);

      if(rf_shmem_open_ipc(state, channel) < 0)
       {
         RF_SHMEM_WARN("did not find channel %u", channel);
       }
      else
       {
         RF_SHMEM_INFO("found channel %u", channel);

         ++num_opened_channels;
       }
    }

   *h = state;

   return num_opened_channels > 0 ? 0 : -1;
 }


int rf_shmem_close(void *h)
 {
   // XXX this does not seem to get called on shutdown as othen as I'd expect

   RF_SHMEM_GET_STATE(h);

   for(uint32_t channel = 0; channel < _state->nof_channels; ++channel)
    {
      char shmem_name[256] = {0};

      // enb creats/cleans up all shared resources
      if(rf_shmem_is_enb(_state))
       {
         if(_state->shm_dl[channel])
          {
            munmap(_state->shm_dl[channel], RF_SHMEM_DATA_SEGMENT_SIZE);

            snprintf(shmem_name, sizeof(shmem_name), RF_SHMEM_DL_FMT, _state->ports[channel]);

            shm_unlink(shmem_name);

            close(_state->shm_dl_fd[channel]);

            _state->shm_dl_fd[channel] = -1;

            _state->shm_dl[channel] = NULL;
          }

         if(_state->shm_ul[channel])
          {
            munmap(_state->shm_ul[channel], RF_SHMEM_DATA_SEGMENT_SIZE);

            snprintf(shmem_name, sizeof(shmem_name), RF_SHMEM_UL_FMT, _state->ports[channel]);

            shm_unlink(shmem_name);

            close(_state->shm_ul_fd[channel]);

            _state->shm_ul_fd[channel] = -1;

            _state->shm_ul[channel] = NULL;
          }
       }

      for(int sf = 0; sf < RF_SHMEM_NUM_SF_X_FRAME; ++sf)
       {
        if(sem[channel][sf])
         {
           snprintf(shmem_name, sizeof(shmem_name), RF_SHMEM_SEM_FMT, _state->ports[channel], sf);

           sem_close(sem[channel][sf]);

           sem[channel][sf] = NULL;
         }
      }
    }

   return 0;
 }


int rf_shmem_set_rx_gain(void *h, double gain)
 {
   RF_SHMEM_GET_STATE(h);

   if(_state->rx_gain != gain) {
     RF_SHMEM_DBUG("gain %3.2lf to %3.2lf", _state->rx_gain, gain);

     _state->rx_gain = gain;
   }

   return SRSRAN_SUCCESS;
 }


int rf_shmem_set_tx_gain(void *h, double gain)
 {
   RF_SHMEM_GET_STATE(h);

   if(_state->tx_gain != gain) {
     RF_SHMEM_DBUG("gain %3.2lf to %3.2lf", _state->tx_gain, gain);

     _state->tx_gain = gain;
   }

   return SRSRAN_SUCCESS;
 }


srsran_rf_info_t * rf_shmem_get_rf_info(void *h)
  {
     RF_SHMEM_GET_STATE(h);

     RF_SHMEM_DBUG("tx_gain min/max %3.2lf/%3.2lf, rx_gain min/max %3.2lf/%3.2lf",
                  _state->rf_info.min_tx_gain,
                  _state->rf_info.max_tx_gain,
                  _state->rf_info.min_rx_gain,
                  _state->rf_info.max_rx_gain);

     return &_state->rf_info;
  }


double rf_shmem_get_rx_gain(void *h)
 {
   RF_SHMEM_GET_STATE(h);

   RF_SHMEM_DBUG("gain %3.2lf", _state->rx_gain);

   return _state->rx_gain;
 }


double rf_shmem_get_tx_gain(void *h)
 {
   RF_SHMEM_GET_STATE(h);

   RF_SHMEM_DBUG("gain %3.2lf", _state->tx_gain);

   return _state->tx_gain;
 }


double rf_shmem_set_rx_srate(void *h, double rate)
 {
   RF_SHMEM_GET_STATE(h);

   pthread_mutex_lock(&_state->state_lock);

   if(_state->rx_srate != rate) {
     RF_SHMEM_INFO("srate %4.2lf MHz to %4.2lf MHz", 
                   _state->rx_srate / 1e6, rate / 1e6);

     _state->rx_srate = rate;
   }

   pthread_mutex_unlock(&_state->state_lock);

   return _state->rx_srate;
 }


double rf_shmem_set_tx_srate(void *h, double rate)
 {
   RF_SHMEM_GET_STATE(h);

   pthread_mutex_lock(&_state->state_lock);

   if(_state->tx_srate != rate) {
     RF_SHMEM_INFO("srate %4.2lf MHz to %4.2lf MHz", 
                   _state->tx_srate / 1e6, rate / 1e6);

     _state->tx_srate = rate;
   }

   pthread_mutex_unlock(&_state->state_lock);

   return _state->tx_srate;
 }


double rf_shmem_set_rx_freq(void *h, uint32_t ch, double freq)
 {
   RF_SHMEM_GET_STATE(h);

   RF_SHMEM_INFO("ch %u, freq %4.2lf MHz to %4.2lf MHz", 
                 ch, _state->rx_freq[ch] / 1e6, freq / 1e6);

   _state->rx_freq[ch] = freq;

   return _state->rx_freq[ch];
 }


double rf_shmem_set_tx_freq(void *h, uint32_t ch, double freq)
 {
   RF_SHMEM_GET_STATE(h);

   if(_state->tx_freq[ch] != freq) {
     RF_SHMEM_INFO("ch %u, freq %4.2lf MHz to %4.2lf MHz", 
                   ch, _state->tx_freq[ch] / 1e6, freq / 1e6);

     _state->tx_freq[ch] = freq;
   }

   return _state->tx_freq[ch];
 }



void rf_shmem_get_time(void *h, time_t *full_secs, double *frac_secs)
 {
   RF_SHMEM_GET_STATE(h);

   rf_shmem_tv_to_fs(&_state->tv_this_tti, full_secs, frac_secs);
 }



int rf_shmem_recv_with_time(void *h, void *data, uint32_t nsamples, 
                            bool blocking, time_t *full_secs, double *frac_secs)
 {
   void *d[SRSRAN_MAX_PORTS] = {data, NULL, NULL, NULL};

   return rf_shmem_recv_with_time_multi(h, 
                                       d,
                                       nsamples, 
                                       blocking,
                                       full_secs,
                                       frac_secs);
 }



int rf_shmem_recv_with_time_multi(void *h, void **data, uint32_t nsamples, 
                                  bool blocking, time_t *full_secs, double *frac_secs)
 {
   RF_SHMEM_GET_STATE(h);

   pthread_mutex_lock(&_state->state_lock);

   // nof bytes requested
   const uint32_t nbytes = RF_SHMEM_BYTES_X_SAMPLE(nsamples);

   // working in units of subframes
   const int nof_sf = (nsamples / (_state->rx_srate / 1000.0f));

   RF_SHMEM_DBUG("nof_samples %u, nbytes %u, nof_sf %d", nsamples, nbytes, nof_sf);

   uint32_t offset[_state->nof_channels];

   for(uint32_t channel = 0; channel < _state->nof_channels; ++channel)
    {
      offset[channel] = 0;
    }

   struct timeval tv_now;

   // for each requested subframe
   for(int sf = 0; sf < nof_sf; ++sf)
    { 
      gettimeofday(&tv_now, NULL);

      // wait for the next tti
      rf_shmem_wait_next_tti(h, &tv_now);

      // find sf_bin for this tti
      const uint32_t sf_bin = get_sf_bin(&_state->tv_this_tti);

      for(uint32_t channel = 0; channel < _state->nof_channels; ++channel)
       { 
         if(data[channel] && _state->rx_freq[channel])
          {
            // lock this sf_bin
            if(sem_wait(sem[channel][sf_bin]) < 0)
             {
               RF_SHMEM_WARN("sem_wait error %s", strerror(errno));
             }

            memset(data[channel], 0x0, nbytes);

            rf_shmem_element_t * element = &_state->rx_segment[channel]->elements[sf_bin];
 
#if 0
            char logbuff[256] = {0};
            RF_SHMEM_INFO("RX, TTI %ld:%06ld, channel %u, sf_bin %u, offset %u, %s", 
                           _state->tv_this_tti.tv_sec, _state->tv_this_tti.tv_usec, 
                           channel, sf_bin, offset[channel], printMsg(element, logbuff, sizeof(logbuff)));
#endif
      
            // check current tti w/sf_bin tti 
            if(timercmp(&_state->tv_this_tti, &element->meta.tv_tx_tti, ==))
             {
               const int result = rf_shmem_resample(element->meta.tx_srate,
                                                    _state->rx_srate,
                                                    element->iqdata,
                                                    ((uint8_t*)data[channel]) + offset[channel],
                                                    element->meta.nof_bytes);

               offset[channel] += result;
             }

            if(rf_shmem_is_enb(_state) && (channel == _state->nof_channels - 1))
             {
               // last enb worker clear ul sf_bin
               memset(element, 0x0, sizeof(*element));
             }

            // unlock
            sem_post(sem[channel][sf_bin]);
          }
       }
    }

   // set rx timestamp to this tti
   rf_shmem_tv_to_fs(&_state->tv_this_tti, full_secs, frac_secs);

   pthread_mutex_unlock(&_state->state_lock);

   return nsamples;
 }


int rf_shmem_send_timed(void *h, void *data, int nsamples,
                       time_t full_secs, double frac_secs, bool has_time_spec,
                       bool blocking, bool is_sob, bool is_eob)
 {
   void *d[SRSRAN_MAX_PORTS] = {data, NULL, NULL, NULL};

   return rf_shmem_send_timed_multi(h, d, nsamples, full_secs, frac_secs, has_time_spec, blocking, is_sob, is_eob);
 }


int rf_shmem_send_timed_multi(void *h, void **data, int nsamples,
                              time_t full_secs, double frac_secs, bool has_time_spec,
                              bool blocking, bool is_sob, bool is_eob)
 {
   RF_SHMEM_GET_STATE(h);

   RF_SHMEM_DBUG("msg len %d, sob %d, eob %d", nsamples, is_sob, is_eob);

   if(nsamples == 0)
    {
      return SRSRAN_SUCCESS;
    }

   // some random loss the higher the number the more loss (0-100)
   if(_state->tx_loss > 0)
    {
      if((rand() % 100) < _state->tx_loss)
       {
         ++_state->tx_nof_drop;
         return SRSRAN_SUCCESS;
       }
    } 

   struct timeval tv_now, tv_tx_tti;

   // all tx are 4 tti in the future
   // code base may advance timespec slightly which can mess up our sf_bin index
   // so we just force the tx_time here to be exactly 4 sf ahead
   timeradd(&_state->tv_this_tti, &tv_4sf, &tv_tx_tti);

   gettimeofday(&tv_now, NULL);

   // this msg tx tti time has passed, running late
   if(timercmp(&tv_tx_tti, &tv_now, <))
    {
      struct timeval tv_diff;

      timersub(&tv_tx_tti, &tv_now, &tv_diff);

      if((++_state->tx_nof_late % 10) == 0)
        RF_SHMEM_WARN("TX late, tx_tti %ld:%06ld, overrun %6.6lf, total late %zu",
                      tv_tx_tti.tv_sec,
                      tv_tx_tti.tv_usec,
                      -rf_shmem_get_fs(&tv_diff),
                      _state->tx_nof_late);
    }
   else
    {
      const uint32_t nbytes = RF_SHMEM_BYTES_X_SAMPLE(nsamples);

      // get the sf_bin for this tx_tti
      const uint32_t sf_bin = get_sf_bin(&tv_tx_tti);

      for(uint32_t channel = 0; channel < _state->nof_channels; ++channel)
       {
         if(data[channel] && _state->tx_freq[channel])
          {
            // lock this sf_bin
            if(sem_wait(sem[channel][sf_bin]) < 0)
             {
               RF_SHMEM_WARN("sem_wait error %s", strerror(errno));
             }

            rf_shmem_element_t * element = &_state->tx_segment[channel]->elements[sf_bin];

            // first enb worker clear bin
            if(rf_shmem_is_enb(_state) && (channel == 0))
             {
               // clear dl sf_bin 
               memset(element, 0x0, sizeof(*element));
             }

            // is a fresh sf_bin entry
            if(element->meta.nof_sf == 0)
             {
               // load the bin with i/q data
               memcpy(element->iqdata, data[channel], nbytes);

               // set meta data
               element->meta.is_sob     = is_sob;
               element->meta.is_eob     = is_eob;
               element->meta.tx_srate   = _state->tx_srate;
               element->meta.seqnum     = _state->tx_seqnum++;
               element->meta.nof_bytes  = nbytes;
               element->meta.tv_tx_time = tv_now;
               element->meta.tv_tx_tti  = tv_tx_tti;
             }
            else
             {
               cf_t * q = (cf_t*)data[channel];

               // combine
               for(int i = 0; i < nsamples; ++i)
                {
                  element->iqdata[i] += q[i];
                }
             }

            // bump write count
            ++element->meta.nof_sf;

            // unlock
            sem_post(sem[channel][sf_bin]);

#if 0
            char logbuff[256] = {0};
            RF_SHMEM_INFO("TX, TTI %ld:%06ld, channel %u, sf_bin %u, %s", 
                          _state->tv_this_tti.tv_sec, _state->tv_this_tti.tv_usec, channel, sf_bin, printMsg(element, logbuff, sizeof(logbuff)));
#endif

          }
       }
    }

   return SRSRAN_SUCCESS;
 }
