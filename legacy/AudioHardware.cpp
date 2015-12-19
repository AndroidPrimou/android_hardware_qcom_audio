/*
** Copyright 2008, The Android Open-Source Project
** Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
** Not a Contribution, Apache license notifications and license are retained
** for attribution purposes only.
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#define LOG_TAG "AudioHardware7x30"
//#define LOG_NDDEBUG 0

#include <dlfcn.h>
#include <fcntl.h>
#include <math.h>
#include <media/AudioSystem.h>
#include <cutils/properties.h>

extern "C" {
#include "control.h"
#ifdef WITH_QCOM_CALIBRATION
#include "initialize_audcal7x30.h"
#endif
#ifdef HTC_AUDIO
#include <linux/spi_aic3254.h>
#include <linux/tpa2051d3.h>
#endif
}

// hardware specific functions

#include "AudioHardware.h"

#define ECHO_SUPRESSION "ec_supported"
#define DUALMIC_KEY "dualmic_enabled"
#define TTY_MODE_KEY "tty_mode"
#define BTHEADSET_VGS "bt_headset_vgs"
#ifdef HTC_AUDIO
#define DSP_EFFECT_KEY "dolby_srs_eq"
#endif

#define AAC_DEVICE_IN "/dev/msm_aac_in"
#define FM_DEVICE  "/dev/msm_fm"

#define FM_A2DP_REC 1
#define FM_FILE_REC 2

namespace android_audio_legacy {

Mutex   mDeviceSwitchLock;
#ifdef HTC_AUDIO
Mutex   mAIC3254ConfigLock;
#endif
static int audpre_index, tx_iir_index;
static void * acoustic;
const uint32_t AudioHardware::inputSamplingRates[] = {
        8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};
static uint32_t INVALID_DEVICE = 65535;
static uint32_t SND_DEVICE_CURRENT =-1;
static uint32_t SND_DEVICE_HANDSET = 0;
static uint32_t SND_DEVICE_SPEAKER = 1;
static uint32_t SND_DEVICE_HEADSET = 2;
static uint32_t SND_DEVICE_FM_HANDSET = 3;
static uint32_t SND_DEVICE_FM_SPEAKER = 4;
static uint32_t SND_DEVICE_FM_HEADSET = 5;
static uint32_t SND_DEVICE_BT = 6;
static uint32_t SND_DEVICE_HEADSET_AND_SPEAKER = 7;
static uint32_t SND_DEVICE_NO_MIC_HEADSET = 8;
static uint32_t SND_DEVICE_IN_S_SADC_OUT_HANDSET = 9;
static uint32_t SND_DEVICE_IN_S_SADC_OUT_SPEAKER_PHONE = 10;
static uint32_t SND_DEVICE_TTY_HEADSET = 11;
static uint32_t SND_DEVICE_TTY_HCO = 12;
static uint32_t SND_DEVICE_TTY_VCO = 13;
static uint32_t SND_DEVICE_TTY_FULL = 14;
static uint32_t SND_DEVICE_HDMI = 15;
static uint32_t SND_DEVICE_FM_TX = 16;
static uint32_t SND_DEVICE_FM_TX_AND_SPEAKER = 17;
static uint32_t SND_DEVICE_HEADPHONE_AND_SPEAKER = 18;
#ifdef HTC_AUDIO
static uint32_t SND_DEVICE_CARKIT = 19;
static uint32_t SND_DEVICE_HANDSET_BACK_MIC = 20;
static uint32_t SND_DEVICE_SPEAKER_BACK_MIC = 21;
static uint32_t SND_DEVICE_NO_MIC_HEADSET_BACK_MIC = 28;
static uint32_t SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC = 30;
static uint32_t SND_DEVICE_I2S_SPEAKER = 32;
static uint32_t SND_DEVICE_BT_EC_OFF = 45;
static uint32_t SND_DEVICE_HAC = 252;
static uint32_t SND_DEVICE_USB_HEADSET = 253;
#else
static uint32_t SND_DEVICE_CARKIT = -1;
static uint32_t SND_DEVICE_BT_EC_OFF = -1;
#endif
#ifdef SAMSUNG_AUDIO
static uint32_t SND_DEVICE_VOIP_HANDSET = 50;
static uint32_t SND_DEVICE_VOIP_SPEAKER = 51;
static uint32_t SND_DEVICE_VOIP_HEADSET = 52;
static uint32_t SND_DEVICE_CALL_HANDSET = 60;
static uint32_t SND_DEVICE_CALL_SPEAKER = 61;
static uint32_t SND_DEVICE_CALL_HEADSET = 62;
#endif
#ifdef SEMC_AUDIO
static uint32_t SND_DEVICE_SPEAKER_DUAL_MIC_REAL_STEREO = 19;
#endif

static uint32_t DEVICE_HANDSET_RX = 0;           /* handset_rx */
static uint32_t DEVICE_HANDSET_TX = 1;           /* handset_tx */
static uint32_t DEVICE_SPEAKER_RX = 2;           /* speaker_stereo_rx
                                                    htc: speaker_mono_rx
                                                    sam: speaker_rx */
static uint32_t DEVICE_SPEAKER_TX = 3;           /* speaker_mono_tx
                                                    sam: speaker_tx */
static uint32_t DEVICE_HEADSET_RX = 4;           /* headset_stereo_rx
                                                    sam: headset_rx */
static uint32_t DEVICE_HEADSET_TX = 5;           /* headset_mono_tx
                                                    sam: headset_tx */
static uint32_t DEVICE_FMRADIO_HANDSET_RX = 6;   /* fmradio_handset_rx */
static uint32_t DEVICE_FMRADIO_HEADSET_RX = 7;   /* fmradio_headset_rx */
static uint32_t DEVICE_FMRADIO_SPEAKER_RX = 8;   /* fmradio_speaker_rx */
static uint32_t DEVICE_DUALMIC_HANDSET_TX = 9;   /* handset_dual_mic_endfire_tx */
static uint32_t DEVICE_DUALMIC_SPEAKER_TX = 10;  /* speaker_dual_mic_endfire_tx */
static uint32_t DEVICE_TTY_HEADSET_MONO_RX = 11; /* tty_headset_mono_rx */
static uint32_t DEVICE_TTY_HEADSET_MONO_TX = 12; /* tty_headset_mono_tx */
static uint32_t DEVICE_SPEAKER_HEADSET_RX = 13;  /* headset_stereo_speaker_stereo_rx
                                                    htc: headset_speaker_stereo_rx
                                                    sam: speaker_headset_rx */
static uint32_t DEVICE_FMRADIO_STEREO_TX = 14;
static uint32_t DEVICE_HDMI_STERO_RX = 15;       /* hdmi_stereo_rx */
static uint32_t DEVICE_FMRADIO_STEREO_RX = 16;
static uint32_t DEVICE_BT_SCO_RX = 17;           /* bt_sco_rx */
static uint32_t DEVICE_BT_SCO_TX = 18;           /* bt_sco_tx */
#ifdef SAMSUNG_AUDIO
static uint32_t DEVICE_HANDSET_VOIP_RX = 40;     /* handset_voip_rx */
static uint32_t DEVICE_HANDSET_VOIP_TX = 41;     /* handset_voip_tx */
static uint32_t DEVICE_SPEAKER_VOIP_RX = 42;     /* speaker_voip_rx */
static uint32_t DEVICE_SPEAKER_VOIP_TX = 43;     /* speaker_voip_tx */
static uint32_t DEVICE_HEADSET_VOIP_RX = 44;     /* headset_voip_rx */
static uint32_t DEVICE_HEADSET_VOIP_TX = 45;     /* headset_voip_tx */
static uint32_t DEVICE_HANDSET_CALL_RX = 60;     /* handset_call_rx */
static uint32_t DEVICE_HANDSET_CALL_TX = 61;     /* handset_call_tx */
static uint32_t DEVICE_SPEAKER_CALL_RX = 62;     /* speaker_call_rx */
static uint32_t DEVICE_SPEAKER_CALL_TX = 63;     /* speaker_call_tx */
static uint32_t DEVICE_HEADSET_CALL_RX = 64;     /* headset_call_rx */
static uint32_t DEVICE_HEADSET_CALL_TX = 65;     /* headset_call_tx */
static uint32_t DEVICE_COUNT = DEVICE_HEADSET_CALL_TX + 1;
#elif defined(SEMC_AUDIO)
static uint32_t DEVICE_SPEAKER_DUAL_MIC_REAL_STEREO_TX = 19;
static uint32_t DEVICE_COUNT = DEVICE_SPEAKER_DUAL_MIC_REAL_STEREO_TX + 1;
#elif defined(HTC_AUDIO)
static const uint32_t DEVICE_USB_HEADSET_RX = 19;      /* usb_headset_stereo_rx */
static const uint32_t DEVICE_HAC_RX = 20;              /* hac_mono_rx */
static const uint32_t DEVICE_ALT_RX = 21;              /* alt_mono_rx */
static const uint32_t DEVICE_VR_HANDSET = 22;          /* handset_vr_tx */
static const uint32_t DEVICE_COUNT = DEVICE_VR_HANDSET +1;
#else
static uint32_t DEVICE_COUNT = DEVICE_BT_SCO_TX + 1;
#endif

#ifdef HTC_AUDIO
static bool support_aic3254 = true;
static bool aic3254_enabled = true;
int (*set_sound_effect)(const char* effect);
static bool support_tpa2051 = true;
static bool support_htc_backmic = true;
static bool fm_enabled = false;
static int alt_enable = 0;
static int hac_enable = 0;
static uint32_t cur_aic_tx = UPLINK_OFF;
static uint32_t cur_aic_rx = DOWNLINK_OFF;
static int cur_tpa_mode = 0;
#endif

int dev_cnt = 0;
const char ** name = NULL;
int mixer_cnt = 0;
static uint32_t cur_tx = INVALID_DEVICE;
static uint32_t cur_rx = INVALID_DEVICE;
bool vMicMute = false;
typedef struct routing_table
{
    unsigned short dec_id;
    int dev_id;
    int dev_id_tx;
    int stream_type;
    bool active;
    struct routing_table *next;
} Routing_table;
Routing_table* head;
Mutex       mRoutingTableLock;

typedef struct device_table
{
    int dev_id;
    int class_id;
    int capability;
}Device_table;
Device_table* device_list;

static unsigned char build_id[20];

static void amr_transcode(unsigned char *src, unsigned char *dst);

enum STREAM_TYPES {
PCM_PLAY=1,
PCM_REC,
VOICE_CALL,
FM_RADIO,
FM_REC,
FM_A2DP,
INVALID_STREAM
};

typedef struct ComboDeviceType
{
    uint32_t DeviceId;
    STREAM_TYPES StreamType;
}CurrentComboDeviceStruct;
CurrentComboDeviceStruct CurrentComboDeviceData;
Mutex   mComboDeviceLock;

enum FM_STATE {
    FM_INVALID=1,
    FM_OFF,
    FM_ON
};

FM_STATE fmState = FM_INVALID;
static uint32_t fmDevice = INVALID_DEVICE;

#define DEV_ID(X) device_list[X].dev_id
void addToTable(int decoder_id,int device_id,int device_id_tx,int stream_type,bool active) {
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = (Routing_table* ) malloc(sizeof(Routing_table));
    temp_ptr->next = NULL;
    temp_ptr->dec_id = decoder_id;
    temp_ptr->dev_id = device_id;
    temp_ptr->dev_id_tx = device_id_tx;
    temp_ptr->stream_type = stream_type;
    temp_ptr->active = active;
    //add new Node to head.
    temp_ptr->next =head->next;
    head->next = temp_ptr;
}
bool isStreamOn(int Stream_type) {
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type)
                return true;
        temp_ptr=temp_ptr->next;
    }
    return false;
}
bool isStreamOnAndActive(int Stream_type) {
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type) {
            if(temp_ptr->active == true) {
                return true;
            }
            else {
                return false;
            }
        }
        temp_ptr=temp_ptr->next;
    }
    return false;
}
bool isStreamOnAndInactive(int Stream_type) {
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type) {
            if(temp_ptr->active == false) {
                return true;
            }
            else {
                return false;
            }
        }
        temp_ptr=temp_ptr->next;
    }
    return false;
}
Routing_table*  getNodeByStreamType(int Stream_type) {
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type) {
            return temp_ptr;
        }
        temp_ptr=temp_ptr->next;
    }
    return NULL;
}
void modifyActiveStateOfStream(int Stream_type, bool Active) {
    Routing_table* temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type) {
            temp_ptr->active = Active;
            return;
        }
        temp_ptr=temp_ptr->next;
    }
}
void modifyActiveDeviceOfStream(int Stream_type,int Device_id,int Device_id_tx) {
    Routing_table* temp_ptr;
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        if(temp_ptr->stream_type == Stream_type) {
            temp_ptr->dev_id = Device_id;
            temp_ptr->dev_id_tx = Device_id_tx;
            return;
        }
        temp_ptr=temp_ptr->next;
    }
}
void printTable()
{
    Routing_table * temp_ptr;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head->next;
    while(temp_ptr!=NULL) {
        printf("%d %d %d %d %d\n",temp_ptr->dec_id,temp_ptr->dev_id,temp_ptr->dev_id_tx,temp_ptr->stream_type,temp_ptr->active);
        temp_ptr = temp_ptr->next;
    }
}
void deleteFromTable(int Stream_type) {
    Routing_table *temp_ptr,*temp1;
    Mutex::Autolock lock(mRoutingTableLock);
    temp_ptr = head;
    while(temp_ptr->next!=NULL) {
        if(temp_ptr->next->stream_type == Stream_type) {
            temp1 = temp_ptr->next;
            temp_ptr->next = temp_ptr->next->next;
            free(temp1);
            return;
        }
        temp_ptr=temp_ptr->next;
    }

}

bool isDeviceListEmpty() {
    if(head->next == NULL)
        return true;
    else
        return false;
}

int enableDevice(int device,short enable) {
    ALOGV("value of device and enable is %d %d ALSA dev id:%d",device,enable,DEV_ID(device));
    if( msm_en_device(DEV_ID(device), enable)) {
        ALOGE("msm_en_device(%d, %d) failed errno = %d",DEV_ID(device), enable, errno);
        return -1;
    }
    return 0;
}

#ifdef HTC_AUDIO
void updateACDB(uint32_t new_rx_device, uint32_t new_tx_device,
                uint32_t new_rx_acdb, uint32_t new_tx_acdb) {

    ALOGD("updateACDB: (%d, %d, %d, %d) ", new_tx_device, new_rx_device, new_tx_acdb, new_rx_acdb);

    int rc = -1;
    int (*update_acdb_id)(uint32_t, uint32_t, uint32_t, uint32_t);

    update_acdb_id = (int (*)(uint32_t, uint32_t, uint32_t, uint32_t))::dlsym(acoustic, "update_acdb_id");
    if ((*update_acdb_id) == 0)
        ALOGE("Could not open update_acdb_id()");
    else {
        rc = update_acdb_id(new_tx_device, new_rx_device, new_tx_acdb, new_rx_acdb);
        if (rc < 0)
            ALOGE("Could not set update_acdb_id: %d", rc);
    }
}

static status_t updateDeviceInfo(int rx_device,int tx_device,
                                 uint32_t rx_acdb_id, uint32_t tx_acdb_id) {
#else
static status_t updateDeviceInfo(int rx_device,int tx_device) {
#endif
    ALOGV("updateDeviceInfo: E rx_device %d and tx_device %d", rx_device, tx_device);
    bool isRxDeviceEnabled = false,isTxDeviceEnabled = false;
    Routing_table *temp_ptr,*temp_head;
    int tx_dev_prev = INVALID_DEVICE;
    temp_head = head;
    Mutex::Autolock lock(mDeviceSwitchLock);

    if (!getNodeByStreamType(VOICE_CALL) && !getNodeByStreamType(PCM_PLAY)
            && !getNodeByStreamType(FM_RADIO)) {
        ALOGV("No active voicecall/playback, disabling cur_rx %d", cur_rx);
        if(cur_rx != INVALID_DEVICE && enableDevice(cur_rx, 0)) {
            ALOGE("Disabling device failed for cur_rx %d", cur_rx);
        }
        cur_rx = rx_device;
    }

    if(!getNodeByStreamType(VOICE_CALL) && !getNodeByStreamType(PCM_REC)) {
        ALOGV("No active voicecall/recording, disabling cur_tx %d", cur_tx);
        if(cur_tx != INVALID_DEVICE && enableDevice(cur_tx, 0)) {
            ALOGE("Disabling device failed for cur_tx %d", cur_tx);
        }
        cur_tx = tx_device;
    }
    Mutex::Autolock lock_1(mRoutingTableLock);

    while(temp_head->next != NULL) {
        temp_ptr = temp_head->next;
        switch(temp_ptr->stream_type) {
            case PCM_PLAY:
            case FM_RADIO:
                ALOGD("The node type is %d and cur device %d new device %d ", temp_ptr->stream_type, temp_ptr->dev_id, rx_device);
                if(rx_device == INVALID_DEVICE)
                    return -1;
                if(rx_device == temp_ptr->dev_id)
                    break;
                ALOGV("rx_device = %d,temp_ptr->dev_id = %d",rx_device,temp_ptr->dev_id);
                if(isRxDeviceEnabled == false) {
                    enableDevice(temp_ptr->dev_id,0);
                    enableDevice(rx_device,1);
                    isRxDeviceEnabled = true;
                }
                if(msm_route_stream(PCM_PLAY,temp_ptr->dec_id,DEV_ID(temp_ptr->dev_id),0)) {
                    ALOGE("msm_route_stream(PCM_PLAY,%d,%d,0) failed",temp_ptr->dec_id,DEV_ID(temp_ptr->dev_id));
                }
                if(msm_route_stream(PCM_PLAY,temp_ptr->dec_id,DEV_ID(rx_device),1)) {
                    ALOGE("msm_route_stream(PCM_PLAY,%d,%d,1) failed",temp_ptr->dec_id,DEV_ID(rx_device));
                }
                modifyActiveDeviceOfStream(temp_ptr->stream_type,rx_device,INVALID_DEVICE);
                cur_tx = tx_device ;
                cur_rx = rx_device ;
                break;
            case PCM_REC:

                ALOGD("case PCM_REC");
                if(tx_device == INVALID_DEVICE)
                    return -1;
                if(tx_device == temp_ptr->dev_id)
                    break;

                if(isTxDeviceEnabled == false) {
                    enableDevice(temp_ptr->dev_id,0);
                    enableDevice(tx_device,1);
                   isTxDeviceEnabled = true;
                }
                if(msm_route_stream(PCM_REC,temp_ptr->dec_id,DEV_ID(temp_ptr->dev_id),0)) {
                    ALOGE("msm_route_stream(PCM_REC,%d,%d,0) failed",temp_ptr->dec_id,DEV_ID(temp_ptr->dev_id));
                }
                if(msm_route_stream(PCM_REC,temp_ptr->dec_id,DEV_ID(tx_device),1)) {
                    ALOGE("msm_route_stream(PCM_REC,%d,%d,1) failed",temp_ptr->dec_id,DEV_ID(tx_device));
                }
                modifyActiveDeviceOfStream(PCM_REC,tx_device,INVALID_DEVICE);
                tx_dev_prev = cur_tx;
                cur_tx = tx_device ;
                cur_rx = rx_device ;
                break;
            case VOICE_CALL:

                ALOGD("case VOICE_CALL");
                if(rx_device == INVALID_DEVICE || tx_device == INVALID_DEVICE)
                    return -1;
                if(rx_device == temp_ptr->dev_id && tx_device == temp_ptr->dev_id_tx)
                    break;

#ifdef HTC_AUDIO
                updateACDB(rx_device, tx_device, rx_acdb_id, tx_acdb_id);
#endif

                msm_route_voice(DEV_ID(rx_device),DEV_ID(tx_device),1);

                // Temporary work around for Speaker mode. The driver is not
                // supporting Speaker Rx and Handset Tx combo
                if(isRxDeviceEnabled == false) {
                    if (rx_device != temp_ptr->dev_id)
                    {
                        enableDevice(temp_ptr->dev_id,0);
                    }
                    isRxDeviceEnabled = true;
                }
                if(isTxDeviceEnabled == false) {
                    if (tx_device != temp_ptr->dev_id_tx)
                    {
                        enableDevice(temp_ptr->dev_id_tx,0);
                    }
                    isTxDeviceEnabled = true;
                }

                if (rx_device != temp_ptr->dev_id)
                {
                    enableDevice(rx_device,1);
                }

                if (tx_device != temp_ptr->dev_id_tx)
                {
                    enableDevice(tx_device,1);
                }

                cur_rx = rx_device;
                cur_tx = tx_device;
                modifyActiveDeviceOfStream(VOICE_CALL,cur_rx,cur_tx);
                break;
            default:
                break;
        }
        temp_head = temp_head->next;
    }

    ALOGV("updateDeviceInfo: X cur_rx %d cur_tx %d", cur_rx, cur_tx);
    return NO_ERROR;
}

void freeMemory() {
    Routing_table *temp;
    while(head != NULL) {
        temp = head->next;
        free(head);
        head = temp;
    }
free(device_list);
}

//
// ----------------------------------------------------------------------------

AudioHardware::AudioHardware() :
    mInit(false), mMicMute(true), mFmFd(-1),
    mBluetoothNrec(true), mBluetoothVGS(false), mBluetoothId(0),
    mVoiceVolume(1), mOutput(0),
    mCurSndDevice(SND_DEVICE_CURRENT), mDualMicEnabled(false),
    mTtyMode(TTY_OFF)
#ifdef HTC_AUDIO
    , mHACSetting(false), mBluetoothIdTx(0), mBluetoothIdRx(0),
    mRecordState(false), mEffectEnabled(false)
#endif
{
#ifdef HTC_AUDIO
        int (*snd_get_num)();
        int (*snd_get_bt_endpoint)(msm_bt_endpoint *);
        int (*set_acoustic_parameters)();
        int (*set_tpa2051_parameters)();
        int (*set_aic3254_parameters)();
        int (*support_back_mic)();

        struct msm_bt_endpoint *ept;
#endif

        int control;
        int i = 0,index = 0;

        head = (Routing_table* ) malloc(sizeof(Routing_table));
        head->next = NULL;

#ifdef HTC_AUDIO
        acoustic =:: dlopen("/system/lib/libhtc_acoustic.so", RTLD_NOW);
        if (acoustic == NULL ) {
            ALOGD("Could not open libhtc_acoustic.so");
            /* this is not really an error on non-htc devices... */
            mNumBTEndpoints = 0;
            support_aic3254 = false;
            support_tpa2051 = false;
            support_htc_backmic = false;
        }
#endif

        ALOGD("msm_mixer_open: Opening the device");
        control = msm_mixer_open("/dev/snd/controlC0", 0);
        if(control< 0)
                ALOGE("ERROR opening the device");

        if((fp = fopen("/sys/devices/system/soc/soc0/build_id","r")) == NULL) {
            ALOGE("Cannot open build_id file.");
        }
        else {
            (void)fgets((char *)build_id,sizeof(build_id),fp);
        }

        if(msm_reset_all_device() < 0)
            ALOGE("msm_reset_all_device() failed");

        mixer_cnt = msm_mixer_count();
        ALOGD("msm_mixer_count:mixer_cnt =%d",mixer_cnt);

        dev_cnt = msm_get_device_count();
        ALOGV("got device_count %d",dev_cnt);
        if (dev_cnt <= 0) {
                ALOGE("NO devices registered\n");
                return;
        }
        name = msm_get_device_list();
        device_list = (Device_table* )malloc(sizeof(Device_table)*DEVICE_COUNT);
        if(device_list == NULL) {
            ALOGE("malloc failed for device list");
            return;
        }
        for(i = 0;i<DEVICE_COUNT;i++)
            device_list[i].dev_id = INVALID_DEVICE;

        for(i = 0; i < dev_cnt;i++) {
            if(strcmp((char* )name[i],"handset_rx") == 0)
                index = DEVICE_HANDSET_RX;
            else if(strcmp((char* )name[i],"handset_tx") == 0)
                index = DEVICE_HANDSET_TX;
#ifndef WITH_STEREO_HW_SPEAKER
            else if(strcmp((char* )name[i],"speaker_mono_rx") == 0)
                index = DEVICE_SPEAKER_RX;
#endif
            else if((strcmp((char* )name[i],"speaker_stereo_rx") == 0) ||
                    (strcmp((char* )name[i],"speaker_rx") == 0))
                index = DEVICE_SPEAKER_RX;
            else if((strcmp((char* )name[i],"speaker_mono_tx") == 0) ||
                    (strcmp((char* )name[i],"speaker_tx") == 0))
                index = DEVICE_SPEAKER_TX;
            else if((strcmp((char* )name[i],"headset_stereo_rx") == 0) ||
                    (strcmp((char* )name[i],"headset_rx") == 0))
                index = DEVICE_HEADSET_RX;
            else if((strcmp((char* )name[i],"headset_mono_tx") == 0) ||
                    (strcmp((char* )name[i],"headset_tx") == 0))
                index = DEVICE_HEADSET_TX;
            else if(strcmp((char* )name[i],"fmradio_handset_rx") == 0)
                index = DEVICE_FMRADIO_HANDSET_RX;
            else if(strcmp((char* )name[i],"fmradio_headset_rx") == 0)
                index = DEVICE_FMRADIO_HEADSET_RX;
            else if(strcmp((char* )name[i],"fmradio_speaker_rx") == 0)
                index = DEVICE_FMRADIO_SPEAKER_RX;
            else if(strcmp((char* )name[i],"handset_dual_mic_broadside_tx") == 0)
                index = DEVICE_DUALMIC_HANDSET_TX;
            else if(strcmp((char* )name[i],"handset_dual_mic_endfire_tx") == 0)
                index = DEVICE_DUALMIC_HANDSET_TX;
            else if(strcmp((char* )name[i],"speaker_dual_mic_broadside_tx") == 0)
                index = DEVICE_DUALMIC_SPEAKER_TX;
            else if(strcmp((char* )name[i],"speaker_dual_mic_endfire_tx") == 0)
                index = DEVICE_DUALMIC_SPEAKER_TX;
            else if(strcmp((char* )name[i],"tty_headset_mono_rx") == 0)
                index = DEVICE_TTY_HEADSET_MONO_RX;
            else if(strcmp((char* )name[i],"tty_headset_mono_tx") == 0)
                index = DEVICE_TTY_HEADSET_MONO_TX;
            else if(strcmp((char* )name[i],"bt_sco_rx") == 0)
                index = DEVICE_BT_SCO_RX;
            else if(strcmp((char* )name[i],"bt_sco_tx") == 0)
                index = DEVICE_BT_SCO_TX;
            else if((strcmp((char*)name[i],"headset_stereo_speaker_stereo_rx") == 0) ||
                    (strcmp((char*)name[i],"headset_speaker_stereo_rx") == 0) ||
                    (strcmp((char*)name[i],"speaker_headset_rx") == 0))
                index = DEVICE_SPEAKER_HEADSET_RX;
            else if(strcmp((char*)name[i],"fmradio_stereo_tx") == 0)
                index = DEVICE_FMRADIO_STEREO_TX;
            else if(strcmp((char*)name[i],"hdmi_stereo_rx") == 0)
                index = DEVICE_HDMI_STERO_RX;
            else if(strcmp((char*)name[i],"fmradio_stereo_rx") == 0)
                index = DEVICE_FMRADIO_STEREO_RX;
#ifdef SAMSUNG_AUDIO
            else if(strcmp((char* )name[i], "handset_voip_rx") == 0)
                index = DEVICE_HANDSET_VOIP_RX;
            else if(strcmp((char* )name[i], "handset_voip_tx") == 0)
                index = DEVICE_HANDSET_VOIP_TX;
            else if(strcmp((char* )name[i], "speaker_voip_rx") == 0)
                index = DEVICE_SPEAKER_VOIP_RX;
            else if(strcmp((char* )name[i], "speaker_voip_tx") == 0)
                index = DEVICE_SPEAKER_VOIP_TX;
            else if(strcmp((char* )name[i], "headset_voip_rx") == 0)
                index = DEVICE_HEADSET_VOIP_RX;
            else if(strcmp((char* )name[i], "headset_voip_tx") == 0)
                index = DEVICE_HEADSET_VOIP_TX;
            else if(strcmp((char* )name[i], "handset_call_rx") == 0)
                index = DEVICE_HANDSET_CALL_RX;
            else if(strcmp((char* )name[i], "handset_call_tx") == 0)
                index = DEVICE_HANDSET_CALL_TX;
            else if(strcmp((char* )name[i], "speaker_call_rx") == 0)
                index = DEVICE_SPEAKER_CALL_RX;
            else if(strcmp((char* )name[i], "speaker_call_tx") == 0)
                index = DEVICE_SPEAKER_CALL_TX;
            else if(strcmp((char* )name[i], "headset_call_rx") == 0)
                index = DEVICE_HEADSET_CALL_RX;
            else if(strcmp((char* )name[i], "headset_call_tx") == 0)
                index = DEVICE_HEADSET_CALL_TX;
#endif
#ifdef SEMC_AUDIO
            else if(strcmp((char* )name[i], "speaker_dual_mic_endfire_tx_real_stereo") == 0)
                index = DEVICE_SPEAKER_DUAL_MIC_REAL_STEREO_TX;
#endif
            else
                continue;
            ALOGV("index = %d",index);

            device_list[index].dev_id = msm_get_device((char* )name[i]);
            if(device_list[index].dev_id >= 0) {
                    ALOGV("Found device: %s:index = %d,dev_id: %d",( char* )name[i], index,device_list[index].dev_id);
            }
            device_list[index].class_id = msm_get_device_class(device_list[index].dev_id);
            device_list[index].capability = msm_get_device_capability(device_list[index].dev_id);
            ALOGV("class ID = %d,capablity = %d for device %d",device_list[index].class_id,device_list[index].capability,device_list[index].dev_id);
        }
#ifdef WITH_QCOM_CALIBRATION
        audcal_initialize();
#endif
        mInit = true;

        CurrentComboDeviceData.DeviceId = INVALID_DEVICE;
        CurrentComboDeviceData.StreamType = INVALID_STREAM;

#ifdef HTC_AUDIO
    // HTC specific functions
    set_acoustic_parameters = (int (*)(void))::dlsym(acoustic, "set_acoustic_parameters");
    if ((*set_acoustic_parameters) == 0 ) {
        ALOGE("Could not open set_acoustic_parameters()");
        return;
    }

    int rc = set_acoustic_parameters();
    if (rc < 0) {
        ALOGD("Could not set acoustic parameters to share memory: %d", rc);
    }

    char value[PROPERTY_VALUE_MAX];
    /* Check the system property for enable or not the ALT function */
    property_get("htc.audio.alt.enable", value, "0");
    alt_enable = atoi(value);
    ALOGV("Enable ALT function: %d", alt_enable);

    /* Check the system property for enable or not the HAC function */
    property_get("htc.audio.hac.enable", value, "0");
    hac_enable = atoi(value);
    ALOGV("Enable HAC function: %d", hac_enable);

    set_tpa2051_parameters = (int (*)(void))::dlsym(acoustic, "set_tpa2051_parameters");
    if ((*set_tpa2051_parameters) == 0) {
        ALOGI("set_tpa2051_parameters() not present");
        support_tpa2051 = false;
    }

    if (support_tpa2051) {
        if (set_tpa2051_parameters() < 0) {
            ALOGI("Speaker amplifies tpa2051 is not supported");
            support_tpa2051 = false;
        }
    }

    set_aic3254_parameters = (int (*)(void))::dlsym(acoustic, "set_aic3254_parameters");
    if ((*set_aic3254_parameters) == 0 ) {
        ALOGI("set_aic3254_parameters() not present");
        support_aic3254 = false;
    }

    if (support_aic3254) {
        if (set_aic3254_parameters() < 0) {
            ALOGI("AIC3254 DSP is not supported");
            support_aic3254 = false;
        }
    }

    if (support_aic3254) {
        set_sound_effect = (int (*)(const char*))::dlsym(acoustic, "set_sound_effect");
        if ((*set_sound_effect) == 0 ) {
            ALOGI("set_sound_effect() not present");
            ALOGI("AIC3254 DSP is not supported");
            support_aic3254 = false;
        } else
            strcpy(mEffect, "\0");
    }

    support_back_mic = (int (*)(void))::dlsym(acoustic, "support_back_mic");
    if ((*support_back_mic) == 0 ) {
        ALOGI("support_back_mic() not present");
        support_htc_backmic = false;
    }

    if (support_htc_backmic) {
        if (support_back_mic() != 1) {
            ALOGI("HTC DualMic is not supported");
            support_htc_backmic = false;
        }
    }

    snd_get_num = (int (*)(void))::dlsym(acoustic, "snd_get_num");
    if ((*snd_get_num) == 0 ) {
        ALOGD("Could not open snd_get_num()");
    }

    mNumBTEndpoints = snd_get_num();
    ALOGV("mNumBTEndpoints = %d", mNumBTEndpoints);
    mBTEndpoints = new msm_bt_endpoint[mNumBTEndpoints];
    ALOGV("constructed %d SND endpoints)", mNumBTEndpoints);
    ept = mBTEndpoints;
    snd_get_bt_endpoint = (int (*)(msm_bt_endpoint *))::dlsym(acoustic, "snd_get_bt_endpoint");
    if ((*snd_get_bt_endpoint) == 0 ) {
        mInit = true;
        ALOGE("Could not open snd_get_bt_endpoint()");
        return;
    }
    snd_get_bt_endpoint(mBTEndpoints);

    for (int i = 0; i < mNumBTEndpoints; i++) {
        ALOGV("BT name %s (tx,rx)=(%d,%d)", mBTEndpoints[i].name, mBTEndpoints[i].tx, mBTEndpoints[i].rx);
    }
#endif
}

AudioHardware::~AudioHardware()
{
    for (size_t index = 0; index < mInputs.size(); index++) {
        closeInputStream((AudioStreamIn*)mInputs[index]);
    }
    mInputs.clear();
    closeOutputStream((AudioStreamOut*)mOutput);
    if (acoustic) {
        ::dlclose(acoustic);
        acoustic = 0;
    }
    msm_mixer_close();
#ifdef WITH_QCOM_CALIBRATION
    audcal_deinitialize();
#endif
    freeMemory();
    fclose(fp);
    mInit = false;
}

status_t AudioHardware::initCheck()
{
    return mInit ? NO_ERROR : NO_INIT;
}

// default implementation calls its "without flags" counterpart
AudioStreamOut* AudioHardware::openOutputStreamWithFlags(uint32_t devices,
                                          audio_output_flags_t flags,
                                          int *format,
                                          uint32_t *channels,
                                          uint32_t *sampleRate,
                                          status_t *status)
{
    return openOutputStream(devices, format, channels, sampleRate, status);
}

AudioStreamOut* AudioHardware::openOutputStream(
        uint32_t devices,  int *format, uint32_t *channels,
        uint32_t *sampleRate, status_t *status)

{
     ALOGD("AudioHardware::openOutputStream devices %x format %d channels %d samplerate %d",
        devices, *format, *channels, *sampleRate);

     audio_output_flags_t flags = static_cast<audio_output_flags_t> (*status);

     if (!audio_is_output_device(devices))
        return 0;


    { // scope for the lock
        Mutex::Autolock lock(mLock);

        ALOGV(" AudioHardware::openOutputStream AudioStreamOutMSM72xx output stream \n");
        // only one output stream allowed
        if (mOutput) {
            if (status) {
                *status = INVALID_OPERATION;
            }
            ALOGE(" AudioHardware::openOutputStream Only one output stream allowed \n");
            return 0;
        }

        // create new output stream
        AudioStreamOutMSM72xx* out = new AudioStreamOutMSM72xx();
        status_t lStatus = out->set(this, devices, format, channels, sampleRate);
        if (status) {
            *status = lStatus;
        }
        if (lStatus == NO_ERROR) {
            mOutput = out;
        } else {
            delete out;
        }
        return mOutput;
    }
}

void AudioHardware::closeOutputStream(AudioStreamOut* out) {
    Mutex::Autolock lock(mLock);
    if (mOutput == 0 || mOutput != out) {
        ALOGW("Attempt to close invalid output stream");
    } else {
        delete mOutput;
        mOutput = 0;
    }
}

AudioStreamIn* AudioHardware::openInputStream(
        uint32_t devices, int *format, uint32_t *channels, uint32_t *sampleRate, status_t *status,
        AudioSystem::audio_in_acoustics acoustic_flags)
{
    // check for valid input source
    ALOGD("AudioHardware::openInputStream devices %x format %d channels %d samplerate %d",
        devices, *format, *channels, *sampleRate);

    if (!audio_is_input_device(devices))
        return 0;

    mLock.lock();

    AudioStreamInMSM72xx* in72xx = new AudioStreamInMSM72xx();
    status_t lStatus = in72xx->set(this, devices, format, channels, sampleRate, acoustic_flags);
    if (status) {
        *status = lStatus;
    }
    if (lStatus != NO_ERROR) {
        ALOGE("Error creating Audio stream AudioStreamInMSM72xx \n");
        mLock.unlock();
        delete in72xx;
        return 0;
    }
    mInputs.add(in72xx);
    mLock.unlock();
    return in72xx;
}

void AudioHardware::closeInputStream(AudioStreamIn* in) {
    Mutex::Autolock lock(mLock);

    ssize_t index = mInputs.indexOf((AudioStreamInMSM72xx *)in);
    if (index >= 0) {
        ALOGV("closeInputStream AudioStreamInMSM72xx");
        mLock.unlock();
        delete mInputs[index];
        mLock.lock();
        mInputs.removeAt(index);
    } else {
        ALOGE("Attempt to close invalid input stream");
     }
}

status_t AudioHardware::setMode(int mode)
{
    status_t status = AudioHardwareBase::setMode(mode);
    if (status == NO_ERROR) {
        // make sure that doAudioRouteOrMute() is called by doRouting()
        // even if the new device selected is the same as current one.
        clearCurDevice();
    }
    return status;
}

status_t AudioHardware::setMasterMute(bool muted) {
    //TODO: enable when supported by driver
    return INVALID_OPERATION;
}

int AudioHardware::createAudioPatch(unsigned int num_sources,
        const struct audio_port_config *sources,
        unsigned int num_sinks,
        const struct audio_port_config *sinks,
        audio_patch_handle_t *handle) {
    //TODO: enable when supported by driver
    return INVALID_OPERATION;
}

int AudioHardware::releaseAudioPatch(audio_patch_handle_t handle) {
    //TODO: enable when supported by driver
    return INVALID_OPERATION;
}

int AudioHardware::getAudioPort(struct audio_port *port) {
    //TODO: enable when supported by driver
    return INVALID_OPERATION;
}

int AudioHardware::setAudioPortConfig(
        const struct audio_port_config *config) {
    //TODO: enable when supported by driver
    return INVALID_OPERATION;
}

bool AudioHardware::checkOutputStandby()
{
    if (mOutput)
        if (!mOutput->checkStandby())
            return false;

    return true;
}

status_t AudioHardware::setMicMute(bool state)
{
    Mutex::Autolock lock(mLock);
    return setMicMute_nosync(state);
}

// always call with mutex held
status_t AudioHardware::setMicMute_nosync(bool state)
{
    if (mMicMute != state) {
        mMicMute = state;
        ALOGD("setMicMute_nosync calling voice mute with the mMicMute %d", mMicMute);
        msm_set_voice_tx_mute(mMicMute);
    }
    return NO_ERROR;
}

status_t AudioHardware::getMicMute(bool* state)
{
    *state = mMicMute;
    return NO_ERROR;
}

status_t AudioHardware::setParameters(const String8& keyValuePairs)
{
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 value;
    String8 key;
    const char BT_NREC_KEY[] = "bt_headset_nrec";
    const char BT_NAME_KEY[] = "bt_headset_name";
    const char BT_NREC_VALUE_ON[] = "on";

#ifdef HTC_AUDIO
    const char ACTIVE_AP[] = "active_ap";
    const char EFFECT_ENABLED[] = "sound_effect_enable";
#endif

    ALOGV("setParameters() %s", keyValuePairs.string());

    if (keyValuePairs.length() == 0) return BAD_VALUE;

    key = String8(BT_NREC_KEY);
    if (param.get(key, value) == NO_ERROR) {
        if (value == BT_NREC_VALUE_ON) {
            mBluetoothNrec = true;
        } else {
            mBluetoothNrec = false;
            ALOGI("Turning noise reduction and echo cancellation off for BT "
                 "headset");
        }
    }
    key = String8(BTHEADSET_VGS);
    if (param.get(key, value) == NO_ERROR) {
        if (value == BT_NREC_VALUE_ON) {
            mBluetoothVGS = true;
        } else {
            mBluetoothVGS = false;
        }
    }
    key = String8(BT_NAME_KEY);
    if (param.get(key, value) == NO_ERROR) {
#ifdef HTC_AUDIO
        mBluetoothIdTx = 0;
        mBluetoothIdRx = 0;
        for (int i = 0; i < mNumBTEndpoints; i++) {
            if (!strcasecmp(value.string(), mBTEndpoints[i].name)) {
                mBluetoothIdTx = mBTEndpoints[i].tx;
                mBluetoothIdRx = mBTEndpoints[i].rx;
                ALOGD("Using custom acoustic parameters for %s", value.string());
                break;
            }
        }
        if (mBluetoothIdTx == 0) {
            ALOGD("Using default acoustic parameters "
                 "(%s not in acoustic database)", value.string());
        }
#endif
       doRouting(NULL, 0);
   }
    key = String8(DUALMIC_KEY);
    if (param.get(key, value) == NO_ERROR) {
        if (value == "true") {
            mDualMicEnabled = true;
            ALOGI("DualMike feature Enabled");
        } else {
            mDualMicEnabled = false;
            ALOGI("DualMike feature Disabled");
        }
        doRouting(NULL, 0);
    }

    key = String8(TTY_MODE_KEY);
    if (param.get(key, value) == NO_ERROR) {
        if (value == "full" || value == "tty_full") {
            mTtyMode = TTY_FULL;
        } else if (value == "hco" || value == "tty_hco") {
            mTtyMode = TTY_HCO;
        } else if (value == "vco" || value == "tty_vco") {
            mTtyMode = TTY_VCO;
        } else {
            mTtyMode = TTY_OFF;
        }
        if(mMode != AUDIO_MODE_IN_CALL){
           return NO_ERROR;
        }
        ALOGI("Changed TTY Mode=%s", value.string());
        if((mMode == AUDIO_MODE_IN_CALL) &&
          (cur_rx == DEVICE_HEADSET_RX) &&
          (cur_tx == DEVICE_HEADSET_TX))
          doRouting(NULL, 0);
    }

#ifdef HTC_AUDIO
    key = String8(ACTIVE_AP);
    if (param.get(key, value) == NO_ERROR) {
        const char* active_ap = value.string();
        ALOGD("Active AP = %s", active_ap);
        strcpy(mActiveAP, active_ap);

        const char* dsp_effect = "\0";
        key = String8(DSP_EFFECT_KEY);
        if (param.get(key, value) == NO_ERROR) {
            ALOGD("DSP Effect = %s", value.string());
            dsp_effect = value.string();
            strcpy(mEffect, dsp_effect);
        }

        key = String8(EFFECT_ENABLED);
        if (param.get(key, value) == NO_ERROR) {
            const char* sound_effect_enable = value.string();
            ALOGD("Sound Effect Enabled = %s", sound_effect_enable);
            if (value == "on") {
                mEffectEnabled = true;
                if (support_aic3254)
                    aic3254_config(get_snd_dev());
            } else {
                strcpy(mEffect, "\0");
                mEffectEnabled = false;
            }
        }
    }
#endif

    return NO_ERROR;
}

String8 AudioHardware::getParameters(const String8& keys)
{
    AudioParameter param = AudioParameter(keys);
    String8 value;

    String8 key = String8(DUALMIC_KEY);
    if (param.get(key, value) == NO_ERROR) {
        value = String8(mDualMicEnabled ? "true" : "false");
        param.add(key, value);
    }
    key = String8(BTHEADSET_VGS);
    if (param.get(key, value) == NO_ERROR) {
        if(mBluetoothVGS)
           param.addInt(String8("isVGS"), true);
    }

#ifdef HTC_AUDIO
    key = String8(DSP_EFFECT_KEY);
    if (param.get(key, value) == NO_ERROR) {
        value = String8(mCurDspProfile);
        param.add(key, value);
    }
#endif

    ALOGV("AudioHardware::getParameters() %s", param.toString().string());
    return param.toString();
}


static unsigned calculate_audpre_table_index(unsigned index)
{
    switch (index) {
        case 48000:    return SAMP_RATE_INDX_48000;
        case 44100:    return SAMP_RATE_INDX_44100;
        case 32000:    return SAMP_RATE_INDX_32000;
        case 24000:    return SAMP_RATE_INDX_24000;
        case 22050:    return SAMP_RATE_INDX_22050;
        case 16000:    return SAMP_RATE_INDX_16000;
        case 12000:    return SAMP_RATE_INDX_12000;
        case 11025:    return SAMP_RATE_INDX_11025;
        case 8000:    return SAMP_RATE_INDX_8000;
        default:     return -1;
    }
}
size_t AudioHardware::getInputBufferSize(uint32_t sampleRate, int format, int channelCount)
{

    if ((format != AUDIO_FORMAT_PCM_16_BIT) &&
        (format != AUDIO_FORMAT_AAC)) {
        ALOGW("getInputBufferSize bad format: %d", format);
        return 0;
    }
    if (channelCount < 1 || channelCount > 2) {
        ALOGW("getInputBufferSize bad channel count: %d", channelCount);
        return 0;
    }

    if (format == AUDIO_FORMAT_AAC)
       return 2048;
    else
    {
        if (build_id[17] == '1') {
            /*
            Return pcm record buffer size based on the sampling rate:
            If sampling rate >= 44.1 Khz, use 512 samples/channel pcm recording and
            If sampling rate < 44.1 Khz, use 256 samples/channel pcm recording
            */
           if(sampleRate>=44100)
               return 1024*channelCount;
           else
               return 512*channelCount;
        }
        else {
           return 2048*channelCount;
        }
    }
    return 0;
}
static status_t set_volume_rpc(uint32_t device,
                               uint32_t method,
                               uint32_t volume)
{
    ALOGV("set_volume_rpc(%d, %d, %d)\n", device, method, volume);

    if (device == -1UL) return NO_ERROR;
     return NO_ERROR;
}

status_t AudioHardware::setVoiceVolume(float v)
{
    if (v < 0.0) {
        ALOGW("setVoiceVolume(%f) under 0.0, assuming 0.0\n", v);
        v = 0.0;
    } else if (v > 1.0) {
        ALOGW("setVoiceVolume(%f) over 1.0, assuming 1.0\n", v);
        v = 1.0;
    }

    mVoiceVolume = v;

    int vol = lrint(v * 100.0);
    ALOGD("setVoiceVolume(%f)\n", v);
    ALOGI("Setting in-call volume to %d (available range is 0 to 100)\n", vol);

    if(msm_set_voice_rx_vol(vol)) {
        ALOGE("msm_set_voice_rx_vol(%d) failed errno = %d",vol,errno);
        return -1;
    }
    ALOGV("msm_set_voice_rx_vol(%d) succeeded",vol);
    return NO_ERROR;
}

#ifdef QCOM_FM_ENABLED
status_t AudioHardware::setFmVolume(float value)
{
    mFmVolume = value;

    Routing_table *temp = getNodeByStreamType(FM_RADIO);
    if(temp == NULL)
        return NO_ERROR;

    float vol;
    if (value < 0.0) {
        ALOGW("%s: (%f) Under 0.0, assuming 0.0\n", __FUNCTION__, value);
        value = 0.0;
    } else if (value > 1.0) {
        ALOGW("%s: (%f) Over 1.0, assuming 1.0\n", __FUNCTION__, value);
        value = 1.0;
    }

    vol = value * 100;

    ALOGD("Setting FM volume to %f", vol);

    if(msm_set_volume(temp->dec_id, vol)) {
        ALOGE("msm_set_volume(%d) failed for FM errno = %d", vol, errno);
        return -1;
    }

    return NO_ERROR;
}
#endif

status_t AudioHardware::setMasterVolume(float v)
{
    Mutex::Autolock lock(mLock);
    int vol = ceil(v * 7.0);
    ALOGI("Set master volume to %d.\n", vol);

    set_volume_rpc(SND_DEVICE_HANDSET, SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_SPEAKER, SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_BT,      SND_METHOD_VOICE, vol);
    set_volume_rpc(SND_DEVICE_HEADSET, SND_METHOD_VOICE, vol);
    //TBD - does HDMI require this handling

    // We return an error code here to let the audioflinger do in-software
    // volume on top of the maximum volume that we set through the SND API.
    // return error - software mixer will handle it
    return -1;
}

#ifdef HTC_AUDIO
status_t get_batt_temp(int *batt_temp) {
    ALOGD("Enable ALT for speaker");

    int i, fd, len;
    char get_batt_temp[6] = { 0 };
    const char *fn[] = {
         "/sys/devices/platform/rs30100001:00000000.0/power_supply/battery/batt_temp",
         "/sys/devices/platform/rs30100001:00000000/power_supply/battery/batt_temp" };

    for (i = 0; i < 2; i++) {
       if ((fd = open(fn[i], O_RDONLY)) >= 0)
           break;
    }
    if (fd <= 0) {
       ALOGE("Couldn't open sysfs file batt_temp");
       return UNKNOWN_ERROR;
    }

    if ((len = read(fd, get_batt_temp, sizeof(get_batt_temp))) <= 1) {
        ALOGE("read battery temp fail: %s", strerror(errno));
        close(fd);
        return BAD_VALUE;
    }

    *batt_temp = strtol(get_batt_temp, NULL, 10);
    ALOGD("ALT batt_temp = %d", *batt_temp);

    close(fd);
    return NO_ERROR;
}

status_t do_tpa2051_control(int inCall)
{
    int fd, rc;
    int tpa_mode = TPA2051_MODE_OFF;
    int batt_temp = 0;

    if (inCall) {
        if (cur_rx == DEVICE_HEADSET_RX)
            tpa_mode = TPA2051_MODE_VOICECALL_HEADSET;
        else if (cur_rx == DEVICE_SPEAKER_RX)
            tpa_mode = TPA2051_MODE_VOICECALL_SPKR;
    } else {
        if (cur_rx == DEVICE_FMRADIO_HEADSET_RX)
                tpa_mode = TPA2051_MODE_FM_HEADSET;
        else if (cur_rx == DEVICE_FMRADIO_SPEAKER_RX)
                tpa_mode = TPA2051_MODE_FM_SPKR;
        else if (cur_rx == DEVICE_SPEAKER_HEADSET_RX)
                tpa_mode = TPA2051_MODE_RING;
        else if (cur_rx == DEVICE_HEADSET_RX)
                tpa_mode = TPA2051_MODE_PLAYBACK_HEADSET;
        else if (cur_rx == DEVICE_SPEAKER_RX)
                tpa_mode = TPA2051_MODE_PLAYBACK_SPKR;
    }

    fd = open("/dev/tpa2051d3", O_RDWR);
    if (fd < 0) {
        ALOGE("can't open /dev/tpa2051d3 %d", fd);
        return -1;
    }

    if (tpa_mode != cur_tpa_mode) {
        cur_tpa_mode = tpa_mode;
        rc = ioctl(fd, TPA2051_SET_MODE, &tpa_mode);
        if (rc < 0)
            ALOGE("ioctl TPA2051_SET_MODE failed: %s", strerror(errno));
        else
            ALOGD("update TPA2051_SET_MODE to mode %d success", tpa_mode);
    }

    if (alt_enable && cur_rx == DEVICE_SPEAKER_RX) {
        if (get_batt_temp(&batt_temp) == NO_ERROR) {
            if (batt_temp < 50) {
                tpa_mode = 629276672;
                rc = ioctl(fd, TPA2051_SET_CONFIG, &tpa_mode);
                if (rc < 0)
                    ALOGE("ioctl TPA2051_SET_CONFIG failed: %s", strerror(errno));
                else
                    ALOGD("update TPA2051_SET_CONFIG to mode %d success", tpa_mode);
            }
        }
    }

    close(fd);
    return 0;
}

static status_t do_route_audio_rpc(uint32_t device,
                                   bool ear_mute, bool mic_mute,
                                   uint32_t rx_acdb_id, uint32_t tx_acdb_id)
#else
static status_t do_route_audio_rpc(uint32_t device,
                                   bool ear_mute, bool mic_mute)
#endif
{
    if(device == -1)
        return 0;

    int new_rx_device = INVALID_DEVICE,new_tx_device = INVALID_DEVICE,fm_device = INVALID_DEVICE;
    Routing_table* temp = NULL;
    ALOGV("do_route_audio_rpc(%d, %d, %d)", device, ear_mute, mic_mute);

    if(device == SND_DEVICE_HANDSET) {
        new_rx_device = DEVICE_HANDSET_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        ALOGV("In HANDSET");
    }
    else if(device == SND_DEVICE_SPEAKER) {
        new_rx_device = DEVICE_SPEAKER_RX;
        new_tx_device = DEVICE_SPEAKER_TX;
        ALOGV("In SPEAKER");
    }
    else if(device == SND_DEVICE_HEADSET) {
        new_rx_device = DEVICE_HEADSET_RX;
        new_tx_device = DEVICE_HEADSET_TX;
        ALOGV("In HEADSET");
    }
    else if(device == SND_DEVICE_NO_MIC_HEADSET) {
        new_rx_device = DEVICE_HEADSET_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        ALOGV("In NO MIC HEADSET");
    }
    else if (device == SND_DEVICE_FM_HANDSET) {
        fm_device = DEVICE_FMRADIO_HANDSET_RX;
        ALOGV("In FM HANDSET");
    }
    else if(device == SND_DEVICE_FM_SPEAKER) {
        fm_device = DEVICE_FMRADIO_SPEAKER_RX;
        ALOGV("In FM SPEAKER");
    }
    else if(device == SND_DEVICE_FM_HEADSET) {
        fm_device = DEVICE_FMRADIO_HEADSET_RX;
        ALOGV("In FM HEADSET");
    }
    else if(device == SND_DEVICE_IN_S_SADC_OUT_HANDSET) {
        new_rx_device = DEVICE_HANDSET_RX;
        new_tx_device = DEVICE_DUALMIC_HANDSET_TX;
        ALOGV("In DUALMIC_HANDSET");
    }
    else if(device == SND_DEVICE_IN_S_SADC_OUT_SPEAKER_PHONE) {
        new_rx_device = DEVICE_SPEAKER_RX;
        new_tx_device = DEVICE_DUALMIC_SPEAKER_TX;
        ALOGV("In DUALMIC_SPEAKER");
    }
    else if(device == SND_DEVICE_TTY_FULL) {
        new_rx_device = DEVICE_TTY_HEADSET_MONO_RX;
        new_tx_device = DEVICE_TTY_HEADSET_MONO_TX;
        ALOGV("In TTY_FULL");
    }
    else if(device == SND_DEVICE_TTY_VCO) {
        new_rx_device = DEVICE_TTY_HEADSET_MONO_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        ALOGV("In TTY_VCO");
    }
    else if(device == SND_DEVICE_TTY_HCO) {
        new_rx_device = DEVICE_HANDSET_RX;
        new_tx_device = DEVICE_TTY_HEADSET_MONO_TX;
        ALOGV("In TTY_HCO");
    }
    else if((device == SND_DEVICE_BT) ||
            (device == SND_DEVICE_BT_EC_OFF)) {
        new_rx_device = DEVICE_BT_SCO_RX;
        new_tx_device = DEVICE_BT_SCO_TX;
        ALOGV("In BT_HCO");
    }
    else if(device == SND_DEVICE_HEADSET_AND_SPEAKER) {
        new_rx_device = DEVICE_SPEAKER_HEADSET_RX;
        new_tx_device = DEVICE_HEADSET_TX;
        ALOGV("In DEVICE_SPEAKER_HEADSET_RX and DEVICE_HEADSET_TX");
    }
    else if(device == SND_DEVICE_HEADPHONE_AND_SPEAKER) {
        new_rx_device = DEVICE_SPEAKER_HEADSET_RX;
        new_tx_device = DEVICE_HANDSET_TX;
        ALOGV("In DEVICE_SPEAKER_HEADSET_RX and DEVICE_HANDSET_TX");
    }
    else if (device == SND_DEVICE_HDMI) {
        new_rx_device = DEVICE_HDMI_STERO_RX;
        new_tx_device = cur_tx;
        ALOGV("In DEVICE_HDMI_STERO_RX and cur_tx");
    }
    else if(device == SND_DEVICE_FM_TX) {
        new_rx_device = DEVICE_FMRADIO_STEREO_RX;
        new_tx_device = cur_tx;
        ALOGV("In DEVICE_FMRADIO_STEREO_RX and cur_tx");
    }
#ifdef SAMSUNG_AUDIO
    else if (device == SND_DEVICE_VOIP_HANDSET) {
        new_rx_device = DEVICE_HANDSET_VOIP_RX;
        new_tx_device = DEVICE_HANDSET_VOIP_TX;
        ALOGV("In VOIP HANDSET");
    }
    else if (device == SND_DEVICE_VOIP_SPEAKER) {
        new_rx_device = DEVICE_SPEAKER_VOIP_RX;
        new_tx_device = DEVICE_SPEAKER_VOIP_TX;
        ALOGV("In VOIP SPEAKER");
    }
    else if (device == SND_DEVICE_VOIP_HEADSET) {
        new_rx_device = DEVICE_HEADSET_VOIP_RX;
        new_tx_device = DEVICE_HEADSET_VOIP_TX;
        ALOGV("In VOIP HEADSET");
    }
    else if (device == SND_DEVICE_CALL_HANDSET) {
        new_rx_device = DEVICE_HANDSET_CALL_RX;
        new_tx_device = DEVICE_HANDSET_CALL_TX;
        ALOGV("In CALL HANDSET");
    }
    else if (device == SND_DEVICE_CALL_SPEAKER) {
        new_rx_device = DEVICE_SPEAKER_CALL_RX;
        new_tx_device = DEVICE_SPEAKER_CALL_TX;
        ALOGV("In CALL SPEAKER");
    }
    else if (device == SND_DEVICE_CALL_HEADSET) {
        new_rx_device = DEVICE_HEADSET_CALL_RX;
        new_tx_device = DEVICE_HEADSET_CALL_TX;
        ALOGV("In CALL HEADSET");
    }
#endif
#ifdef SEMC_AUDIO
    else if (device == SND_DEVICE_SPEAKER_DUAL_MIC_REAL_STEREO) {
        new_rx_device = cur_rx;
        new_tx_device = DEVICE_SPEAKER_DUAL_MIC_REAL_STEREO_TX;
        ALOGV("In dual mic real stereo endfire");
    }
#endif

    if(new_rx_device != INVALID_DEVICE)
        ALOGD("new_rx = %d", DEV_ID(new_rx_device));
    if(new_tx_device != INVALID_DEVICE)
        ALOGD("new_tx = %d", DEV_ID(new_tx_device));

    if (ear_mute == false && !isStreamOn(VOICE_CALL)) {
        ALOGV("Going to enable RX/TX device for voice stream");
            // Routing Voice
            if ( (new_rx_device != INVALID_DEVICE) && (new_tx_device != INVALID_DEVICE))
            {
                ALOGD("Starting voice on Rx %d and Tx %d device", DEV_ID(new_rx_device), DEV_ID(new_tx_device));

#ifdef HTC_AUDIO
                updateACDB(new_rx_device, new_tx_device, rx_acdb_id, tx_acdb_id);
#endif

                msm_route_voice(DEV_ID(new_rx_device),DEV_ID(new_tx_device), 1);
            }
            else
            {
                return -1;
            }

            if(cur_rx == INVALID_DEVICE || new_rx_device == INVALID_DEVICE)
                return -1;

            if(cur_tx == INVALID_DEVICE || new_tx_device == INVALID_DEVICE)
                return -1;

           //Enable RX device
           if(new_rx_device != cur_rx) {
               enableDevice(cur_rx,0);
           }
           enableDevice(new_rx_device,1);

           //Enable TX device
           if(new_tx_device != cur_tx) {
               enableDevice(cur_tx,0);
           }
           enableDevice(new_tx_device,1);

            // start Voice call
            ALOGD("Starting voice call and UnMuting the call");
            msm_start_voice();
            msm_set_voice_tx_mute(0);
            cur_rx = new_rx_device;
            cur_tx = new_tx_device;
            addToTable(0,cur_rx,cur_tx,VOICE_CALL,true);
#ifdef HTC_AUDIO
            updateDeviceInfo(new_rx_device,new_tx_device, rx_acdb_id, tx_acdb_id);
#else
            updateDeviceInfo(new_rx_device,new_tx_device);
#endif
    }
    else if (ear_mute == true && isStreamOnAndActive(VOICE_CALL)) {
        ALOGV("Going to disable RX/TX device during end of voice call");
        temp = getNodeByStreamType(VOICE_CALL);
        if(temp == NULL)
            return 0;

        // Ending voice call
        ALOGD("Ending Voice call");
        msm_end_voice();
        deleteFromTable(VOICE_CALL);
#ifdef HTC_AUDIO
        updateDeviceInfo(new_rx_device,new_tx_device, 0, 0);
#else
        updateDeviceInfo(new_rx_device,new_tx_device);
#endif
        if(new_rx_device != INVALID_DEVICE && new_tx_device != INVALID_DEVICE) {
            cur_rx = new_rx_device;
            cur_tx = new_tx_device;
        }
    }
    else {
#ifdef HTC_AUDIO
        updateDeviceInfo(new_rx_device,new_tx_device, rx_acdb_id, tx_acdb_id);
    }

    if (support_tpa2051)
        do_tpa2051_control(ear_mute ^1);
#else
        updateDeviceInfo(new_rx_device,new_tx_device);
    }
#endif

    return NO_ERROR;
}

#ifdef HTC_AUDIO
status_t AudioHardware::doAudioRouteOrMuteHTC(uint32_t device)
{
    uint32_t rx_acdb_id = 0;
    uint32_t tx_acdb_id = 0;

    if (device == SND_DEVICE_BT) {
        if (!mBluetoothNrec)
            device = SND_DEVICE_BT_EC_OFF;
    }

    if (support_aic3254) {
        aic3254_config(device);
        do_aic3254_control(device);
    }

    if (device == SND_DEVICE_BT) {
        if (mBluetoothIdTx != 0) {
            rx_acdb_id = mBluetoothIdRx;
            tx_acdb_id = mBluetoothIdTx;
        } else {
            /* use default BT entry defined in AudioBTID.csv */
            rx_acdb_id = mBTEndpoints[0].rx;
            tx_acdb_id = mBTEndpoints[0].tx;
            ALOGD("Update ACDB ID to default BT setting");
        }
    } else if (device == SND_DEVICE_CARKIT ||
               device == SND_DEVICE_BT_EC_OFF) {
        if (mBluetoothIdTx != 0) {
            rx_acdb_id = mBluetoothIdRx;
            tx_acdb_id = mBluetoothIdTx;
        } else {
            /* use default carkit entry defined in AudioBTID.csv */
            rx_acdb_id = mBTEndpoints[1].rx;
            tx_acdb_id = mBTEndpoints[1].tx;
            ALOGD("Update ACDB ID to default carkit setting");
        }
    } else if (isInCall() && hac_enable && mHACSetting &&
               device == SND_DEVICE_HANDSET) {
        ALOGD("Update acdb id to hac profile.");
        rx_acdb_id = ACDB_ID_HAC_HANDSET_SPKR;
        tx_acdb_id = ACDB_ID_HAC_HANDSET_MIC;
    } else {
        if (isInCall()) {
            rx_acdb_id = getACDB(MOD_RX, device);
            tx_acdb_id = getACDB(MOD_TX, device);
        } else {
            if (!checkOutputStandby())
                rx_acdb_id = getACDB(MOD_PLAY, device);

            if (mRecordState)
                tx_acdb_id = getACDB(MOD_REC, device);
        }
    }


    ALOGV("doAudioRouteOrMuteHTC() rx acdb %d, tx acdb %d", rx_acdb_id, tx_acdb_id);
    ALOGV("doAudioRouteOrMuteHTC() device %x, mMode %d, mMicMute %d",
            device, mMode, mMicMute);
    return do_route_audio_rpc(device, !isInCall(), mMicMute, rx_acdb_id, tx_acdb_id);
}
#endif

// always call with mutex held
status_t AudioHardware::doAudioRouteOrMute(uint32_t device)
{
// BT acoustics is not supported. This might be used by OEMs. Hence commenting
// the code and not removing it.
#if 0
    if (device == (uint32_t)SND_DEVICE_BT || device == (uint32_t)SND_DEVICE_CARKIT) {
        if (mBluetoothId) {
            device = mBluetoothId;
        } else if (!mBluetoothNrec) {
            device = SND_DEVICE_BT_EC_OFF;
        }
    }
#endif

    status_t ret = NO_ERROR;

#ifdef HTC_AUDIO
    ret = doAudioRouteOrMuteHTC(device);
#else
    ALOGV("doAudioRouteOrMute() device %x, mMode %d, mMicMute %d", device, mMode, mMicMute);
    ret = do_route_audio_rpc(device, !isInCall(), mMicMute);
#endif

    if (isStreamOnAndActive(VOICE_CALL) && mMicMute == false)
        msm_set_voice_tx_mute(0);

    if (isInCall())
        setVoiceVolume(mVoiceVolume);

    return ret;
}

#ifdef HTC_AUDIO
status_t AudioHardware::get_mMode(void) {
    return mMode;
}

status_t AudioHardware::set_mRecordState(bool onoff) {
    mRecordState = onoff;
    return 0;
}

status_t AudioHardware::get_mRecordState(void) {
    return mRecordState;
}

status_t AudioHardware::get_snd_dev(void) {
    return mCurSndDevice;
}

uint32_t AudioHardware::getACDB(int mode, uint32_t device) {

    uint32_t acdb_id = 0;
    int batt_temp = 0;
    int vol = lrint(mVoiceVolume * 100.0);

    ALOGD("getACDB(%d)\n", vol);

    if (mMode == AudioSystem::MODE_IN_CALL &&
        device <= SND_DEVICE_NO_MIC_HEADSET) {
        if (mode == MOD_RX) {
            if(device == SND_DEVICE_HANDSET) {
                acdb_id = vol / 20 + 201;
            }
            else if(device == SND_DEVICE_HEADSET || device == SND_DEVICE_NO_MIC_HEADSET) {
                acdb_id = vol / 20 + 401;
            }
            else if(device == SND_DEVICE_SPEAKER) {
                acdb_id = vol / 20 + 601;
            }
        } else if (mode == MOD_TX) {
            if(device == SND_DEVICE_HANDSET) {
                acdb_id = vol / 20 + 101;
            }
            else if(device == SND_DEVICE_HEADSET || device == SND_DEVICE_NO_MIC_HEADSET) {
                acdb_id = vol / 20 + 301;
            }
            else if(device == SND_DEVICE_SPEAKER) {
                acdb_id = vol / 20 + 501;
            }
        }
    } else {
        if (mode == MOD_PLAY) {
            if(device == SND_DEVICE_HEADSET || device == SND_DEVICE_NO_MIC_HEADSET || device == SND_DEVICE_NO_MIC_HEADSET_BACK_MIC || device == SND_DEVICE_FM_HEADSET) {
                acdb_id = ACDB_ID_HEADSET_PLAYBACK;
            }
            else if(device == SND_DEVICE_SPEAKER || device == SND_DEVICE_FM_SPEAKER || device == SND_DEVICE_SPEAKER_BACK_MIC) {
                acdb_id = ACDB_ID_SPKR_PLAYBACK;
            }
            else if(device == SND_DEVICE_HEADSET_AND_SPEAKER || device == SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC || device == SND_DEVICE_HEADPHONE_AND_SPEAKER) {
                acdb_id = ACDB_ID_HEADSET_RINGTONE_PLAYBACK;
            }
        } else if (mode == MOD_REC) {
            if(device == SND_DEVICE_HEADSET || device == SND_DEVICE_FM_HEADSET || device == SND_DEVICE_FM_SPEAKER || device == SND_DEVICE_HEADSET_AND_SPEAKER) {
                acdb_id = ACDB_ID_EXT_MIC_REC;
            }
            else if(device == SND_DEVICE_HANDSET || device == SND_DEVICE_NO_MIC_HEADSET || device == SND_DEVICE_SPEAKER) {
                acdb_id = ACDB_ID_INT_MIC_REC;
            }
            else if(device == SND_DEVICE_SPEAKER_BACK_MIC || device == SND_DEVICE_NO_MIC_HEADSET_BACK_MIC || device == SND_DEVICE_HANDSET_BACK_MIC || device == SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC) {
                acdb_id = ACDB_ID_CAMCORDER;
            }
        }
    }
    ALOGD("getACDB, return ID %d", acdb_id);
    return acdb_id;
}

status_t AudioHardware::do_aic3254_control(uint32_t device) {
    ALOGD("do_aic3254_control device: %d mode: %d record: %d", device, mMode, mRecordState);
    uint32_t new_aic_txmode = UPLINK_OFF;
    uint32_t new_aic_rxmode = DOWNLINK_OFF;

    Mutex::Autolock lock(mAIC3254ConfigLock);

    if (mMode == AudioSystem::MODE_IN_CALL) {
        if(device == SND_DEVICE_HANDSET) {
            new_aic_rxmode = CALL_DOWNLINK_EMIC_HEADSET;
            new_aic_txmode = CALL_UPLINK_EMIC_HEADSET;
        }
        else if(device == SND_DEVICE_SPEAKER || device == SND_DEVICE_SPEAKER_BACK_MIC) {
            new_aic_rxmode = CALL_DOWNLINK_IMIC_SPEAKER;
            new_aic_txmode = CALL_UPLINK_IMIC_SPEAKER;
        }
        else if(device == SND_DEVICE_HEADSET_AND_SPEAKER || device == SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC) {
            new_aic_rxmode = RING_HEADSET_SPEAKER;
        }
        else if(device == SND_DEVICE_NO_MIC_HEADSET || device == SND_DEVICE_NO_MIC_HEADSET_BACK_MIC) {
            new_aic_rxmode = CALL_DOWNLINK_IMIC_HEADSET;
            new_aic_txmode = CALL_UPLINK_IMIC_HEADSET;
        }
        else if(device == SND_DEVICE_HANDSET || device == SND_DEVICE_HANDSET_BACK_MIC) {
            new_aic_rxmode = CALL_DOWNLINK_IMIC_RECEIVER;
            new_aic_txmode = CALL_UPLINK_IMIC_RECEIVER;
        }
    } else {
        if (checkOutputStandby()) {
            if (device == SND_DEVICE_FM_HEADSET) {
                new_aic_rxmode = FM_OUT_HEADSET;
                new_aic_txmode = FM_IN_HEADSET;
            } else if (device == SND_DEVICE_FM_SPEAKER) {
                new_aic_rxmode = FM_OUT_SPEAKER;
                new_aic_txmode = FM_IN_SPEAKER;
            }
        } else {
            if(device == SND_DEVICE_HEADSET_AND_SPEAKER || device == SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC || device == SND_DEVICE_HEADPHONE_AND_SPEAKER) {
                new_aic_rxmode = RING_HEADSET_SPEAKER;
            }
            else if(device == SND_DEVICE_SPEAKER || device == SND_DEVICE_SPEAKER_BACK_MIC) {
                new_aic_rxmode = PLAYBACK_SPEAKER;
            }
            else if(device == SND_DEVICE_HANDSET || device == SND_DEVICE_HANDSET_BACK_MIC) {
                new_aic_rxmode = PLAYBACK_RECEIVER;
            }
            else if(device == SND_DEVICE_HEADSET || device == SND_DEVICE_NO_MIC_HEADSET || device == SND_DEVICE_NO_MIC_HEADSET_BACK_MIC) {
                new_aic_rxmode = PLAYBACK_HEADSET;
            }
        }

        if (mRecordState) {
            if(device == SND_DEVICE_HEADSET) {
                new_aic_txmode = VOICERECORD_EMIC;
            }
            else if(device == SND_DEVICE_HANDSET_BACK_MIC || device == SND_DEVICE_SPEAKER_BACK_MIC || device == SND_DEVICE_NO_MIC_HEADSET_BACK_MIC || device == SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC) {
                new_aic_txmode = VIDEORECORD_IMIC;
            }
            else if(device == SND_DEVICE_HANDSET || device == SND_DEVICE_SPEAKER || device == SND_DEVICE_NO_MIC_HEADSET || device == SND_DEVICE_HEADSET_AND_SPEAKER) {
                new_aic_txmode = VOICERECORD_IMIC;
            }
        }
    }
    ALOGD("aic3254_ioctl: new_aic_rxmode %d cur_aic_rx %d", new_aic_rxmode, cur_aic_rx);
    if (new_aic_rxmode != cur_aic_rx)
        if (aic3254_ioctl(AIC3254_CONFIG_RX, new_aic_rxmode) >= 0)
            cur_aic_rx = new_aic_rxmode;

    ALOGD("aic3254_ioctl: new_aic_txmode %d cur_aic_tx %d", new_aic_txmode, cur_aic_tx);
    if (new_aic_txmode != cur_aic_tx)
        if (aic3254_ioctl(AIC3254_CONFIG_TX, new_aic_txmode) >= 0)
            cur_aic_tx = new_aic_txmode;

    if (cur_aic_tx == UPLINK_OFF && cur_aic_rx == DOWNLINK_OFF && aic3254_enabled) {
        strcpy(mCurDspProfile, "\0");
        aic3254_enabled = false;
        aic3254_powerdown();
    } else if (cur_aic_tx != UPLINK_OFF || cur_aic_rx != DOWNLINK_OFF)
        aic3254_enabled = true;
    return NO_ERROR;

}

bool AudioHardware::isAic3254Device(uint32_t device) {
    if(device == SND_DEVICE_HEADSET || device == SND_DEVICE_SPEAKER || device == SND_DEVICE_HEADSET || device == SND_DEVICE_NO_MIC_HEADSET || device == SND_DEVICE_FM_HEADSET || device == SND_DEVICE_HEADSET_AND_SPEAKER || device == SND_DEVICE_FM_SPEAKER || device == SND_DEVICE_HEADPHONE_AND_SPEAKER || device == SND_DEVICE_HANDSET_BACK_MIC || device == SND_DEVICE_SPEAKER_BACK_MIC || device == SND_DEVICE_NO_MIC_HEADSET_BACK_MIC || device == SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC) {
        return true;
    } else {
        return false;
    }
}

status_t AudioHardware::aic3254_config(uint32_t device) {
    ALOGD("aic3254_config: device %d enabled %d", device, aic3254_enabled);
    char name[22] = "\0";
    char aap[9] = "\0";

    if ((!isAic3254Device(device) ||
         !aic3254_enabled) &&
        strlen(mCurDspProfile) != 0)
        return NO_ERROR;

    Mutex::Autolock lock(mAIC3254ConfigLock);

    if (mMode == AudioSystem::MODE_IN_CALL) {
#ifdef WITH_SPADE_DSP_PROFILE
        if (support_htc_backmic) {
            strcpy(name, "DualMic_Phone");
            if(device == SND_DEVICE_HANDSET || device == SND_DEVICE_HANDSET_BACK_MIC || device == SND_DEVICE_HEADSET || device == SND_DEVICE_HEADSET_AND_SPEAKER || device == SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC || device == SND_DEVICE_NO_MIC_HEADSET) {
                strcat(name, "_EP");
            }
            else if(device == SND_DEVICE_SPEAKER) {
                strcat(name, "_SPK");
            }
        } else {
            strcpy(name, "Original_Phone");
        }
#else
        strcpy(name, "Original_Phone");
        if(device == SND_DEVICE_HANDSET || device == SND_DEVICE_HANDSET_BACK_MIC) {
            strcat(name, "_REC");
        }
        else if(device == SND_DEVICE_HEADSET || device == SND_DEVICE_HEADSET_AND_SPEAKER || device == SND_DEVICE_HEADSET_AND_SPEAKER_BACK_MIC || device == SND_DEVICE_NO_MIC_HEADSET) {
            strcat(name, "_HP");
        }
        else if(device == SND_DEVICE_SPEAKER) {
            strcat(name, "_SPK");
        }
#endif
    } else {
#ifdef WITH_SPADE_DSP_PROFILE
        if (mRecordState) {
#else
        if ((strcasecmp(mActiveAP, "Camcorder") == 0)) {
            if (strlen(mEffect) != 0) {
                strcpy(name, "Recording_");
                strcat(name, mEffect);
            } else
                strcpy(name, "Original");
        } else if (mRecordState) {
#endif
#ifdef WITH_SPADE_DSP_PROFILE
            strcpy(name, "Original");
#else
            strcpy(name, "Original_Recording");
#endif
        } else if (strlen(mEffect) == 0 && !mEffectEnabled)
           strcpy(name, "Original");
        else {
            if (mEffectEnabled)
                strcpy(name, mEffect);

            if ((strcasecmp(name, "Srs") == 0) ||
                (strcasecmp(name, "Dolby") == 0)) {
                strcpy(mEffect, name);
                if (strcasecmp(mActiveAP, "Music") == 0)
                    strcat(name, "_a");
                else if (strcasecmp(mActiveAP, "Video") == 0)
                    strcat(name, "_v");
                if (device == SND_DEVICE_SPEAKER)
                    strcat(name, "_spk");
                else
                    strcat(name, "_hp");
            }
        }
    }

    if (strcasecmp(mCurDspProfile, name)) {
        ALOGD("aic3254_config: loading effect %s", name);
        strcpy(mCurDspProfile, name);
    } else {
        ALOGD("aic3254_config: effect %s already loaded", name);
        return NO_ERROR;
    }

    int rc = set_sound_effect(name);
    if (rc < 0) {
        ALOGE("Could not set sound effect %s: %d", name, rc);
        return rc;
    }
    return NO_ERROR;
}

int AudioHardware::aic3254_ioctl(int cmd, const int argc) {
    int rc = -1;
    int (*set_aic3254_ioctl)(int, const int*);

    ALOGD("aic3254_ioctl()");

    set_aic3254_ioctl = (int (*)(int, const int*))::dlsym(acoustic, "set_aic3254_ioctl");
    if ((*set_aic3254_ioctl) == 0) {
        ALOGE("Could not open set_aic3254_ioctl()");
        return rc;
    }

    ALOGD("aic3254_ioctl: try ioctl 0x%x with arg %d", cmd, argc);
    rc = set_aic3254_ioctl(cmd, &argc);
    if (rc < 0)
        ALOGE("aic3254_ioctl failed");

    return rc;
}

void AudioHardware::aic3254_powerdown() {
    ALOGD("aic3254_powerdown");
    int rc = aic3254_ioctl(AIC3254_POWERDOWN, 0);
    if (rc < 0)
        ALOGE("aic3254_powerdown failed");
}

int AudioHardware::aic3254_set_volume(int volume) {
    ALOGD("aic3254_set_volume = %d", volume);

    if (aic3254_ioctl(AIC3254_CONFIG_VOLUME_L, volume) < 0)
        ALOGE("aic3254_set_volume: could not set aic3254 LEFT volume %d", volume);

    int rc = aic3254_ioctl(AIC3254_CONFIG_VOLUME_R, volume);
    if (rc < 0)
        ALOGE("aic3254_set_volume: could not set aic3254 RIGHT volume %d", volume);
    return rc;
}
#endif


status_t AudioHardware::doRouting(AudioStreamInMSM72xx *input, uint32_t outputDevices)
{
    Mutex::Autolock lock(mLock);
    status_t ret = NO_ERROR;
    int audProcess = (ADRC_DISABLE | EQ_DISABLE | RX_IIR_DISABLE);
    int sndDevice = -1;

    if (!outputDevices)
        outputDevices = mOutput->devices();

    ALOGD("outputDevices = %x", outputDevices);

    if (input != NULL) {
        uint32_t inputDevice = input->devices();
        ALOGI("do input routing device %x\n", inputDevice);
        // ignore routing device information when we start a recording in voice
        // call
        // Recording will happen through currently active tx device
        if((inputDevice == AUDIO_DEVICE_IN_VOICE_CALL)
#ifdef QCOM_FM_ENABLED
           || (inputDevice == AUDIO_DEVICE_IN_FM_RX)
           || (inputDevice == AUDIO_DEVICE_IN_FM_RX_A2DP)
#endif
          )
            return NO_ERROR;
        if (inputDevice & AUDIO_DEVICE_BIT_IN) {
            if (inputDevice == AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET) {
                ALOGI("Routing audio to Bluetooth PCM\n");
                sndDevice = SND_DEVICE_BT;
#ifdef SEMC_AUDIO
            // Camcorder case
            } else if (inputDevice & 0x80000004) {
                ALOGI("Routing audio to dual mic real stereo\n");
                sndDevice = SND_DEVICE_SPEAKER_DUAL_MIC_REAL_STEREO;
#endif
            } else if (inputDevice == AUDIO_DEVICE_IN_WIRED_HEADSET) {
                if ((outputDevices & AUDIO_DEVICE_OUT_WIRED_HEADSET) &&
                    (outputDevices & AUDIO_DEVICE_OUT_SPEAKER)) {
                    ALOGI("Routing audio to Wired Headset and Speaker\n");
                    sndDevice = SND_DEVICE_HEADSET_AND_SPEAKER;
                    audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE | MBADRC_ENABLE);
                } else {
                    ALOGI("Routing audio to Wired Headset\n");
                    sndDevice = SND_DEVICE_HEADSET;
                }
            } else {
                if (outputDevices == AUDIO_DEVICE_OUT_WIRED_HEADPHONE) {
                    ALOGI("Routing audio to Speakerphone\n");
                    sndDevice = SND_DEVICE_NO_MIC_HEADSET;
                } else if (outputDevices & AUDIO_DEVICE_OUT_EARPIECE) {
                    ALOGI("Routing audio to Handset\n");
                    sndDevice = SND_DEVICE_HANDSET;
                } else {
                    ALOGI("Routing audio to Speakerphone\n");
                    sndDevice = SND_DEVICE_SPEAKER;
                }
            }
        }
    }

    if (sndDevice == -1) {
        if (outputDevices & (outputDevices - 1)) {
            if ((outputDevices & AUDIO_DEVICE_OUT_SPEAKER) == 0) {
                ALOGW("Hardware does not support requested route combination (%#X),"
                     " picking closest possible route...", outputDevices);
            }
        }
        if ((mTtyMode != TTY_OFF) && (mMode == AUDIO_MODE_IN_CALL) &&
                (outputDevices & AUDIO_DEVICE_OUT_WIRED_HEADSET)) {
            if (mTtyMode == TTY_FULL) {
                ALOGI("Routing audio to TTY FULL Mode\n");
                sndDevice = SND_DEVICE_TTY_FULL;
            } else if (mTtyMode == TTY_VCO) {
                ALOGI("Routing audio to TTY VCO Mode\n");
                sndDevice = SND_DEVICE_TTY_VCO;
            } else if (mTtyMode == TTY_HCO) {
                ALOGI("Routing audio to TTY HCO Mode\n");
                sndDevice = SND_DEVICE_TTY_HCO;
            }
        } else if (outputDevices &
                   (AUDIO_DEVICE_OUT_BLUETOOTH_SCO | AUDIO_DEVICE_OUT_BLUETOOTH_SCO_HEADSET)) {
            ALOGI("Routing audio to Bluetooth PCM\n");
            sndDevice = SND_DEVICE_BT;
        } else if (outputDevices & AUDIO_DEVICE_OUT_BLUETOOTH_SCO_CARKIT) {
            ALOGI("Routing audio to Bluetooth PCM\n");
            sndDevice = SND_DEVICE_CARKIT;
        } else if (outputDevices & AUDIO_DEVICE_OUT_AUX_DIGITAL) {
            ALOGI("Routing audio to HDMI\n");
            sndDevice = SND_DEVICE_HDMI;
        } else if ((outputDevices & AUDIO_DEVICE_OUT_WIRED_HEADSET) &&
                   (outputDevices & AUDIO_DEVICE_OUT_SPEAKER)) {
            ALOGI("Routing audio to Wired Headset and Speaker\n");
            sndDevice = SND_DEVICE_HEADSET_AND_SPEAKER;
            audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE | MBADRC_ENABLE);
        }
#ifdef QCOM_FM_ENABLED
        else if ((outputDevices & AUDIO_DEVICE_OUT_FM_TX) &&
                   (outputDevices & AUDIO_DEVICE_OUT_SPEAKER)) {
            ALOGI("Routing audio to FM Tx and Speaker\n");
            sndDevice = SND_DEVICE_FM_TX_AND_SPEAKER;
            enableComboDevice(sndDevice,1);
            audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE | MBADRC_ENABLE);
        }
#endif
        else if (outputDevices & AUDIO_DEVICE_OUT_WIRED_HEADPHONE) {
            if (outputDevices & AUDIO_DEVICE_OUT_SPEAKER) {
                ALOGI("Routing audio to No microphone Wired Headset and Speaker (%d,%x)\n", mMode, outputDevices);
                sndDevice = SND_DEVICE_HEADPHONE_AND_SPEAKER;
                audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE | MBADRC_ENABLE);
            } else {
                ALOGI("Routing audio to No microphone Wired Headset (%d,%x)\n", mMode, outputDevices);
                sndDevice = SND_DEVICE_NO_MIC_HEADSET;
            }
        } else if (outputDevices & AUDIO_DEVICE_OUT_WIRED_HEADSET) {
            ALOGI("Routing audio to Wired Headset\n");
            sndDevice = SND_DEVICE_HEADSET;
            audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE | MBADRC_ENABLE);
        } else if (outputDevices & AUDIO_DEVICE_OUT_SPEAKER) {
            ALOGI("Routing audio to Speakerphone\n");
            sndDevice = SND_DEVICE_SPEAKER;
            audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE | MBADRC_ENABLE);
        } else if(outputDevices & AUDIO_DEVICE_OUT_EARPIECE){
            ALOGI("Routing audio to Handset\n");
            sndDevice = SND_DEVICE_HANDSET;
            audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE | MBADRC_ENABLE);
        }
#ifdef QCOM_FM_ENABLED
         else if(outputDevices & AUDIO_DEVICE_OUT_FM_TX){
            ALOGI("Routing audio to FM Tx Device\n");
            sndDevice = SND_DEVICE_FM_TX;
            audProcess = (ADRC_ENABLE | EQ_ENABLE | RX_IIR_ENABLE | MBADRC_ENABLE);
        }
#endif
    }

    if (mDualMicEnabled && mMode == AUDIO_MODE_IN_CALL) {
        if (sndDevice == SND_DEVICE_HANDSET) {
            ALOGI("Routing audio to handset with DualMike enabled\n");
            sndDevice = SND_DEVICE_IN_S_SADC_OUT_HANDSET;
        } else if (sndDevice == SND_DEVICE_SPEAKER) {
            ALOGI("Routing audio to speakerphone with DualMike enabled\n");
            sndDevice = SND_DEVICE_IN_S_SADC_OUT_SPEAKER_PHONE;
        }
    }

#ifdef SAMSUNG_AUDIO
    if (mMode == AUDIO_MODE_IN_CALL) {
        if (sndDevice == SND_DEVICE_HANDSET) {
            ALOGI("Routing audio to Call Handset\n");
            sndDevice = SND_DEVICE_CALL_HANDSET;
        } else if (sndDevice == SND_DEVICE_SPEAKER) {
            ALOGI("Routing audio to Call Speaker\n");
            sndDevice = SND_DEVICE_CALL_SPEAKER;
        } else if (sndDevice == SND_DEVICE_HEADSET) {
            ALOGI("Routing audio to Call Headset\n");
            sndDevice = SND_DEVICE_CALL_HEADSET;
        }
    } else if (mMode == AUDIO_MODE_IN_COMMUNICATION) {
        if (sndDevice == SND_DEVICE_HANDSET) {
            ALOGI("Routing audio to VOIP handset\n");
            sndDevice = SND_DEVICE_VOIP_HANDSET;
        } else if (sndDevice == SND_DEVICE_SPEAKER) {
            ALOGI("Routing audio to VOIP speaker\n");
            sndDevice = SND_DEVICE_VOIP_SPEAKER;
        } else if (sndDevice == SND_DEVICE_HEADSET) {
            ALOGI("Routing audio to VOIP headset\n");
            sndDevice = SND_DEVICE_VOIP_HEADSET;
        }
    }
#endif

    if ((CurrentComboDeviceData.DeviceId == INVALID_DEVICE) &&
        (sndDevice == SND_DEVICE_FM_TX_AND_SPEAKER )){
        /* speaker rx is already enabled change snd device to the fm tx
         * device and let the flow take the regular route to
         * updatedeviceinfo().
         */
        Mutex::Autolock lock_1(mComboDeviceLock);

        CurrentComboDeviceData.DeviceId = SND_DEVICE_FM_TX_AND_SPEAKER;
        sndDevice = DEVICE_FMRADIO_STEREO_RX;
    }
    else if(CurrentComboDeviceData.DeviceId != INVALID_DEVICE){
        /* time to disable the combo device */
        enableComboDevice(CurrentComboDeviceData.DeviceId,0);
        Mutex::Autolock lock_2(mComboDeviceLock);
        CurrentComboDeviceData.DeviceId = INVALID_DEVICE;
        CurrentComboDeviceData.StreamType = INVALID_STREAM;
    }

    if (sndDevice == SND_DEVICE_HEADSET)
    {
        char value[PROPERTY_VALUE_MAX];
        property_get("persist.service.audio.hs_intmic", value, "0");
        if (!strcmp(value, "1"))
            sndDevice = SND_DEVICE_NO_MIC_HEADSET;
    }

    if (sndDevice != -1 && sndDevice != mCurSndDevice) {
        ret = doAudioRouteOrMute(sndDevice);
        mCurSndDevice = sndDevice;
    }

    return ret;
}
status_t AudioHardware::enableComboDevice(uint32_t sndDevice, bool enableOrDisable)
{
    ALOGD("enableComboDevice %u",enableOrDisable);
    status_t status = NO_ERROR;
    Routing_table *PcmNode = getNodeByStreamType(PCM_PLAY);

    if(SND_DEVICE_FM_TX_AND_SPEAKER == sndDevice){

        if(getNodeByStreamType(VOICE_CALL) || getNodeByStreamType(FM_RADIO) ||
           getNodeByStreamType(FM_A2DP)){
            ALOGE("voicecall/FM radio active bailing out");
            return NO_ERROR;
        }

        if (!PcmNode) {
            ALOGE("No active playback session active bailing out ");
            return NO_ERROR;
        }

        Mutex::Autolock lock_1(mComboDeviceLock);

        Routing_table* temp = NULL;

        if (enableOrDisable == 1) {

            if(enableDevice(DEVICE_SPEAKER_RX, 1)) {
                ALOGE("enableDevice failed for device %d", DEVICE_SPEAKER_RX);
                return -1;
            }


            if(CurrentComboDeviceData.StreamType == INVALID_STREAM){
                if (PcmNode){
                    temp = PcmNode;
                    CurrentComboDeviceData.StreamType = PCM_PLAY;
                    ALOGD("PCM_PLAY session Active ");
                } else {
                    ALOGE("no PLAYback session Active ");
                    return -1;
                }
            }else
                temp = getNodeByStreamType(CurrentComboDeviceData.StreamType);

            if(temp == NULL){
                ALOGE("null check:fatal error:temp cannot be null");
                return -1;
            }

            ALOGD("combo:msm_route_stream(%d,%d,1)",temp->dec_id,
                DEV_ID(DEVICE_SPEAKER_RX));
            if(msm_route_stream(PCM_PLAY, temp->dec_id, DEV_ID(DEVICE_SPEAKER_RX),
                1)) {
                ALOGE("msm_route_stream failed");
                return -1;
            }

        }else if(enableOrDisable == 0) {
            temp = getNodeByStreamType(CurrentComboDeviceData.StreamType);


            if(temp == NULL){
                ALOGE("null check:fatal error:temp cannot be null");
                return -1;
            }

            ALOGD("combo:de-route msm_route_stream(%d,%d,0)",temp->dec_id,
                DEV_ID(DEVICE_SPEAKER_RX));
            if(msm_route_stream(PCM_PLAY, temp->dec_id,
                DEV_ID(DEVICE_SPEAKER_RX), 0)) {
                ALOGE("msm_route_stream failed");
                return -1;
            }

            if(enableDevice(DEVICE_SPEAKER_RX, 0)) {
                ALOGE("enableDevice failed for device %d", DEVICE_SPEAKER_RX);
                return -1;
            }
        }

    }

    return status;
}

status_t AudioHardware::enableFM()
{
    ALOGD("enableFM");
    status_t status = NO_INIT;
    unsigned short session_id = INVALID_DEVICE;
    status = ::open(FM_DEVICE, O_RDWR);
    if (status < 0) {
           ALOGE("Cannot open FM_DEVICE errno: %d", errno);
           goto Error;
    }
    mFmFd = status;
    if(ioctl(mFmFd, AUDIO_GET_SESSION_ID, &session_id)) {
           ALOGE("AUDIO_GET_SESSION_ID failed*********");
           goto Error;
    }

    if(enableDevice(DEVICE_FMRADIO_STEREO_TX, 1)) {
           ALOGE("enableDevice failed for device %d", DEVICE_FMRADIO_STEREO_TX);
           goto Error;
    }
    if(msm_route_stream(PCM_PLAY, session_id, DEV_ID(DEVICE_FMRADIO_STEREO_TX), 1)) {
           ALOGE("msm_route_stream failed");
           goto Error;
    }
    addToTable(session_id,cur_rx,INVALID_DEVICE,FM_RADIO,true);

#ifdef QCOM_FM_ENABLED
    setFmVolume(mFmVolume);
#endif

    enableDevice(cur_rx, 1);
    msm_route_stream(PCM_PLAY,session_id,DEV_ID(cur_rx),1);

    status = ioctl(mFmFd, AUDIO_START, 0);
    if (status < 0) {
            ALOGE("Cannot do AUDIO_START");
            goto Error;
    }
    return NO_ERROR;
    Error:
    if (mFmFd >= 0) {
        ::close(mFmFd);
        mFmFd = -1;
    }
    return NO_ERROR;
}

status_t AudioHardware::disableFM()
{
    ALOGD("disableFM");
    Routing_table* temp = NULL;
    temp = getNodeByStreamType(FM_RADIO);
    if(temp == NULL)
        return 0;
    if (mFmFd >= 0) {
            ::close(mFmFd);
            mFmFd = -1;
    }
    if(msm_route_stream(PCM_PLAY, temp->dec_id, DEV_ID(DEVICE_FMRADIO_STEREO_TX), 0)) {
           ALOGE("msm_route_stream failed");
           return 0;
    }
    if(!getNodeByStreamType(FM_A2DP)){
        if(enableDevice(DEVICE_FMRADIO_STEREO_TX, 0)) {
            ALOGE("Disabling device failed for device %d", DEVICE_FMRADIO_STEREO_TX);
        }
    }
    deleteFromTable(FM_RADIO);
#ifdef HTC_AUDIO
    updateDeviceInfo(cur_rx, cur_tx, 0, 0);
#else
    updateDeviceInfo(cur_rx, cur_tx);
#endif
    return NO_ERROR;
}


status_t AudioHardware::checkMicMute()
{
    Mutex::Autolock lock(mLock);
    if (mMode != AUDIO_MODE_IN_CALL) {
        setMicMute_nosync(true);
    }

    return NO_ERROR;
}

status_t AudioHardware::dumpInternals(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    result.append("AudioHardware::dumpInternals\n");
    snprintf(buffer, SIZE, "\tmInit: %s\n", mInit? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmMicMute: %s\n", mMicMute? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmBluetoothNrec: %s\n", mBluetoothNrec? "true": "false");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmBluetoothId: %d\n", mBluetoothId);
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

status_t AudioHardware::dump(int fd, const Vector<String16>& args)
{
    dumpInternals(fd, args);
    for (size_t index = 0; index < mInputs.size(); index++) {
        mInputs[index]->dump(fd, args);
    }

    if (mOutput) {
        mOutput->dump(fd, args);
    }
    return NO_ERROR;
}

uint32_t AudioHardware::getInputSampleRate(uint32_t sampleRate)
{
    uint32_t i;
    uint32_t prevDelta;
    uint32_t delta;

    for (i = 0, prevDelta = 0xFFFFFFFF; i < sizeof(inputSamplingRates)/sizeof(uint32_t); i++, prevDelta = delta) {
        delta = abs(sampleRate - inputSamplingRates[i]);
        if (delta > prevDelta) break;
    }
    // i is always > 0 here
    return inputSamplingRates[i-1];
}


AudioHardware::AudioStreamOutMSM72xx::AudioStreamOutMSM72xx() :
    mHardware(0), mFd(-1), mStartCount(0), mRetryCount(0), mStandby(true), mDevices(0)
{
}

status_t AudioHardware::AudioStreamOutMSM72xx::set(
        AudioHardware* hw, uint32_t devices, int *pFormat, uint32_t *pChannels, uint32_t *pRate)
{
    int lFormat = pFormat ? *pFormat : 0;
    uint32_t lChannels = pChannels ? *pChannels : 0;
    uint32_t lRate = pRate ? *pRate : 0;

    mHardware = hw;

    // fix up defaults
    if (lFormat == 0) lFormat = format();
    if (lChannels == 0) lChannels = channels();
    if (lRate == 0) lRate = sampleRate();

    // check values
    if ((lFormat != format()) ||
        (lChannels != channels()) ||
        (lRate != sampleRate())) {
        if (pFormat) *pFormat = format();
        if (pChannels) *pChannels = channels();
        if (pRate) *pRate = sampleRate();
        ALOGE("%s: Setting up correct values", __func__);
        return NO_ERROR;
    }

    if (pFormat) *pFormat = lFormat;
    if (pChannels) *pChannels = lChannels;
    if (pRate) *pRate = lRate;

    mDevices = devices;

    return NO_ERROR;
}

AudioHardware::AudioStreamOutMSM72xx::~AudioStreamOutMSM72xx()
{
    if (mFd >= 0) close(mFd);
}

ssize_t AudioHardware::AudioStreamOutMSM72xx::write(const void* buffer, size_t bytes)
{
    // ALOGD("AudioStreamOutMSM72xx::write(%p, %u)", buffer, bytes);
    status_t status = NO_INIT;
    size_t count = bytes;
    const uint8_t* p = static_cast<const uint8_t*>(buffer);
    unsigned short dec_id = INVALID_DEVICE;

    if (mStandby) {

        // open driver
        ALOGV("open driver");
        status = ::open("/dev/msm_pcm_out", O_WRONLY/*O_RDWR*/);
        if (status < 0) {
            ALOGE("Cannot open /dev/msm_pcm_out errno: %d", errno);
            goto Error;
        }
        mFd = status;

        // configuration
        ALOGV("get config");
        struct msm_audio_config config;
        status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
        if (status < 0) {
            ALOGE("Cannot read config");
            goto Error;
        }

        ALOGV("set config");
        config.channel_count = AudioSystem::popCount(channels());
        config.sample_rate = sampleRate();
        config.buffer_size = bufferSize();
        config.buffer_count = AUDIO_HW_NUM_OUT_BUF;
        config.type = CODEC_TYPE_PCM;

        status = ioctl(mFd, AUDIO_SET_CONFIG, &config);
        if (status < 0) {
            ALOGE("Cannot set config");
            goto Error;
        }

        ALOGV("buffer_size: %u", config.buffer_size);
        ALOGV("buffer_count: %u", config.buffer_count);
        ALOGV("channel_count: %u", config.channel_count);
        ALOGV("sample_rate: %u", config.sample_rate);

        // fill 2 buffers before AUDIO_START
        mStartCount = AUDIO_HW_NUM_OUT_BUF;
        mStandby = false;

#ifdef HTC_AUDIO
        if (support_tpa2051)
            do_tpa2051_control(0);
#endif
    }

    while (count) {
        ssize_t written = ::write(mFd, p, count);
        if (written >= 0) {
            count -= written;
            p += written;
        } else {
            if (errno != EAGAIN) return written;
            mRetryCount++;
            ALOGW("EAGAIN - retry");
        }
    }

    // start audio after we fill 2 buffers
    if (mStartCount) {
        if (--mStartCount == 0) {
            if(ioctl(mFd, AUDIO_GET_SESSION_ID, &dec_id)) {
                ALOGE("AUDIO_GET_SESSION_ID failed*********");
                return 0;
            }
            ALOGV("dec_id = %d\n",dec_id);
            if(cur_rx == INVALID_DEVICE)
                return 0;

            Mutex::Autolock lock(mDeviceSwitchLock);

#ifdef HTC_AUDIO
            int snd_dev = mHardware->get_snd_dev();
            if (support_aic3254)
                mHardware->do_aic3254_control(snd_dev);
#endif

            ALOGV("cur_rx for pcm playback = %d",cur_rx);
            if(enableDevice(cur_rx, 1)) {
                ALOGE("enableDevice failed for device cur_rx %d", cur_rx);
                return 0;
            }

#ifdef HTC_AUDIO
            uint32_t rx_acdb_id = mHardware->getACDB(MOD_PLAY, snd_dev);
            updateACDB(cur_rx, cur_tx, rx_acdb_id, 0);
#endif

            ALOGV("msm_route_stream(PCM_PLAY,%d,%d,1)",dec_id,DEV_ID(cur_rx));
            if(msm_route_stream(PCM_PLAY, dec_id, DEV_ID(cur_rx), 1)) {
                ALOGE("msm_route_stream failed");
                return 0;
            }
            Mutex::Autolock lock_1(mComboDeviceLock);

            if(CurrentComboDeviceData.DeviceId == SND_DEVICE_FM_TX_AND_SPEAKER){

                ALOGD("Routing PCM stream to speaker for combo device");
                ALOGD("combo:msm_route_stream(PCM_PLAY,session id:%d,dev id:%d,1)",dec_id,
                    DEV_ID(DEVICE_SPEAKER_RX));

                if(msm_route_stream(PCM_PLAY, dec_id, DEV_ID(DEVICE_SPEAKER_RX),
                    1)) {
                    ALOGE("msm_route_stream failed");
                    return -1;
                }
                CurrentComboDeviceData.StreamType = PCM_PLAY;
            }
            addToTable(dec_id,cur_rx,INVALID_DEVICE,PCM_PLAY,true);
            ioctl(mFd, AUDIO_START, 0);
        }
    }
    return bytes;

Error:
    if (mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
    // Simulate audio output timing in case of error
    usleep(bytes * 1000000 / frameSize() / sampleRate());

    return status;
}

status_t AudioHardware::AudioStreamOutMSM72xx::standby()
{
    Routing_table* temp = NULL;
    ALOGV("AudioStreamOutMSM72xx::standby()");
    status_t status = NO_ERROR;

    temp = getNodeByStreamType(PCM_PLAY);

    if(temp == NULL)
        return NO_ERROR;

    ALOGV("Deroute pcm out stream");
    if(msm_route_stream(PCM_PLAY, temp->dec_id,DEV_ID(temp->dev_id), 0)) {
        ALOGE("could not set stream routing\n");
        deleteFromTable(PCM_PLAY);
        return -1;
    }
    deleteFromTable(PCM_PLAY);
#ifdef HTC_AUDIO
    updateDeviceInfo(cur_rx, cur_tx, 0, 0);
#else
    updateDeviceInfo(cur_rx, cur_tx);
#endif

    if (!mStandby && mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }

    mStandby = true;

#ifdef HTC_AUDIO
    if (support_aic3254)
        mHardware->do_aic3254_control(mHardware->get_snd_dev());
#endif

    return status;
}

status_t AudioHardware::AudioStreamOutMSM72xx::dump(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    result.append("AudioStreamOutMSM72xx::dump\n");
    snprintf(buffer, SIZE, "\tsample rate: %d\n", sampleRate());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tbuffer size: %d\n", bufferSize());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tchannels: %d\n", channels());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tformat: %d\n", format());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmHardware: %p\n", mHardware);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmFd: %d\n", mFd);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmStartCount: %d\n", mStartCount);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmRetryCount: %d\n", mRetryCount);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmStandby: %s\n", mStandby? "true": "false");
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

bool AudioHardware::AudioStreamOutMSM72xx::checkStandby()
{
    return mStandby;
}


status_t AudioHardware::AudioStreamOutMSM72xx::setParameters(const String8& keyValuePairs)
{
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 key;
    status_t status = NO_ERROR;
    int device;
    ALOGV("AudioStreamOutMSM72xx::setParameters() %s", keyValuePairs.string());

#ifdef QCOM_FM_ENABLED
    float fm_volume;
    key = String8(AUDIO_PARAMETER_KEY_FM_VOLUME);
    if (param.getFloat(key, fm_volume) == NO_ERROR) {
        mHardware->setFmVolume(fm_volume);
        param.remove(key);
    }

    key = String8(AUDIO_PARAMETER_KEY_HANDLE_FM);
    if (param.getInt(key, device) == NO_ERROR) {
        if (device & AUDIO_DEVICE_OUT_FM) {
            mDevices |= device;
            mHardware->enableFM();
        } else {
            mHardware->disableFM();
            mDevices &= device;
        }
        param.remove(key);
    }
#endif

    key = String8(AudioParameter::keyRouting);
    if (param.getInt(key, device) == NO_ERROR) {
        mDevices = device;
        ALOGV("set output routing %x", mDevices);
        status = mHardware->doRouting(NULL, device);
        param.remove(key);
    }

    if (param.size()) {
        status = BAD_VALUE;
    }
    return status;
}

String8 AudioHardware::AudioStreamOutMSM72xx::getParameters(const String8& keys)
{
    AudioParameter param = AudioParameter(keys);
    String8 value;
    String8 key = String8(AudioParameter::keyRouting);

    if (param.get(key, value) == NO_ERROR) {
        ALOGV("get routing %x", mDevices);
        param.addInt(key, (int)mDevices);
    }

    ALOGV("AudioStreamOutMSM72xx::getParameters() %s", param.toString().string());
    return param.toString();
}

status_t AudioHardware::AudioStreamOutMSM72xx::getRenderPosition(uint32_t *dspFrames)
{
    //TODO: enable when supported by driver
    return INVALID_OPERATION;
}

status_t AudioHardware::AudioStreamOutMSM72xx::getPresentationPosition(uint64_t *frames, struct timespec *timestamp)
{
    //TODO: enable when supported by driver
    return INVALID_OPERATION;
}

// ----------------------------------------------------------------------------

AudioHardware::AudioStreamInMSM72xx::AudioStreamInMSM72xx() :
    mHardware(0), mFd(-1), mState(AUDIO_INPUT_CLOSED), mRetryCount(0),
    mFormat(AUDIO_HW_IN_FORMAT), mChannels(AUDIO_HW_IN_CHANNELS),
    mSampleRate(AUDIO_HW_IN_SAMPLERATE), mBufferSize(AUDIO_HW_IN_BUFFERSIZE),
    mAcoustics((AudioSystem::audio_in_acoustics)0), mDevices(0)
{
}

status_t AudioHardware::AudioStreamInMSM72xx::set(
        AudioHardware* hw, uint32_t devices, int *pFormat, uint32_t *pChannels, uint32_t *pRate,
        AudioSystem::audio_in_acoustics acoustic_flags)
{
    if ((pFormat == 0) ||
        ((*pFormat != AUDIO_HW_IN_FORMAT) &&
         (*pFormat != AUDIO_FORMAT_AAC)))
    {
        *pFormat = AUDIO_HW_IN_FORMAT;
        return BAD_VALUE;
    }

    if((*pFormat == AUDIO_FORMAT_AAC) && (*pChannels & (AUDIO_CHANNEL_IN_VOICE_DNLINK |  AUDIO_CHANNEL_IN_VOICE_UPLINK))) {
        ALOGE("voice call recording in AAC format does not support");
        return BAD_VALUE;
    }

    if (pRate == 0) {
        return BAD_VALUE;
    }
    uint32_t rate = hw->getInputSampleRate(*pRate);
    if (rate != *pRate) {
        *pRate = rate;
        return BAD_VALUE;
    }

    if (pChannels == 0 || (*pChannels & (AUDIO_CHANNEL_IN_MONO | AUDIO_CHANNEL_IN_STEREO)) == 0) {
        *pChannels = AUDIO_HW_IN_CHANNELS;
        return BAD_VALUE;
    }

    mHardware = hw;

    ALOGV("AudioStreamInMSM72xx::set(%d, %d, %u)", *pFormat, *pChannels, *pRate);
    if (mFd >= 0) {
        ALOGE("Audio record already open");
        return -EPERM;
    }
    status_t status =0;
    struct msm_voicerec_mode voc_rec_cfg;
#ifdef QCOM_FM_ENABLED
    if(devices == AUDIO_DEVICE_IN_FM_RX_A2DP) {
        status = ::open("/dev/msm_a2dp_in", O_RDONLY);
        if (status < 0) {
            ALOGE("Cannot open /dev/msm_a2dp_in errno: %d", errno);
            goto Error;
        }
        mFd = status;
        // configuration
        ALOGV("get config");
        struct msm_audio_config config;
        status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
        if (status < 0) {
            ALOGE("Cannot read config");
            goto Error;
        }

        ALOGV("set config");
        config.channel_count = AudioSystem::popCount((*pChannels) & (AUDIO_CHANNEL_IN_STEREO | AUDIO_CHANNEL_IN_MONO));
        config.sample_rate = *pRate;
        config.buffer_size = bufferSize();
        config.buffer_count = 2;
        config.type = CODEC_TYPE_PCM;
        status = ioctl(mFd, AUDIO_SET_CONFIG, &config);
        if (status < 0) {
            ALOGE("Cannot set config");
            if (ioctl(mFd, AUDIO_GET_CONFIG, &config) == 0) {
                if (config.channel_count == 1) {
                    *pChannels = AUDIO_CHANNEL_IN_MONO;
                } else {
                    *pChannels = AUDIO_CHANNEL_IN_STEREO;
                }
                *pRate = config.sample_rate;
            }
            goto Error;
        }

        ALOGV("confirm config");
        status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
        if (status < 0) {
            ALOGE("Cannot read config");
            goto Error;
        }
        ALOGV("buffer_size: %u", config.buffer_size);
        ALOGV("buffer_count: %u", config.buffer_count);
        ALOGV("channel_count: %u", config.channel_count);
        ALOGV("sample_rate: %u", config.sample_rate);

        mDevices = devices;
        mFormat = AUDIO_HW_IN_FORMAT;
        mChannels = *pChannels;
        mSampleRate = config.sample_rate;
        mBufferSize = config.buffer_size;
    }
    else
#endif
    if(*pFormat == AUDIO_HW_IN_FORMAT)
    {
        // open audio input device
        status = ::open("/dev/msm_pcm_in", O_RDONLY);
        if (status < 0) {
            ALOGE("Cannot open /dev/msm_pcm_in errno: %d", errno);
            goto Error;
        }
        mFd = status;

        // configuration
        ALOGV("get config");
        struct msm_audio_config config;
        status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
        if (status < 0) {
            ALOGE("Cannot read config");
            goto Error;
        }

        ALOGV("set config");
        config.channel_count = AudioSystem::popCount((*pChannels) & (AUDIO_CHANNEL_IN_STEREO | AUDIO_CHANNEL_IN_MONO));
        config.sample_rate = *pRate;
        config.buffer_size = bufferSize();
        config.buffer_count = 2;
        config.type = CODEC_TYPE_PCM;
        if (build_id[17] == '1') {//build 4.1
           /*
             Configure pcm record buffer size based on the sampling rate:
             If sampling rate >= 44.1 Khz, use 512 samples/channel pcm recording and
             If sampling rate < 44.1 Khz, use 256 samples/channel pcm recording
           */
            if(*pRate>=44100)
                config.buffer_size = 1024 * config.channel_count;
            else
                config.buffer_size = 512 * config.channel_count;
        }
        status = ioctl(mFd, AUDIO_SET_CONFIG, &config);
        if (status < 0) {
            ALOGE("Cannot set config");
            if (ioctl(mFd, AUDIO_GET_CONFIG, &config) == 0) {
                if (config.channel_count == 1) {
                    *pChannels = AUDIO_CHANNEL_IN_MONO;
                } else {
                    *pChannels = AUDIO_CHANNEL_IN_STEREO;
                }
                *pRate = config.sample_rate;
            }
            goto Error;
        }

        ALOGV("confirm config");
        status = ioctl(mFd, AUDIO_GET_CONFIG, &config);
        if (status < 0) {
            ALOGE("Cannot read config");
            goto Error;
        }
        ALOGV("buffer_size: %u", config.buffer_size);
        ALOGV("buffer_count: %u", config.buffer_count);
        ALOGV("channel_count: %u", config.channel_count);
        ALOGV("sample_rate: %u", config.sample_rate);

        mDevices = devices;
        mFormat = AUDIO_HW_IN_FORMAT;
        mChannels = *pChannels;
        if (mDevices == AUDIO_DEVICE_IN_VOICE_CALL)
         {
            if ((mChannels & AUDIO_CHANNEL_IN_VOICE_DNLINK) &&
                (mChannels & AUDIO_CHANNEL_IN_VOICE_UPLINK)) {
                 ALOGI("Recording Source: Voice Call Both Uplink and Downlink");
                 voc_rec_cfg.rec_mode = VOC_REC_BOTH;
            } else if (mChannels & AUDIO_CHANNEL_IN_VOICE_DNLINK) {
                 ALOGI("Recording Source: Voice Call DownLink");
                 voc_rec_cfg.rec_mode = VOC_REC_DOWNLINK;
            } else if (mChannels & AUDIO_CHANNEL_IN_VOICE_UPLINK) {
                 ALOGI("Recording Source: Voice Call UpLink");
                 voc_rec_cfg.rec_mode = VOC_REC_UPLINK;
            }
            if (ioctl(mFd, AUDIO_SET_INCALL, &voc_rec_cfg))
            {
                ALOGE("Error: AUDIO_SET_INCALL failed\n");
                goto  Error;
            }
        }
        mSampleRate = config.sample_rate;
        mBufferSize = config.buffer_size;
    }
    else if (*pFormat == AUDIO_FORMAT_AAC)
    {
          ALOGI("Recording format: AAC");
          // open aac input device
          status = ::open(AAC_DEVICE_IN, O_RDWR);
          if (status < 0) {
              ALOGE("Cannot open aac device for read");
              goto Error;
          }
          mFd = status;

          struct msm_audio_stream_config config;
          if(ioctl(mFd, AUDIO_GET_STREAM_CONFIG, &config))
          {
            ALOGE(" Error getting buf config param AUDIO_GET_STREAM_CONFIG \n");
            goto  Error;
          }

          ALOGE("The Config buffer size is %d", config.buffer_size);
          ALOGE("The Config buffer count is %d", config.buffer_count);


          struct msm_audio_aac_enc_config aac_enc_cfg;
          if (ioctl(mFd, AUDIO_GET_AAC_ENC_CONFIG, &aac_enc_cfg))
          {
            ALOGE("Error: AUDIO_GET_AAC_ENC_CONFIG failed\n");
            goto  Error;
          }

          ALOGV("The Config channels is %d", aac_enc_cfg.channels);
          ALOGV("The Config sample_rate is %d", aac_enc_cfg.sample_rate);
          ALOGV("The Config bit_rate is %d", aac_enc_cfg.bit_rate);
          ALOGV("The Config stream_format is %d", aac_enc_cfg.stream_format);

          mDevices = devices;
          mChannels = *pChannels;
          aac_enc_cfg.sample_rate = mSampleRate = *pRate;
          mFormat = *pFormat;
          mBufferSize = 2048;
          if (*pChannels & (AUDIO_CHANNEL_IN_MONO))
              aac_enc_cfg.channels =  1;
          else if (*pChannels & (AUDIO_CHANNEL_IN_STEREO))
              aac_enc_cfg.channels =  2;
          aac_enc_cfg.bit_rate = 128000;

          ALOGV("Setting the Config channels is %d", aac_enc_cfg.channels);
          ALOGV("Setting the Config sample_rate is %d", aac_enc_cfg.sample_rate);
          ALOGV("Setting the Config bit_rate is %d", aac_enc_cfg.bit_rate);
          ALOGV("Setting the Config stream_format is %d", aac_enc_cfg.stream_format);

          if (ioctl(mFd, AUDIO_SET_AAC_ENC_CONFIG, &aac_enc_cfg))
          {
            ALOGE("Error: AUDIO_SET_AAC_ENC_CONFIG failed\n");
            goto  Error;
          }
    }
    //mHardware->setMicMute_nosync(false);
    mState = AUDIO_INPUT_OPENED;
#ifdef HTC_AUDIO
    mHardware->set_mRecordState(true);
#endif

    if (!acoustic)
        return NO_ERROR;


    int (*msm72xx_set_audpre_params)(int, int);
    msm72xx_set_audpre_params = (int (*)(int, int))::dlsym(acoustic, "msm72xx_set_audpre_params");
    if ((*msm72xx_set_audpre_params) == 0) {
        ALOGI("msm72xx_set_audpre_params not present");
        return NO_ERROR;
    }

    int (*msm72xx_enable_audpre)(int, int, int);
    msm72xx_enable_audpre = (int (*)(int, int, int))::dlsym(acoustic, "msm72xx_enable_audpre");
    if ((*msm72xx_enable_audpre) == 0) {
        ALOGI("msm72xx_enable_audpre not present");
        return NO_ERROR;
    }

    audpre_index = calculate_audpre_table_index(mSampleRate);
    tx_iir_index = (audpre_index * 2) + (hw->checkOutputStandby() ? 0 : 1);
    ALOGD("audpre_index = %d, tx_iir_index = %d\n", audpre_index, tx_iir_index);

    /**
     * If audio-preprocessing failed, we should not block record.
     */
    status = msm72xx_set_audpre_params(audpre_index, tx_iir_index);
    if (status < 0)
        ALOGE("Cannot set audpre parameters");

    mAcoustics = acoustic_flags;
    status = msm72xx_enable_audpre((int)acoustic_flags, audpre_index, tx_iir_index);
    if (status < 0)
        ALOGE("Cannot enable audpre");

    return NO_ERROR;

Error:
    if (mFd >= 0) {
        ::close(mFd);
        mFd = -1;
    }
    return status;
}

AudioHardware::AudioStreamInMSM72xx::~AudioStreamInMSM72xx()
{
    ALOGV("AudioStreamInMSM72xx destructor");
    standby();
}

ssize_t AudioHardware::AudioStreamInMSM72xx::read( void* buffer, ssize_t bytes)
{
    unsigned short dec_id = INVALID_DEVICE;
    ALOGV("AudioStreamInMSM72xx::read(%p, %ld)", buffer, bytes);
    if (!mHardware) return -1;

    size_t count = bytes;
    size_t  aac_framesize= bytes;
    uint8_t* p = static_cast<uint8_t*>(buffer);
    uint32_t* recogPtr = (uint32_t *)p;
    uint16_t* frameCountPtr;
    uint16_t* frameSizePtr;

    if (mState < AUDIO_INPUT_OPENED) {
        AudioHardware *hw = mHardware;
        hw->mLock.lock();
        status_t status = set(hw, mDevices, &mFormat, &mChannels, &mSampleRate, mAcoustics);
        if (status != NO_ERROR) {
            hw->mLock.unlock();
            return -1;
        }
#ifdef QCOM_FM_ENABLED
        if((mDevices == AUDIO_DEVICE_IN_FM_RX) || (mDevices == AUDIO_DEVICE_IN_FM_RX_A2DP) ){
            if(ioctl(mFd, AUDIO_GET_SESSION_ID, &dec_id)) {
                ALOGE("AUDIO_GET_SESSION_ID failed*********");
                hw->mLock.unlock();
                return -1;
            }

            if(enableDevice(DEVICE_FMRADIO_STEREO_TX, 1)) {
                ALOGE("enableDevice failed for device %d",DEVICE_FMRADIO_STEREO_TX);
                hw->mLock.unlock();
                return -1;
             }

            if(msm_route_stream(PCM_REC, dec_id, DEV_ID(DEVICE_FMRADIO_STEREO_TX), 1)) {
                ALOGE("msm_route_stream failed");
                hw->mLock.unlock();
                return -1;
            }
            mFirstread = false;
            if (mDevices == AUDIO_DEVICE_IN_FM_RX_A2DP) {
                addToTable(dec_id,cur_tx,INVALID_DEVICE,FM_A2DP,true);
                mFmRec = FM_A2DP_REC;
            }
            else {
                addToTable(dec_id,cur_tx,INVALID_DEVICE,FM_REC,true);
                mFmRec = FM_FILE_REC;
            }
            hw->mLock.unlock();
        }
        else
#endif
{
            hw->mLock.unlock();
            if(ioctl(mFd, AUDIO_GET_SESSION_ID, &dec_id)) {
                ALOGE("AUDIO_GET_SESSION_ID failed*********");
                return -1;
            }
            ALOGV("dec_id = %d,cur_tx= %d",dec_id,cur_tx);
            if(cur_tx == INVALID_DEVICE)
                cur_tx = DEVICE_HANDSET_TX;

            Mutex::Autolock lock(mDeviceSwitchLock);

            if(enableDevice(cur_tx, 1)) {
                ALOGE("enableDevice failed, device %d",cur_tx);
                return -1;
            }
            if(msm_route_stream(PCM_REC, dec_id, DEV_ID(cur_tx), 1)) {
                ALOGE("msm_route_stream failed");
                return -1;
            }
            addToTable(dec_id,cur_tx,INVALID_DEVICE,PCM_REC,true);
            mFirstread = false;
        }
    }


    if (mState < AUDIO_INPUT_STARTED) {
        // force routing to input device
        mHardware->clearCurDevice();
        mHardware->doRouting(this, 0);
#ifdef HTC_AUDIO
        if (support_aic3254) {
            int snd_dev = mHardware->get_snd_dev();
            mHardware->aic3254_config(snd_dev);
            mHardware->do_aic3254_control(snd_dev);
        }
#endif
        if (ioctl(mFd, AUDIO_START, 0)) {
            ALOGE("Error starting record");
            standby();
            return -1;
        }
        mState = AUDIO_INPUT_STARTED;
    }

    bytes = 0;
    if(mFormat == AUDIO_HW_IN_FORMAT)
    {
        while (count) {
            ssize_t bytesRead = ::read(mFd, buffer, count);
            if (bytesRead >= 0) {
                count -= bytesRead;
                p += bytesRead;
                bytes += bytesRead;
                if(!mFirstread)
                {
                   mFirstread = true;
                   break;
                }
            } else {
                if (errno != EAGAIN) return bytesRead;
                mRetryCount++;
                ALOGW("EAGAIN - retrying");
            }
        }
    }
    else if (mFormat == AUDIO_FORMAT_AAC)
    {
        *((uint32_t*)recogPtr) = 0x51434F4D ;// ('Q','C','O', 'M') Number to identify format as AAC by higher layers
        recogPtr++;
        frameCountPtr = (uint16_t*)recogPtr;
        *frameCountPtr = 0;
        p += 3*sizeof(uint16_t);
        count -= 3*sizeof(uint16_t);

        while (count > 0) {
            frameSizePtr = (uint16_t *)p;
            p += sizeof(uint16_t);
            if(!(count > 2)) break;
            count -= sizeof(uint16_t);

            ssize_t bytesRead = ::read(mFd, p, count);
            if (bytesRead > 0) {
                ALOGV("Number of Bytes read = %d", bytesRead);
                count -= bytesRead;
                p += bytesRead;
                bytes += bytesRead;
                ALOGV("Total Number of Bytes read = %d", bytes);

                *frameSizePtr =  bytesRead;
                (*frameCountPtr)++;
                if(!mFirstread)
                {
                   mFirstread = true;
                   break;
                }
                /*Typical frame size for AAC is around 250 bytes. So we have
                 * taken the minimum buffer size as twice of this size i.e.
                 * 512 to avoid short reads from driver */
                if(count < 512)
                {
                   ALOGI("buffer passed to driver %d, is less than the min 512 bytes", count);
                   break;
                }
            }
            else if(bytesRead == 0)
            {
             ALOGI("Bytes Read = %d ,Buffer no longer sufficient",bytesRead);
             break;
            } else {
                if (errno != EAGAIN) return bytesRead;
                mRetryCount++;
                ALOGW("EAGAIN - retrying");
            }
        }
    }

    if (mFormat == AUDIO_FORMAT_AAC)
         return aac_framesize;

        return bytes;
}

status_t AudioHardware::AudioStreamInMSM72xx::standby()
{
    bool isDriverClosed = false;
    ALOGD("AudioStreamInMSM72xx::standby()");
    Routing_table* temp = NULL;
    if (!mHardware) return -1;

#ifdef HTC_AUDIO
    mHardware->set_mRecordState(false);
    if (support_aic3254) {
        int snd_dev = mHardware->get_snd_dev();
        mHardware->aic3254_config(snd_dev);
        mHardware->do_aic3254_control(snd_dev);
    }
#endif

    if (mState > AUDIO_INPUT_CLOSED) {
        if (mFd >= 0) {
            ::close(mFd);
            mFd = -1;
            ALOGV("driver closed");
            isDriverClosed = true;
        }
        //mHardware->checkMicMute();
        mState = AUDIO_INPUT_CLOSED;
    }
    if (mFmRec == FM_A2DP_REC) {
        //A2DP Recording
        temp = getNodeByStreamType(FM_A2DP);
        if(temp == NULL)
            return NO_ERROR;
        if(msm_route_stream(PCM_PLAY, temp->dec_id, DEV_ID(DEVICE_FMRADIO_STEREO_TX), 0)) {
           ALOGE("msm_route_stream failed");
           return 0;
        }
        deleteFromTable(FM_A2DP);
        if(enableDevice(DEVICE_FMRADIO_STEREO_TX, 0)) {
            ALOGE("Disabling device failed for device %d", DEVICE_FMRADIO_STEREO_TX);
        }
    }
    if (mFmRec == FM_FILE_REC) {
        //FM Recording
        temp = getNodeByStreamType(FM_REC);
        if(temp == NULL)
            return NO_ERROR;
        if(msm_route_stream(PCM_PLAY, temp->dec_id, DEV_ID(DEVICE_FMRADIO_STEREO_TX), 0)) {
           ALOGE("msm_route_stream failed");
           return 0;
        }
        deleteFromTable(FM_REC);
    }
    temp = getNodeByStreamType(PCM_REC);
    if(temp == NULL)
        return NO_ERROR;

    if(isDriverClosed){
        ALOGD("Deroute pcm in stream");
        if(msm_route_stream(PCM_REC, temp->dec_id,DEV_ID(temp->dev_id), 0)) {
            ALOGE("could not set stream routing\n");
            deleteFromTable(PCM_REC);
            return -1;
        }
        ALOGV("Disable device");
        deleteFromTable(PCM_REC);
#ifdef HTC_AUDIO
        updateDeviceInfo(cur_rx, cur_tx, 0, 0);
#else
        updateDeviceInfo(cur_rx, cur_tx);
#endif
    }//mRecordingSession condition.
    // restore output routing if necessary
    mHardware->clearCurDevice();
    mHardware->doRouting(this, 0);
    return NO_ERROR;
}

status_t AudioHardware::AudioStreamInMSM72xx::dump(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    result.append("AudioStreamInMSM72xx::dump\n");
    snprintf(buffer, SIZE, "\tsample rate: %d\n", sampleRate());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tbuffer size: %d\n", bufferSize());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tchannels: %d\n", channels());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tformat: %d\n", format());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmHardware: %p\n", mHardware);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmFd count: %d\n", mFd);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmState: %d\n", mState);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmRetryCount: %d\n", mRetryCount);
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

status_t AudioHardware::AudioStreamInMSM72xx::setParameters(const String8& keyValuePairs)
{
    AudioParameter param = AudioParameter(keyValuePairs);
    String8 key = String8(AudioParameter::keyRouting);
    status_t status = NO_ERROR;
    int device;
    ALOGV("AudioStreamInMSM72xx::setParameters() %s", keyValuePairs.string());

    if (param.getInt(key, device) == NO_ERROR) {
        ALOGV("set input routing %x", device);
        if (device & (device - 1)) {
            status = BAD_VALUE;
        } else {
            mDevices = device;
            status = mHardware->doRouting(this, device);
        }
        param.remove(key);
    }

    if (param.size()) {
        status = BAD_VALUE;
    }
    return status;
}

String8 AudioHardware::AudioStreamInMSM72xx::getParameters(const String8& keys)
{
    AudioParameter param = AudioParameter(keys);
    String8 value;
    String8 key = String8(AudioParameter::keyRouting);

    if (param.get(key, value) == NO_ERROR) {
        ALOGV("get routing %x", mDevices);
        param.addInt(key, (int)mDevices);
    }

    ALOGV("AudioStreamInMSM72xx::getParameters() %s", param.toString().string());
    return param.toString();
}

// getActiveInput_l() must be called with mLock held
AudioHardware::AudioStreamInMSM72xx *AudioHardware::getActiveInput_l()
{
    for (size_t i = 0; i < mInputs.size(); i++) {
        // return first input found not being in standby mode
        // as only one input can be in this state
        if (mInputs[i]->state() > AudioStreamInMSM72xx::AUDIO_INPUT_CLOSED) {
            return mInputs[i];
        }
    }

    return NULL;
}

// ----------------------------------------------------------------------------


extern "C" AudioHardwareInterface* createAudioHardware(void) {
    return new AudioHardware();
}

}; // namespace android
