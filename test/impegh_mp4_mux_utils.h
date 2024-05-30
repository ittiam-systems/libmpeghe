/* 	Copyright (c) [2022] Ittiam Systems Pvt. Ltd.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted (subject to the limitations in the
   disclaimer below) provided that the following conditions are met:
   •	Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   •	Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
   •	Neither the names of Dolby Laboratories, Inc. (or its affiliates),
   Ittiam Systems Pvt. Ltd. nor the names of its contributors may be used
   to endorse or promote products derived from this software without
   specific prior written permission.

   NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED
   BY THIS LICENSE. YOUR USE OF THE SOFTWARE MAY REQUIRE ADDITIONAL PATENT
   LICENSE(S) BY THIRD PARTIES, INCLUDING, WITHOUT LIMITATION, DOLBY
   LABORATORIES, INC. OR ANY OF ITS AFFILIATES. THIS SOFTWARE IS PROVIDED
   BY ITTIAM SYSTEMS LTD. AND ITS CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
   IN NO EVENT SHALL ITTIAM SYSTEMS LTD OR ITS CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------
*/

#ifndef IMPEGH_MP4_MUX_UTILS_H
#define IMPEGH_MP4_MUX_UTILS_H
#include <setjmp.h>
#include "impeghe_type_def.h"
#include "impeghe_bitbuffer.h"
#include "impeghe_mhas_write.h"
#include "impegh_error_codes_mux.h"
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX_SAMPLE_RATE (96000)
#define MAX_MAE_BUFFER_SIZE (10000)
#define MAX_MAE_NUM_DATASETS (15)

#define ID_MAE_GROUP_DESCRIPTION (0)
#define ID_MAE_SWITCHGROUP_DESCRIPTION (1)
#define ID_MAE_GROUP_CONTENT (2)
#define ID_MAE_GROUP_COMPOSITE (3)
#define ID_MAE_SCREEN_SIZE (4)
#define ID_MAE_GROUP_PRESET_DESCRIPTION (5)
#define ID_MAE_DRC_UI_INFO (6)
#define ID_MAE_SCREEN_SIZE_EXTENSION (7)
#define ID_MAE_GROUP_PRESET_EXTENSION (8)
#define ID_MAE_LOUDNESS_COMPENSATION (9)

// Values obtained from Table 11 of section 4.8.2.2 of specification - 23008-3
// Restrictions applicable for LC profile
#define MAX_NUM_GROUPS (28)
#define MAX_NUM_SWITCH_GROUPS (14)
#define MAX_NUM_GROUPS_PRESETS (16)
#define MAX_GROUP_PRESET_NUM_CONDITIONS (16)
#define MAX_NUM_PRESET_GROUP_EXTENSIONS (16)
#define MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT (32)

// Group Definition
#define MAX_GROUP_NUM_MEMBERS (128)
#define MAX_SWITCH_GROUP_NUM_MEMBERS (32)

#define MAX_NUM_CONTENT_DATA_BLOCKS (128)

#define MAX_NUM_DESCRIPTOIN_BLOCKS (128)
#define MAX_NUM_CONTENT_BLOCKS (128)
#define MAX_DESCR_LANGUAGE_DATA_LEN (16)
#define MAX_DESCRIPTON_DATA_LEN (256)
#define MAX_NUM_COMPOSITE_PAIRS (128)
#define MAX_NUM_TGT_LOUDNESS_CONDITIONS (7)
#define MAX_NUM_PRESET_PROD_SCREENS (31)
#define MAX_MAE_CONFIG_EXTENSIONS (16)
#define MAX_MAE_NUM_DATA_SETS (10)
#define MAX_NUM_SIGNAL_GROUPS (32)
#define MAX_NUM_OAM_OBJS (24)
#define MAX_METADATA_ELEMENT_ID_COUNT (128)
#define MAX_GAIN_SET_COUNT (64)
#define MAX_BAND_COUNT (16)
#define MAX_ADDITIONAL_DMIX_COUNT (12)
#define MAX_BASE_CHANNEL_COUNT (128)
#define MAX_DOWNMIX_ID_COUNT (32)
#define MAX_DOWNMIX_MATRIX_COUNT (16)
#define MAX_DOWNMIX_MATRIX_LEN (512)
#define MAX_LOUDNESS_INFO_COUNT (64)
#define MAX_MEASUREMENT_COUNT (16)

#ifdef NO_PROFILE_LIMIT
#define MAX_DRC_COEFFICIENTS_UNI_DRC_COUNT (8)
#define MAX_DRC_INSTRUCTIONS_UNI_DRC_COUNT (64)
#define MAX_LOUD_SPEAKER_LAYOUT_INDX (100)
#define MAX_USAC_CONFIG_EXT_TYPES (256)
#define MAX_NUM_DESCR_LANGUAGES (16)
#else
#define MAX_DRC_COEFFICIENTS_UNI_DRC_COUNT (4)
#define MAX_DRC_INSTRUCTIONS_UNI_DRC_COUNT (32)
#define MAX_LOUD_SPEAKER_LAYOUT_INDX (20)
#define MAX_USAC_CONFIG_EXT_TYPES (7)
#define MAX_NUM_DESCR_LANGUAGES (8)
#endif

// error codes
#define IMPEGHD_MHAS_SYNCWORD_MISMATCH (-1)

#define TP_MPEGH_MAX_SIGNAL_GROUPS (2 * 28)
#define TP_MAX_CHANNELS_PER_SIGNAL_GROUP (28)
#define TP_USAC_MAX_EXT_ELEMENTS ((2 * (2 * 28) + ((2 * 28)) / 2 + 4 * 1))

#define TP_USAC_MAX_ELEMENTS ((2 * 28) + TP_USAC_MAX_EXT_ELEMENTS)


#define USAC_ID_BIT 16 /** USAC element IDs start at USAC_ID_BIT */
typedef enum {
  /* mp4 element IDs */
  ID_NONE = -1, /**< Invalid Element helper ID.             */
  ID_SCE = 0,   /**< Single Channel Element.                */
  ID_CPE = 1,   /**< Channel Pair Element.                  */
  ID_CCE = 2,   /**< Coupling Channel Element.              */
  ID_LFE = 3,   /**< LFE Channel Element.                   */
  ID_DSE = 4,   /**< Currently one Data Stream Element for ancillary data is supported. */
  ID_PCE = 5,   /**< Program Config Element.                */
  ID_FIL = 6,   /**< Fill Element.                          */
  ID_END = 7,   /**< Arnie (End Element = Terminator).      */
  ID_EXT = 8,   /**< Extension Payload (ER only).           */
  ID_SCAL = 9,  /**< AAC scalable element (ER only).        */
  /* USAC element IDs */
  ID_USAC_SCE = 0 + USAC_ID_BIT, /**< Single Channel Element.                */
  ID_USAC_CPE = 1 + USAC_ID_BIT, /**< Channel Pair Element.                  */
  ID_USAC_LFE = 2 + USAC_ID_BIT, /**< LFE Channel Element.                   */
  ID_USAC_EXT = 3 + USAC_ID_BIT, /**< Extension Element.                     */
  ID_USAC_END = 4 + USAC_ID_BIT, /**< Arnie (End Element = Terminator).      */
  ID_LAST
} MP4_ELEMENT_ID;

typedef enum {
  /* usac */
  ID_EXT_ELE_FILL = 0x00,
  ID_EXT_ELE_MPEGS = 0x01,
  ID_EXT_ELE_SAOC = 0x02,
  ID_EXT_ELE_AUDIOPREROLL = 0x03,
  ID_EXT_ELE_UNI_DRC = 0x04,
  /* mpegh3da */
  ID_EXT_ELE_OBJ_METADATA = 0x05,
  ID_EXT_ELE_SAOC_3D = 0x06,
  ID_EXT_ELE_HOA = 0x07,
  ID_EXT_ELE_FMT_CNVRTR = 0x08,
  ID_EXT_ELE_MCT = 0x09,
  ID_EXT_ELE_TCC = 0x0A,
  ID_EXT_ELE_HOA_ENH_LAYER = 0x0B,
  ID_EXT_ELE_HREP = 0x0C,
  ID_EXT_ELE_ENHANCED_OBJ_METADATA = 0x0d,
  ID_EXT_ELE_PROD_METADATA = 0x0e,
  /* reserved for use outside of ISO scope */
  ID_EXT_ELE_VR_METADATA = 0x81,
  ID_EXT_ELE_UNKNOWN = 0xFF
} USAC_EXT_ELEMENT_TYPE;

typedef enum {
  /* USAC and MPEG-H 3DA */
  ID_CONFIG_EXT_FILL = 0,
  /* MPEG-H 3DA */
  ID_CONFIG_EXT_DOWNMIX = 1,
  ID_CONFIG_EXT_LOUDNESS_INFO = 2,
  ID_CONFIG_EXT_AUDIOSCENE_INFO = 3,
  ID_CONFIG_EXT_HOA_MATRIX = 4,
  ID_CONFIG_EXT_ICG = 5,
  ID_CONFIG_EXT_SIG_GROUP_INFO = 6,
  ID_CONFIG_EXT_COMPATIBLE_PROFILELVL_SET = 7
  /* 8-127 => reserved for ISO use */
  /* > 128 => reserved for use outside of ISO scope */
} CONFIG_EXT_ID;

typedef struct packet_info
{
  WORD32 sync_packet_bits;
  WORD32 sync_packet_length;
  WORD32 cnfg_box_complete;
  WORD32 asi_box_complete;
  WORD32 asi_packet_length;
  WORD32 asi_packet_bits;
  UWORD8 mhaD_buff[MAX_MAE_BUFFER_SIZE];
  UWORD8 mhaP_buff[MAX_MAE_BUFFER_SIZE];
  UWORD8 maeg_buff[MAX_MAE_BUFFER_SIZE];
  UWORD8 maes_buff[MAX_MAE_BUFFER_SIZE];
  UWORD8 maep_buff[MAX_MAE_BUFFER_SIZE];
  UWORD8 mael_buff[MAX_MAE_BUFFER_SIZE];
  WORD32 maeg_bits;
  WORD32 maes_bits;
  WORD32 maep_bits;
  WORD32 mael_bits;
  WORD32 mhaP_bits;
  WORD32 mhaD_bits;
  WORD32 maei_present;
  WORD8 maei_parse;
  WORD32 group_definition_length;
  WORD32 switch_group_length;
  WORD32 group_preset_length;
  WORD32 config_packet_bits;
  WORD32 config_packet_length;
  WORD32 config_packet_found;
  WORD32 config_packet_start_position;
  WORD32 mhac_content_size;
  WORD32 other_packet_bits;
  WORD32 other_packet_length;
  WORD32 frame_packet_bits;
  WORD32 sampling_freq;
  UWORD32 profile_info;
  UWORD32 spaker_layout;
  WORD8 mhaP_data_present;
  WORD8 mhaD_data_present;
  WORD32 max_frame_data_size;
  WORD32 total_frame_data_size;
  WORD32 frame_count;
} packet_info;

typedef struct
{
  WORD8 mae_groupID;
  WORD8 mae_allowOnOff;
  WORD8 mae_defaultOnOff;
  WORD8 mae_allowPositionInteractivity;
  WORD8 mae_interactivityMinAzOffset;
  WORD8 mae_interactivityMaxAzOffset;
  WORD8 mae_interactivityMinElOffset;
  WORD8 mae_interactivityMaxElOffset;
  WORD8 mae_interactivityMinDistFactor;
  WORD8 mae_interactivityMaxDistFactor;
  WORD8 mae_allowGainInteractivity;
  WORD8 mae_interactivityMinGain;
  WORD8 mae_interactivityMaxGain;
  WORD8 mae_bsGroupNumMembers;
  WORD8 mae_hasConjunctMembers;
  WORD8 mae_startID;
  WORD8 mae_metaDataElementID[MAX_METADATA_ELEMENT_ID_COUNT];
} ia_maeG_data_struct;

typedef struct
{
  WORD8 mae_switchGroupID;
  WORD8 mae_switchGroupAllowOnOff;
  WORD8 mae_switchGroupDefaultOnOff;
  WORD8 mae_bsSwitchGroupNumMembers;
  WORD8 mae_switchGroupMemberID[MAX_SWITCH_GROUP_NUM_MEMBERS];
  WORD8 mae_switchGroupDefaultGroupID;
} ia_maeS_data_struct;

typedef struct
{
  WORD8 mae_groupPresetID;
  WORD8 mae_groupPresetKind;
  WORD8 mae_bsGroupPresetNumConditions;
  WORD8 mae_groupPresetReferenceID[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 mae_groupPresetConditionOnOff[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 mae_groupPresetDisableGainInteractivity[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 mae_groupPresetGainFlag[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD16 mae_groupPresetGain[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD16 mae_groupPresetDisablePositionInteractivity[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 mae_groupPresetPositionFlag[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD16 mae_groupPresetAzOffset[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 mae_groupPresetElOffset[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 mae_groupPresetDistFactor[MAX_GROUP_PRESET_NUM_CONDITIONS];
} ia_maeP_data_struct;

typedef struct
{
  WORD8 mae_bsNumDescriptionBlocks;
  WORD8 mae_descriptionID[MAX_NUM_DESCRIPTOIN_BLOCKS];//mae_descriptionGroupID,mae_descriptionSwitchGroupID,mae_descriptionGroupPresetID
  WORD8 mae_bsNumDescLanguages[MAX_NUM_DESCRIPTOIN_BLOCKS];
  WORD32 mae_bsDescriptionLanguage[MAX_NUM_DESCRIPTOIN_BLOCKS][MAX_NUM_DESCR_LANGUAGES];
  WORD8 mae_bsDescriptionDataLength[MAX_NUM_DESCRIPTOIN_BLOCKS][MAX_NUM_DESCR_LANGUAGES];
  WORD16 mae_descriptionData[MAX_NUM_DESCRIPTOIN_BLOCKS][MAX_NUM_DESCR_LANGUAGES][MAX_DESCRIPTON_DATA_LEN];
} ia_description_data_struct;

typedef struct
{
  WORD8 mae_bsNumContentDataBlocks;
  WORD8 mae_ContentDataGroupID[MAX_NUM_CONTENT_BLOCKS];
  WORD8 mae_contentKind[MAX_NUM_CONTENT_BLOCKS];
  WORD8 mae_hasContentLanguage[MAX_NUM_CONTENT_BLOCKS];
  WORD32 mae_contentLanguage[MAX_NUM_CONTENT_BLOCKS];
} ia_content_data_structure;

typedef struct
{
  WORD8 mae_numDataSets;
  WORD8 mae_dataType[MAX_MAE_NUM_DATA_SETS];
  WORD32 mae_dataLength[MAX_MAE_NUM_DATA_SETS];
  ia_description_data_struct group_desc_data;
  ia_description_data_struct switch_desc_data;
  ia_description_data_struct group_preset_desc_data;
  ia_content_data_structure content_data;
} ia_mae_data_struct;

typedef struct
{
  WORD8 mae_isMainStream;
  WORD8 mae_audioSceneInfoIDPresent;
  WORD32 mae_audioSceneInfoID;
  WORD32 mae_numGroups;
  WORD32 mae_numSwitchGroups;
  WORD32 mae_numGroupPresets;
  ia_maeG_data_struct maeG_data[MAX_NUM_GROUPS];
  ia_maeS_data_struct maeS_data[MAX_NUM_SWITCH_GROUPS];
  ia_maeP_data_struct maeP_data[MAX_NUM_GROUPS_PRESETS];
  ia_mae_data_struct mae_data;
  WORD32 mae_metaDataElementIDoffset;
  WORD32 mae_metaDataElementIDmaxAvail;
} ia_audio_scene_data;


typedef struct
{
  WORD8 isCICPspeakerIdx;
  WORD8 CICPspeakerIdx;
  WORD8 ElevationClass;
  WORD8 ElevationAngleIdx;
  WORD8 ElevationDirection;
  WORD8 AzimuthAngleIdx;
  WORD8 AzimuthDirection;
  WORD8 isLFE;
  WORD8 el;
  WORD8 az;
} ia_mpegh3daSpeakerDescription;

typedef struct
{
  WORD8 angularPrecision;
  ia_mpegh3daSpeakerDescription mpegh3daSpeakerDescription_data[100];//check max number of speaker value
  WORD8 alsoAddSymmetricPair[100];//check max number of speaker value

} ia_mpegh3daFlexibleSpeakerConfig;


typedef struct
{
  WORD8 speakerLayoutType;
  WORD8 CICPspeakerLayoutIdx;
  WORD32 numSpeakers;
  WORD8 CICPspeakerIdx[MAX_LOUD_SPEAKER_LAYOUT_INDX];
  ia_mpegh3daFlexibleSpeakerConfig mpegh3daFlexibleSpeakerConfig_data;
} ia_SpeakerConfig3d;


typedef struct
{
  WORD32 numAudioChannels;
  WORD32 numAudioObjects;
  WORD32 numSAOCTransportChannels;
  WORD32 numHOATransportChannels;


  WORD8 bsNumSignalGroups;
  WORD8 signalGroupType[MAX_NUM_SIGNAL_GROUPS];
  WORD32 signal_groupID[MAX_NUM_SIGNAL_GROUPS];
  WORD32 bsNumberOfSignals[MAX_NUM_SIGNAL_GROUPS];
  WORD32 differsFromReferenceLayout[MAX_NUM_SIGNAL_GROUPS];
  WORD32 audioChannelLayout[MAX_NUM_SIGNAL_GROUPS];
  ia_SpeakerConfig3d SpeakerConfig3d_data[MAX_NUM_SIGNAL_GROUPS];
  WORD8 saocDmxLayoutPresent;
  WORD8 saocDmxChannelLayout[32];
} ia_FrameworkConfig3d;

typedef struct
{
  WORD8 hasDiffuseness;
  WORD8 hasCommonGroupDiffuseness;
  WORD8 hasExcludedSectors;
  WORD8 hasCommonGroupExcludedSectors;
  WORD8 useOnlyPredefinedSectors[MAX_NUM_OAM_OBJS];
  WORD8 hasClosestSpeakerCondition;
  WORD8 closestSpeakerThresholdAngle;
  WORD8 hasDivergence[MAX_NUM_OAM_OBJS];
  WORD8 divergenceAzimuthRange[MAX_NUM_OAM_OBJS];
} ia_EnhancedObjectMetadataConfig;

typedef struct
{
  WORD8 drcLocation;
  WORD8 drcFrameSizePresent;
  WORD16 bsDrcFrameSize;
  WORD8 gainSetCount;
  WORD8 gainSetIndex[MAX_GAIN_SET_COUNT];
  WORD8 gainCodingProfile[MAX_GAIN_SET_COUNT];
  WORD8 gainInterpolationType[MAX_GAIN_SET_COUNT];
  WORD8 fullFrame[MAX_GAIN_SET_COUNT];
  WORD8 timeAlignment[MAX_GAIN_SET_COUNT];
  WORD8 timeDeltaMinPresent[MAX_GAIN_SET_COUNT];
  WORD16 bsTimeDeltaMin[MAX_GAIN_SET_COUNT];
  WORD8 bandCount[MAX_GAIN_SET_COUNT];
  WORD8 drcBandType[MAX_GAIN_SET_COUNT];
  WORD8 drcCharacteristic[MAX_GAIN_SET_COUNT][MAX_BAND_COUNT];
  WORD8 crossoverFreqIndex[MAX_GAIN_SET_COUNT][MAX_BAND_COUNT];
  WORD8 startSubBandIndex[MAX_GAIN_SET_COUNT][MAX_BAND_COUNT];
  WORD32 gainSequenceCount;
} ia_drcCoefficientsUniDrc;

typedef struct
{
  WORD8 drcSetId;
  WORD8 drcLocation;
  WORD8 downmixId;
  WORD8 additionalDownmixIdPresent;
  WORD8 additionalDownmixIdCount;
  WORD8 additionalDownmixId[MAX_ADDITIONAL_DMIX_COUNT];
  WORD32 drcSetEffect;
  WORD8 limiterPeakTargetPresent;
  WORD16 bsLimiterPeakTarget;
  WORD8 drcSetTargetLoudnessPresent;
  WORD8 bsDrcSetTargetLoudnessValueUpper;
  WORD8 drcSetTargetLoudnessValueLowerPresent;
  WORD8 bsDrcSetTargetLoudnessValueLower;
  WORD8 dependsOnDrcSetPresent;
  WORD8 dependsOnDrcSet;
  WORD8 noIndependentUse;
  WORD8 bsGainSetIndex[MAX_BASE_CHANNEL_COUNT];
  WORD8 repeatGainSetIndex[MAX_BASE_CHANNEL_COUNT];

  //nDrcChannelGroups
  WORD8 bsRepeatGainSetIndexCount[MAX_BASE_CHANNEL_COUNT];
  WORD8 duckingScalingPresent[MAX_BASE_CHANNEL_COUNT];
  WORD8 bsDuckingScaling[MAX_BASE_CHANNEL_COUNT];
  WORD8 repeatParameters[MAX_BASE_CHANNEL_COUNT];
  WORD8 bsRepeatParametersCount[MAX_BASE_CHANNEL_COUNT];
  WORD8 nDrcChannelGroups[MAX_BASE_CHANNEL_COUNT];
  WORD8 targetCharacteristicLeftIndex[MAX_BASE_CHANNEL_COUNT];
  WORD8 targetCharacteristicRightPresent[MAX_BASE_CHANNEL_COUNT];
  WORD8 targetCharacteristicRightIndex[MAX_BASE_CHANNEL_COUNT];
  WORD8 gainScalingPresent[MAX_BASE_CHANNEL_COUNT];
  WORD8 bsAttenuationScaling[MAX_BASE_CHANNEL_COUNT];
  WORD8 bsAmplificationScaling[MAX_BASE_CHANNEL_COUNT];
  WORD8 gainOffsetPresent[MAX_BASE_CHANNEL_COUNT];
  WORD8 bsGainOffset[MAX_BASE_CHANNEL_COUNT];
  WORD8 shapeFilterPresent[MAX_BASE_CHANNEL_COUNT];
  WORD8 shapeFilterIndex[MAX_BASE_CHANNEL_COUNT];
} ia_drcInstructionsUniDrc;



typedef struct
{
  WORD8 uniDrcConfigExtType;
  WORD8 bitSizeLen;
  WORD32 bitSize;
  WORD8 parametricDrcInstructionsCount;
  WORD8 downmixInstructionsV1Present;
  WORD8 downmixInstructionsV1Count;
  WORD8 drcCoeffsAndInstructionsUniDrcV1Present;
  WORD8 drcCoefficientsUniDrcV1Count;
  WORD8 drcInstructionsUniDrcV1Count;
  WORD8 loudEqInstructionsPresent;
  WORD8 loudEqInstructionsCount;
  WORD8 eqPresent;
  WORD8 eqInstructionsCount;
  WORD8 otherBit[64];
} ia_uniDrcConfigExtension;

typedef struct
{
  WORD8 version;
  WORD8 drcSetId;
  WORD8 eqSetId;
  WORD8 downmixId;
  WORD8 samplePeakLevelPresent;
  WORD16 bsSamplePeakLevel;
  WORD8 truePeakLevelPresent;
  WORD16 bsTruePeakLevel;
  WORD8 measurementSystem;
  WORD8 reliability;
  WORD8 measurementCount;
  WORD8 methodDefinition[MAX_LOUDNESS_INFO_COUNT];
  WORD16 methodValue[MAX_LOUDNESS_INFO_COUNT];
  WORD8 measurementSystemArr[MAX_LOUDNESS_INFO_COUNT];
  WORD8 reliabilityArr[MAX_LOUDNESS_INFO_COUNT];
} ia_loudnessInfo;

typedef struct
{
  WORD8 loudnessInfoSetExtType;
  WORD8 bitSizeLen;
  WORD32 bitSize;
  WORD8 loudnessInfoV1AlbumCount;
  ia_loudnessInfo loudnessInfoV1Album[MAX_LOUDNESS_INFO_COUNT];
  WORD8 loudnessInfoV1Count;
  ia_loudnessInfo loudnessInfoV1[MAX_LOUDNESS_INFO_COUNT];
  WORD8 otherBit[MAX_LOUDNESS_INFO_COUNT];
} ia_loudnessInfoSetExtension;

typedef struct
{
  WORD8 loudnessInfoCount;
  WORD8 loudnessInfoType[MAX_LOUDNESS_INFO_COUNT];
  WORD8 mae_groupID[MAX_LOUDNESS_INFO_COUNT];
  WORD8 mae_groupPresetID[MAX_LOUDNESS_INFO_COUNT];
  ia_loudnessInfo loudnessInfo[MAX_LOUDNESS_INFO_COUNT];
  WORD8 loudnessInfoAlbumPresent;
  WORD8 loudnessInfoAlbumCount;
  ia_loudnessInfo loudnessInfoAlbum[MAX_LOUDNESS_INFO_COUNT];
  WORD8 loudnessInfoSetExtensionPresent;
  ia_loudnessInfoSetExtension loudnessInfoSetExtension;
} ia_mpegh3daLoudnessInfoSet;

typedef struct
{
  WORD8 baseChannelCount;
} ia_mpegh3daUniDrcChannelLayout;

typedef struct
{
  WORD8 downmixId;
  WORD8 targetChannelCount;
  WORD8 targetLayout;
  WORD8 downmixCoefficientsPresent;
} ia_downmixInstruction;

typedef struct
{
  WORD8 drcCoefficientsUniDrcCount;
  WORD8 drcInstructionsUniDrcCount;
  WORD8 downmixInstructionsCount;
  ia_mpegh3daUniDrcChannelLayout mpegh3daUniDrcChannelLayout_data;
  ia_drcCoefficientsUniDrc drcCoefficientsUniDrc_data[MAX_DRC_COEFFICIENTS_UNI_DRC_COUNT];
  ia_downmixInstruction downmixInstructions[32];
  WORD8 drcInstructionsType[32];
  WORD8 mae_groupID[32];
  WORD8 mae_groupPresetID[32];
  ia_drcInstructionsUniDrc drcInstructionsUniDrc_data[MAX_DRC_INSTRUCTIONS_UNI_DRC_COUNT];
  WORD8 uniDrcConfigExtPresent;
  ia_uniDrcConfigExtension  uniDrcConfigExtension_data;
  WORD8 loudnessInfoSetPresent;
  ia_mpegh3daLoudnessInfoSet mpegh3daLoudnessInfoSet_data;

} ia_mpegh3daUniDrcConfig;

typedef struct
{
  WORD32 usacExtElementType;
  WORD32 usacExtElementConfigLength;
  WORD32 usacExtElementDefaultLengthPresent;
  WORD32 usacExtElementDefaultLength;
  WORD8 usacExtElementPayloadFrag;
  ia_EnhancedObjectMetadataConfig EnhancedObjectMetadataConfig_data;
  ia_mpegh3daUniDrcConfig mpegh3daUniDrcConfig_data;

} ia_mpegh3daExtElementConfig;

typedef struct
{
  WORD8 tw_mdct;
  WORD8 fullbandLpd;
  WORD8 noiseFilling;
  WORD8 enhancedNoiseFilling;
  WORD8 igfUseEnf;
  WORD8 igfUseHighRes;
  WORD8 igfUseWhitening;
  WORD8 igfAfterTnsSynth;
  WORD8 igfStartIndex;
  WORD8 igfStopIndex;
  WORD8 igfIndependentTiling;
  WORD8 qceIndex;
  WORD8 shiftIndex0;
  WORD32 shiftChannel0;
  WORD8 shiftIndex1;
  WORD32 shiftChannel1;
  WORD32 lpdStereoIndex;
  ia_mpegh3daExtElementConfig mpegh3daExtElementConfig_data;
} ia_mpegh3daCoreConfig;

typedef struct
{
  WORD32 numAudioChannels;
  WORD32 numAudioObjects;
  WORD32 numSAOCTransportChannels;
  WORD32 numHOATransportChannels;


  WORD32 numElements;
  WORD8 elementLengthPresent;
  MP4_ELEMENT_ID usacElementType[TP_USAC_MAX_ELEMENTS];
  WORD8 mpegh3daUniDrcConfigPresent;
  ia_mpegh3daCoreConfig mpegh3daCoreConfig_data[TP_USAC_MAX_ELEMENTS];
} ia_mpegh3daDecoderConfig;

typedef struct
{
  WORD8 CompatibleProfileLevelSet_data_present;
  WORD8 bsNumCompatibleSets;
  WORD8 numCompatibleSets;
  WORD8 CompatibleSetIndication[16];
} ia_CompatibleProfileLevelSet;

typedef struct
{
  WORD8 downmixIdCount;
  WORD8 downmixId[MAX_DOWNMIX_ID_COUNT];
  WORD8 downmixType[MAX_DOWNMIX_ID_COUNT];
  WORD8 CICPspeakerLayoutIdx[MAX_DOWNMIX_ID_COUNT];
  WORD8 bsDownmixMatrixCount[MAX_DOWNMIX_ID_COUNT];
  WORD16 bsNumAssignedGroupIDs[MAX_DOWNMIX_ID_COUNT][MAX_DOWNMIX_MATRIX_COUNT];
  WORD8 signal_groupID[MAX_DOWNMIX_ID_COUNT][MAX_DOWNMIX_MATRIX_COUNT][MAX_DOWNMIX_MATRIX_LEN];
  WORD32 dmxMatrixLenBits[MAX_DOWNMIX_ID_COUNT][MAX_DOWNMIX_MATRIX_COUNT];
} ia_downmixMatrixSet;

typedef struct
{
  WORD8 downmixConfigType;
  WORD8 passiveDownmixFlag;
  WORD8 phaseAlignStrength;
  WORD8 immersiveDownmixFlag;
  ia_downmixMatrixSet dowmixMatrixSet;
} ia_downmixConfig;

typedef struct
{
  WORD32 numConfigExtensions;
  WORD16 usacConfigExtType[MAX_USAC_CONFIG_EXT_TYPES];
  WORD16 usacConfigExtLength[MAX_USAC_CONFIG_EXT_TYPES];
  ia_CompatibleProfileLevelSet CompatibleProfileLevelSet_data;
  ia_downmixConfig downmixConfig;
} ia_mpegh3daConfigExtension;


typedef struct
{
  WORD8 mpegh3daProfileLevelIndication;
  WORD8 usacSamplingFrequencyIndex;
  WORD32 usacSamplingFrequency;
  WORD8 coreSbrFrameLengthIndex;
  WORD8 cfg_reserved;
  WORD8 receiverDelayCompensation;
  WORD8 referenceLayout;
  ia_SpeakerConfig3d SpeakerConfig3d_data;
  ia_FrameworkConfig3d FrameworkConfig3d_data;
  ia_mpegh3daDecoderConfig mpegh3daDecoderConfig_data;
  WORD8 usacConfigExtensionPresent;
  ia_mpegh3daConfigExtension mpegh3daConfigExtension_data;

} ia_3d_audio_cnfg_data;

WORD32 impegh_read_bits_buf(ia_bit_buf_struct *it_bit_buff, WORD no_of_bits);
UWORD32 impegh_read_escape_value(ia_bit_buf_struct *it_bit_buff,
  UWORD32 no_bits1,
  UWORD32 no_bits2, UWORD32 no_bits3, WORD32 *bits_read);

WORD32 impegh_skip_bits_buf(ia_bit_buf_struct *it_bit_buff, WORD no_of_bits);

ia_bit_buf_struct *impegh_create_bit_buf(ia_bit_buf_struct *it_bit_buff,
                                           UWORD8 *ptr_bit_buf_base, WORD32 bit_buf_size);

VOID impegh_create_init_bit_buf(ia_bit_buf_struct *it_bit_buff, UWORD8 *ptr_bit_buf_base,
                                WORD32 bit_buf_size);

WORD32 impegh_mhas_parse(ia_bit_buf_struct *ptr_bit_buf, ia_mhas_pac_info *ptr_pac_info,
                         packet_info *header_info);

WORD32 impegh_file_parse(ia_bit_buf_struct *ptr_bit_buf);

#endif
