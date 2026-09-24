# 1. Introduction

The libmpeghe encoder sample application has command line support for writing DRC information. It expects the DRC information to be presented in txt format. Refer to the command line option `-idrc:`.

The `-idrc:<path to drc_file.tx>` argument is an optional argument that allows the user to specify an TXT file containing detailed drc information for audio scene. This file provides metadata that defines drc characteristics and can be used along with ASI input file (refer command line option `-iasi:`) to represent DRC characteristics for each group or preset. 

The input file follows structure to pass the DRC related information(Instruction set and Coefficient set).

# 2. Input Structure
The following input structure describes how inputs are passed to encoder.

## 2.1 uniDRC configuration

### 2.1.1 DRC instructions

```plaintext
drc_instructions_uni_drc_count:(int)
└── downmix_id:(int)
└── drc_set_effect:(int)
└── gain_set_channels:(int)
    └──gain_set_index: (int)
└──num_drc_channel_groups:(int)
└──drc_instructions_type:(int)
    └──mae_group_id/mae_group_preset_id:(int)(if drc_instructions_type: != 0 )
```
### 2.1.2 DRC coefficients

```plaintext
drc_coefficients_uni_drc_count:(int)
└── gain_set_count:(int)
    └── band_count:(int)
        └── nb_points:(int)
          └── x:(float)
          └── y:(float)
        └── width:(float)
        └── attack:(float)
        └── decay:(float)
        └── start_sub_band_index:(int)(if band_count > 1)
```

The following is an example for single DRC gain set interface TXT file:

```plaintext
##str_drc_instructions_uni_drc##
drc_instructions_uni_drc_count:1
downmix_id:0
drc_set_effect:0x0020
gain_set_channels:4
gain_set_index:0
gain_set_index:0
gain_set_index:0
gain_set_index:0
num_drc_channel_groups:1
drc_instructions_type:0
##str_drc_coefficients_uni_drc##
drc_coefficients_uni_drc_count:1
gain_set_count:1
band_count:1
nb_points:3
x:-60.0
y:-40.0
x:-30.0
y:-30.0
x:0.0
y:-20.0
width:0.01
attack:10.0
decay:20.0
```
### 2.1.3 Element Descriptions

Here is a brief description of each element in the DRC interface txt file.

| **Sl. No.** | **Element name** | **Element description** | **Element type** | 
|-----|------|------|------|
| 1 |`drc_instructions_uni_drc_count:` | Number of DRC instructions set. | Integer | 
| 2 |`downmix_id:` | Downmix ID. | Integer |
| 3 |`drc_set_effect:` | Set DRC effect. | Integer |
| 4 |`gain_set_channels:` | Total number of channels. Can be equivalent to total channels in audio scene/ total channels in a group/ total channels in preset. | Integer |
| 5 |`gain_set_index:` | Gain Set index for indiviual channel. This is mapped to coeffecient set. | Integer |
| 6 |`num_drc_channel_groups:` | Total number if unique set groups.(Unique Set index). | Integer |
| 7 |`drc_instructions_type:` | Type of DRC instruction. `0` Audio scene, `2` MAE Group, `3` MAE Group preset. | Integer |
| 8 |`mae_group_id/mae_group_preset_id:` | Group id in case of drc_instructions_type set to `2`. Preset id in case drc_instructions_type set to `3`. | Integer |

| **Sl. No.** | **Element name** | **Element description** | **Element type** | 
|-----|------|------|------|
| 1 |`drc_coefficients_uni_drc_count:` | Number of DRC coefficients set. | Integer | 
| 2 |`gain_set_count:` | Gain Set count. | Integer |
| 3 |`band_count:` | Band count. | Integer |
| 4 |`nb_points:` | Number of points .| Integer |
| 5 |`x:` | `X` value. | Float |
| 6 |`y:` | `Y` value. | Float |
| 7 |`width:` | Width value. | Float |
| 8 |`attack:` | Attack value. | Float |
| 9 |`decay:` | Decay value. | Float |
| 10 |`start_sub_band_index:` | Start sub band index of band count is more than `1`. | Integer |

## 2.2 Loundness configuration

### 2.2.1 loudness info
```plaintext
loudness_info_count: (int)
└── drc_set_id: (int)
└── downmix_id: (int)
└── sample_peak_level_present: (int)
    └── sample_peak_level: (float)
└── true_peak_level_present: (int)
    └── true_peak_level: (float)
    └── true_peak_level_measurement_system: (int)
    └── true_peak_level_reliability: (int)
└── measurement_count: (int)
    └── method_definition: (int)
    └── method_value: (float)
    └── measurement_system: (int)
    └── reliability: (int)
```
### 2.2.2 loudness info album
```plaintext
loudness_info_album_count: (int)
└── drc_set_id: (int)
└── downmix_id: (int)
└── sample_peak_level_present: (int)
    └── sample_peak_level: (float)
└── true_peak_level_present: (int)
    └── true_peak_level: (float)
    └── true_peak_level_measurement_system: (int)
    └── true_peak_level_reliability: (int)
└── measurement_count: (int)
    └── method_definition: (int)
    └── method_value: (float)
    └── measurement_system: (int)
    └── reliability: (int)
```

### 2.2.3 Element description of loudness configuration parameters
Here is a brief description of each element in loudness configuration.
| **Sl. No.** | **Element name** | **Element description** | **Element type** | 
|-----|------|------|------|
| 1 |`loudness_info_count:` | loudnessInfo count. Valid values are 0 to 31. | Integer | 
| 2 |`drc_set_id:` | Defines the DRC set relevant to the loudness data. | Integer |
| 3 |`downmix_id:` | Defines the downmix configuration relevant to the loudness data. | Integer |
| 4 |`sample_peak_level_present:` | Flag to indicate if sample peak level is present. Valid values are 0 and 1. | Integer |
| 5 |`sample_peak_level:` | Defines the maximum sample peak level in the audio signal (in dB.) | Float |
| 6 |`true_peak_level_present:` | Flag to indicate if sample peak level is present. Valid values are 0 and 1. | Integer |
| 7 |`true_peak_level:` | Defines the maximum true peak level of the audio signal (in dB). | Float |
| 8 |`true_peak_level_measurement_system:` |  Defines the measurement system used to determine the true peak level. | Integer |
| 9 |`true_peak_level_reliability:` | Defines the reliability level of the true peak measurement. | Integer |
| 10 |`measurement_count:` | Defines the number of measurements used to calculate the loudness value. Valid values are 0 to 14 | Integer |
| 11 |`method_definition:` | Defines the method used to measure loudness level. | Integer |
| 12 |`method_value:` | Defines the loudness level value associated with the measurement method. | Float |
| 13 |`measurement_system:` | Defines the measurement system employed to measure the loudness value. | Integer |
| 14 |`reliability:` | Defines the reliability level of the measured value. | Integer |

Following is an example with loudness info configured in the DRC file:
```plaintext
#loudness info parameters
loudness_info_count:1
#n=0
drc_set_id:1
downmix_id:0
sample_peak_level_present:1
sample_peak_level:0.0
true_peak_level_present:1
true_peak_level:0.0
true_peak_level_measurement_system:2
true_peak_level_reliability:3
measurement_count:1
#m=0
method_definition:1
method_value:-16.0
measurement_system:2
reliability:3
#end loudness info parameters

#loudness info album parameters
loudness_info_album_count:0
#end of loudness info album parameters
```
