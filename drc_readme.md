# 1. Introduction

The libmpeghe encoder sample application has command line support for writing DRC information. It expects the DRC information to be presented in txt format. Refer to the command line option `-idrc:`.

The `-idrc:<path to drc_file.tx>` argument is an optional argument that allows the user to specify an TXT file containing detailed drc information for audio scene. This file provides metadata that defines drc characteristics and can be used along with ASI input file (refer command line option `-iasi:`) to represent DRC characteristics for each group or preset. 

The input file follows structure to pass the DRC related information(Instruction set and Coefficient set).

# 2. Input Structure
The following input structure describes how inputs are passed to encoder.

# 2.1 DRC instructions

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
# 2.2 DRC coefficients

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
# 3. Element Descriptions

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

