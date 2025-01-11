# 1. Introduction

The MPEG-H encoder supports encoding of metadata audio elements. All the static metadata is organized in the "Audio Scene Info (ASI)" which is present in the bit-stream as a config extension.

The libmpeghe encoder sample application expects the audio scene information to be presented in text format - refer to the command line option `-iasi:`.
Each entry of the audio scene info data, as described in section 15 of the specification document - ISO/IEC 23008-3, has a specific syntax in the libmpeghe interface. This document serves as a user guide to map different elements of the ASI file and will be useful in composing a custom ASI text file.

Common info about the syntax followed in the ASI file:
- All the field names and their corresponding values are separated by `:`.
- In cases where a field has multiple entries (or) values comma - `,` is used as separator.
- No spaces are allowed between comma separated values.
- Any line which starts with `#` or `//` is considered as a comment.

# 2. Encoder interface

The audio scene information is passed to the sample application in a text format. This section describes the entries of supported by the ASI file and their description.

Each of the sub-sections have an associated "Example usage:" that illustrates the usage of the fields in a actual ASI file. The case picked for these illustrations has 2 Channel-based signal groups, each 2.0 channel configuration.

## 2.1 Audio Scene Information - Mandatory fields

An audio scene information that pertains to a main stream of MPEG-H audio, expects some mandatory fields. The following table lists these mandatory fields and their description.

| **Sl. No.** | **Text File Entry** | **Description** |
|-----|------|------|
| 1 |`main_stream_flag` | Signals if the actual MPEG-H stream is the main stream. In multi stream scenarios the main and side-streams are determined by this flag |
| 2 |`mae_id_offset` | Indicates the signal number offset for streams where `main_stream_flag` is `0`. This field is `0` for main Stream. |
| 3 |`mae_id_max_avail` | Maximum value of the group IDs in the audio scene information. |
| 4 |`num_groups` | Number of groups in the audio scene information. |
| 5 |`num_switch_groups` | Number of switch groups in the audio scene information. |
| 6 |`num_group_presets` | Number of group presets in the audio scene information. |
| 7 |`num_data_sets` | Number of MAE data sets in the ASI. |

### Example usage:
```
main_stream_flag:1
mae_id_offset:0
mae_id_max_avail:4
```

## 2.2 Group definition

Each of the entries under the "group definition" block should have as many values as the number of groups.

| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 |`group_definition.group_id` | IDs of the signal groups in the stream. | `0` - `127` |
| 2 |`group_definition.allow_on_off` | Flag that indicates if the user is allowed to switch a group on and off. | `0` or `1`|
| 3 |`group_definition.default_on_off` | Flag that indicates default ON(`=1`) and OFF(`=0`) status of a group. | `0` or `1`|
| 4 |`group_definition.allow_pos_interact` | Flag that indicates if the user is allowed to change position of elements of the group.| `0` or `1`|
| 5 |`group_definition.min_az_offset` | Minimum azimuth offset for changing the position of members of a group. Valid only when the corresponding position interaction flag is on.| `-180` - `0`|
| 6 |`group_definition.max_az_offset` | Maximum azimuth offset for changing the position of members of a group. Valid only when the corresponding position interaction flag is on.| `0` - `180`|
| 7 |`group_definition.min_el_offset` | Minimum elevation offset for changing the position of members of a group. Valid only when the corresponding position interaction flag is on.| `-90` - `0`|
| 8 |`group_definition.max_el_offset` | Maximum elevation offset for changing the position of members of a group. Valid only when the corresponding position interaction flag is on.| `0` - `90`|
| 9 |`group_definition.min_dist_factor` | Minimum distance change factor for interactively changing the position of the members of a group. Valid only when the corresponding position interaction flag is on. The actual minimum distance is calculated using the formula minDist = Pow(2, min_dist_factor - 12)| `0` - `15` (0.00025m to 8)|
| 10 |`group_definition.max_dist_factor` | Maximum distance change factor for interactively changing the position of the members of a group. Valid only when the corresponding position interaction flag is on. The actual maximum distance is calculated using the formula maxDist = Pow(2, max_dist_factor - 12)| `0` - `15` (0.00025m to 8)|
| 11 |`group_definition.allow_gain_interact` | Flag that indicates if the user is allowed to change the gain of a group| `0` or `1`|
| 12 |`group_definition.min_gain` | Minimum gain of the members of a metadata element group. Valid only when the corresponding gain interaction flag is on.| `-63` - `0` (steps of 1dB)|
| 13 |`group_definition.max_gain` | Maximum gain of the members of a metadata element group. Valid only when the corresponding gain interaction flag is on.| `0` - `31` (steps of 1dB)|
| 14 |`group_definition.start_id` | Start ID of the first element of the corresponding group | Depends on number of groups. |
| 15 |`group_definition.has_conjunct_member` | Flag that indicates if the members of a group are coded consecutively in the bitstream. | `0` or `1` |
| 16 |`group_definition.group_num` | Number of members in the group. | - |
| 17 |`group_definition.metadata_ele_id` | Element IDs of the members of the groups which do not have conjunct members.| - |

### Example usage:
```
num_groups:2                             // 2- Channel based signal groups
group_definition.group_id:0,1            // IDs of the two groups
group_definition.allow_on_off:1,1        // On / off allowed for both the groups
group_definition.default_on_off:1,0      // Group '0' is ON by default
group_definition.allow_pos_interact:0,0  // Position interactivity disabled for both groups
group_definition.allow_gain_interact:0,0 // Gain interactivity disabled for both groups
group_definition.group_num:2,2           // Number of members in each group 2 since each group has 2 channels
group_definition.has_conjunct_member:1,1 // Both the groups has conjunct members
group_definition.start_id:0,2            // Start ID of the elements of group '0' and group '1'
```

## 2.3 Switch Group definition

Each of the entries under the "switch group definition" block should have as many values as the number of switch groups.
Some of the fields are expected to be repeated in "sets". The number of sets of these data blocks in the ASI text file should be same as the number of presets.
The first column entry of all such items are marked * in the table below.

| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 |`switch_group_definition.group_id`| IDs of the switch groups in the stream.| `0` - `31` |
| 2 |`switch_group_definition.allow_on_off`| Flag that indicates if the user is allowed to completely disable the playback of the switch group. If set to `0` then one member of the group is always played back, if set to `1`, either none or one member of the group is played back.| `0` or `1`|
| 3 |`switch_group_definition.group_num_members`|Indicates the number of members of a switch group.| `0` - `31`|
| 4* |`switch_group_definition.member_id`|Indicates the group IDs of the members of the switch group.| `0` - `127`|
| 5* |`switch_group_definition.default_on_off`|Flag that indicates if the switch group is enabled or disabled for playback by default.| `0` or `1`|
| 6* |`switch_group_definition.default_grp_id`|Indicates ID of the default member of the switch group. |`0` - `127`|

### Example usage:
```
num_switch_groups:1                         // Number of switch groups in the Audio scene
switch_group_definition.group_id:0          // ID of the switch group
switch_group_definition.allow_on_off:0      // ON / OFF disabled for the switch group
switch_group_definition.default_on_off:0    // Switch group is off by default
switch_group_definition.group_num_members:2 // Number of members of the switch group
switch_group_definition.member_id:0,1       // IDs of the members of the switch group
switch_group_definition.default_grp_id:0    // ID of the group that is played back by default
```

## 2.4 Preset definition

Each of the entries under the "preset definition" block should have as many values as the number of presets.
Some of the fields are expected to be repeated in "sets". The number of sets of these data blocks in the ASI text file should be same as the number of presets.
The first column entry of all such items are marked * in the table below.

| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 |`group_preset_definition.grp_id`| IDs of the group preset.| `0` - `31` |
| 2 |`group_preset_definition.preset_kind`| Kind of content of a group preset. Refer to the [table](#241-preset-kind) for more details.| `0` or `1`|
| 3 |`group_preset_definition.num_conditions`|Indicates number of group conditions that are associated with a group preset.| `0` - `31`|
| 4* |`group_preset_definition.reference_id`|Indicates the groups or switch groups associated with a group preset.By default, this reference is interpreted as a groupID. The reference can be defined to be interpreted as a switchGroupID by extension metadata.| `0` - `127`|
| 5* |`group_preset_definition.cond_on_off`|This flag describes the required on/off status of a group or a switch group associated with a group preset or a group preset extension. If the flag is 1 and the referenced ID is defined to be interpreted as a groupID, the associated group has to be switched on to validate the group preset or group preset extension. If the referenced ID is defined to be interpreted as a switchGroupID, a groupPresetConditionOnOff  with value 1 means that one member of the switch group has to be switched on to validate the group preset If the flag is 0, the associated group or switch group has to be switched off.| `0` or `1`|
| 6* |`group_preset_definition.disable_gain_interact`|Indicates whether the gain interactivity of the currently referenced group or of the members of the referenced switch group shall be disabled (flag is equal to 1) or shall stay enabled (flag is equal to 0) if the preset or preset extension is chosen/valid. |`0` or `1`|
| 7* |`group_preset_definition.gain_flag`|Indicates whether the corresponding preset or preset extension specifies an initial gain of the members of a metadata element group or of the element members of the group members of the referenced switch group. It shall only be 1 if the flag mae_allowGainInteractivity of the corresponding group or of all the members of the referenced switch group is set to 1. . |`0` or `1`|
| 8* |`group_preset_definition.gain`|Indicates the initial gain of the members of a metadata element group or of the element members of the group members of the referenced switch group when the corresponding preset or preset extension is selected. |`0` - `255`|
| 9* |`group_preset_definition.disable_position_interact`|Indicates whether the position interactivity of the currently referenced group or of the members of the referenced switch group shall be disabled (flag is equal to 1) or shall stay enabled (flag is equal to 0) if the preset or preset extension is chosen/valid.|`0` or `1`|
| 10* |`group_preset_definition.position_interact`|Indicates whether initial position interactivity data (azimuth offset, elevation offset and distance factor) is present that shall be applied to the members of a metadata element group or to the element members of the group members of the referenced switch group. It shall only be 1 if the flag mae_allowPositionInteractivity of the corresponding group or of all the members of the referenced switch group is set to 1. |`0` or `1`|
| 11* |`group_preset_definition.azimuth_offset`|Indicates the additional azimuth offset that shall be applied to the currently referenced group or the members of the referenced switch group if the preset or preset extension is chosen/valid. |`-180` - `+180`|
| 12* |`group_preset_definition.elevation_offset`|Indicates Indicates the additional elevation offset that shall be applied to the currently referenced group or the members of the referenced switch group if the preset or preset extension is chosen/valid. |`-90` - `+90`|
| 13* |`group_preset_definition.dist_factor`|Indicates Indicates the additional distance change factor that shall be applied to the currently referenced group or the members of the referenced switch group if the preset or preset extension is chosen/valid. The actual distance is calculated using the formula `Dist = Pow(2, dist_factor - 12)` |`0` - `15` (0.00025m to 8)|

### 2.4.1 Preset kind
| **Group Preset Kind** | **Description** |
|------|-----|
| 0 |undefined|
| 1 |integrated TV loudspeaker|
| 2 |high quality loudspeaker|
| 3 |mobile loudspeakers|
| 4 |mobile headphones|
| 5 |hearing impaired (light)|
| 6 |hearing impaired (heavy)|
| 7 |visually impaired / audio description|
| 8 |spoken subtitles|
| 9 |loudness/DRC|
| 10-25 |/* reserved for ISO use */|
| 26-30 |/* reserved for use outside ISO scope */|
| 31 |other|

### Example usage:
```
num_group_presets:2                                   // Number of presets
group_preset_definition.grp_id:0,1                    // Preset IDs
group_preset_definition.preset_kind:0,0               // Preset Kind
group_preset_definition.num_conditions:2,2            // Number of conditions in each preset
// Data corresponding to preset 0
group_preset_definition.reference_id:0,1              // Group IDs referenced in Preset 0
group_preset_definition.cond_on_off:1,0               // ON OFF status of the groups
group_preset_definition.gain_flag:0,0                 // Gain flag status of the groups
group_preset_definition.gain:4,4                      // Gain value - ignored since the gain flag is set to 0 in this example
group_preset_definition.disable_gain_interact:1,1     // gain interaction disable flag for the preset '0'
group_preset_definition.disable_position_interact:1,1 // position interaction disable flag for the preset '0'
group_preset_definition.position_interact:0,0         // initial position interactivity data flag for the preset '0'
// Data corresponding to preset 1
group_preset_definition.reference_id:0,1              // Group IDs referenced in Preset 1
group_preset_definition.cond_on_off:0,1               // ON OFF status of the groups
group_preset_definition.gain_flag:0,0                 // Gain flag status of the groups
group_preset_definition.gain:4,4                      // Gain value - ignored since the gain flag is set to 0 in this example
group_preset_definition.disable_gain_interact:1,1     // gain interaction disable flag for the preset '1'
group_preset_definition.disable_position_interact:1,1 // position interaction disable flag for the preset '1'
group_preset_definition.position_interact:0,0         // initial position interactivity data flag for the preset '1'
```

## 2.5 MAE Data
Data corresponding to different elements can be transmitted through this bitstream block in the ASI. For most of the popular use cases, Group description, Switch group description and Preset Descrition data sets are transmitted in the bit-stream. Though there is no specific order in which these data sets are expected to be present in the encoded bit-stream, for better readability of the ASI information in the form of text, description blocks pertaining to a particular category(group, switch group, preset) are grouped together and presented in the bit-stream.
The following table describes the `data_type` field. This occurs once at the start of the data set block.

| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 | `data_type` | Indicates the type of description that follows. Refer to [table](#data-type) below for more details.| `0` - `15`|

#### Data type

| **MAE data type value** | **Description** |
|------|-----|
| 0 |Group description|
| 1 |Switch group description|
| 2 |Group content information|
| 3 |Composite pair information|
| 4 |Information about local screen size|
| 5 |Group preset description|
| 6 |Extension metadata with DRC user interface information|
| 7 |Extension metadata of the screen size information|
| 8 |Extension metadata of the group preset definition|
| 9 |Loudness compensation information|
| 10-15 | n/a |


### 2.5.1 MAE Group Description
The following table lists the different entries of group description data
| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 | `num_grp_def_decription_blocks` | Number of description blocks to be written into the bit-stream. | `0` - `127` |
| 2 | `grp_def_decription_grp_id` | Group IDs of the blocks corresponding to the description. | `0` - `127` |
| 3 | `num_grp_def_decription_language` | Number of available languages for the description text | `1` - `16` - Maximum of 8 for LC level 4 encoder|
| 4 | `grp_def_decription_languages` | A 3-character code  as  specified  by  ISO 639-2. EXAMPLE: French has 3-character code "fre", English has a 3-character code "eng". | - |
| 5 | `grp_def_decription_data_length` | Length of the group description data that follows.| `1` - `256` |
| 6 | `grp_def_decription_data` | A description of a metadata element group or a switch group, i.e. a string describing the content by a high-level description. The format shall follow UTF-8 according to ISO/IEC 10646. | - |

### 2.5.2 MAE Switch Group Description
The following table lists the different entries of switch group description data
| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 | `num_switch_grp_decription_blocks` | Number of switch group description blocks to be written into the bit-stream. | `0` - `127` |
| 2 | `switch_grp_decription_grp_id` | Switch group IDs of the blocks corresponding to the description. | `0` - `31` |
| 3 | `num_switch_grp_decription_language` | Number of available languages for the description text | `1` - `16` - Maximum of 8 for LC level 4 encoder|
| 4 | `switch_grp_decription_languages` | A 3-character code  as  specified  by  ISO 639-2. EXAMPLE: French has 3-character code "fre", English has a 3-character code "eng". | - |
| 5 | `switch_grp_decription_data_length` | Length of the group description data that follows.| `1` - `256` |
| 6 | `switch_grp_decription_data` | A description of a metadata element group or a switch group, i.e. a string describing the content by a high-level description. The format shall follow UTF-8 according to ISO/IEC 10646. | - |

### 2.5.3 MAE Preset Description
The following table lists the different entries of preset description data
| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 | `num_preset_decription_blocks` | Number of preset description blocks to be written into the bit-stream. | `0` - `127` |
| 2 | `preset_decription_grp_id` | Preset IDs of the blocks corresponding to the description. | `0` - `31` |
| 3 | `num_preset_decription_language` | Number of available languages for the description text | `1` - `16` - Maximum of 8 for LC level 4 encoder|
| 4 | `preset_decription_languages` | A 3-character code  as  specified  by  ISO 639-2. EXAMPLE: French has 3-character code "fre", English has a 3-character code "eng". | - |
| 5 | `preset_decription_data_length` | Length of the group description data that follows.| `1` - `256` |
| 6 | `preset_decription_data` | A description of a metadata element group or a switch group, i.e. a string describing the content by a high-level description. The format shall follow UTF-8 according to ISO/IEC 10646. | - |

### Example Usage:
```
num_data_sets:3                        // Number of data sets
data_type:0,1,5                        // 0-Group Description, 1- Switch Group description, 5-Preset description
num_grp_def_decription_blocks:2        // Number of description blocks
grp_def_decription_grp_id:0,1          // Group IDs referenced by the description block
num_grp_def_decription_language:1,1    // Number of description languages for each block
grp_def_decription_languages:eng,eng   // 3-character code for language description
grp_def_decription_data_length:5,5     // Length of description data
grp_def_decription_data:2.0 A,2.0 B    // Description data
num_switch_grp_decription_blocks:1     // Number of switch group description blocks
switch_grp_decription_grp_id:0         // Switch group IDs referenced by the description block
num_switch_grp_decription_language:1   // Number of description languages for each block
switch_grp_decription_languages:eng    // 3-character code for language description
switch_grp_decription_data_length:9    // Length of description data
switch_grp_decription_data:Selection   // Description data
num_preset_decription_blocks:1         // Number of preset description blocks
preset_decription_grp_id:0             // preset IDs referenced by the description block
num_preset_decription_language:1       // Number of description languages for each block
preset_decription_languages:eng        // 3-character code for language description
preset_decription_data_length:7        // Length of description data
preset_decription_data:Default         // Description data
```

### 2.5.4 MAE Content Data Blocks
The following table lists the different entries of preset description data
| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 | `num_content_data_blocks` | Number of content data blocks. | `0` - `127` |
| 2 | `content_group_id` | Specifies the mae_groupID of the group to which the ContentData block applies.| `0` - `127` |
| 3 | `content_kind` | Defines the kind of content of a metadata element group. Refer [table](#2541-content-kind) below for more details. | `0` - `15`|
| 4 | `has_content_language` | Flag that indicates if the actual metadata element group has a language assigned to its content | `0` or `1` |
| 5 | `content_language` | A 3-character code  as  specified  by  ISO 639-2. EXAMPLE: French has 3-character code "fre", English has a 3-character code "eng". | - |

#### 2.5.4.1 Content kind

| **MAE content kind value** | **Description** |
|------|-----|
| 0 |undefined|
| 1 |complete main|
| 2 |dialogue|
| 3 |music|
| 4 |effect|
| 5 |mixed|
| 6 |LFE|
| 7 |voiceover|
| 8 |spokensubtitle|
| 9 |audiodescription/visually impaired|
| 10 |commentary|
| 11 |hearing impaired|
| 12 |emergency|
| 13-15 | reserved |

The next sections (2.5.5 to 2.5.7) present blocks that are not so common as the blocks presented above.

### 2.5.5 MAE Composite pairs
Composite pairs are a pair of metadata elements. The first element in the pair is considered independent object. The second element in the pair is called dependant object.
The dependant object will be given the metadata of the independent object. Each element may be either a discrete object or an SAOC object.
| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 | `num_comp_pairs` | Number of composite pairs. | `0` - `127` |
| 2 | `element_id` | Element IDs forming the composite pairs.| `0` - `127` |

### 2.5.6 MAE Screen Size
| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 | `has_non_std_screen_size` | This flag specifies whether a nominal screen size is defined that is different than the standard screen size. The definition is done via viewing angles corresponding to the screen edges. In case hasNonStandardScreenSize is zero, the following values are used as default (assuming a 4k display and an optimal viewing distance): \ nominal Azimuth left = 29.0 \ nominal Azimuth right = -29.0 \ nominal Elevation top = 17.5 \ nominal Elevation bottom = -17.5 | `0` or `1` |
| 2 | `screen_size_az` | Defines the azimuth corresponding to the left and right screen edge.| `0` - `180` |
| 3 | `screen_size_el` | Defines the elevation corresponding to the top screen edge.| `0` - `90` |
| 4 | `screen_size_bot_el` | Defines the elevation corresponding to the bottom screen edge.| `0` - `90` |

### 2.5.7 MAE Group Preset Definition Extension
This data block is written to distinguish the references in preset definition as group / switch group IDs.
| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 | `group_preset_ext_has_sw_grp_cond` | Flag that indicates if switch group conditions are present for the preset. As many values as the number of presets in the scene. | `0` or `1` |
| 2 | `group_preset_ext_is_sw_grp_cond` | Indicates whether the condition from original preset definition or a group preset extension references a groupID (0) or if the referenced ID shall be interpreted as a switchGroupID (1). | `0` or 1` |
| 3 | `group_preset_ext_has_downmix_id` | Indicates whether a group preset has layout-dependent extensions. | `0` or `1` |
| 4 | `num_downmix_id_group_preset_ext` | Number of downmix extensions available for the preset. | `1` - `16` (restriction for level 4) |
| 5 | `group_preset_downmix_id` |References a downmixId, for which the current group preset extension is applicable.| `0` - `127` |
| 6 | `group_preset_ext_num_conditions` |Indicates number of group conditions that are associated with a group preset.| `0` - `31`|
| 7 | `group_preset_ext_disable_gain_interact` |Indicates whether the gain interactivity of the currently referenced group or of the members of the referenced switch group shall be disabled (flag is equal to 1) or shall stay enabled (flag is equal to 0) if the preset or preset extension is chosen/valid.|`0` or `1`|
| 8 | `group_preset_ext_gain_flag` | Indicates whether the gain of the currently referenced group or of the members of the referenced switch group is present (flag is equal to 1).|`0` or `1`|
| 9 | `group_preset_ext_gain` | Indicates the initial gain of the members of a metadata element group or of the element members of the group members of the referenced switch group when the corresponding preset or preset extension is selected. |`0` - `255`|
| 10 | `group_preset_ext_disable_pos_interact` | Indicates whether the position interactivity of the currently referenced group or of the members of the referenced switch group shall be disabled (flag is equal to 1) or shall stay enabled (flag is equal to 0) if the preset or preset extension is chosen/valid. | `0` - `15` (each step for 3dB) |
| 11 | `group_preset_ext_pos_flag` | Indicates whether the position info of the currently referenced group or of the members of the referenced switch group is present (flag is equal to 1). |`0` or `1`|
| 12 | `group_preset_ext_az_offset` | Indicates additional azimuth offset that shall be applied to the currently referenced group or the members of the referenced switch group if the preset or preset extension is chosen/valid. | `-180` - `180`.|
| 13 | `group_preset_ext_el_offset` | Indicates additional elevation offset that shall be applied to the currently referenced group or the members of the referenced switch group if the preset or preset extension is chosen/valid. | `-90` - `90`.|
| 14 | `group_preset_ext_dist_fact` | Indicates additional distance change factor that shall be applied to the currently referenced group or the members of the referenced switch group if the preset or preset extension is chosen/valid. | `0` - `15`.|


### 2.5.8 MAE Loudness Compensation Data
| **Sl. No.** | **Text File Entry** | **Description** | **Allowed values** |
|-----|------|------|-----|
| 1 | `grp_loudness` | loudness value for the current metadata element group (groupID). Actual loudness is calculated as loudness = 0.25 * grp_loudness - 57.75 | `0` - `255`  |
| 2 | `defaultlt_param_present` |Indicates whether loudness compensation parameters for the default scene.| `0` or 1` |
| 3 | `default_include_group` | Indicates whether the current metadata element group (groupID) shall be incorporated in the computation of the loudness compensation gain of the default scene is present. As many entries as the number of groups.| `0` or `1` |
| 4 | `default_min_max_gain_present` | Indicates whether min/max values for loudness compensation gain of the default scene is present.| `0` or `1` |
| 5 | `default_min_gain` | Indicates a minimum value for the loudness compensation gain of the default scene. Can take values between 0 and minus 42 dB in 3 dB steps.| `0` - `15` (each step for 3dB) |
| 6 | `default_max_gain` | Indicates a maximum value for the loudness compensation gain of the default scene. Can take values between 0 and minus 42 dB in 3 dB steps.| `0` - `15` (each step for 3dB) |
| 7 | `preset_param_present` |  Indicates whether loudness compensation parameters for the current preset is present. As many entries as the number of presets. | `0` or `1` |
| 8 | `preset_include_group` | Indicates whether the current metadata element group (groupID) shall be incorporated in the computation of the loudness compensation gain of the current preset (groupPresetID). | `0` or `1`|
| 9 | `preset_min_max_present` | Indicates whether min/max values for loudness compensation gain of the current preset (groupPresetID) | `0` or `1`.|
| 10 | `preset_min_gain` | Indicates a minimum value for the loudness compensation gain of the specific preset. Can take values between 0 and minus 42 dB in 3 dB steps.| `0` - `15` (each step for 3dB) |
| 11 | `preset_max_gain` | Indicates a maximum value for the loudness compensation gain of the specific preset. Can take values between 0 and minus 42 dB in 3 dB steps.| `0` - `15` (each step for 3dB) |
