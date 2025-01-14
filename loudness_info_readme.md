# 1. Introduction

The libmpeghe encoder sample application has command line support for loudness information. It expects the loudness information to be presented in XML format. Refer to the command line option `-iloudness:`.

The `-iloudness:<path to loudness_file.xml>` argument is an optional argument that allows the user to specify an XML file containing detailed loudness configuration information for each audio element group in the MPEG-H encoded file. This file provides metadata that defines loudness characteristics and can be used along with ASI input file (refer command line option `-iasi:`) for preset related loudness configurations. 

The input file follows a traditional XML file format with tags and XML structure defined in the following section.

# 2. XML Structure

The following XML tree describes how the XML tags should be structured and which tags are mandatory along with their tag value type.

```plaintext
loudness_config
└── loudness (one or more)
    ├── loudness_info_type (int)
    ├── mae_group_id OR mae_group_preset_id (int)
    ├── sample_peak_level (float)
    ├── true_peak_level (float)
    └── measurement (one or more)
        ├── method_val (float)
        ├── method_def (int)
        └── measurement_system (int)
```

The following is an example loudness XML file:

```xml
<loudness_config>
    <loudness>
        <loudness_info_type>1</loudness_info_type>
        <mae_group_id>0</mae_group_id>
        <true_peak_level>-110.5</true_peak_level>
        <measurement>
            <method_val>-10.5</method_val>
            <method_def>1</method_def>
            <measurement_system>2</measurement_system>
        </measurement>
        <measurement>
            <method_val>-30.5</method_val>
            <method_def>1</method_def>
            <measurement_system>3</measurement_system>
        </measurement>
    </loudness>
    <loudness>
        <loudness_info_type>1</loudness_info_type>
        <mae_group_id>1</mae_group_id>
        <sample_peak_level>-100.5</sample_peak_level>
        <measurement>
            <method_val>-20.3</method_val>
            <method_def>1</method_def>
            <measurement_system>2</measurement_system>
        </measurement>
    </loudness>
</loudness_config>
```

Notice that there can be multiple `<loudness>` sections present which will correspond to the total number of loudness info count, which is not required to be mentioned in the XML file. There can also be multiple `<measurement>` blocks inside one `<loudness>` block which means that that particular loudness config has multiple measurements with different methods or measurement systems used.

# 3. Element Descriptions

Here is a brief description of each element in the loudness XML file.

- `loudness_config`: Root element that encapsulates all loudness configurations.
    - `loudness`: Contains loudness information for a specific audio group. Multiple <loudness> elements can be defined.
        - `loudness_info_type`: Specifies the type of loudness information.
        - `mae_group_id`: ID of the signal group to which this loudness information corresponds to.
        - `mae_group_preset_id`: ID of the audio scene (preset) to which this loudness information corresponds to.
        - `sample_peak_level`: Sample peak value in dBFS.
        - `true_peak_value`: True peak value in dBTP.
        - `measurement`: Encapsulates measurement-specific parameters.
            - `method_val`: Loudness value measured using the corresponding method and measurement system.
            - `method_def`: Identifies the measurement method used.
            - `measurement_system`: Identifies the measurement system.

The following table lists further description of each element's tag value.

| **Sl. No.** | **XML Tag Name** | **XML Tag Value** | **Note** |
|-----|------|------|------|
| 1 |`loudness_config` | Encapsulates one or more `loudness` elements | Mandatory tag
| 2 |`loudness` | Encapsulates loudness information for one loudness block. | There can be one or more `loudness` blocks present in one `loudness_config` block. |
| 3 |`loudness_info_type` | Allowed values: `0`, `1` or `2` | If not mentioned, defaults to `0`. Refer to [section 3.1](#31-loudness-info-type) for further details. |
| 4 |`mae_group_id` | Allowed values: `0` to `127` | Mandatory tag when `loudness_info_type` is `1` or `2`  |
| 5 |`mae_group_preset_id` | Allowed values: `0` to `31` | Mandatory tag when `loudness_info_type` is `3` |
| 6 |`sample_peak_level` | Allowed values: `-107.0` to `20.0`. Step size: `0.0312` | Optional tag |
| 7 |`true_peak_level` | Allowed values: `-107.0` to `20.0`. Step size: `0.0312` |  Optional tag
| 8 |`measurement` | Encapsulates measurement information for current loudness block. | There can be one or more `measurement` blocks present in one `loudness` block. |
| 9 |`method_val` | Value of loudness derived from the method definition. Range depends on the measurement system used. Refer to ISO/IEC 23003-4 for further details.| Mandatory tag |
| 10 |`method_def` | Allowed values: `0` to `9` | Mandatory tag. Refer to [section 3.2](#32-method-definition) for further details. |
| 11 |`measurement_system` | Allowed values: `0` to `11` | Mandatory tag. Refer to [section 3.3](#33-measurement-system) for further details. |


## 3.1 Loudness Info Type

The following table describes the allowed values of `loudness_info_type` and the mapping of these values to the type of loudness information described.

| **`loudness_info_type` value** | **Meaning** |
|-----|------|
| `0` | Loudness information for fixed audio scene (default audio scene) |
| `1` | Loudness information for single audio element defined by `mae_group_id` |
| `2` | Loudness information for audio scene which includes a specific `mae_group_id` |
| `3` | Loudness information for audio scene defined by `mae_group_preset_id` |


## 3.2 Method Definition

The following table describes the allowed values of `method_def` and what each value corresponds to.

| **`method_def` value** | **`method_val` type** | **Meaning** |
|-----|------|------|
| `0` | N/A | Unknown/other |
| `1` | Loudness | Program Loudness (programLoudness as defined in ISO/IEC 23091-3) |
| `2` | Loudness | Anchor Loudness (anchorLoudness as defined in ISO/IEC 23091-3) |
| `3` | Loudness | Maximum of the range, i.e. the 95th percentile of the loudness distribution according to EBU R-128 |
| `4` | Loudness | Maximum momentary loudness, measured using a 0.4s window according to ITU-R BS.1771–1 or EBU R-128 |
| `5` | Loudness | Maximum short-term loudness, measured using a 3s window according to ITU-R BS.1771–1 or EBU R-128 |
| `6` | Loudness range | Loudness range derived from EBU R-128 |
| `7` | Sound pressure level | Production mixing level measured |
| `8` | Index | Production room type |
| `9` | Loudness | Short-term loudness, measured using a 3s window according to ITU-R BS.1771–1 or EBU R-128, The 3s window shall include the current DRC frame. |

## 3.3 Measurement System

The following table describes the allowed values of `measurement_system` and what each value corresponds to.

| **`measurement_system` value** | **Meaning** |
|-----|------|
| `0` | Unknown/other |
| `1` | EBU R-128 |
| `2` | ITU-R BS.1770–4 |
| `3` | ITU-R BS.1770–4 with pre-processing. The pre-processor is a 4th order Linkwitz-Riley filter with a cutoff frequency of 500 Hz. |
| `4` | User |
| `5` | Expert/panel |
| `6` | ITU-R BS.1771–1 |
| `7` | Reserved Measurement System A (RMS_A) |
| `8` | Reserved Measurement System B (RMS_B) |
| `9` | Reserved Measurement System C (RMS_C) |
| `10` | Reserved Measurement System D (RMS_D) |
| `11` | Reserved Measurement System E (RMS_E) |

# 4. Example Cases

The following list showcases examples for each `loudness_info_type`.

## 4.1 `loudness_info_type` - 0

In this case, a loudness value of `-20.3` will be applied to the entire default audio scene. Method definition value `1` corresponds to program loudness which indicates the overall loudness of the corresponding audio program. `measurement_system` having value `2` means that the method value is measured using the ITU-R BS.1770–4 standard. Since the loudness applies to the entire audio scene, there is no need of mentioning any ASI information such as the `mae_group_id` or `mae_group_preset_id`.

```xml
<loudness_config>
    <loudness>
        <loudness_info_type>0</loudness_info_type>
        <measurement>
            <method_val>-20.3</method_val>
            <method_def>1</method_def>
            <measurement_system>2</measurement_system>
        </measurement>
    </loudness>
</loudness_config>
```

## 4.2 `loudness_info_type` - 1

In this case, there are two loudness information blocks each corresponding to a different signal group, indicated by the IDs `0` and `1`. Assuming there are no presets or switch groups involved, the decoder will select the loudness information corresponding to the maximum group ID among the available groupd IDs. So in this case, the maximum group ID is `1` and the overall loudness of the program would be set to `-30.5`. 

```xml
<loudness_config>
    <loudness>
        <loudness_info_type>1</loudness_info_type>
        <mae_group_id>0</mae_group_id>
        <measurement>
            <method_val>-20.3</method_val>
            <method_def>1</method_def>
            <measurement_system>2</measurement_system>
        </measurement>
    </loudness>
    <loudness>
        <loudness_info_type>1</loudness_info_type>
        <mae_group_id>1</mae_group_id>
        <measurement>
            <method_val>-30.5</method_val>
            <method_def>1</method_def>
            <measurement_system>2</measurement_system>
        </measurement>
    </loudness>
</loudness_config>
```

## 4.3 `loudness_info_type` - 2

In this case, there are two loudness information blocks each corresponding to a different signal group, indicated by the IDs `0` and `1`. Consider a case where the ASI has two presets involved with IDs `10` and `20`, and preset `10` selects signal group `0` and preset `20` selects signal group `1`. In this case, the first loudness information will be applied to preset `10` with a method value of `-20.3` and the second loudness information will be applied to preset `20` with a method value of `-30.5`.

```xml
<loudness_config>
    <loudness>
        <loudness_info_type>2</loudness_info_type>
        <mae_group_id>0</mae_group_id>
        <measurement>
            <method_val>-20.3</method_val>
            <method_def>1</method_def>
            <measurement_system>2</measurement_system>
        </measurement>
    </loudness>
    <loudness>
        <loudness_info_type>2</loudness_info_type>
        <mae_group_id>1</mae_group_id>
        <measurement>
            <method_val>-30.5</method_val>
            <method_def>1</method_def>
            <measurement_system>2</measurement_system>
        </measurement>
    </loudness>
</loudness_config>
```

## 4.4 `loudness_info_type` - 3

In this case, there are two loudness information blocks each corresponding to a different presets, indicated by the IDs `10` and `20`. In this case, the first loudness information will be applied to preset `10` with a method value of `-20.3` and the second loudness information will be applied to preset `20` with a method value of `-30.5`.

```xml
<loudness_config>
    <loudness>
        <loudness_info_type>3</loudness_info_type>
        <mae_group_preset_id>10</mae_group_preset_id>
        <measurement>
            <method_val>-20.3</method_val>
            <method_def>1</method_def>
            <measurement_system>2</measurement_system>
        </measurement>
    </loudness>
    <loudness>
        <loudness_info_type>3</loudness_info_type>
        <mae_group_preset_id>20</mae_group_preset_id>
        <measurement>
            <method_val>-30.5</method_val>
            <method_def>1</method_def>
            <measurement_system>2</measurement_system>
        </measurement>
    </loudness>
</loudness_config>
```