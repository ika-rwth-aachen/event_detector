# *Event Detector —* A Modular Event Detection Framework for ROS 2

<p align="center">
  <!--<img src="https://img.shields.io/github/v/release/ika-rwth-aachen/event_detector"/>-->
  <img src="https://img.shields.io/github/license/ika-rwth-aachen/event_detector"/>
  <!--<a href="https://github.com/ika-rwth-aachen/event_detector/actions/workflows/docker-ros.yml"><img src="https://github.com/ika-rwth-aachen/event_detector/actions/workflows/docker-ros.yml/badge.svg"/></a>-->
  <a href="https://ika-rwth-aachen.github.io/event_detector/"><img src="https://github.com/ika-rwth-aachen/event_detector/actions/workflows/doc.yml/badge.svg"/></a>
  <img src="https://img.shields.io/badge/ROS 2-jazzy-293754"/>
  <a href="https://github.com/ika-rwth-aachen/event_detector"><img src="https://img.shields.io/github/stars/ika-rwth-aachen/event_detector?style=social"/></a>
</p>

<p align="center">
⸻ <b><i><a href="#quick-start">Quick Start</a></i></b> | <b><i><a href="#action-plugins">Action Plugins</a></i></b> |  <b><i><a href="#installation">Installation</a></i></b> | <b><i><a href="#documentation">Documentation</a></i></b> | <b><i><a href="#research-article">Research Article</a></i></b> | <b><i><a href="#acknowledgements">Acknowledgements</a></i></b> ⸻
</p>

The ***event detector*** is a generic and modular event detection framework for ROS 2 applications. With the event detector, you can automatically ...
- ***buffer*** ROS messages of arbitrary message type;
- ***detect*** events by analyzing the buffer contents with custom developer-defined ***analysis rules***;
- ***trigger*** custom developer-defined ***actions*** in response to detected events using modular ***action plugins***.

In order to give an example, with the event detector you can automatically ...
- buffer robot sensor data;
- detect malfunction identified by some quantity exceeding a certain threshold;
- trigger the storage of all buffered data leading up to the incident.

The example illustrates that the event detector framework is highly modular and customizable.
- You can process data of arbitrary ROS message type, including custom type definitions.
- You can use any of the existing *analysis rules* to detect an event (e.g., *value exceeding threshold*) or define your own logic to detect events by implementing a single C++ function in a custom *analysis rule*.
- You can trigger an *action* using any of the existing [*action plugins*](#action-plugins) (e.g., for recording a ROS bag) or you can define your own logic to trigger something by implementing a custom *action plugin*.

> [!IMPORTANT]  
> This repository is open-sourced and maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/).  
> **Advanced C-ITS Use Cases** are one of many research topics within our [*Vehicle Intelligence & Automated Driving*](https://www.ika.rwth-aachen.de/en/competences/fields-of-research/vehicle-intelligence-automated-driving.html) domain.  
> If you would like to learn more about how we can support your advanced driver assistance and automated driving efforts, feel free to reach out to us!  
> :email: ***opensource@ika.rwth-aachen.de***


## Quick Start

> [!TIP]
> Check out the [examples repository](https://github.com/ika-rwth-aachen/event_detector_examples) to see the event detector and all of its action plugins in action!  
> The examples also give a good idea of potential use cases for the event detector.


## Action Plugins

Event detector action plugins implement the resulting actions that should be triggered upon the detection of a specific event. The detection of any specific event is closely tied to the action that should be triggered. For this reason, any developer-defined custom analysis rule for event detection is associated with a specific action plugin.

As part of the event detector framework, we provide an initial set of action plugins that cover common uses cases, all listed in the table below. Nevertheless, we highlight that the event detector is designed to be easily extensible with new action plugins or new developer-defined analysis rules.

| Common Name | Action Plugin | Purpose |
| --- | --- | --- |
| [Database Recording](https://github.com/ika-rwth-aachen/event_detector_db_recording) | `event_detector_db_recording` | write data from buffer to a database |
| ROS Bag Recording | *work in progress* | write data from buffer to ROS bag file |
| Recording Trigger | *work in progress* | trigger a (remote) data recording |
| Kubernetes Operator | *work in progress* | request deployment of applications in Kubernetes |

<img src="./assets/architecture.png" alt="architecture" width="600"/>


## Installation

> [!WARNING]
> The core event detector implemented in this repository is not supposed to be used by itself. Please check out the dedicated instructions for the respective [action plugin](#action-plugins) you are interested in.

You can integrate the event detector into your existing ROS 2 workspace by cloning the repository, installing all dependencies using [*rosdep*](http://wiki.ros.org/rosdep), and then building the event detector from source.

```bash
# ROS workspace$
git clone https://github.com/ika-rwth-aachen/event_detector.git src/event_detector
rosdep install -r --ignore-src --from-paths src/event_detector
colcon build --packages-up-to event_detector --cmake-args -DCMAKE_BUILD_TYPE=Release
```


## Documentation

### Code Documentation

Browsable Doxygen code documentation is available [here](https://ika-rwth-aachen.github.io/event_detector).

### Topics, Services, Parameters

<details><summary><i>Click to show</i></summary>

#### Subscribed Topics

Subscriptions for buffering data are specified via the parameter [`client_params.<CLIENT_NAME>.data_type_params.<DATA_TYPE>.topics`](#parameters). Action plugins may define additional subscribers.

#### Published Topics

None by default. Action plugins may define additional publishers.

#### Services

| Service | Type | Description |
| --- | --- | --- |
| `~/<RULE/NAME>/evaluate` | `std_srvs/srv/Empty` | allows to trigger a rule evaluation on-demand |

#### Parameters

| Parameter | Type | Default | Description | Options |
| --- | --- | --- | --- | --- |
| `use_sim_time` | `bool` | `false` | whether to use ROS simulation time |  |
| `startup_state` | `int` | `1` | initial lifecycle state | `1` (unconfigured), `2` (inactive), `3` (active) |
| `buffer.default_time` | `float` | `10.0` | default buffer length (in seconds) |  |
| `buffer.default_queue_size` | `int` | `20` | default subscriber queue size |  |
| `buffer.initialization_bags` | `string[]` | `[]` | ROS bag files to initialize buffer with |  |
| `buffer.use_msg_stamp` | `bool` | `true` | whether to use message stamp for sorting, if present |  |
| `analysis.default_period` | `float` | `1.0` | default period (in seconds) between subsequent analysis rule evaluations |  |
| `clients` | `string[]` | `[]` | client names |  |
| `client_params.<CLIENT_NAME>.base_frame` | `string` | `""` | client base frame used for storing static transforms |  |
| `client_params.<CLIENT_NAME>.tf_prefix` | `string` | `""` | client-specific prefix of frames found in dynamic transforms |  |
| `client_params.<CLIENT_NAME>.data_types` | `string[]` | `[]` | data types to subscribe | see second column in [`datatypes.macro`](./event_detector/include/event_detector/datatypes.macro) |
| `client_params.<CLIENT_NAME>.data_type_params.<DATA_TYPE>.topics` | `string[]` | `[]` | topic names |  |
| `client_params.<CLIENT_NAME>.data_type_params.<DATA_TYPE>.buffer_times` | `float[]` | `[]` | buffer length (in seconds) for each topic |  |
| `client_params.<CLIENT_NAME>.data_type_params.<DATA_TYPE>.queue_sizes` | `int[]` | `[]` | queue size for each topic |  |
| `rules` | `string[]` | `[]` | rule names |  |
| `rule_params.<RULE_NAME>.enabled` | `bool` | `false` | whether rule is enabled |  |
| `rule_params.<RULE_NAME>.parameters` | `dict` | `{}` | custom rule parameters | see rule documentation |

</details>

### Supported data types

<details><summary><i>Click to show</i></summary>

The event detector natively supports most common ROS message types, see the list of supported message packages below. You can also add support for custom data types, see [*How to add support for a new data type*](#how-to-add-support-for-a-new-data-type).
- [builtin_interfaces](https://index.ros.org/p/builtin_interfaces/)
- [diagnostic_msgs](https://index.ros.org/p/diagnostic_msgs/)
- [etsi_its_cam_msgs](https://index.ros.org/p/etsi_its_cam_msgs)
- [etsi_its_cam_ts_msgs](https://index.ros.org/p/etsi_its_cam_ts_msgs)
- [etsi_its_cpm_ts_msgs](https://index.ros.org/p/etsi_its_cpm_ts_msgs)
- [etsi_its_denm_msgs](https://index.ros.org/p/etsi_its_denm_msgs)
- [etsi_its_mapem_ts_msgs](https://index.ros.org/p/etsi_its_mapem_ts_msgs)
- [etsi_its_spatem_ts_msgs](https://index.ros.org/p/etsi_its_spatem_ts_msgs)
- [etsi_its_vam_ts_msgs](https://index.ros.org/p/etsi_its_vam_ts_msgs)
- [geometry_msgs](https://index.ros.org/p/geometry_msgs)
- [nav_msgs](https://index.ros.org/p/nav_msgs)
- [perception_msgs](https://github.com/ika-rwth-aachen/perception_interfaces)
- [sensor_msgs](https://index.ros.org/p/sensor_msgs)
- [shape_msgs](https://index.ros.org/p/shape_msgs)
- [std_msgs](https://index.ros.org/p/std_msgs)
- [stereo_msgs](https://index.ros.org/p/stereo_msgs)
- [tf2_msgs](https://index.ros.org/p/tf2_msgs)
- [trajectory_msgs](https://index.ros.org/p/trajectory_msgs)
- [vision_msgs](https://index.ros.org/p/vision_msgs)
- [visualization_msgs](https://index.ros.org/p/visualization_msgs)

</details>

### How to add support for a new data type

<details><summary><i>Click to show</i></summary>

The event detector natively supports most common ROS message types, see [*Supported data types*](#supported-data-types). Through clever application of preprocessor macros, adding support for a new ROS message type only requires 6 lines of code.
1. Add a package dependency on your message package in [`./event_detector/package.xml`](./event_detector/package.xml).
2. Add your message package to the `find_package`, `ament_target_dependencies`, and `ament_export_dependencies` sections of [`./event_detector/CMakeLists.txt`](./event_detector/CMakeLists.txt).
3. Include the required message type headers in [`./event_detector/include/event_detector/datatypes.hpp`](./event_detector/include/event_detector/datatypes.hpp).
4. Define a new data type by calling the `DATATYPE(TYPE, VAR)` macro for each new message type in [`./event_detector/include/event_detector/datatypes.macro`](./event_detector/include/event_detector/datatypes.macro).

</details>

### How to implement a custom analysis rule

<details><summary><i>Click to show</i></summary>

The detection of any specific event is closely tied to the action that should be triggered. For this reason, any developer-defined custom analysis rule for event detection is associated with a specific action plugin. If the [supported action plugins](#action-plugins) do not work for you, see [*How to implement a custom action plugin*](#how-to-implement-a-custom-action-plugin).

The steps below describe how to add a custom analysis rule to an existing action plugin, e.g., the [database recording plugin](https://github.com/ika-rwth-aachen/event_detector_db_recording). The steps listed here are generally applicable to any action plugin, but specific action plugins may require plugin-specific steps that are documented in the dedicated plugin repositories.
1. Create a new header (`.hpp`) and a new implementation file (`.cpp`) for the new analysis rule, e.g., `src/rules/new_custom_rule/NewCustomRule.cpp`. Usually, existing action plugins provide a `TemplateRule` (e.g., [here, for the database recording plugin](https://github.com/ika-rwth-aachen/event_detector_db_recording/blob/main/event_detector_db_recording_plugin/include/event_detector_db_recording_plugin/rules/template)) that you can copy-paste.
2. Add the new implementation file to the build process in the plugin's `CMakeLists.txt`.
    ```cmake
    add_library(${TARGET_NAME} SHARED
      # ...
      src/rules/new_custom_rule/NewCustomRule.cpp
    )
    ```
3. Declare a new plugin class in the plugin's `plugins.xml`.
    ```xml
    <library path="<ACTION_PLUGIN>">
      <!-- ... -->
      <class type="<ACTION_PLUGIN>::NewCustomRule" base_class_type="event_detector::AnalysisRule" />
    </library>
    ```
4. Add a new rule-specific section to the plugin's parameter file `config/params.yml`, where you can enable the new analysis rule and specify parameters.
    ```yml
    # ...
    rules:
      - <ACTION_PLUGIN>::NewCustomRule
    rule_params:
      enabled: true
      parameters:
        new_custom_param: 3.14
    ```
5. Implement the new custom analysis rule. Follow any plugin-specific instructions and follow the plugin's template rule documentation, if existing.
6. Build the action plugin.
    ```bash
    # ROS workspace$
    colcon build --packages-up-to <ACTION_PLUGIN_PKG>
    ```

</details>

### How to implement a custom action plugin

<details><summary><i>Click to show</i></summary>

The event detector action plugin architecture is built with [ROS 2's `pluginlib` mechanism](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Pluginlib.html). All action plugins are separate ROS packages that build the action plugins as shared libraries, which are loaded by the core event detector node at runtime.

The steps below describe how to create a custom action plugin.
1. Create a new ROS package, e.g., `event_detector_custom_action_plugin`, with dependencies on `event_detector` and `pluginlib` in its `package.xml`.
    ```xml
    <!-- ... -->
    <depend>event_detector</depend>
    <depend>pluginlib</depend>
    <!-- ... -->
    ```
2. Create a new header (`.hpp`) and a new implementation file (`.cpp`) for a new analysis rule, e.g., `src/rules/custom_action_plugin_rule/CustomActionPluginRule.cpp`.
3. Declare a new plugin class in a new file `plugins.xml` on the package's top level.
    ```xml
    <library path="event_detector_custom_action_plugin">
      <class type="event_detector_custom_action_plugin::CustomActionPluginRule" base_class_type="event_detector::AnalysisRule" />
    </library>
    ```
4. Set up the package's `CMakeLists.txt` to build a shared library that is exported as a plugin.
    ```cmake
    project(event_detector_custom_action_plugin)
    # ...
    set(TARGET_NAME ${PROJECT_NAME})
    add_library(${TARGET_NAME} SHARED
      src/rules/custom_action_plugin_rule/CustomActionPluginRule.cpp
    )
    # ...
    ament_target_dependencies(${TARGET_NAME}
      event_detector
      pluginlib
      # ...
    )
    # ...
    pluginlib_export_plugin_description_file(event_detector plugins.xml)
    # ...
    ```
5. Add a new rule-specific section to the plugin's parameter file `config/params.yml`, where you can enable the new analysis rule and specify parameters.
    ```yml
    # ...
    rules:
      - event_detector_custom_action_plugin::CustomActionPluginRule
    rule_params:
      enabled: true
      parameters:
        new_custom_param: 3.14
    ```
6. Implement the action plugin logic in the core analysis rule. The analysis rule has to inherit `event_detector::AnalysisRule` and override a set of abstract functions. A minimal implementation is given below. The most relevant function to implement is `CustomActionPluginRule::evaluate()`, which is periodically called as part of rule evaluation. For reference, check out the documentation of the [core abstract `AnalysisRule` class](./event_detector/include/event_detector/AnalysisRule.hpp).
    ```cpp
    // include/event_detector_custom_action_plugin/rules/custom_action_plugin_rule/CustomActionPluginRule.hpp
    #pragma once
    #include <event_detector/AnalysisRule.hpp>
    namespace event_detector_custom_action_plugin {
    class CustomActionPluginRule : public event_detector::AnalysisRule {
     public:
      std::string getRuleName() const override;
      void loadRuleParameters() override;
      void onInitialize() override;
     protected:
      void evaluate() override;
    }
    }
    ```
    ```cpp
    // src/rules/custom_action_plugin_rule/CustomActionPluginRule.cpp
    #include <event_detector_custom_action_plugin/rules/custom_action_plugin_rule/CustomActionPluginRule.hpp>
    #include <pluginlib/class_list_macros.hpp>
    PLUGINLIB_EXPORT_CLASS(event_detector_custom_action_plugin::CustomActionPluginRule,
                           event_detector::AnalysisRule)
    namespace event_detector_custom_action_plugin {
    std::string TemplateRule::getRuleName() const {
      return "event_detector_custom_action_plugin::CustomActionPluginRule";
    }
    // ...
    }
    ```
7. Build the action plugin.
    ```bash
    # ROS workspace$
    colcon build --packages-up-to event_detector_custom_action_plugin
    ```

</details>


## Research Article

If you are interested in the role of event detection in modern automated driving systems and Cooperative Intelligent Transport Systems (C-ITS), please check out our associated research article on the topic and consider citing it if you are using the event detector for your own research.

> **Event Detection in C-ITS: Classification, Use Cases, and Reference Implementation**  
> *([ResearchGate](https://www.researchgate.net/publication/388993060_Event_Detection_in_C-ITS_Classification_Use_Cases_and_Reference_Implementation))*  
>
> [Lennart Reiher](https://www.ika.rwth-aachen.de/en/institute/team/vehicle-intelligence-automated-driving/reiher.html), [Bastian Lampe](https://www.ika.rwth-aachen.de/en/institute/team/vehicle-intelligence-automated-driving/lampe.html), [Lukas Zanger](https://www.ika.rwth-aachen.de/en/institute/team/vehicle-intelligence-automated-driving/zanger.html), [Timo Woopen](https://www.ika.rwth-aachen.de/de/institut/team/fahrzeugintelligenz-automatisiertes-fahren/woopen.html), [Lutz Eckstein](https://www.ika.rwth-aachen.de/en/institute/team/univ-prof-dr-ing-lutz-eckstein.html)  
> [Institute for Automotive Engineering (ika), RWTH Aachen University](https://www.ika.rwth-aachen.de/en/)
>
> <sup>*Abstract* – The transition from traditional hardware-centric vehicles to software-defined vehicles is largely driven by a switch to modern architectural patterns of software, including service orientation and microservices. Automated driving systems (ADS), and even more so, Cooperative Intelligent Transport Systems (C-ITS), come with requirements for scalability, modularity, and adaptability that cannot be met with conventional software architectures. The complexity and dynamics of future mobility systems also suggest to employ ideas of the event-driven architecture paradigm: distributed systems need to be able to detect and respond to events in real-time and in an asynchronous manner. In this paper, we therefore stress the importance of data-driven event detection in the context of ADS and C-ITS. First, we propose a classification scheme for event-detection use cases. We then describe a diverse set of possible use cases and apply the classification scheme to a selection of concrete, innovative examples. Last, we present a modular event detection software framework that we publish as open-source software to foster further research and development of complex C-ITS use cases, but also for robotics in general.</sup>


## Acknowledgements

This work is accomplished within the projects *6GEM* (FKZ 16KISK036K), *autotech.agil* (FKZ 01IS22088A), and *UNICAR.agil* (FKZ 16EMO0284K). We acknowledge the financial support for the projects by the *Federal Ministry of Education and Research of Germany (BMBF)*.
