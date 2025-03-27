/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "event_detector/AnalysisRule.hpp"
#include "event_detector/BufferManager.hpp"
#include "event_detector/common.hpp"

namespace event_detector {

class EventDetector;

/**
 * @brief Component of the Event Detector responsible for identifying relevant data
 *
 * The AnalysisManager analyzes the data buffered in the BufferManager.
 * It is responsible for detecting data-based events.
 * To this end, it evaluates pre-defined rules, implemented by plugins.
 */
class AnalysisManager {
 public:
  /**
   * @brief Creates a new AnalysisManager with pre-defined rules.
   *
   * @param   ed              EventDetector node
   * @param   buffer          BufferManager instance
   * @param   default_period  default analysis interval
   */
  AnalysisManager(EventDetector* ed, std::shared_ptr<BufferManager> buffer, const double default_period);

  /**
   * @brief Toggle analysis on/off for all rules.
   *
   * @param enable_periodic whether to enable periodic analysis
   * @param enable_service whether to enable service-based analysis
   */
  void toggleAnalysis(const bool enable_periodic, const bool enable_service);

 protected:
  /**
   * @brief Loads all enabled rules into the manager.
   *
   * First checks if a rule is enabled by querying the parameter server.
   * Then creates and configures the rule by passing parameters to the rule.
   */
  void loadRules();

  /**
   * @brief Loads a rule into the manager.
   *
   * First, checks if a rule is enabled by querying the parameter server.
   * Then creates and configures the rule by passing parameters to the rule.
   * Last, sets up a periodic callback to run rule evaluation.
   *
   * @param   rule_name  rule name
   */
  void loadRule(const std::string& rule_name);

  /**
   * @brief Toggle periodic analysis for a specific rule.
   *
   * @param rule_name rule name
   * @param enable whether to enable periodic analysis
   */
  void togglePeriodicAnalysis(const std::string& rule_name, const bool enable);

  /**
   * @brief Toggle service-based analysis for a specific rule.
   *
   * @param rule_name rule name
   * @param enable whether to enable service-based analysis
   */
  void toggleAnalysisByService(const std::string& rule_name, const bool enable);

  /**
   * @brief Queries the parameter server to determine whether a rule is enabled.
   *
   * @param   rule_name   name of the rule
   *
   * @return  bool        whether rule is enabled
   */
  bool isRuleEnabled(const std::string& rule_name);

  /**
   * @brief Loads analysis interval for a specific rule from the parameter server.
   *
   * @param   rule_name  name of the rule
   * @return  double     analysis interval
   */
  double loadRuleAnalysisPeriod(const std::string& rule_name);

  /**
   * @brief Evaluates a single rule.
   *
   * The rule is evaluted, data is requested, and then this data is passed to
   * the DatabaseInterface for insertion into the database.
   *
   * @param  rule_name  rule name
   */
  void runAnalysis(const std::string& rule_name);

 protected:
  /**
   * @brief EventDetector node
   */
  EventDetector* ed_;

  /**
   * @brief BufferManager component
   */
  std::shared_ptr<BufferManager> buffer_;

  /**
   * @brief pluginlib class loader for AnalysisRule plugins
  */
  std::shared_ptr<pluginlib::ClassLoader<AnalysisRule>> rule_loader_;

  /**
   * @brief default analysis interval
   */
  double default_period_ = 1.0;

  /**
   * @brief ROS timers triggering rule analysis, mapped to rule name
   */
  std::map<std::string, rclcpp::TimerBase::SharedPtr> analysis_timers_;

  /**
   * @brief pre-defined rules to evaluate, mapped to rule name
   */
  std::map<std::string, std::shared_ptr<AnalysisRule>> rules_;

  /**
   * @brief services that can be used to trigger rule evaluation, mapped to rule name
  */
  std::map<std::string, std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>>> trigger_services_;
};

}  // namespace event_detector
