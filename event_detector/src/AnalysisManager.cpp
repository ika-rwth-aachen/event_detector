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

#include "event_detector/AnalysisManager.hpp"
#include "event_detector/EventDetector.hpp"

#include <algorithm>
#include <iterator>
#include <sstream>


namespace event_detector
{

AnalysisManager::AnalysisManager(EventDetector* ed, std::shared_ptr<BufferManager> buffer, const double default_period)
    : ed_(ed), buffer_(buffer), default_period_(default_period) {
  rule_loader_ =
      std::make_shared<pluginlib::ClassLoader<AnalysisRule>>("event_detector", "event_detector::AnalysisRule");

  this->loadRules();
}

void AnalysisManager::toggleAnalysis(const bool enable_periodic, const bool enable_service) {
  for (const auto& rule : rules_) {
    this->togglePeriodicAnalysis(rule.first, enable_periodic);
  }
  for (const auto& rule : rules_) {
    this->toggleAnalysisByService(rule.first, enable_service);
  }
}

void AnalysisManager::loadRules() {
  std::vector<std::string> rules;
  ed_->get_parameter_or("rules", rules, {});
  for (const auto& rule : rules) loadRule(rule);

  if (rules_.empty()) RCLCPP_ERROR(ed_->get_logger(), "No rules enabled, nothing will be evaluated");
}

void AnalysisManager::loadRule(const std::string& rule_name) {
  // create rule
  if (!this->isRuleEnabled(rule_name)) return;
  try {
    std::shared_ptr<AnalysisRule> rule = rule_loader_->createSharedInstance(rule_name);
    rule->initialize(ed_, buffer_);
    rules_[rule_name] = rule;
  } catch (pluginlib::PluginlibException& exc) {
    RCLCPP_ERROR(ed_->get_logger(), "Failed to load rule: %s", exc.what());
  }

  // load rule parameters
  rules_[rule_name]->loadRuleParameters();
}

void AnalysisManager::togglePeriodicAnalysis(const std::string& rule_name, const bool enable) {
  if (rules_.find(rule_name) == rules_.end()) return;

  if (enable) {
    // Set up timer to trigger periodic rule analysis
    double period = this->loadRuleAnalysisPeriod(rule_name);
    if (period > 0.0) {
      std::function<void()> callback = std::bind(&AnalysisManager::runAnalysis, this, rule_name);

      // Create a timer that uses the node's clock (which respects use_sim_time)
      rclcpp::TimerBase::SharedPtr timer =
          rclcpp::create_timer(ed_->get_node_base_interface(),
                               ed_->get_node_timers_interface(),
                               ed_->get_clock(),
                               rclcpp::Duration::from_seconds(period),
                               std::move(callback),
                               ed_->get_node_base_interface()->get_default_callback_group());

      analysis_timers_[rule_name] = timer;
      RCLCPP_INFO(ed_->get_logger(), "Started periodic execution of rule '%s', will run analysis every %.3fs",
                  rule_name.c_str(), period);
    } else {
      // Stop periodic analysis if the period is non-positive
      auto it = analysis_timers_.find(rule_name);
      if (it != analysis_timers_.end()) {
        analysis_timers_.erase(it);
        RCLCPP_INFO(ed_->get_logger(), "Stopped periodic execution of rule '%s' due to non-positive period",
                    rule_name.c_str());
      }
    }
  } else {
    // Stop the timer if 'enable' is false
    auto it = analysis_timers_.find(rule_name);
    if (it != analysis_timers_.end()) {
      analysis_timers_.erase(it);
      RCLCPP_INFO(ed_->get_logger(), "Stopped periodic execution of rule '%s' as it was disabled", rule_name.c_str());
    }
  }
}

void AnalysisManager::toggleAnalysisByService(const std::string& rule_name, const bool enable) {
  if (rules_.find(rule_name) == rules_.end()) return;

  if (enable) {
    // set up service to trigger rule analysis on demand
    std::string rule_name_non_const = rule_name;
    std::string srv_name = "~/" + rule_name_non_const.replace(rule_name.find("::"), 2, "/") + "/evaluate";
    trigger_services_[rule_name] = ed_->create_service<std_srvs::srv::Empty>(
        srv_name, [this, rule_name](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                    std::shared_ptr<std_srvs::srv::Empty::Response>) { this->runAnalysis(rule_name); });
    RCLCPP_INFO(ed_->get_logger(), "Started service for on-demand execution of rule '%s' at '%s'", rule_name.c_str(),
                srv_name.c_str());
  } else {
    // destroy service
    trigger_services_.erase(rule_name);
    RCLCPP_INFO(ed_->get_logger(), "Stopped service for on-demand execution of rule '%s'", rule_name.c_str());
  }
}

bool AnalysisManager::isRuleEnabled(const std::string& rule_name) {
  const std::string parameter_name = "rule_params." + rule_name + ".enabled";

  bool enabled;
  bool found = ed_->get_parameter_or(parameter_name, enabled, false);
  if (!found) {
    RCLCPP_WARN(ed_->get_logger(), "Rule parameter '%s' not set, disabling", parameter_name.c_str());
  }
  return enabled;
}

double AnalysisManager::loadRuleAnalysisPeriod(const std::string& rule_name) {
  double period;
  ed_->get_parameter_or("rule_params." + rule_name + ".period", period, default_period_);
  return period;
}

void AnalysisManager::runAnalysis(const std::string& rule_name) {
  { /* lock all buffers during rule evaluation */

    // lock writing to all buffers
    buffer_->lockBuffers(false);

    // evaluate rule and gather evaluation result
    auto& rule = rules_[rule_name];
    rclcpp::Time t0 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    rule->evaluate();
    rclcpp::Time t1 = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    RCLCPP_INFO(ed_->get_logger(), "Analyzed rule '%s' in %fs", rule_name.c_str(), (t1 - t0).seconds());

    // unlock writing to all buffers
    buffer_->unlockBuffers(false);
  }
}

}  // namespace event_detector
