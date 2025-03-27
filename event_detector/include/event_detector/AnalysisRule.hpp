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

#include "event_detector/BufferManager.hpp"
#include "event_detector/common.hpp"

namespace event_detector {

class EventDetector;

/**
 * @brief Abstract class defining a rule for identifying relevant data
 */
class AnalysisRule {
 public:
  /**
   * @brief Creates a new AnalysisRule.
   *
   * Initialization takes place in the initialize() method.
  */
  AnalysisRule();

  /**
   * @brief Defines the rule's name.
   *
   * Must be overridden by derived rules. Should match the plugin's name.
   *
   * @return std::string rule name
   */
  virtual std::string getRuleName() const = 0;

  /**
   * @brief Initializes the rule.
   *
   * Called by the AnalysisManager when the rule is loaded into the manager.
   * Constructor is not allowed to take arguments.
   *
   * @param ed      EventDetector node
   * @param buffer  BufferManager component
   */
  void initialize(EventDetector* ed, std::shared_ptr<BufferManager> buffer);

  /**
   * @brief Sets the rule's parameters.
   *
   * Must be overridden by derived rules. Will be automatically called by the
   * AnalysisManager when the rule is loaded into the manager.
   */
  virtual void loadRuleParameters() = 0;

  /**
   * @brief Evaluates the rule.
   *
   * Must be overridden by derived rules. Will be automatically called by the
   * AnalysisManager as part of rule evaluation.
   */
  virtual void evaluate() = 0;

 protected:
  /**
   * @brief Callback for when the rule is initialized.
   *
   * Must be overridden by derived rules. Will be automatically called during
   * initialization. Allows derived rules to perform additional initialization.
   */
  virtual void onInitialize() = 0;

  /**
   * @brief Loads a rule parameter.
   *
   * @param parameter_name parameter name
   *
   * @return rclcpp::Parameter parameter value
   */
  rclcpp::Parameter loadRuleParameter(const std::string& parameter_name);

  /**
   * @brief Loads a rule parameter if set, else uses default value.
   *
   * @param parameter_name parameter name
   * @param out_value      variable to store parameter value
   * @param default_value  default value to use if parameter is not set
   *
   * @return true if parameter was set, false otherwise
   */
  template <typename T>
  bool loadRuleParameter(const std::string& parameter_name, T& out_value, const T& default_value);

 protected:
  /**
   * @brief EventDetector node
   */
  EventDetector* ed_;

  /**
   * @brief BufferManager component
   */
  std::shared_ptr<BufferManager> buffer_;
};

}  // namespace event_detector

#include "event_detector/AnalysisRule.tpp"