// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DIAGNOSTIC_UPDATER__PUBLISHER_HPP_
#define DIAGNOSTIC_UPDATER__PUBLISHER_HPP_

#include <diagnostic_updater/update_functions.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <string>
#include <memory>

namespace diagnostic_updater {

/**
 * \brief A class to facilitate making diagnostics for a topic using a
 * FrequencyStatus.
 *
 * The word "headerless" in the class name refers to the fact that it is
 * mainly designed for use with messages that do not have a header, and
 * that cannot therefore be checked using a TimeStampStatus.
 */

class HeaderlessTopicDiagnostic : public CompositeDiagnosticTask {
public:
  /**
   * \brief Constructs a HeaderlessTopicDiagnostic.
   *
   * \param name The name of the topic that is being diagnosed.
   *
   * \param diag The diagnostic_updater that the CompositeDiagnosticTask
   * should add itself to.
   *
   * \param freq The parameters for the FrequencyStatus class that will be
   * computing statistics.
   */

  HeaderlessTopicDiagnostic(
      std::string name, diagnostic_updater::Updater &diag,
      const diagnostic_updater::FrequencyStatusParam &freq)
      : CompositeDiagnosticTask(name + " topic status"), freq_(freq) {
    addTask(&freq_);
    diag.add(*this);
  }

  virtual ~HeaderlessTopicDiagnostic() {}

  /**
         * \brief Signals that a publication has occurred.
         */

  virtual void tick() { freq_.tick(); }

  /**
         * \brief Clears the frequency statistics.
         */

  virtual void clear_window() { freq_.clear(); }

private:
  diagnostic_updater::FrequencyStatus freq_;
};

/**
 * \brief A class to facilitate making diagnostics for a topic using a
 * FrequencyStatus and TimeStampStatus.
 */

class TopicDiagnostic : public HeaderlessTopicDiagnostic {
public:
  /**
   * \brief Constructs a TopicDiagnostic.
   *
   * \param name The name of the topic that is being diagnosed.
   *
   * \param diag The diagnostic_updater that the CompositeDiagnosticTask
   * should add itself to.
   *
   * \param freq The parameters for the FrequencyStatus class that will be
   * computing statistics.
   *
   * \param stamp The parameters for the TimeStampStatus class that will be
   * computing statistics.
   */

  TopicDiagnostic(std::string name, diagnostic_updater::Updater &diag,
                  const diagnostic_updater::FrequencyStatusParam &freq,
                  const diagnostic_updater::TimeStampStatusParam &stamp)
      : HeaderlessTopicDiagnostic(name, diag, freq), stamp_(stamp) {
    addTask(&stamp_);
  }

  virtual ~TopicDiagnostic() {}

  /**
         * This method should never be called on a TopicDiagnostic as a
   * timestamp
         * is needed to collect the timestamp diagnostics. It is defined here to
         * prevent the inherited tick method from being used accidentally.
         */
  virtual void tick() {
    // ROS_FATAL("tick(void) has been called on a TopicDiagnostic. This is never
    // correct. Use tick(rclcpp::Time &) instead.");
  }

  /**
         * \brief Collects statistics and publishes the message.
         *
         * \param stamp Timestamp to use for interval computation by the
         * TimeStampStatus class.
         */
  virtual void tick(const rclcpp::Time &stamp) {
    stamp_.tick(stamp);
    HeaderlessTopicDiagnostic::tick();
  }

private:
  TimeStampStatus stamp_;
};

/**
 * \brief A TopicDiagnostic combined with a ros::Publisher.
 *
 * For a standard ros::Publisher, this class allows the ros::Publisher and
 * the TopicDiagnostic to be combined for added convenience.
 */

template <class T> class DiagnosedPublisher : public TopicDiagnostic {
public:
  /**
   * \brief Constructs a DiagnosedPublisher.
   *
   * \param pub The publisher on which statistics are being collected.
   *
   * \param diag The diagnostic_updater that the CompositeDiagnosticTask
   * should add itself to.
   *
   * \param freq The parameters for the FrequencyStatus class that will be
   * computing statistics.
   *
   * \param stamp The parameters for the TimeStampStatus class that will be
   * computing statistics.
   */

  DiagnosedPublisher(const rclcpp::Publisher<
                         diagnostic_msgs::msg::DiagnosticArray>::SharedPtr &pub,
                     diagnostic_updater::Updater &diag,
                     const diagnostic_updater::FrequencyStatusParam &freq,
                     const diagnostic_updater::TimeStampStatusParam &stamp)
      : TopicDiagnostic(pub->get_topic_name(), diag, freq, stamp),
        publisher_(pub) {}

  virtual ~DiagnosedPublisher() {}

  /**
         * \brief Collects statistics and publishes the message.
         *
         * The timestamp to be used by the TimeStampStatus class will be
         * extracted from message.header.stamp.
         */
  virtual void publish(const std::shared_ptr<T> &message) {
    tick(message->header.stamp);
    publisher_->publish(message);
  }

  /**
         * \brief Collects statistics and publishes the message.
         *
         * The timestamp to be used by the TimeStampStatus class will be
         * extracted from message.header.stamp.
         */
  virtual void publish(const T &message) {
    tick(message.header.stamp);
    publisher_->publish(message);
  }

  /**
         * \brief Returns the publisher.
         */
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
  getPublisher() const {
    return publisher_;
  }

  /**
         * \brief Changes the publisher.
         */
  void setPublisher(
      rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub) {
    publisher_ = pub;
  }

private:
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      publisher_;
};
};  // namespace diagnostic_updater

#endif  // DIAGNOSTIC_UPDATER__PUBLISHER_HPP_
