#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "main.h"

#include "myactuator_rmd/driver/driver.hpp"
#include "myactuator_rmd/exceptions.hpp"

/**
 * \brief Driver implementation that adapts the vendor SDK to the STM32 HAL FDCAN peripheral.
 */
class Stm32CanDriver : public myactuator_rmd::Driver {
 public:
  explicit Stm32CanDriver(FDCAN_HandleTypeDef* handle, std::uint32_t rx_timeout_ms = 10U);

  void addId(std::uint32_t actuator_id) override;
  void send(myactuator_rmd::Message const& msg, std::uint32_t actuator_id) override;
  [[nodiscard]] std::array<std::uint8_t, 8> sendRecv(myactuator_rmd::Message const& request,
                                                    std::uint32_t actuator_id) override;
  [[nodiscard]] float getLastWaitTimeUs() const { return last_wait_time_us_; }

 private:
  void flushRxFifo();
  void configureFilter(std::uint32_t actuator_id);
  [[nodiscard]] std::array<std::uint8_t, 8> waitForResponse(std::uint32_t expected_can_id);

  FDCAN_HandleTypeDef* hfdcan_;
  std::uint32_t rx_timeout_ms_;
  std::uint32_t next_filter_index_ {0U};
  float last_wait_time_us_ {0.0f};
};
