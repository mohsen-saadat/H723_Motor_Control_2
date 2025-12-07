#include "stm32_can_driver.hpp"

#include <string>

namespace {
constexpr std::uint32_t kRequestBaseId = 0x140U;
constexpr std::uint32_t kResponseBaseId = 0x240U;
constexpr std::uint32_t kStandardIdMask = 0x7FFU;
}

Stm32CanDriver::Stm32CanDriver(FDCAN_HandleTypeDef* handle, std::uint32_t rx_timeout_ms)
    : hfdcan_{handle}, rx_timeout_ms_{rx_timeout_ms} {
  if (hfdcan_ == nullptr) {
    throw myactuator_rmd::Exception{"FDCAN handle is null"};
  }

  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan_, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
    throw myactuator_rmd::Exception{"Failed to configure global CAN filter"};
  }
}

void Stm32CanDriver::addId(std::uint32_t actuator_id) {
  if ((actuator_id < 1U) || (actuator_id > 32U)) {
    throw myactuator_rmd::Exception{"Actuator ID out of range"};
  }
  configureFilter(actuator_id);
}

void Stm32CanDriver::send(myactuator_rmd::Message const& msg, std::uint32_t actuator_id) {
  FDCAN_TxHeaderTypeDef header {};
  header.Identifier = kRequestBaseId + actuator_id;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0;

  auto data = msg.getData();
  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &header, data.data()) != HAL_OK) {
    throw myactuator_rmd::Exception{"Failed to queue CAN message"};
  }
}

std::array<std::uint8_t, 8> Stm32CanDriver::sendRecv(myactuator_rmd::Message const& request,
                                                      std::uint32_t actuator_id) {
  flushRxFifo();
  send(request, actuator_id);
  auto const expected_id = kResponseBaseId + actuator_id;
  return waitForResponse(expected_id);
}

void Stm32CanDriver::flushRxFifo() {
  FDCAN_RxHeaderTypeDef rx_header {};
  std::array<std::uint8_t, 8> dummy {};
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, FDCAN_RX_FIFO0) > 0U) {
    if (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header, dummy.data()) != HAL_OK) {
      break;
    }
  }
}

void Stm32CanDriver::configureFilter(std::uint32_t actuator_id) {
  if (next_filter_index_ >= hfdcan_->Init.StdFiltersNbr) {
    throw myactuator_rmd::Exception{"No free CAN filter slots"};
  }

  FDCAN_FilterTypeDef filter {};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = next_filter_index_++;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0U;
  filter.FilterID2 = 0U;
  filter.RxBufferIndex = 0;

  if (HAL_FDCAN_ConfigFilter(hfdcan_, &filter) != HAL_OK) {
    throw myactuator_rmd::Exception{"Failed to configure CAN filter"};
  }
}

std::array<std::uint8_t, 8> Stm32CanDriver::waitForResponse(std::uint32_t expected_can_id) {
  std::array<std::uint8_t, 8> data {};
  FDCAN_RxHeaderTypeDef rx_header {};
  auto const start = HAL_GetTick();
  while ((HAL_GetTick() - start) < rx_timeout_ms_) {
    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, FDCAN_RX_FIFO0) == 0U) {
      continue;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header, data.data()) != HAL_OK) {
      continue;
    }

    if (rx_header.Identifier == expected_can_id) {
      return data;
    }
  }

  throw myactuator_rmd::Exception{"Timed out waiting for CAN response"};
}
