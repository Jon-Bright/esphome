#pragma once

#include <deque>
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include <esp32-hal-timer.h>
#endif

namespace esphome {
namespace dali_bus {

// DALI uses Manchester encoding, meaning every bit has two halves.
// Frames start with a start bit, which is always low-high
// Data bits follow. A zero is encoded as high-low, a one is encoded as low-high.
// There may be 8, 16, or 24 data bits.
// Frames end with a stop bit, which is the bus being idle for >=2450us.
// Half-bits should be between 366.7us and 466.7us, typically 416.7us.
// Double half-bits (when a zero follows a one or vice-versa) should be between 733.3us and 933.3us, typically 833.3us.

// DALIRecvState represents where in a received bitstream we currently are
enum DALIRecvState : uint8_t {
  rsIdle,        // Nothing is happening on the bus
  rsSending,     // We're sending on the bus (we should ignore anything we receive, it's us)
  rsStartBitH1,  // We're receiving the first half of a start bit (DALI low)
  rsStartBitH2,  // We're receiving the second half of a start bit (DALI high)
  rsFirstHalf,   // We're receiving the first half of a normal data bit
  rsSecondHalf,  // We're receiving the second half of a normal data bit
  rsFrameReady,  // We've seen a stop bit, so our data frame is ready
};

// DALITime represents what the time between two edges on the input pin can validly represent
enum DALITime : uint8_t {
  // The time between edges on the bus was:
  tiTooShort,   // too short to represent even half a bit
  tiHalfBit,    // valid for half a bit
  tiInvalid,    // too long to be half a bit but not long enough to be two half-bits
  ti2HalfBits,  // valid for two half-bits
  tiTooLong,    // too long to represent even two half-bits
};

// DALISendState is the state of an ongoing send
enum DALISendState : uint8_t {
  ssStartBit,
  ssDataBits,
  ssStopBit,
  ssSuccess,
  ssFailed,
  ssNone,
};

// DALIAddr represents the address of a device. There are a number of "special" addresses,
// see the more complete description of those in the code.
typedef uint8_t DALIAddr;

// DALIPri represents the priority of a given message. This affects how long the bus needs to be idle
// before said message is sent (thereby also providing crude time-domain mediation of the bus).
enum DALIPri : uint8_t {
  priTxn = 1,  // "used for all forward frames within a transaction [...] except for the first"
  priUser,     // "used to execute user instigated actions"
  priConfig,   // "used for configuration of a bus unit" (also events that aren't User or Auto)
  priAuto,     // "used to execute automatic actions"
  priQuery,    // "used for periodic query commands"
};

// DALIMsg represents all available DALI messages.
enum DALIMsg : uint8_t {
  msgOff = 0x00,
  msgUp,
  msgDown,
  msgStepUp,
  msgStepDown,
  msgRecallMax,
  msgRecallMin,
  msgStepDownOff,
  msgOnStepUp,
  msgEnableDapcSeq,
  msgGoToLastActiveLevel,  // v2

  msgGoToScene = 0x10,  // ...and up to 0x1f, for different scenes.

  msgReset = 0x20,
  msgStoreActualLevelDtr0,
  msgSavePersistentVars,  // v2
  msgSetOperatingMode,    // v2
  msgResetMemoryBank,     // v2
  msgIdentifyDevice,      // v2

  msgSetMaxLevel = 0x2a,
  msgSetMinLevel,
  msgSetSystemFailureLevel,
  msgSetPowerOnLevel,
  msgSetFadeTime,
  msgSetFadeRate,
  msgSetExtendedFadeTime,  // v2

  msgSetScene = 0x40,  // ...and up to 0x4f, for different scenes.

  msgRemoveFromScene = 0x50,  // ...and up to 0x5f, for different scenes.

  msgAddToGroup = 0x60,  // ...and up to 0x6f, for different groups.

  msgRemoveFromGroup = 0x70,  // ...and up to 0x7f, for different groups.

  msgSetShortAddr = 0x80,
  msgEnableWriteMemory,

  msgQueryStatus = 0x90,
  msgQueryControlGearPresent,
  msgQueryLampFailure,
  msgQueryLampPowerOn,
  msgQueryLimitError,
  msgQueryResetState,
  msgQueryMissingShortAddr,
  msgQueryVersionNo,
  msgQueryContentDtr0,
  msgQueryDeviceType,
  msgQueryPhysicalMin,
  msgQueryPowerFailure,
  msgQueryContentDtr1,
  msgQueryContentDtr2,
  msgQueryOperatingMode,    // v2
  msgQueryLightSourceType,  // v2

  msgQueryActualLevel,
  msgQueryMaxLevel,
  msgQueryMinLevel,
  msgQueryPowerOnLevel,
  msgQuerySystemFailureLevel,
  msgQueryFadeTimeRate,
  msgQueryMfrSpecificMode,            // v2
  msgQueryNextDeviceType,             // v2
  msgQueryExtendedFadeTime,           // v2
  msgQueryControlGearFailure = 0xaa,  // v2

  msgQuerySceneLevel = 0xb0,  // ...and up to 0xbf, for different scenes.

  msgQueryGroup0_7 = 0xc0,
  msgQueryGroup8_15,
  msgQueryRandomAddrH,
  msgQueryRandomAddrM,
  msgQueryRandomAddrL,
  msgReadMemoryLoc,

  msgAppExtCmdBase = 0xe0,
};

enum DALIRecvDebug : uint8_t {
  rdNoDebugInfo,
  rdWrongStartH1Time,
  rdWrongDataH1OneTime,
  rdWrongDataH2ZeroTime,
  rdWrongStartH2Time,
  rdWrongDataH1ZeroTime,
  rdWrongDataH2OneTime,
  rdUnexpectedStop,
};

struct DALIInterrupt {
  ISRInternalGPIOPin in_pin;
  ISRInternalGPIOPin out_pin;
#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  hw_timer_t *timer;
#endif
  volatile DALIRecvDebug debug_recv_err;
  volatile uint32_t debug_recv_time;
  volatile DALIRecvState debug_recv_state;

  volatile uint32_t last_dali_high{0};
  volatile uint32_t last_dali_low{0};
  volatile DALIRecvState recv_state{rsIdle};

  volatile uint8_t rcvd_bits{0};
  volatile uint32_t rcvd_val{0};

  volatile uint8_t send_half_bits{0};
  volatile uint32_t send_val{0};
  volatile uint32_t low_time_at_start_of_high{0};
  volatile DALISendState send_state{ssSuccess};

  static void gpio_intr(DALIInterrupt *d);
  static void timer_intr(DALIInterrupt *d);
#ifdef USE_ESP_IDF
  static bool timer_intr_bool(void *d);
#endif
  void received_bit(bool bit);
  void dali_high();
  void dali_low();
  void dali_idle();
  inline void set_dali_high() __attribute__((always_inline));
  inline void set_dali_low() __attribute__((always_inline));
  void send_next_half_bit();
  void start_stop_bit_timer(void);
  void start_half_bit_timer(void);
  void stop_stop_bit_timer(void);
  void begin_send(uint32_t to_send, uint32_t send_bits);
  void log_any_recv_errors();

  DALITime get_bit_time(void);
};

using msg_callback_t = std::function<void(bool success, uint8_t reply)>;

enum SendMsgState : uint8_t {
  smsAwaitSend1,
  smsAwaitSend2,
  smsAwaitBackFrame,
  smsDone,
};

struct SendMsg {
  // The message itself
  DALIPri pri;
  DALIAddr addr;
  DALIMsg msg;

  msg_callback_t callback;

  // Stuff about the message
  uint32_t wait_us;
};

class DALIBusComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  void set_scan(bool scan) { scan_ = scan; }
  void set_dali_out_pin(InternalGPIOPin *out_pin) { out_pin_ = out_pin; }
  void set_dali_in_pin(InternalGPIOPin *in_pin) { in_pin_ = in_pin; }

  void send_reset(DALIAddr addr);

 protected:
  void setup_timer_();
  void wait_then_send_(SendMsg msg);
  void send_forward_message_(DALIAddr addr, DALIMsg msg);
  void process_sent_message_();
  void send_message_if_ready_();

  InternalGPIOPin *out_pin_;
  InternalGPIOPin *in_pin_;
  bool scan_;
  DALIInterrupt store_;
  struct SendMsg sending_;
  SendMsgState send_state_;
  std::deque<struct SendMsg> msg_queue_;
};

}  // namespace dali_bus
}  // namespace esphome
