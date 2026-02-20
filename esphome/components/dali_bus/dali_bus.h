#pragma once

#include <deque>
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include <esp32-hal-timer.h>
#endif

#ifdef USE_ESP_IDF
#include "driver/gptimer.h"
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
  rsError,       // We saw an error
};

enum DALICallbackResult : uint8_t {
  crSuccess,        // Success, no back frame
  crSendFailed,     // Send failed
  crGoodBackFrame,  // Sucess, back frame received in time, no encoding errors, 8 bits
  crWrongLength,    // Back frame had more or less than 8 bits
  crTimingError,    // Back frame had some kind of timing error
  crNoBackFrame,    // We just didn't see anything. Might not be an error.
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

  msgReset = 0x20,  // Send twice - all messages from here to EnableWriteMemory below
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
  msgEnableWriteMemory,  // Send twice - all message from Reset above to here

  msgQueryStatus = 0x90,  // Back frame - all messages from here to ReadMemoryLoc below
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
  msgReadMemoryLoc,  // Back frame - all messages from here to ReadMemoryLoc below

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
  bool in_shorted_state;   // The input pin state when the bus is shorted
  bool out_shorted_state;  // The output pin state when the bus is shorted
#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  hw_timer_t *timer;
#endif
  volatile uint32_t timer_cnt{0};

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
  volatile DALISendState send_state{ssNone};

  static void gpio_intr(DALIInterrupt *d);
  static void timer_intr(DALIInterrupt *d);
#ifdef USE_ESP_IDF
  static bool IRAM_ATTR HOT timer_intr_bool(gptimer_t *timer, const gptimer_alarm_event_data_t *ad, void *d);
#endif
#ifdef ESP8266
  static DALIInterrupt *instance;
  static void timer_intr_void();
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

using msg_callback_t = std::function<void(DALICallbackResult success, uint8_t reply)>;

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
  bool expect_back_frame;
  bool send_twice;

  msg_callback_t callback;

  // Stuff about the message
  uint32_t wait_us;
};

// Where we are in the process of readdressing lamps
enum AddressState : uint8_t {
  asInactive,
  asReset,
  asResetWait,
  asLampOff,
  asInitialise,
  asRandomise,
  asRandomiseWait,
  asResetParams,
  asReadySend,
  asSearchAddrH,
  asSearchAddrM,
  asSearchAddrL,
  asCompare,
  asProgramShortAddr,
  asVerifyShortAddr,
  asWithdraw,
};

class DALIBusComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }
  bool is_ready() { return this->addr_state_ == asInactive; }

  void set_scan(bool scan) { scan_ = scan; }
  void set_dali_scan_delay(int scan_delay) { scan_delay_ = scan_delay; }
  void set_dali_out_pin(InternalGPIOPin *out_pin) { out_pin_ = out_pin; }
  void set_dali_out_invert(bool invert) { out_inverted_ = invert; }
  void set_dali_in_pin(InternalGPIOPin *in_pin) { in_pin_ = in_pin; }
  void set_dali_in_invert(bool invert) { in_inverted_ = invert; }

  void send_reset(DALIAddr addr, msg_callback_t cb);
  void send_lamp_off(DALIAddr addr, msg_callback_t cb);
  void send_dapc(DALIAddr addr, uint8_t level, msg_callback_t cb);
  void send_dtr0(uint8_t dtr0, msg_callback_t cb);
  void send_enable_write_memory(DALIAddr addr, msg_callback_t cb);
  void send_set_fade_time(DALIAddr addr, msg_callback_t cb);
  void send_query_max_level(DALIAddr addr, msg_callback_t cb);
  void send_query_min_level(DALIAddr addr, msg_callback_t cb);

 protected:
  void setup_timer_();
  void wait_then_send_(SendMsg msg);
  void send_forward_message_(DALIAddr addr, DALIMsg msg);
  void process_sent_message_();
  void process_back_frames_();
  void send_message_if_ready_();
  void addressing_cb_(DALICallbackResult cr, uint8_t reply);
  void terminate_addressing_(bool success);
  void process_addr_wait_();

  InternalGPIOPin *out_pin_;
  bool out_inverted_;
  InternalGPIOPin *in_pin_;
  bool in_inverted_;
  bool scan_;
  int scan_delay_;
  DALIInterrupt store_;
  struct SendMsg sending_;
  SendMsgState send_state_{smsDone};
  uint32_t back_frame_wait_start_;
  uint32_t reset_time_{0};
  std::deque<struct SendMsg> msg_queue_;

  AddressState addr_state_;
  uint32_t addr_wait_start_;
  uint32_t addr_min_;
  uint32_t addr_max_;
  uint32_t addr_mid_;
  DALIMsg addr_short_;
  msg_callback_t addr_cb_;
};

}  // namespace dali_bus
}  // namespace esphome
