#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// =============================================================================
// ATmega4809 電磁クラッチ制御システム (Crystal-less 20MHz, UPDI)
// =============================================================================

// =============================================================================
// 1. ピンアサイン & 論理定数定義
// =============================================================================
namespace Pins {
    // クラッチ制御出力 (MOSFET 駆動: HIGH = ON, LOW = OFF)
    constexpr uint8_t CLUTCH_DOWN     = PIN_PA0;  // 載荷用下降クラッチ (※USART0_TXD共用)
    constexpr uint8_t CLUTCH_UP       = PIN_PA1;  // 載荷用上昇クラッチ (※USART0_RXD共用)
    constexpr uint8_t CLUTCH_BREAK    = PIN_PA2;  // 載荷用保持クラッチ (BREAK)
    constexpr uint8_t CLUTCH_FAST     = PIN_PA3;  // 移動用クラッチ (FAST)

    // 移動速度ノブ制御 (LOW: NC接点でショート/速度0固定, HIGH: リレー励磁でノブ有効)
    constexpr uint8_t FAST_KNOB_CTRL  = PIN_PA4;

    // スイッチ入力 (セーフティ/リミット: 5V=NORMAL / 0V=ACTIVE)
    constexpr uint8_t SW_MIN          = PIN_PC4;  // 下限リミット
    constexpr uint8_t SW_MAX          = PIN_PC5;  // 上限リミット
    constexpr uint8_t SW_FREE         = PIN_PD0;  // フリーSW
    constexpr uint8_t SW_KILL         = PIN_PD1;  // 非常停止/キルSW

    // スイッチ入力 (制御系)
    constexpr uint8_t SW_COM_DIR      = PIN_PD2;  // COM方向 (HIGH: UP / LOW: DOWN)
    constexpr uint8_t SW_COM_ENA      = PIN_PD3;  // COMイネーブル (HIGH: UP/DOWN / LOW: BREAK)
    constexpr uint8_t SW_MAN_ENA      = PIN_PD4;  // 手動イネーブル (HIGH: UP/DOWN / LOW: BREAK)
    constexpr uint8_t SW_FAST_ENA     = PIN_PD5;  // 高速イネーブル (HIGH: FAST / LOW: NORMAL)
    constexpr uint8_t SW_CTRL_COM     = PIN_PD6  // 制御モード (HIGH: COMPUTER / LOW: MANUAL)
    constexpr uint8_t SW_MAN_DIR      = PIN_PD7;  // 手動方向 (HIGH: UP / LOW: DOWN)

    // 外部出力信号 (内部ステータス通知用)
    constexpr uint8_t EXT_STOP_BAR    = PIN_PF6;  // 外部通知: STOP状態 (HIGH: RUN / LOW: STOP)
    constexpr uint8_t EXT_CTRL_COM    = PIN_PF5;  // 外部通知: 制御モード (HIGH: COMPUTER / LOW: MANUAL)

    // アナログ入力 (遅延時間調整POT)
    constexpr uint8_t DELAY_BREAK     = PIN_PE0;  // AIN8: ブレーキOFF遅延
    constexpr uint8_t DELAY_UP        = PIN_PE1;  // AIN9: 上昇OFF遅延
    constexpr uint8_t DELAY_DOWN      = PIN_PE2;  // AIN10: 下降OFF遅延

    // LED 表示出力 (HIGH = 点灯, LOW = 消灯)
    constexpr uint8_t LED_CTRL_MANUAL = PIN_PE3;  // 手動モードLED
    constexpr uint8_t LED_CTRL_COM    = PIN_PF0;  // PC制御モードLED
    constexpr uint8_t LED_ERR_KILL    = PIN_PF1;  // キル/フリー要因LED
    constexpr uint8_t LED_ERR_LIMIT   = PIN_PF2;  // リミット要因LED
    constexpr uint8_t LED_RUN         = PIN_PF3;  // 正常稼働中(RUN)LED (STOPのNOT)
    constexpr uint8_t LED_ERR_STOP    = PIN_PF4;  // 総合停止(STOP)LED (全要因のOR合成)
}

namespace Logic {
    constexpr uint8_t SW_ACTIVE_LOW   = LOW;
    constexpr uint8_t DIR_UP          = HIGH;
    constexpr uint8_t DIR_DOWN        = LOW;
    constexpr uint8_t ENA_MOVE        = HIGH;
    constexpr uint8_t ENA_BREAK       = LOW;
    constexpr uint8_t MODE_COMPUTER   = HIGH;
    constexpr uint8_t MODE_MANUAL     = LOW;
    constexpr uint8_t FAST_ENABLE     = HIGH;
    constexpr uint8_t FAST_NORMAL     = LOW;

    constexpr uint8_t KNOB_SHORT      = LOW;   // リレー非通電(NC短絡)で速度ゼロ
    constexpr uint8_t KNOB_RELEASE    = HIGH;  // リレー励磁(接点開放)でノブ有効

    constexpr uint8_t LED_ON          = HIGH;
    constexpr uint8_t LED_OFF         = LOW;
}

// =============================================================================
// 2. クラス設計
// =============================================================================

// -----------------------------------------------------------------------------
// [Class] AnalogDelaySampler : バックグラウンドADC + IIR-LPFサンプラー
// -----------------------------------------------------------------------------
class AnalogDelaySampler {
public:
    enum Channel {
        CH_BREAK = 0,
        CH_UP    = 1,
        CH_DOWN  = 2,
        COUNT    = 3
    };

private:
    static constexpr uint8_t LPF_SHIFT = 5; // 1/32 指数移動平均
    static inline const ADC_MUXPOS_t ADC_MUX_LIST[COUNT] = {
        ADC_MUXPOS_AIN8_gc,  // PE0
        ADC_MUXPOS_AIN9_gc,  // PE1
        ADC_MUXPOS_AIN10_gc  // PE2
    };

    static volatile uint16_t s_lpfAcc[COUNT];
    static volatile uint16_t s_filteredAdc[COUNT];
    static volatile uint8_t  s_currentChannel;

public:
    static void init() {
        PORTE.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
        PORTE.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
        PORTE.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;

        ADC0.CTRLC    = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc;
        ADC0.CTRLB    = ADC_SAMPNUM_ACC1_gc;
        ADC0.CTRLD    = ADC_INITDLY_DLY16_gc;
        ADC0.SAMPCTRL = 15;
        ADC0.INTCTRL  = ADC_RESRDY_bm;
        ADC0.CTRLA    = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;

        ADC0.MUXPOS   = ADC_MUX_LIST[CH_BREAK];
        ADC0.COMMAND  = ADC_STCONV_bm;
    }

    // ADC割り込みサービスルーチンから呼び出し
    static void handleInterrupt() {
        uint16_t raw = ADC0.RES;

        if (s_lpfAcc[s_currentChannel] == 0) {
            s_lpfAcc[s_currentChannel] = raw << LPF_SHIFT;
        } else {
            s_lpfAcc[s_currentChannel] += raw - (s_lpfAcc[s_currentChannel] >> LPF_SHIFT);
        }
        s_filteredAdc[s_currentChannel] = s_lpfAcc[s_currentChannel] >> LPF_SHIFT;

        s_currentChannel++;
        if (s_currentChannel >= COUNT) {
            s_currentChannel = 0;
        }

        ADC0.MUXPOS  = ADC_MUX_LIST[s_currentChannel];
        ADC0.COMMAND = ADC_STCONV_bm;
    }

    // 0V -> 1.0s (1000ms), 5V -> 0.1s (100ms) ※電圧逆転
    static uint16_t getDelayMs(Channel ch) {
        uint16_t adcVal;
        cli();
        adcVal = s_filteredAdc[ch];
        sei();
        return 1000 - (uint16_t)(( (uint32_t)adcVal * 900UL ) / 1023UL);
    }
};

volatile uint16_t AnalogDelaySampler::s_lpfAcc[AnalogDelaySampler::COUNT]       = {0, 0, 0};
volatile uint16_t AnalogDelaySampler::s_filteredAdc[AnalogDelaySampler::COUNT] = {0, 0, 0};
volatile uint8_t  AnalogDelaySampler::s_currentChannel                        = 0;

ISR(ADC0_RESRDY_vect) {
    AnalogDelaySampler::handleInterrupt();
}

// -----------------------------------------------------------------------------
// [Class] DelayedClutch : OFF-DELAY クロスオーバー制御ステートマシン
// -----------------------------------------------------------------------------
class DelayedClutch {
public:
    enum class State {
        IDLE = 0,       // 出力 OFF
        ACTIVE,         // 出力 ON
        OFF_DELAYING    // OFF遅延中 (HIGH保持 -> タイマー満了でIDLEへ)
    };

private:
    const uint8_t                  _pin;
    const AnalogDelaySampler::Channel _delayCh;
    State                          _state;
    uint32_t                       _delayStartTime;
    uint16_t                       _latchedDelayMs;

public:
    DelayedClutch(uint8_t pin, AnalogDelaySampler::Channel delayCh)
        : _pin(pin), _delayCh(delayCh), _state(State::IDLE), _delayStartTime(0), _latchedDelayMs(0) {}

    void init() {
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin, LOW);
        forceOff();
    }

    void forceOff() {
        _state = State::IDLE;
        _delayStartTime = 0;
        _latchedDelayMs = 0;
        digitalWrite(_pin, LOW);
    }

    void update(bool targetOn, uint32_t now) {
        switch (_state) {
            case State::IDLE:
                if (targetOn) {
                    _state = State::ACTIVE;
                    digitalWrite(_pin, HIGH);
                }
                break;

            case State::ACTIVE:
                if (!targetOn) {
                    // ON -> OFF 立ち下がりエッジ時に遅延時間をラッチ(固定)
                    _state = State::OFF_DELAYING;
                    _delayStartTime = now;
                    _latchedDelayMs = AnalogDelaySampler::getDelayMs(_delayCh);
                }
                break;

            case State::OFF_DELAYING:
                if (targetOn) {
                    _state = State::ACTIVE;
                } else if (now - _delayStartTime >= _latchedDelayMs) {
                    // ラッチされた遅延時間で満了判定
                    _state = State::IDLE;
                    digitalWrite(_pin, LOW);
                }
                break;
        }
    }

    bool isActive() const { return _state != State::IDLE; }
};

// -----------------------------------------------------------------------------
// [Class] SafetyMonitor : 安全停止要因の判定・総合STOP/LED管理
// -----------------------------------------------------------------------------
class SafetyMonitor {
public:
    struct Status {
        bool isLimitErr;     // 上限・下限リミット
        bool isKillFreeErr;  // 非常停止・フリーSW
        bool isStop;         // 総合停止 (上記OR合成)
    };

    static void init() {
        const uint8_t safetyInputs[] = { Pins::SW_MIN, Pins::SW_MAX, Pins::SW_FREE, Pins::SW_KILL };
        for (uint8_t pin : safetyInputs) pinMode(pin, INPUT);

        const uint8_t statusOutputs[] = {
            Pins::LED_ERR_LIMIT, Pins::LED_ERR_KILL, Pins::LED_ERR_STOP, Pins::LED_RUN, Pins::EXT_STOP_BAR
        };
        for (uint8_t pin : statusOutputs) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }
    }

    static Status evaluate() {
        bool swMin  = digitalRead(Pins::SW_MIN);
        bool swMax  = digitalRead(Pins::SW_MAX);
        bool swFree = digitalRead(Pins::SW_FREE);
        bool swKill = digitalRead(Pins::SW_KILL);

        Status status;
        status.isLimitErr    = (swMin  == Logic::SW_ACTIVE_LOW || swMax  == Logic::SW_ACTIVE_LOW);
        status.isKillFreeErr = (swFree == Logic::SW_ACTIVE_LOW || swKill == Logic::SW_ACTIVE_LOW);
        status.isStop        = status.isLimitErr || status.isKillFreeErr;

        // LED / 外部通知出力の更新
        digitalWrite(Pins::LED_ERR_LIMIT, status.isLimitErr    ? Logic::LED_ON : Logic::LED_OFF);
        digitalWrite(Pins::LED_ERR_KILL,  status.isKillFreeErr ? Logic::LED_ON : Logic::LED_OFF);
        digitalWrite(Pins::LED_ERR_STOP,  status.isStop        ? Logic::LED_ON : Logic::LED_OFF);
        digitalWrite(Pins::LED_RUN,      !status.isStop        ? Logic::LED_ON : Logic::LED_OFF);

        // 外部通知: HIGH: RUN / LOW: STOP
        digitalWrite(Pins::EXT_STOP_BAR, !status.isStop        ? HIGH : LOW);

        return status;
    }
};

// -----------------------------------------------------------------------------
// [Class] MachineController : システム全体の統合コーディネータ
// -----------------------------------------------------------------------------
class MachineController {
private:
    DelayedClutch _clutchBreak;
    DelayedClutch _clutchUp;
    DelayedClutch _clutchDown;

public:
    MachineController()
        : _clutchBreak(Pins::CLUTCH_BREAK, AnalogDelaySampler::CH_BREAK),
          _clutchUp   (Pins::CLUTCH_UP,    AnalogDelaySampler::CH_UP),
          _clutchDown (Pins::CLUTCH_DOWN,  AnalogDelaySampler::CH_DOWN) {}

    void init() {
        // PA0/PA1のUSARTペリフェラル強制無効化 (GPIO解放)
        USART0.CTRLB = 0;

        _clutchBreak.init();
        _clutchUp.init();
        _clutchDown.init();

        // 移動クラッチ & ノブ制御
        pinMode(Pins::CLUTCH_FAST, OUTPUT);
        digitalWrite(Pins::CLUTCH_FAST, LOW);

        pinMode(Pins::FAST_KNOB_CTRL, OUTPUT);
        digitalWrite(Pins::FAST_KNOB_CTRL, Logic::KNOB_SHORT);

        // 制御入力 & モード表示
        const uint8_t ctrlInputs[] = {
            Pins::SW_COM_DIR, Pins::SW_COM_ENA, Pins::SW_MAN_ENA,
            Pins::SW_FAST_ENA, Pins::SW_CTRL_COM, Pins::SW_MAN_DIR
        };
        for (uint8_t pin : ctrlInputs) pinMode(pin, INPUT);

        pinMode(Pins::LED_CTRL_MANUAL, OUTPUT);
        pinMode(Pins::LED_CTRL_COM,    OUTPUT);
        pinMode(Pins::EXT_CTRL_COM,    OUTPUT);

        SafetyMonitor::init();
        AnalogDelaySampler::init();

        sei(); // 全体割り込み有効化
    }

    void update() {
        uint32_t now = millis();

        // ---------------------------------------------------------------------
        // 1. 安全停止要因の評価 (STOP_SIGNAL)
        // ---------------------------------------------------------------------
        SafetyMonitor::Status safety = SafetyMonitor::evaluate();

        // 制御モード判定 & 表示
        bool isComputerMode = (digitalRead(Pins::SW_CTRL_COM) == Logic::MODE_COMPUTER);
        digitalWrite(Pins::LED_CTRL_COM,    isComputerMode ? Logic::LED_ON : Logic::LED_OFF);
        digitalWrite(Pins::LED_CTRL_MANUAL, !isComputerMode ? Logic::LED_ON : Logic::LED_OFF);
        digitalWrite(Pins::EXT_CTRL_COM,    isComputerMode ? HIGH : LOW);

        // ---------------------------------------------------------------------
        // 2. 優先度1: 非常停止・リミット安全停止
        // ---------------------------------------------------------------------
        if (safety.isStop) {
            forceAllOff();
            return;
        }

        // ---------------------------------------------------------------------
        // 3. 優先度2: 移動モード (FAST 有効時)
        // ---------------------------------------------------------------------
        bool isFastEna = (digitalRead(Pins::SW_FAST_ENA) == Logic::FAST_ENABLE);
        if (isFastEna) {
            digitalWrite(Pins::CLUTCH_FAST, HIGH);                  // 移動クラッチ即時ON
            digitalWrite(Pins::FAST_KNOB_CTRL, Logic::KNOB_RELEASE);// ノブ開放(リレー励磁)
            _clutchBreak.forceOff();                                // 載荷系は即時解放
            _clutchUp.forceOff();
            _clutchDown.forceOff();
            return;
        }

        // FAST 無効時 (載荷モード)
        digitalWrite(Pins::CLUTCH_FAST, LOW);
        digitalWrite(Pins::FAST_KNOB_CTRL, Logic::KNOB_SHORT);      // ノブ短絡(速度0固定)

        // ---------------------------------------------------------------------
        // 4. 優先度3: 通常載荷運転 (NORMAL) - 目標クラッチのFSM更新
        // ---------------------------------------------------------------------
        bool activeEna = isComputerMode ? digitalRead(Pins::SW_COM_ENA) : digitalRead(Pins::SW_MAN_ENA);
        bool activeDir = isComputerMode ? digitalRead(Pins::SW_COM_DIR) : digitalRead(Pins::SW_MAN_DIR);

        bool targetBreak = false;
        bool targetUp    = false;
        bool targetDown  = false;

        if (activeEna == Logic::ENA_BREAK) {
            targetBreak = true; // 載荷保持
        } else {
            if (activeDir == Logic::DIR_UP) targetUp   = true;  // 載荷上昇
            else                            targetDown = true;  // 載荷下降
        }

        // OFF DELAY クロスオーバーFSMの実行
        _clutchBreak.update(targetBreak, now);
        _clutchUp.update(targetUp, now);
        _clutchDown.update(targetDown, now);
    }

private:
    void forceAllOff() {
        _clutchBreak.forceOff();
        _clutchUp.forceOff();
        _clutchDown.forceOff();
        digitalWrite(Pins::CLUTCH_FAST, LOW);
        digitalWrite(Pins::FAST_KNOB_CTRL, Logic::KNOB_SHORT);
        digitalWrite(Pins::LED_CTRL_MANUAL, Logic::LED_OFF);
        digitalWrite(Pins::LED_CTRL_COM,    Logic::LED_OFF);
    }
};

// =============================================================================
// 3. Arduino エントリポイント
// =============================================================================
static MachineController g_machine;

void setup() {
    g_machine.init();
}

void loop() {
    g_machine.update();
    delay(1);
}
