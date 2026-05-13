/**
 * @file  main.cpp
 * @brief ARES runtime bootstrap.
 *
 * Initialises subsystems (sensors, radio, storage, WiFi, API, AMS)
 * and then remains idle until external commands arrive through the
 * REST API / mission runtime.
 *
 * Board-specific driver instantiation is delegated to the active
 * board implementation in src/boards/.  Select a board by defining
 * BOARD_ARES_V1 or BOARD_MYBOARD in platformio.ini build_flags.
 *
 * All objects are statically allocated (no `new`) so heap
 * usage stays at zero (PO10-3).
 */

#include "boards/board.h"
#include "config.h"
#include "sys/led/status_led.h"
#include "sys/wifi/wifi_ap.h"
#include "sys/storage/littlefs_storage.h"
#include "api/api_server.h"
#include "ams/mission_script_engine.h"
#include "comms/ares_radio_protocol.h"
#include "comms/radio_dispatcher.h"

#include <Arduino.h>
#include <freertos/task.h>
#include <esp_system.h>

#include "debug/ares_log.h"

// ── Application-layer singletons (not board-specific) ──────────
// WifiAp and LittleFsStorage are SoC-level, not tied to board hardware.
static WifiAp          wifiAp;
static LittleFsStorage storage;

// ── File-scope pointers for loop() access ──────────────────────
// These are set once in setup() and remain valid for the lifetime of
// the program.  Static locals in setup() live in .bss, not the stack.
static ares::ams::MissionScriptEngine* gEngine     = nullptr;
static ApiServer*                      gApi        = nullptr;
static ares::RadioDispatcher*          gDispatch   = nullptr;

// ═══════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(ares::SERIAL_BAUD);

    ares::board::initBuses();
    ares::board::beginAll();
    auto& b = ares::board::get();

    StorageInterface& storageIf = storage;
    (void)storageIf.begin();

    // LoRa radio transceiver
    // (radio hardware begin() is called by board::beginAll(); here we
    //  just reference the interface for application-layer objects.)

    // WiFi AP — must be up before API server
    (void)wifiAp.begin();

    // AMS runtime (IDLE by default, waits for API activation)
    // static: one-time construction in .bss, not on the stack.
    static ares::ams::MissionScriptEngine engine(
        storageIf,
        b.gpsDrivers,  b.gpsCount,
        b.baroDrivers, b.baroCount,
        b.comDrivers,  b.comCount,
        b.imuDrivers,  b.imuCount,
        b.pulse,
        b.servo);
    gEngine = &engine;
    (void)engine.begin();

    // REST API server
    static ApiServer apiServer(wifiAp,
                               *b.primaryBaro, *b.primaryGps, *b.primaryImu,
                               &storageIf, gEngine,
                               b.statusLed,
                               b.wire0, b.wire1,
                               b.gpsSerial, b.radioSerial,
                               b.radio,
                               b.pulse);
    gApi = &apiServer;
    (void)apiServer.begin();

    // Radio dispatcher — polls the LoRa receive FIFO and dispatches
    // inbound APUS frames.  Sends ACK/NACK for every COMMAND (APUS-9).
    static ares::RadioDispatcher radioDispatcher(*b.radio, engine, b.pulse);
    gDispatch = &radioDispatcher;

    // Classify the reset cause before acting on the restored checkpoint.
    const esp_reset_reason_t resetCause = esp_reset_reason();
    const bool abnormalReset =
        (resetCause == ESP_RST_PANIC    ||
         resetCause == ESP_RST_INT_WDT  ||
         resetCause == ESP_RST_TASK_WDT ||
         resetCause == ESP_RST_WDT      ||
         resetCause == ESP_RST_BROWNOUT);

    ares::ams::EngineSnapshot bootSnap = {};
    engine.getSnapshot(bootSnap);
    if (bootSnap.status == ares::ams::EngineStatus::RUNNING)
    {
        if (abnormalReset)
        {
            LOG_W("BOOT", "Abnormal reset during flight (cause=%d) — injecting TC.RESET_ABNORMAL",
                  static_cast<int>(resetCause));
            (void)engine.injectTcCommand("RESET_ABNORMAL");
        }
        apiServer.notifyMissionResumed();
    }
    else
    {
        if (b.statusLed) { b.statusLed->setMode(ares::OperatingMode::IDLE); }
    }
}

// ═══════════════════════════════════════════════════════════
void loop()
{
    const uint32_t now = millis();
    auto& b = ares::board::get();

    // GPS bytes must be consumed every iteration to keep
    // the UART FIFO from overflowing (72-byte HW FIFO).
    for (uint8_t i = 0; i < b.gpsCount; ++i) { b.gpsDrivers[i].iface->update(); }

    // Radio receive path — poll LoRa FIFO, decode APUS frames, dispatch
    // commands, and send ACK/NACK responses (APUS-4.4, APUS-9, APUS-14).
    gDispatch->poll(now);

    // AMS script runtime tick (state machine + PUS emission).
    gEngine->tick(now);

    // Auto-return to IDLE when the AMS mission finishes or faults.
    if (gApi->getMode() == ares::OperatingMode::FLIGHT)
    {
        ares::ams::EngineSnapshot snap = {};
        gEngine->getSnapshot(snap);
        if (snap.status == ares::ams::EngineStatus::COMPLETE
            || snap.status == ares::ams::EngineStatus::ERROR)
        {
            gApi->notifyMissionComplete();
        }
    }

    // Adaptive sleep: wake up exactly when the next engine event is due.
    const uint32_t wakeupMs = gEngine->nextWakeupMs(now);
    const uint32_t sleepMs  = (wakeupMs > now) ? (wakeupMs - now) : 1U;
    vTaskDelay(pdMS_TO_TICKS(sleepMs));
}
