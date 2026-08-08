#pragma once

#include <Arduino.h>
#include <atomic>
#include <stddef.h>
#include <stdint.h>
#include "master/protocol.h"

class DaisyUsbTransport
{
  public:
    struct TransportState
    {
        bool link_ready;
        bool engine_responding;
        bool playing;
        uint8_t step;
        uint8_t pattern;
        uint32_t last_response_ms;
        uint32_t tx_packets;
        uint32_t rx_packets;
        uint32_t crc_errors;
        uint32_t framing_errors;
        uint32_t tx_drops;
        bool sd_present;
        uint16_t sample_mask;
        uint8_t xtra_sample_mask;
        char kit_name[32];
        uint8_t sd_mount_result;
        uint8_t sd_root_result;
        uint8_t sd_boot_loaded;
        uint8_t sd_load_failures;
        uint8_t sd_diag_stage;
        uint8_t sd_card_type;
        uint8_t sd_last_command;
        uint8_t sd_last_response;
        uint8_t sd_last_data_token;
        uint16_t sd_spi_errors;
        bool daisy_sd_status_seen;
        uint16_t daisy_sd_sample_mask;
        char daisy_sd_kit[32];
        uint8_t daisy_sd_folder_count;
        char daisy_sd_folders[16][32];
        uint8_t daisy_sd_file_count;
        char daisy_sd_files[20][32];
        uint32_t daisy_sd_revision;
        PodStatePayload pod;
        uint32_t pod_revision;
    };

    void begin();
    void process();

    bool send(uint8_t command, const void* payload = nullptr,
              uint16_t payload_length = 0);
    bool sendU8(uint8_t command, uint8_t value);
    bool sendFloat(uint8_t command, float value);

    void trigger(uint8_t pad, uint8_t velocity);
    void start();
    void stop();
    void reset();
    void stopAll();
    void setTempo(float bpm);
    void selectPattern(uint8_t pattern);
    void queuePattern(uint8_t pattern, uint8_t bars = 0);
    void cancelPatternQueue();
    void setStep(uint8_t pattern, uint8_t track, uint8_t step,
                 bool active, uint8_t velocity = 100,
                 uint8_t note_division = 1, uint8_t probability = 100);
    void uploadTrack(uint8_t pattern, uint8_t track,
                     const bool steps[16], uint8_t velocity = 100);
    void setTrackMute(uint8_t track, bool muted);
    void setTrackSolo(uint8_t track, bool soloed);
    void setTrackVolume(uint8_t track, uint8_t volume);
    void setTrackEngine(uint8_t track, int8_t engine);
    void synthTrigger(uint8_t engine, uint8_t instrument, uint8_t velocity);
    void synthNoteOn(uint8_t engine, uint8_t note, uint8_t velocity,
                     bool accent, bool slide);
    void synthNoteOff(uint8_t engine, uint8_t track, uint8_t note = 0);
    void synthParam(uint8_t engine, uint8_t instrument, uint8_t parameter,
                    float value);
    void synthPreset(uint8_t engine, uint8_t preset);

    bool connected() const { return state_.engine_responding; }
    const TransportState& state() const { return state_; }
    uint32_t sampleEndAckRevision() const
    {
        return sample_end_ack_revision_.load(std::memory_order_acquire);
    }
    uint8_t sampleEndAckPad() const
    {
        return sample_end_ack_pad_.load(std::memory_order_relaxed);
    }
    bool sampleEndAckAccepted() const
    {
        return sample_end_ack_accepted_.load(std::memory_order_relaxed);
    }

  private:
    static uint16_t crc16(const uint8_t* data, uint16_t length);
    void parseByte(uint8_t byte);
    void handleResponse(const uint8_t* packet, uint16_t packet_length);
    void poll();

    std::atomic<uint16_t> sequence_{0};
    uint8_t rx_packet_[SPI_MAX_PAYLOAD + sizeof(SPIPacketHeader)] = {};
    uint16_t rx_length_ = 0;
    uint16_t rx_target_ = 0;
    uint32_t last_ping_ms_ = 0;
    uint32_t last_position_ms_ = 0;
    uint32_t last_status_ms_ = 0;
    uint32_t last_pod_state_ms_ = 0;
    std::atomic<uint32_t> sample_end_ack_revision_{0};
    std::atomic<uint8_t> sample_end_ack_pad_{0xFFu};
    std::atomic<bool> sample_end_ack_accepted_{false};
    TransportState state_ = {};
};

extern DaisyUsbTransport daisyUsb;
