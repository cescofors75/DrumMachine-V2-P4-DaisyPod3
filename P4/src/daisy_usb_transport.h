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
        uint32_t round_trip_ms;
        uint32_t query_timeouts;
        uint32_t stale_responses;
        uint16_t protocol_version;
        uint16_t capability_flags;
        uint32_t daisy_rx_drops;
        uint32_t daisy_protocol_errors;
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
        // From StatusResponse (CMD_GET_STATUS) — already sent periodically
        // by Daisy and already computed there (CpuLoadMeter), just never
        // parsed on this side before the DASHBOARD's CPU/memory panel.
        uint8_t daisy_cpu_load_pct;
        uint8_t daisy_cpu_avg_pct;
        uint8_t daisy_cpu_peak_pct;
        uint32_t daisy_sdram_used_bytes;
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
    void setTrackMuteMask(uint16_t mask);
    void setTrackSoloMask(uint16_t mask);
    void setTrackVolume(uint8_t track, uint8_t volume);
    void setTrackEngine(uint8_t track, int8_t engine);

    // Per-instrument FX (dormant chain on DaisyPod3, CMD_TRACK_* 0x50-0x5A).
    void setTrackFilter(uint8_t track, uint8_t filterType, float cutoffHz,
                        float resonance);
    void clearTrackFilter(uint8_t track);
    void setTrackDistortion(uint8_t track, float amount01);
    void setTrackBitcrush(uint8_t track, uint8_t bits);
    void setTrackReverbSend(uint8_t track, uint8_t percent);
    void setTrackDelaySend(uint8_t track, uint8_t percent);
    void clearTrackFx(uint8_t track);
    void synthTrigger(uint8_t engine, uint8_t instrument, uint8_t velocity);
    void synthNoteOn(uint8_t engine, uint8_t note, uint8_t velocity,
                     bool accent, bool slide);
    void synthNoteOff(uint8_t engine, uint8_t track, uint8_t note = 0);
    void synthParam(uint8_t engine, uint8_t instrument, uint8_t parameter,
                    float value);
    void synthPreset(uint8_t engine, uint8_t preset);
    bool uploadSong(const SongEntry* entries, uint8_t count);
    bool controlSong(uint8_t action); // 0=stop, 1=play, 2=reset
    bool sendMidiMap(const MidiMapEntry* entries, uint8_t count);
    // Pops the next raw MIDI event polled from the Daisy MPD218 monitor.
    // Producer (handleResponse) and consumer both run from loop().
    bool popMidiEvent(MidiMonitorEvent& event);

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
    bool sendPacket(uint8_t command, const void* payload,
                    uint16_t payload_length, uint16_t* assigned_sequence);
    bool sendQuery(uint8_t command, const void* payload = nullptr,
                   uint16_t payload_length = 0);
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
    uint32_t last_midi_events_ms_ = 0;
    MidiMonitorEvent midi_events_[64] = {};
    uint8_t midi_events_head_ = 0;
    uint8_t midi_events_tail_ = 0;
    bool pending_query_ = false;
    uint8_t pending_query_command_ = 0;
    uint16_t pending_query_sequence_ = 0;
    uint32_t pending_query_since_ms_ = 0;
    std::atomic<uint32_t> sample_end_ack_revision_{0};
    std::atomic<uint8_t> sample_end_ack_pad_{0xFFu};
    std::atomic<bool> sample_end_ack_accepted_{false};
    TransportState state_ = {};
};

extern DaisyUsbTransport daisyUsb;
