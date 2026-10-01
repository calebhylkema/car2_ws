// dbw_can.h — Shared CAN link definitions + end-to-end (E2E) command protection
// for the car2 drive-by-wire stack.
//
// One header, included by the master and all three actuator nodes, so every
// participant packs/parses command frames identically. See DESIGN.md §1.3 and
// PROTOCOL.md for the contract.
//
// Command frames (master -> node) are safety-relevant and are protected with a
// simplified AUTOSAR-E2E-Profile-1 scheme:
//     byte[0]        = rolling sequence counter (per CAN ID)
//     byte[1..N]     = payload
//     byte[1+N]      = CRC-8 (SAE J1850) over { dataId, seq, payload }
// where dataId = the CAN ID folded to one byte (id ^ id>>8). Folding (rather than
// just the low byte) keeps 0x100/0x200/0x300 distinct so a mis-routed frame from
// another node fails the CRC (masquerade guard).
// A receiver accepts a frame only if the CRC matches AND the counter advanced
// (not duplicated, not wildly out of sequence). Only accepted frames are treated
// as fresh commands, so corruption/loss naturally drives the node toward its
// watchdog safe state.
//
// Status frames (node -> master) are informational (QM level) and are sent as
// plain bytes; the master judges their validity by ID/length/arrival time.
//
// Set -DDBW_E2E_ENABLE=0 to bring the bus up with plain command frames first.

#ifndef DBW_CAN_H
#define DBW_CAN_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

// ---------------- Bus ----------------
static constexpr uint32_t DBW_CAN_BITRATE = 250000u;

// ---------------- Command IDs (master -> node) ----------------
static constexpr uint32_t DBW_ID_THROTTLE_CMD = 0x100u;
static constexpr uint32_t DBW_ID_STEER_CMD    = 0x200u;
static constexpr uint32_t DBW_ID_BRAKE_CMD    = 0x300u;

// ---------------- Status IDs (node -> master) ----------------
static constexpr uint32_t DBW_ID_THROTTLE_STAT = 0x101u;
static constexpr uint32_t DBW_ID_STEER_STAT    = 0x201u;
static constexpr uint32_t DBW_ID_BRAKE_STAT    = 0x301u;

// ---------------- E2E configuration ----------------
#ifndef DBW_E2E_ENABLE
#define DBW_E2E_ENABLE 1
#endif

// Largest sequence-counter jump still accepted (tolerates a few dropped frames
// while still rejecting a wildly stale/duplicated counter).
static constexpr uint8_t DBW_MAX_SEQ_DELTA = 16u;

// Longest command payload in this system (throttle = estop,throttle,mode).
static constexpr uint8_t DBW_MAX_PAYLOAD = 3u;

// ---------------- CRC-8 / SAE J1850 (poly 0x1D, init 0xFF, xorout 0xFF) ------
inline uint8_t dbw_crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0xFFu;
    for (uint8_t i = 0u; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            if ((crc & 0x80u) != 0u) {
                crc = (uint8_t)((crc << 1) ^ 0x1Du);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return (uint8_t)(crc ^ 0xFFu);
}

// Receiver-side per-ID sequence state.
struct DbwRxState {
    uint8_t last_seq;
    bool    inited;
};

// ---------------- Pack a command frame (master side) ----------------
// seq_counter is advanced by one each call.
inline void dbw_pack_cmd(CAN_message_t& msg, uint32_t id, uint8_t& seq_counter,
                         const uint8_t* payload, uint8_t plen) {
    msg.id = id;
#if DBW_E2E_ENABLE
    const uint8_t seq = seq_counter;
    seq_counter = (uint8_t)(seq_counter + 1u);

    msg.buf[0] = seq;
    for (uint8_t i = 0u; i < plen; ++i) {
        msg.buf[1u + i] = payload[i];
    }

    uint8_t tmp[2u + DBW_MAX_PAYLOAD];
    tmp[0] = (uint8_t)((id ^ (id >> 8)) & 0xFFu);  // folded Data-ID   // dataId
    tmp[1] = seq;
    for (uint8_t i = 0u; i < plen; ++i) {
        tmp[2u + i] = payload[i];
    }
    msg.buf[1u + plen] = dbw_crc8(tmp, (uint8_t)(2u + plen));
    msg.len = (uint8_t)(plen + 2u);
#else
    for (uint8_t i = 0u; i < plen; ++i) {
        msg.buf[i] = payload[i];
    }
    msg.len = plen;
#endif
}

// ---------------- Validate + extract a command frame (node side) ----------------
// Returns true only for a fresh, uncorrupted, in-sequence command.
inline bool dbw_unpack_cmd(const CAN_message_t& msg, uint32_t id,
                           DbwRxState& st, uint8_t* payload, uint8_t plen) {
#if DBW_E2E_ENABLE
    if (msg.len < (uint8_t)(plen + 2u)) {
        return false;
    }
    const uint8_t seq = msg.buf[0];

    uint8_t tmp[2u + DBW_MAX_PAYLOAD];
    tmp[0] = (uint8_t)((id ^ (id >> 8)) & 0xFFu);  // folded Data-ID
    tmp[1] = seq;
    for (uint8_t i = 0u; i < plen; ++i) {
        tmp[2u + i] = msg.buf[1u + i];
    }
    if (dbw_crc8(tmp, (uint8_t)(2u + plen)) != msg.buf[1u + plen]) {
        return false;                       // corrupted
    }
    if (st.inited) {
        const uint8_t delta = (uint8_t)(seq - st.last_seq);
        if (delta == 0u) {
            return false;                   // duplicate
        }
        if (delta > DBW_MAX_SEQ_DELTA) {
            return false;                   // out of sequence
        }
    }
    st.last_seq = seq;
    st.inited = true;

    for (uint8_t i = 0u; i < plen; ++i) {
        payload[i] = msg.buf[1u + i];
    }
    return true;
#else
    if (msg.len < plen) {
        return false;
    }
    (void)id;
    (void)st;
    for (uint8_t i = 0u; i < plen; ++i) {
        payload[i] = msg.buf[i];
    }
    return true;
#endif
}

#endif  // DBW_CAN_H
