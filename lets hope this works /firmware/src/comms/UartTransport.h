#pragma once
#include <stdint.h>
#include <stddef.h>
#include "Config.h"

// RX ring buffer + line assembly. No TX ring — binary packets go via Serial.write() directly,
// ASCII responses via Serial.println() directly. This keeps the TX path simple and bounded.

namespace comms {

class UartTransport {
public:
    // Call once after Serial.begin().
    void begin();

    // Drain Serial RX into ring; assemble complete lines. Call every loop().
    void service(uint32_t now_ms);

    // Returns true if a complete line is waiting.
    bool hasLine() const;

    // Copy next complete line into dst (max len bytes, null-terminated). Removes it from queue.
    // Returns false if no line is waiting.
    bool getLine(char* dst, size_t len);

private:
    static constexpr size_t kRingSize   = core::config::kRxRingSize;
    static constexpr size_t kLineSize   = core::config::kCommandLineSize;
    static constexpr uint32_t kTimeoutMs = core::config::kLineTimeoutMs;

    // RX ring buffer
    uint8_t  m_ring[kRingSize]{};
    size_t   m_head{0};
    size_t   m_tail{0};

    // Line assembly
    char     m_line[kLineSize]{};
    size_t   m_line_len{0};
    uint32_t m_line_start_ms{0};

    // Completed line queue (single slot — commands come slowly)
    char     m_ready[kLineSize]{};
    bool     m_has_ready{false};

    void processByte(uint8_t b, uint32_t now_ms);
};

}  // namespace comms
