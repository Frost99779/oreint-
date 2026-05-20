#include "UartTransport.h"

#include <Arduino.h>
#include <string.h>

namespace comms {

void UartTransport::begin() {
    m_head = m_tail = m_line_len = 0U;
    m_has_ready = false;
}

void UartTransport::service(uint32_t now_ms) {
    // Line timeout: discard partial line
    if (m_line_len > 0U && (now_ms - m_line_start_ms) >= kTimeoutMs) {
        m_line_len = 0U;
    }

    uint32_t budget = core::config::kUartReadBudgetBytes;
    while (budget-- > 0U && Serial.available() > 0) {
        processByte(static_cast<uint8_t>(Serial.read()), now_ms);
    }
}

void UartTransport::processByte(uint8_t b, uint32_t now_ms) {
    if (b == '\r') return;  // ignore CR
    if (b == '\n') {
        if (m_line_len > 0U && !m_has_ready) {
            m_line[m_line_len] = '\0';
            memcpy(m_ready, m_line, m_line_len + 1U);
            m_has_ready = true;
        }
        m_line_len = 0U;
        return;
    }
    if (m_line_len == 0U) m_line_start_ms = now_ms;
    if (m_line_len < kLineSize - 1U) {
        m_line[m_line_len++] = static_cast<char>(b);
    }
    // If line too long, discard accumulation silently
}

bool UartTransport::hasLine() const {
    return m_has_ready;
}

bool UartTransport::getLine(char* dst, size_t len) {
    if (!m_has_ready || dst == nullptr || len == 0U) return false;
    const size_t copy_len = strlen(m_ready);
    const size_t n = (copy_len < len - 1U) ? copy_len : len - 1U;
    memcpy(dst, m_ready, n);
    dst[n] = '\0';
    m_has_ready = false;
    return true;
}

}  // namespace comms
