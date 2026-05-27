#include "NvsCal.h"
#include "Config.h"

#include <math.h>
#include <esp_timer.h>
#include <Preferences.h>
#include <cstddef>
#include <cstring>

using namespace core::config;

CalBlob g_cal{};

static constexpr const char* kNvsNamespace = "orient_cal";
static constexpr const char* kNvsKey       = "cal_blob";

static_assert(sizeof(CalBlob) == 140U, "CalBlob layout changed — bump kCalVersion");

// FNV-1a 32-bit over entire blob with crc32 field = 0.
static uint32_t fnv1a32(const uint8_t* data, size_t len) {
    uint32_t h = 0x811C9DC5UL;
    for (size_t i = 0; i < len; i++) {
        h ^= static_cast<uint32_t>(data[i]);
        h *= 0x01000193UL;
    }
    return h;
}

static void calBlobZeroTailPadding(CalBlob& b) {
    b._pad = 0U;
    b._pad_mount[0] = 0U;
    b._pad_mount[1] = 0U;
    b._pad_mount[2] = 0U;
    b._pad_hdg[0] = 0U;
    b._pad_hdg[1] = 0U;
    b._pad_hdg[2] = 0U;
}

static uint32_t computeCrc(const CalBlob& b) {
    CalBlob tmp = b;
    tmp.crc32 = 0U;
    calBlobZeroTailPadding(tmp);
    return fnv1a32(reinterpret_cast<const uint8_t*>(&tmp), sizeof(CalBlob));
}

static void calBlobFinalizeForStore(CalBlob& b) {
    calBlobZeroTailPadding(b);
    b.magic   = kCalMagic;
    b.version = kCalVersion;
    b.size    = static_cast<uint16_t>(sizeof(CalBlob));
    b.saved_at_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    b.crc32   = 0U;
    b.crc32   = computeCrc(b);
}

void calResetDefaults() {
    std::memset(&g_cal, 0, sizeof(CalBlob));
    g_cal.mag_softmat[0] = 1.0f;
    g_cal.mag_softmat[4] = 1.0f;
    g_cal.mag_softmat[8] = 1.0f;
}

bool calLoad() {
    Preferences prefs;
    if (!prefs.begin(kNvsNamespace, /*readOnly=*/false)) {
        return false;
    }

    CalBlob tmp{};
    std::memset(&tmp, 0, sizeof(CalBlob));
    const size_t got = prefs.getBytes(kNvsKey, &tmp, sizeof(CalBlob));
    prefs.end();

    if (got != sizeof(CalBlob)) {
        return false;
    }
    if (tmp.magic != kCalMagic) {
        return false;
    }
    if (tmp.version != kCalVersion) {
        return false;
    }
    if (tmp.size != static_cast<uint16_t>(sizeof(CalBlob))) {
        return false;
    }

    const uint32_t stored = tmp.crc32;
    const uint32_t computed = computeCrc(tmp);
    if (stored != computed) {
        return false;
    }

    if (tmp.mount_valid != 0U) {
        const float* R = tmp.mount_R;
        const float det =
            R[0]*(R[4]*R[8] - R[5]*R[7])
          - R[1]*(R[3]*R[8] - R[5]*R[6])
          + R[2]*(R[3]*R[7] - R[4]*R[6]);
        if (fabsf(det - 1.0f) > 1e-2f) {
            tmp.mount_valid = 0U;
        }
    }

    g_cal = tmp;
    return true;
}

bool calSave() {
    CalBlob store = g_cal;
    calBlobFinalizeForStore(store);

    Preferences prefs;
    if (!prefs.begin(kNvsNamespace, /*readOnly=*/false)) {
        return false;
    }
    const size_t written = prefs.putBytes(kNvsKey, &store, sizeof(CalBlob));
    prefs.end();

    if (written != sizeof(CalBlob)) {
        return false;
    }
    g_cal = store;
    return true;
}

bool calClear() {
    calResetDefaults();
    Preferences prefs;
    if (!prefs.begin(kNvsNamespace, /*readOnly=*/false)) {
        return false;
    }
    prefs.remove(kNvsKey);
    prefs.end();
    return true;
}
