#include "Inmp441I2sSource.h"

namespace sdl {

esp_err_t Inmp441I2sSource::begin() {
    if (installed_) return ESP_OK;

    i2s_config_t i2s = {};
    i2s.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s.sample_rate = static_cast<uint32_t>(cfg_.sampleRate);
    // 32-bit slots: INMP441 needs 64 BCLK per frame (2 x 32), data in the top 24 bits.
    i2s.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    // Both slots active: slot 0 (WS low) = L/R=GND mic, slot 1 (WS high) = L/R=3.3V mic.
    i2s.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    i2s.communication_format = I2S_COMM_FORMAT_STAND_I2S; // Philips I2S, as INMP441 expects
    i2s.intr_alloc_flags = 0;
    i2s.dma_buf_count = cfg_.dmaBufCount;
    i2s.dma_buf_len = cfg_.dmaBufLen;
    i2s.use_apll = false;
    i2s.tx_desc_auto_clear = false;
    i2s.fixed_mclk = 0;

    esp_err_t err = i2s_driver_install(cfg_.port, &i2s, 0, nullptr);
    if (err != ESP_OK) return err;
    installed_ = true;

    i2s_pin_config_t pins = {};
    pins.mck_io_num = I2S_PIN_NO_CHANGE; // INMP441 needs no MCLK
    pins.bck_io_num = cfg_.bclkPin;
    pins.ws_io_num = cfg_.wsPin;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num = cfg_.dataPin;
    err = i2s_set_pin(cfg_.port, &pins);
    if (err != ESP_OK) {
        end();
        return err;
    }
    i2s_zero_dma_buffer(cfg_.port);
    return ESP_OK;
}

void Inmp441I2sSource::end() {
    if (!installed_) return;
    i2s_driver_uninstall(cfg_.port);
    installed_ = false;
}

bool Inmp441I2sSource::readRaw(std::size_t frames) {
    if (!installed_) return false;
    raw_.resize(frames * 2);
    const std::size_t wanted = raw_.size() * sizeof(int32_t);
    std::size_t got = 0;
    // i2s_read may return fewer bytes than asked; loop until the block is complete so
    // we never split a stereo frame (which would misalign LEFT/RIGHT).
    while (got < wanted) {
        std::size_t n = 0;
        const esp_err_t err = i2s_read(cfg_.port, reinterpret_cast<uint8_t*>(raw_.data()) + got,
                                       wanted - got, &n, portMAX_DELAY);
        if (err != ESP_OK) return false;
        got += n;
    }
    return true;
}

bool Inmp441I2sSource::discard(std::size_t frames) {
    const std::size_t chunk = 256;
    while (frames > 0) {
        const std::size_t n = frames < chunk ? frames : chunk;
        if (!readRaw(n)) return false;
        framesRead_ += n;
        frames -= n;
    }
    return true;
}

bool Inmp441I2sSource::read(StereoBlock& out, std::size_t frames) {
    if (frames == 0 || !readRaw(frames)) return false;

    out.left.resize(frames);
    out.right.resize(frames);
    out.sampleRate = cfg_.sampleRate;
    out.startTimeSec = static_cast<double>(framesRead_) / cfg_.sampleRate;

    // De-interleave. Slot 0 of each frame is the WS-low (LEFT, L/R=GND) mic,
    // slot 1 is the WS-high (RIGHT, L/R=3.3V) mic.
    const int leftSlot = cfg_.swapChannels ? 1 : 0;
    const int rightSlot = 1 - leftSlot;
    for (std::size_t i = 0; i < frames; ++i) {
        out.left[i] = inmp441SlotToFloat(raw_[2 * i + leftSlot]);
        out.right[i] = inmp441SlotToFloat(raw_[2 * i + rightSlot]);
    }
    framesRead_ += frames;
    return true;
}

} // namespace sdl
