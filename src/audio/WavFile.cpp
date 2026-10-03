#include "audio/WavFile.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>

namespace sdl {
namespace {

constexpr uint16_t kFormatPcm = 1;
constexpr uint16_t kFormatFloat = 3;
constexpr uint16_t kFormatExtensible = 0xFFFE;

uint16_t rd16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t rd32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

void wr16(std::ofstream& f, uint16_t v) {
    const char b[2] = {static_cast<char>(v & 0xFF), static_cast<char>(v >> 8)};
    f.write(b, 2);
}
void wr32(std::ofstream& f, uint32_t v) {
    const char b[4] = {static_cast<char>(v & 0xFF), static_cast<char>((v >> 8) & 0xFF),
                       static_cast<char>((v >> 16) & 0xFF), static_cast<char>(v >> 24)};
    f.write(b, 4);
}

float decodeSample(const uint8_t* p, uint16_t format, uint16_t bits) {
    if (format == kFormatPcm) {
        switch (bits) {
            case 8: return (static_cast<int>(p[0]) - 128) / 128.0f;
            case 16: return static_cast<int16_t>(rd16(p)) / 32768.0f;
            case 24: {
                int32_t v = static_cast<int32_t>(p[0] | (p[1] << 8) | (p[2] << 16));
                if (v & 0x800000) v |= ~0xFFFFFF; // sign-extend
                return v / 8388608.0f;
            }
            case 32: return static_cast<float>(static_cast<int32_t>(rd32(p)) / 2147483648.0);
        }
    } else if (format == kFormatFloat) {
        if (bits == 32) {
            const uint32_t u = rd32(p);
            float f;
            std::memcpy(&f, &u, 4);
            return f;
        }
        if (bits == 64) {
            const uint64_t u = static_cast<uint64_t>(rd32(p)) | (static_cast<uint64_t>(rd32(p + 4)) << 32);
            double d;
            std::memcpy(&d, &u, 8);
            return static_cast<float>(d);
        }
    }
    return 0.0f;
}

} // namespace

bool readWav(const std::string& path, WavData& out, std::string& error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        error = "cannot open '" + path + "'";
        return false;
    }
    const std::vector<uint8_t> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (buf.size() < 12 || std::memcmp(buf.data(), "RIFF", 4) != 0 ||
        std::memcmp(buf.data() + 8, "WAVE", 4) != 0) {
        error = "not a RIFF/WAVE file";
        return false;
    }

    uint16_t format = 0, channels = 0, bits = 0, blockAlign = 0;
    uint32_t sampleRate = 0;
    bool haveFmt = false;
    const uint8_t* data = nullptr;
    std::size_t dataSize = 0;

    std::size_t pos = 12;
    while (pos + 8 <= buf.size()) {
        const uint8_t* chunk = buf.data() + pos;
        std::size_t size = rd32(chunk + 4);
        const std::size_t bodyPos = pos + 8;
        const std::size_t available = buf.size() - bodyPos;
        if (size > available) size = available; // tolerate truncated / streaming headers

        if (std::memcmp(chunk, "fmt ", 4) == 0 && size >= 16) {
            const uint8_t* b = buf.data() + bodyPos;
            format = rd16(b);
            channels = rd16(b + 2);
            sampleRate = rd32(b + 4);
            blockAlign = rd16(b + 12);
            bits = rd16(b + 14);
            if (format == kFormatExtensible && size >= 26) format = rd16(b + 24); // SubFormat GUID
            haveFmt = true;
        } else if (std::memcmp(chunk, "data", 4) == 0) {
            data = buf.data() + bodyPos;
            dataSize = size;
        }
        pos = bodyPos + size + (size & 1); // chunks are word aligned
    }

    if (!haveFmt || !data) {
        error = "missing fmt or data chunk";
        return false;
    }
    const bool supported = (format == kFormatPcm && (bits == 8 || bits == 16 || bits == 24 || bits == 32)) ||
                           (format == kFormatFloat && (bits == 32 || bits == 64));
    if (!supported || channels == 0 || sampleRate == 0) {
        error = "unsupported WAV format (format=" + std::to_string(format) +
                ", bits=" + std::to_string(bits) + ", channels=" + std::to_string(channels) + ")";
        return false;
    }

    const std::size_t bytesPerSample = bits / 8;
    if (blockAlign < channels * bytesPerSample) blockAlign = static_cast<uint16_t>(channels * bytesPerSample);
    const std::size_t frames = dataSize / blockAlign;

    out.sampleRate = static_cast<int>(sampleRate);
    out.channels = channels;
    out.channelData.assign(channels, std::vector<float>(frames));
    for (std::size_t i = 0; i < frames; ++i) {
        const uint8_t* frame = data + i * blockAlign;
        for (uint16_t c = 0; c < channels; ++c)
            out.channelData[c][i] = decodeSample(frame + c * bytesPerSample, format, bits);
    }
    return true;
}

bool writeWav(const std::string& path, const std::vector<std::vector<float>>& channels,
              int sampleRate, std::string& error) {
    if (channels.empty()) {
        error = "no channels to write";
        return false;
    }
    const std::size_t frames = channels[0].size();
    for (const auto& ch : channels) {
        if (ch.size() != frames) {
            error = "channel length mismatch";
            return false;
        }
    }

    std::ofstream f(path, std::ios::binary);
    if (!f) {
        error = "cannot create '" + path + "'";
        return false;
    }
    const uint16_t numCh = static_cast<uint16_t>(channels.size());
    const uint32_t dataBytes = static_cast<uint32_t>(frames * numCh * 2);

    f.write("RIFF", 4);
    wr32(f, 36 + dataBytes);
    f.write("WAVE", 4);
    f.write("fmt ", 4);
    wr32(f, 16);
    wr16(f, kFormatPcm);
    wr16(f, numCh);
    wr32(f, static_cast<uint32_t>(sampleRate));
    wr32(f, static_cast<uint32_t>(sampleRate) * numCh * 2);
    wr16(f, static_cast<uint16_t>(numCh * 2));
    wr16(f, 16);
    f.write("data", 4);
    wr32(f, dataBytes);
    for (std::size_t i = 0; i < frames; ++i) {
        for (const auto& ch : channels) {
            float s = ch[i];
            s = s > 1.0f ? 1.0f : (s < -1.0f ? -1.0f : s);
            wr16(f, static_cast<uint16_t>(static_cast<int16_t>(std::lround(s * 32767.0f))));
        }
    }
    if (!f) {
        error = "write failed";
        return false;
    }
    return true;
}

std::vector<float> downmixToMono(const WavData& wav) {
    std::vector<float> mono(wav.frames(), 0.0f);
    if (wav.channels == 0) return mono;
    for (const auto& ch : wav.channelData)
        for (std::size_t i = 0; i < mono.size(); ++i) mono[i] += ch[i];
    const float scale = 1.0f / static_cast<float>(wav.channels);
    for (float& s : mono) s *= scale;
    return mono;
}

} // namespace sdl
