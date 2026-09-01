// Pluggable depth-frame codecs for the NanoPi (RK3528A) write-path experiment.
//
// Every codec turns one packed Z16 frame (width*height*2 bytes, little endian)
// into a self-contained byte blob that the writer appends to depth_XXXXXX.<ext>.
// The index record already stores per-frame offset and byte count, so blobs do
// not need their own framing.
//
// Design constraints coming from the board:
//   * 4x Cortex-A53 @1.8GHz, NEON only, 1GB RAM, no network.
//   * Only libzstd (with headers) and zlib are available for generic entropy
//     coding; lz4 ships without headers.
//   * The MJPEG hardware encoder is already busy with the color stream.
#ifndef DEPTH_CODEC_H
#define DEPTH_CODEC_H

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <zstd.h>

namespace depth_codec
{

    enum class Kind
    {
        RAW_Z16,           // baseline: what camera_cap writes today
        RVL,               // Wilson's Run-Length Variable-length, depth specific
        ZSTD1,             // zstd level 1 on raw Z16
        ZSTD3,             // zstd level 3 on raw Z16
        SPLIT_ZSTD1,       // hi/lo byte planes, then zstd level 1
        SPLIT_ZSTD3,       // hi/lo byte planes, then zstd level 3
        RVL_ZSTD1,         // RVL first, zstd level 1 on the RVL stream
        DELTA_ZSTD1,       // horizontal 16-bit delta, then zstd level 1
        SPLIT_DELTA_ZSTD1, // horizontal delta on the split planes, then zstd 1
        SHIFT1_RVL,        // lossy: >>1 (2mm quantum) then RVL
        SHIFT2_RVL,        // lossy: >>2 (4mm quantum) then RVL
        SHIFT1_SPLIT_ZSTD1,
        SHIFT2_SPLIT_ZSTD1,
        SHIFT1_RVL_ZSTD1,
    };

    struct Info
    {
        Kind kind;
        const char *name;      // used in reports and on the command line
        const char *extension; // depth_XXXXXX.<extension>
        const char *fourcc;    // 4 chars for the index header
        bool lossless;
    };

    inline const std::vector<Info> &registry()
    {
        static const std::vector<Info> table = {
            {Kind::RAW_Z16, "raw_z16", "z16", "Z16 ", true},
            {Kind::RVL, "rvl", "rvl", "RVL ", true},
            {Kind::ZSTD1, "zstd1", "zst", "ZST1", true},
            {Kind::ZSTD3, "zstd3", "zst", "ZST3", true},
            {Kind::SPLIT_ZSTD1, "split_zstd1", "spz", "SPZ1", true},
            {Kind::SPLIT_ZSTD3, "split_zstd3", "spz", "SPZ3", true},
            {Kind::RVL_ZSTD1, "rvl_zstd1", "rvz", "RVZ1", true},
            {Kind::DELTA_ZSTD1, "delta_zstd1", "dlz", "DLZ1", true},
            {Kind::SPLIT_DELTA_ZSTD1, "split_delta_zstd1", "sdz", "SDZ1", true},
            {Kind::SHIFT1_RVL, "shift1_rvl", "rvl1", "RVS1", false},
            {Kind::SHIFT2_RVL, "shift2_rvl", "rvl2", "RVS2", false},
            {Kind::SHIFT1_SPLIT_ZSTD1, "shift1_split_zstd1", "spz1", "SZS1", false},
            {Kind::SHIFT2_SPLIT_ZSTD1, "shift2_split_zstd1", "spz2", "SZS2", false},
            {Kind::SHIFT1_RVL_ZSTD1, "shift1_rvl_zstd1", "rvz1", "RZS1", false},
        };
        return table;
    }

    inline const Info &lookup(const std::string &name)
    {
        for (const Info &info : registry())
        {
            if (name == info.name)
                return info;
        }
        throw std::runtime_error("unknown depth codec: " + name);
    }

    // ---------------------------------------------------------------------------
    // RVL (Wilson 2017). Encodes zero runs, then zig-zag deltas with a variable
    // number of nibbles. Pure integer work, no tables, no allocation.
    // ---------------------------------------------------------------------------
    class RvlEncoder
    {
    public:
        // out must hold at least (count * 3 + 8) bytes for the pathological case.
        static std::size_t worst_case(std::size_t count)
        {
            return count * 3U + 16U;
        }

        std::size_t encode(const std::uint16_t *input, std::size_t count,
                           std::uint8_t *out)
        {
            buffer_ = reinterpret_cast<std::uint32_t *>(out);
            word_index_ = 0;
            word_ = 0;
            nibbles_written_ = 0;

            const std::uint16_t *end = input + count;
            std::uint16_t previous = 0;
            while (input != end)
            {
                std::uint32_t zeros = 0;
                std::uint32_t nonzeros = 0;
                for (; input != end && *input == 0; ++input, ++zeros)
                {
                }
                encode_vle(zeros);
                for (const std::uint16_t *cursor = input;
                     cursor != end && *cursor != 0; ++cursor, ++nonzeros)
                {
                }
                encode_vle(nonzeros);
                for (std::uint32_t i = 0; i < nonzeros; ++i)
                {
                    const std::uint16_t current = *input++;
                    const std::int32_t delta =
                        static_cast<std::int32_t>(current) -
                        static_cast<std::int32_t>(previous);
                    const std::uint32_t positive =
                        static_cast<std::uint32_t>((delta << 1) ^ (delta >> 31));
                    encode_vle(positive);
                    previous = current;
                }
            }
            if (nibbles_written_ != 0)
                buffer_[word_index_++] = word_ << (4U * (8U - nibbles_written_));
            return static_cast<std::size_t>(word_index_) * sizeof(std::uint32_t);
        }

    private:
        void encode_vle(std::uint32_t value)
        {
            do
            {
                std::uint32_t nibble = value & 0x7U;
                value >>= 3;
                if (value != 0)
                    nibble |= 0x8U;
                word_ = (word_ << 4) | nibble;
                if (++nibbles_written_ == 8U)
                {
                    buffer_[word_index_++] = word_;
                    nibbles_written_ = 0;
                    word_ = 0;
                }
            } while (value != 0);
        }

        std::uint32_t *buffer_ = nullptr;
        std::uint32_t word_index_ = 0;
        std::uint32_t word_ = 0;
        std::uint32_t nibbles_written_ = 0;
    };

    class RvlDecoder
    {
    public:
        void decode(const std::uint8_t *in, std::uint16_t *output,
                    std::size_t count)
        {
            buffer_ = reinterpret_cast<const std::uint32_t *>(in);
            word_index_ = 0;
            word_ = 0;
            nibbles_left_ = 0;

            std::uint16_t previous = 0;
            std::size_t produced = 0;
            while (produced < count)
            {
                std::uint32_t zeros = decode_vle();
                for (std::uint32_t i = 0; i < zeros && produced < count; ++i)
                    output[produced++] = 0;
                std::uint32_t nonzeros = decode_vle();
                for (std::uint32_t i = 0; i < nonzeros && produced < count; ++i)
                {
                    const std::uint32_t positive = decode_vle();
                    const std::int32_t delta = static_cast<std::int32_t>(
                        (positive >> 1) ^ -static_cast<std::int32_t>(positive & 1U));
                    previous = static_cast<std::uint16_t>(
                        static_cast<std::int32_t>(previous) + delta);
                    output[produced++] = previous;
                }
                if (zeros == 0 && nonzeros == 0)
                    break; // malformed stream guard
            }
        }

    private:
        std::uint32_t decode_vle()
        {
            std::uint32_t value = 0;
            std::uint32_t bits = 29;
            std::uint32_t nibble = 0;
            do
            {
                if (nibbles_left_ == 0)
                {
                    word_ = buffer_[word_index_++];
                    nibbles_left_ = 8;
                }
                nibble = word_ & 0xf0000000U;
                value |= (nibble << 1) >> bits;
                word_ <<= 4;
                --nibbles_left_;
                bits -= 3;
            } while ((nibble & 0x80000000U) != 0);
            return value;
        }

        const std::uint32_t *buffer_ = nullptr;
        std::uint32_t word_index_ = 0;
        std::uint32_t word_ = 0;
        std::uint32_t nibbles_left_ = 0;
    };

    // ---------------------------------------------------------------------------
    // Codec driver. One instance per writer thread; owns all scratch buffers so
    // the hot path never allocates.
    // ---------------------------------------------------------------------------
    class Codec
    {
    public:
        Codec(Kind kind, std::size_t pixel_count)
            : kind_(kind), pixels_(pixel_count)
        {
            scratch_.resize(pixel_count * 2U + 64U);
            plane_.resize(pixel_count * 2U + 64U);
            output_capacity_ = RvlEncoder::worst_case(pixel_count) +
                               ZSTD_compressBound(pixel_count * 2U) + 128U;
            if (needs_zstd())
            {
                zstd_ctx_ = ZSTD_createCCtx();
                if (zstd_ctx_ == nullptr)
                    throw std::runtime_error("ZSTD_createCCtx failed");
            }
        }

        ~Codec()
        {
            if (zstd_ctx_ != nullptr)
                ZSTD_freeCCtx(zstd_ctx_);
        }

        Codec(const Codec &) = delete;
        Codec &operator=(const Codec &) = delete;

        std::size_t output_capacity() const { return output_capacity_; }

        // Encodes one packed Z16 frame into out (must be output_capacity() bytes).
        std::size_t encode(const std::uint16_t *depth, std::uint8_t *out)
        {
            switch (kind_)
            {
            case Kind::RAW_Z16:
                std::memcpy(out, depth, pixels_ * 2U);
                return pixels_ * 2U;

            case Kind::RVL:
                return rvl_.encode(depth, pixels_, out);

            case Kind::ZSTD1:
                return zstd(depth, pixels_ * 2U, out, 1);

            case Kind::ZSTD3:
                return zstd(depth, pixels_ * 2U, out, 3);

            case Kind::SPLIT_ZSTD1:
                split_planes(depth, false);
                return zstd(plane_.data(), pixels_ * 2U, out, 1);

            case Kind::SPLIT_ZSTD3:
                split_planes(depth, false);
                return zstd(plane_.data(), pixels_ * 2U, out, 3);

            case Kind::RVL_ZSTD1:
            {
                const std::size_t rvl_bytes =
                    rvl_.encode(depth, pixels_, scratch_.data());
                return zstd(scratch_.data(), rvl_bytes, out, 1);
            }

            case Kind::DELTA_ZSTD1:
                delta_rows(depth);
                return zstd(scratch_.data(), pixels_ * 2U, out, 1);

            case Kind::SPLIT_DELTA_ZSTD1:
                split_planes(depth, true);
                return zstd(plane_.data(), pixels_ * 2U, out, 1);

            case Kind::SHIFT1_RVL:
                shift_into_scratch(depth, 1);
                return rvl_.encode(scratch16(), pixels_, out);

            case Kind::SHIFT2_RVL:
                shift_into_scratch(depth, 2);
                return rvl_.encode(scratch16(), pixels_, out);

            case Kind::SHIFT1_SPLIT_ZSTD1:
                shift_into_scratch(depth, 1);
                split_planes(scratch16(), false);
                return zstd(plane_.data(), pixels_ * 2U, out, 1);

            case Kind::SHIFT2_SPLIT_ZSTD1:
                shift_into_scratch(depth, 2);
                split_planes(scratch16(), false);
                return zstd(plane_.data(), pixels_ * 2U, out, 1);

            case Kind::SHIFT1_RVL_ZSTD1:
            {
                shift_into_scratch(depth, 1);
                const std::size_t rvl_bytes =
                    rvl_.encode(scratch16(), pixels_, plane_.data());
                return zstd(plane_.data(), rvl_bytes, out, 1);
            }
            }
            throw std::runtime_error("unhandled depth codec kind");
        }

    private:
        bool needs_zstd() const
        {
            switch (kind_)
            {
            case Kind::RAW_Z16:
            case Kind::RVL:
            case Kind::SHIFT1_RVL:
            case Kind::SHIFT2_RVL:
                return false;
            default:
                return true;
            }
        }

        std::uint16_t *scratch16()
        {
            return reinterpret_cast<std::uint16_t *>(scratch_.data());
        }

        std::size_t zstd(const void *src, std::size_t src_bytes, std::uint8_t *out,
                         int level)
        {
            const std::size_t result = ZSTD_compressCCtx(
                zstd_ctx_, out, output_capacity_, src, src_bytes, level);
            if (ZSTD_isError(result))
            {
                throw std::runtime_error(
                    std::string("zstd compress failed: ") +
                    ZSTD_getErrorName(result));
            }
            return result;
        }

        // Byte-plane separation: all high bytes first, then all low bytes. High
        // bytes of a depth map are strongly spatially correlated, low bytes are
        // near-noise; keeping them apart lets the entropy coder model each well.
        void split_planes(const std::uint16_t *depth, bool delta)
        {
            std::uint8_t *high = plane_.data();
            std::uint8_t *low = plane_.data() + pixels_;
            if (!delta)
            {
                for (std::size_t i = 0; i < pixels_; ++i)
                {
                    const std::uint16_t value = depth[i];
                    high[i] = static_cast<std::uint8_t>(value >> 8);
                    low[i] = static_cast<std::uint8_t>(value & 0xffU);
                }
                return;
            }
            std::uint8_t previous_high = 0;
            std::uint8_t previous_low = 0;
            for (std::size_t i = 0; i < pixels_; ++i)
            {
                const std::uint16_t value = depth[i];
                const auto value_high = static_cast<std::uint8_t>(value >> 8);
                const auto value_low = static_cast<std::uint8_t>(value & 0xffU);
                high[i] = static_cast<std::uint8_t>(value_high - previous_high);
                low[i] = static_cast<std::uint8_t>(value_low - previous_low);
                previous_high = value_high;
                previous_low = value_low;
            }
        }

        void delta_rows(const std::uint16_t *depth)
        {
            auto *out = reinterpret_cast<std::uint16_t *>(scratch_.data());
            std::uint16_t previous = 0;
            for (std::size_t i = 0; i < pixels_; ++i)
            {
                out[i] = static_cast<std::uint16_t>(depth[i] - previous);
                previous = depth[i];
            }
        }

        void shift_into_scratch(const std::uint16_t *depth, unsigned bits)
        {
            auto *out = reinterpret_cast<std::uint16_t *>(scratch_.data());
            for (std::size_t i = 0; i < pixels_; ++i)
                out[i] = static_cast<std::uint16_t>(depth[i] >> bits);
        }

        Kind kind_;
        std::size_t pixels_;
        std::size_t output_capacity_ = 0;
        std::vector<std::uint8_t> scratch_;
        std::vector<std::uint8_t> plane_;
        RvlEncoder rvl_;
        ZSTD_CCtx *zstd_ctx_ = nullptr;
    };

} // namespace depth_codec

#endif // DEPTH_CODEC_H
