/**
 * @file libtrpak.h
 * @brief Public API for libtrpak, a Nintendo 64 Transfer Pak (NUS-019) library.
 *
 * libtrpak talks to a Game Boy / Game Boy Color cartridge plugged into a
 * Transfer Pak that is itself plugged into an N64 controller. It powers the
 * accessory, parses the cartridge header, drives the cartridge's memory bank
 * controller (MBC), and copies ROM or save RAM either into a caller-supplied
 * buffer or, through DMA callbacks, straight into flashcart SDRAM.
 *
 * @section trpak_model Addressing model
 *
 * Every Transfer Pak transaction moves exactly ::TRPAK_TRANSFER_BLOCK_SIZE
 * (32) bytes to or from a 16-bit *Transfer Pak address*, which is NOT a Game
 * Boy bus address. The accessory exposes four control/data regions:
 *
 * | Transfer Pak address | Meaning                                          |
 * | -------------------- | ------------------------------------------------ |
 * | `0x8000`             | Power control (write `0x84` on, read back state). |
 * | `0xA000`             | Bank register: which 16 KiB slice of the Game Boy |
 * |                      | address space appears in the data window.         |
 * | `0xB000`             | Status bitfield (`TRPAK_STATUS_*`) / access mode. |
 * | `0xC000`-`0xFFFF`    | 16 KiB data window onto the selected slice.       |
 *
 * The bank register at `0xA000` selects the slice, so the Game Boy address
 * seen through the window is `slice * 0x4000 + (tp_address - 0xC000)`:
 *
 * | Slice | Game Boy range    | Typical use                                 |
 * | ----- | ----------------- | ------------------------------------------- |
 * | `0`   | `0x0000`-`0x3FFF` | Fixed ROM bank 0 and the MBC write registers. |
 * | `1`   | `0x4000`-`0x7FFF` | Switchable ROM bank (ROM dumping).            |
 * | `2`   | `0x8000`-`0xBFFF` | Cartridge RAM appears at `0xE000`-`0xFFFF`.   |
 *
 * That is why the low-level helpers accept `0xC000`-`0xFFE0` for ROM and
 * `0xE000`-`0xFFE0` for RAM: RAM only exists in the upper half of slice 2.
 *
 * @section trpak_usage Usage
 *
 * 1. Optionally install a custom backend with trpak_configure_io(); otherwise
 *    the libdragon/EverDrive 64 backend is used automatically.
 * 2. Call trpak_init(). On ::TRPAK_OK the global ::trcart describes the
 *    inserted cartridge.
 * 3. Dump or restore data with the bulk helpers.
 * 4. Always call trpak_shutdown(), including on error paths.
 *
 * Every function returns a ::trpak_result code (`0` on success, negative on
 * failure); trpak_error_string() turns one into a printable message.
 *
 * @note The library keeps a single global state (::trcart plus the runtime
 *       backend), so it drives one Transfer Pak at a time and is not
 *       thread-safe or reentrant.
 */
#ifndef LIBTRPAK_H
#define LIBTRPAK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Library version
 *
 * The build version of the library, reported by trpak_version_string() and
 * trpak_version(). All four macros are overridable: define them before
 * including this header — with `-D` on the compiler command line or with a
 * bare `#define` — to change what the accessors return, which is how an
 * application makes the library report the application's version rather than
 * its own.
 * @{
 */
#ifndef TRPAK_VERSION
#define TRPAK_VERSION       "1.0.0"
#endif
#ifndef TRPAK_VERSION_MAJOR
#define TRPAK_VERSION_MAJOR 1
#endif
#ifndef TRPAK_VERSION_MINOR
#define TRPAK_VERSION_MINOR 0
#endif
#ifndef TRPAK_VERSION_PATCH
#define TRPAK_VERSION_PATCH 0
#endif
/** @} */

/**
 * @name Mapper identifiers
 *
 * Values stored in trpak_cart::mapper. They are libtrpak's own identifiers,
 * derived from the raw cartridge type byte at Game Boy address `0x0147`; they
 * are not the cartridge type byte itself. Use trpak_mapper_is_supported() to
 * find out whether a banking path is actually implemented for one of them.
 * @{
 */
#define TRPAK_MAPPER_NONE   0x00u /**< Plain ROM, optionally with unbanked RAM. */
#define TRPAK_MAPPER_MBC1   0x01u /**< MBC1, up to its full 128 banks (2 MiB). */
#define TRPAK_MAPPER_MBC2   0x02u /**< MBC2, including its 512 x 4-bit RAM. */
#define TRPAK_MAPPER_MMM01  0x03u /**< Detected only; no banking implemented. */
#define TRPAK_MAPPER_MBC3   0x04u /**< MBC3/MBC30; RTC registers not driven. */
#define TRPAK_MAPPER_MBC5   0x05u /**< MBC5; rumble motor is never engaged. */
#define TRPAK_MAPPER_CAMERA 0x06u /**< Game Boy Camera; camera regs unsupported. */
#define TRPAK_MAPPER_TAMA5  0x07u /**< Detected only; no banking implemented. */
#define TRPAK_MAPPER_HUC3   0x08u /**< Detected only; no banking implemented. */
#define TRPAK_MAPPER_HUC1   0x09u /**< HuC1; dumping is limited to 64 banks. */
#define TRPAK_MAPPER_MBC4   0x10u /**< Recognized but not implemented. */
#define TRPAK_MAPPER_MBC6   0x11u
#define TRPAK_MAPPER_MBC7   0x12u
/** @} */

/**
 * @name Sizes and defaults
 * @{
 */
/** Size of one Game Boy ROM bank, and of the Transfer Pak data window. */
#define TRPAK_ROM_BANK_SIZE      (16u * 1024u)
/** Payload size of a single Transfer Pak read or write transaction. */
#define TRPAK_TRANSFER_BLOCK_SIZE 32u
/** Size of one cartridge RAM bank as seen through the window. */
#define TRPAK_RAM_BANK_SIZE       (8u * 1024u)
/** Bytes of Game Boy header consumed by the parser (`0x0100`-`0x014F`). */
#define TRPAK_HEADER_SIZE         0x50u
/** Default destination for the DMA helpers: EverDrive 64 SDRAM. */
#define TRPAK_DEFAULT_DMA_BASE    ((uintptr_t)0xB2000000u)
/** @} */

/**
 * @name Transfer Pak status bits
 *
 * Bits of the byte returned by trpak_get_status(), read from Transfer Pak
 * address `0xB000`.
 * @{
 */
#define TRPAK_STATUS_READY        0x01u /**< Cartridge present and accessible. */
#define TRPAK_STATUS_IS_RESETTING 0x04u /**< Cartridge is booting/resetting; wait. */
#define TRPAK_STATUS_WAS_RESET    0x08u /**< A reset happened since last read;
                                             read-and-clear, so each read
                                             reports only new resets. */
#define TRPAK_STATUS_REMOVED      0x40u /**< No cartridge, or it was pulled out. */
#define TRPAK_STATUS_POWERED      0x80u /**< Accessory power is currently on. */
/** @} */

/**
 * @brief Result codes returned by every libtrpak entry point.
 *
 * ::TRPAK_OK is `0`; all failures are negative, so `result != TRPAK_OK` and
 * `result < 0` are equivalent tests.
 */
typedef enum trpak_result {
    TRPAK_OK = 0,                        /**< Operation succeeded. */
    TRPAK_ERR_IO = -1,                   /**< Backend transfer failed or no backend is usable. */
    TRPAK_ERR_POWER_STATE = -2,          /**< Power register reported an unexpected value. */
    TRPAK_ERR_ACCESS_STATE = -3,         /**< Accessory is not ready, or is resetting. */
    TRPAK_ERR_POWER_OFF = -4,            /**< Power dropped in the middle of an operation. */
    TRPAK_ERR_INVALID_ARGUMENT = -10,    /**< NULL pointer, bad size, misaligned or out-of-range address. */
    TRPAK_ERR_BUFFER_TOO_SMALL = -11,    /**< Destination buffer cannot hold the whole transfer. */
    TRPAK_ERR_UNSUPPORTED_CARTRIDGE = -12,/**< Cartridge type or mapper has no safe implementation. */
    TRPAK_ERR_INVALID_BANK = -13,        /**< Requested bank is out of range for the mapper. */
    TRPAK_ERR_NO_RAM = -14,              /**< Cartridge has no usable save RAM. */
    TRPAK_ERR_VERIFY_FAILED = -15,       /**< Data read back differs from the data written. */
    TRPAK_ERR_INVALID_HEADER = -16,      /**< Bad checksum or unknown size code in the header. */
    TRPAK_ERR_NO_CARTRIDGE = -17,        /**< No Game Boy cartridge detected in the Transfer Pak. */
    TRPAK_ERR_TRANSFER_TIMEOUT = -18     /**< Accessory never became ready within the poll budget. */
} trpak_result;

/**
 * @brief Metadata decoded from the Game Boy cartridge header.
 *
 * Filled by trpak_parse_cartridge_header(), and by trpak_init() into the
 * global ::trcart. Fields prefixed with an underscore hold raw header bytes;
 * the remaining fields are decoded values. Only read this after the producing
 * call returned ::TRPAK_OK.
 */
typedef struct trpak_cart {
    uint8_t mapper;         /**< One of the `TRPAK_MAPPER_*` identifiers. */
    uint8_t ram;            /**< Non-zero when the cartridge has usable save RAM.
                                 Cleared when the type byte claims RAM but the
                                 size code at `0x0149` is `0`. */
    uint8_t battery;        /**< Non-zero when the RAM is battery-backed. */
    uint8_t rtc;            /**< Non-zero for MBC3 real-time clock cartridges. */
    uint8_t rumble;         /**< Non-zero for MBC5 rumble cartridges. */
    uint8_t sgb;            /**< Raw Super Game Boy flag byte (`0x0146`). */
    uint8_t gbc;            /**< `0x80` GB/GBC, `0xC0` GBC-only, `0` otherwise. */
    char title[17];         /**< Title: inferred 11/15-byte CGB layout, or 16
                                 bytes otherwise; always NUL-terminated. */
    uint8_t _romsize;       /**< Raw ROM size code (`0x0148`). */
    uint8_t _ramsize;       /**< Raw RAM size code (`0x0149`). */
    uint8_t cartridge_type; /**< Raw cartridge type byte (`0x0147`). */
    uint32_t romsize;       /**< Decoded ROM size in bytes. */
    uint32_t ramsize;       /**< Decoded save RAM size in bytes. */
    uint16_t rombanks;      /**< Number of 16 KiB ROM banks. */
    uint16_t rambanks;      /**< Number of RAM banks (8 KiB each, 1 for MBC2). */
    uint16_t bank;          /**< Last ROM bank successfully selected. */
    uint16_t cpld;          /**< Reserved for future use; always zero. */
} trpak_cart;

/**
 * @brief Reads one 32-byte block from a Transfer Pak address.
 *
 * @param user       Opaque pointer taken from trpak_io::user.
 * @param controller Controller port index, `0`-`3`.
 * @param address    Transfer Pak address; always 32-byte aligned.
 * @param data       Destination for exactly ::TRPAK_TRANSFER_BLOCK_SIZE bytes.
 * @return `0` on success, any non-zero value on failure (mapped to
 *         ::TRPAK_ERR_IO).
 */
typedef int (*trpak_read_block_fn)(
    void *user,
    int controller,
    uint16_t address,
    uint8_t data[TRPAK_TRANSFER_BLOCK_SIZE]
);

/**
 * @brief Writes one 32-byte block to a Transfer Pak address.
 *
 * @param user       Opaque pointer taken from trpak_io::user.
 * @param controller Controller port index, `0`-`3`.
 * @param address    Transfer Pak address; always 32-byte aligned.
 * @param data       Source of exactly ::TRPAK_TRANSFER_BLOCK_SIZE bytes.
 * @return `0` on success, any non-zero value on failure (mapped to
 *         ::TRPAK_ERR_IO).
 */
typedef int (*trpak_write_block_fn)(
    void *user,
    int controller,
    uint16_t address,
    const uint8_t data[TRPAK_TRANSFER_BLOCK_SIZE]
);

/**
 * @brief Busy-waits for the requested number of milliseconds.
 *
 * Used after power changes and while polling for readiness. When absent, the
 * library skips the waits; without it the readiness polls run back to back, so
 * their timeout budget is spent in attempts rather than wall time. That is safe
 * for simulated backends but not recommended on real hardware.
 *
 * @param user         Opaque pointer taken from trpak_io::user.
 * @param milliseconds Delay length; always greater than zero.
 */
typedef void (*trpak_delay_fn)(void *user, unsigned int milliseconds);

/**
 * @brief Stores a block that was read from the cartridge into DMA memory.
 *
 * @param user        Opaque pointer taken from trpak_io::user.
 * @param source      Block just read from the cartridge.
 * @param destination Absolute address, i.e. the configured DMA base plus the
 *                    running byte offset of the dump.
 * @param size        Always ::TRPAK_TRANSFER_BLOCK_SIZE.
 * @return `0` on success, non-zero on failure (mapped to ::TRPAK_ERR_IO).
 */
typedef int (*trpak_dma_store_fn)(
    void *user,
    const uint8_t *source,
    uintptr_t destination,
    size_t size
);

/**
 * @brief Loads a block from DMA memory so it can be written to the cartridge.
 *
 * @param user        Opaque pointer taken from trpak_io::user.
 * @param destination Scratch block filled by the callback.
 * @param source      Absolute address, i.e. the configured DMA base plus the
 *                    running byte offset of the restore.
 * @param size        Always ::TRPAK_TRANSFER_BLOCK_SIZE.
 * @return `0` on success, non-zero on failure (mapped to ::TRPAK_ERR_IO).
 */
typedef int (*trpak_dma_load_fn)(
    void *user,
    uint8_t *destination,
    uintptr_t source,
    size_t size
);

/**
 * @brief Transport backend used for every accessory access.
 *
 * trpak_io::read_block and trpak_io::write_block are mandatory. The remaining
 * callbacks are optional: without trpak_io::delay the library never sleeps,
 * and without the DMA callbacks only the buffer-based API works — the `_dma`
 * helpers then fail with ::TRPAK_ERR_IO.
 */
typedef struct trpak_io {
    trpak_read_block_fn read_block;   /**< Required 32-byte read primitive. */
    trpak_write_block_fn write_block; /**< Required 32-byte write primitive. */
    trpak_delay_fn delay;             /**< Optional millisecond delay. */
    trpak_dma_store_fn dma_store;     /**< Optional cartridge-to-DMA copy. */
    trpak_dma_load_fn dma_load;       /**< Optional DMA-to-cartridge copy. */
    void *user;                       /**< Opaque context passed to callbacks. */
} trpak_io;

/**
 * @brief Metadata of the cartridge described by the last successful
 *        trpak_init().
 *
 * Zeroed at the start of trpak_init() and only meaningful after it returns
 * ::TRPAK_OK. The bulk helpers read this global to know how much data to move
 * and which mapper sequences to use.
 */
extern trpak_cart trcart;

/* ------------------------------------------------------------------------ */
/* Version                                                                   */
/* ------------------------------------------------------------------------ */

/**
 * @brief Returns the library version as a single static string.
 *
 * @return ::TRPAK_VERSION; never NULL.
 */
const char *trpak_version_string(void);

/**
 * @brief Returns the major, minor, and patch components separately.
 *
 * @param major Receives ::TRPAK_VERSION_MAJOR.
 * @param minor Receives ::TRPAK_VERSION_MINOR.
 * @param patch Receives ::TRPAK_VERSION_PATCH.
 * @retval TRPAK_OK                   Components written.
 * @retval TRPAK_ERR_INVALID_ARGUMENT Any output pointer is NULL.
 */
int trpak_version(unsigned int *major, unsigned int *minor, unsigned int *patch);

/* ------------------------------------------------------------------------ */
/* Configuration and lifecycle                                              */
/* ------------------------------------------------------------------------ */

/**
 * @brief Installs a transport backend, controller port, and DMA base address.
 *
 * The structure is copied, so the caller may discard it afterwards. Call this
 * before trpak_init() when targeting a port other than 1, another flashcart,
 * or a simulated accessory.
 *
 * @param io         Backend with at least read and write callbacks set.
 * @param controller Controller port index, `0` (port 1) through `3` (port 4).
 * @param dma_base   Base address used by the `_dma` helpers.
 * @retval TRPAK_OK                  Backend installed.
 * @retval TRPAK_ERR_INVALID_ARGUMENT `io` is NULL, a required callback is
 *                                    missing, or the port is out of range.
 */
int trpak_configure_io(const trpak_io *io, int controller, uintptr_t dma_base);

/**
 * @brief Restores the built-in libdragon backend on port 1 with the default
 *        DMA base (::TRPAK_DEFAULT_DMA_BASE).
 *
 * Also called implicitly the first time an operation runs without a configured
 * backend. When the library is compiled with `TRPAK_NO_DEFAULT_IO` this only
 * clears the runtime state, leaving it unconfigured, so every later call fails
 * with ::TRPAK_ERR_IO until trpak_configure_io() succeeds. DMA callbacks are
 * only installed when compiled with `TRPAK_ENABLE_ED64_DMA`.
 */
void trpak_use_default_io(void);

/**
 * @brief Powers up the accessory, enables access mode, and parses the header.
 *
 * Performs the full bring-up sequence: power off, confirm off, power on,
 * confirm on, enable access mode, poll until the cartridge reports ready
 * (up to ~500 ms with a delay callback installed), select ROM bank 0, read
 * Game Boy addresses `0x0100`-`0x014F` in three blocks, validate the header
 * checksum, and reject mappers with no implemented banking path. On failure it
 * powers the accessory back down before returning, and ::trcart is left zeroed
 * or partially filled.
 *
 * @retval TRPAK_OK                     Cartridge detected and described in ::trcart.
 * @retval TRPAK_ERR_IO                 No usable backend, or a transfer failed.
 * @retval TRPAK_ERR_POWER_STATE        Power register did not follow the requested state.
 * @retval TRPAK_ERR_ACCESS_STATE       Accessory never reached a ready state.
 * @retval TRPAK_ERR_TRANSFER_TIMEOUT   Readiness polling expired.
 * @retval TRPAK_ERR_NO_CARTRIDGE       Removal bit set: no cartridge inserted.
 * @retval TRPAK_ERR_INVALID_HEADER     Checksum mismatch or unknown size code.
 * @retval TRPAK_ERR_UNSUPPORTED_CARTRIDGE Unknown cartridge type or unimplemented mapper.
 */
int trpak_init(void);

/**
 * @brief Disables RAM (when the cartridge has any), leaves access mode, and
 *        powers the accessory off.
 *
 * Every step runs even if an earlier one fails; the first error encountered is
 * the one returned. Safe to call after a failed trpak_init().
 *
 * @return ::TRPAK_OK, or the first error produced by the shutdown steps.
 */
int trpak_shutdown(void);

/**
 * @brief Maps a result code to a short, static English description.
 *
 * @param result Any ::trpak_result value.
 * @return Pointer to a string literal; never NULL and never needs freeing.
 *         Unknown codes yield `"unknown libtrpak error"`.
 */
const char *trpak_error_string(int result);

/* ------------------------------------------------------------------------ */
/* Header parsing and capabilities (hardware-free)                           */
/* ------------------------------------------------------------------------ */

/**
 * @brief Decodes a Game Boy cartridge header into a ::trpak_cart.
 *
 * Pure function over a byte buffer, so it can be unit-tested without an N64.
 * The buffer holds Game Boy addresses `0x0100`-`0x014F`; index `0x47` of the
 * buffer is therefore the cartridge type byte at Game Boy address `0x0147`.
 * The checksum is validated first, and `out` is zeroed before any field is
 * written. `out` may be left partially filled when a later step fails.
 *
 * A cartridge whose type byte claims RAM while the size code at `0x0149` is
 * `0` is decoded as RAM-less — trpak_cart::ram, ::rambanks and ::ramsize all
 * end up zero — rather than rejected, so its ROM stays dumpable. MBC2 is
 * unaffected: its 512 half-bytes are implied by the mapper, not by the code.
 *
 * @param header      Buffer holding Game Boy addresses `0x0100`-`0x014F`.
 * @param header_size Size of that buffer; must be at least ::TRPAK_HEADER_SIZE.
 * @param out         Destination metadata structure.
 * @retval TRPAK_OK                        Header decoded.
 * @retval TRPAK_ERR_INVALID_ARGUMENT      NULL pointer or short buffer.
 * @retval TRPAK_ERR_INVALID_HEADER        Bad checksum, or unknown ROM/RAM size code.
 * @retval TRPAK_ERR_UNSUPPORTED_CARTRIDGE Unknown cartridge type byte.
 */
int trpak_parse_cartridge_header(
    const uint8_t header[TRPAK_HEADER_SIZE],
    size_t header_size,
    trpak_cart *out
);

/**
 * @brief Verifies the Game Boy header checksum.
 *
 * Recomputes the standard algorithm over buffer indices `0x34`-`0x4C` (Game
 * Boy addresses `0x0134`-`0x014C`) and compares it with the stored byte at
 * index `0x4D`.
 *
 * @param header      Buffer holding Game Boy addresses `0x0100`-`0x014F`.
 * @param header_size Size of that buffer; must be at least ::TRPAK_HEADER_SIZE.
 * @return `true` when the checksum matches; `false` on mismatch, a NULL
 *         pointer, or a short buffer.
 */
bool trpak_check_header_checksum(
    const uint8_t header[TRPAK_HEADER_SIZE],
    size_t header_size
);

/**
 * @brief Reports whether a mapper has an implemented banking path.
 *
 * Detection and banking are separate concerns: MMM01, MBC4, HuC3, and TAMA5
 * are recognized by the parser but return `false` here, and trpak_init()
 * rejects them with ::TRPAK_ERR_UNSUPPORTED_CARTRIDGE.
 *
 * @param mapper One of the `TRPAK_MAPPER_*` identifiers.
 * @return `true` when ROM/RAM banking is implemented for that mapper.
 */
bool trpak_mapper_is_supported(uint8_t mapper);

/* ------------------------------------------------------------------------ */
/* Bulk operations into caller-supplied buffers                              */
/* ------------------------------------------------------------------------ */

/**
 * @brief Dumps the whole ROM into a buffer.
 *
 * Walks every bank in `trcart.rombanks`, checking cartridge status before and
 * after each block, and always tries to restore ROM bank 0 afterwards. If the
 * accessory resets mid-bank, the current bank is re-selected and the affected
 * block is retried, since a reset clears the mapper's bank latch while leaving
 * the status reporting a healthy, ready cartridge. A cartridge
 * claiming more banks than its mapper's register can select is refused up
 * front rather than dumped with wrapped banks: ROM-only above 2, MBC2 above
 * 16, HuC1 and Camera above 64, conventional MBC1 above 128, MBC1M above 64,
 * MBC3 above 128, MBC5 above 512.
 *
 * @param destination Buffer of at least `trcart.romsize` bytes; must not be NULL.
 * @param capacity    Size of `destination` in bytes.
 * @param bytes_read  Optional; receives the number of bytes actually copied.
 *                    Always written, including on partial failures and on the
 *                    validation failures below, where it is set to `0`.
 * @retval TRPAK_OK                        Whole ROM copied.
 * @retval TRPAK_ERR_BUFFER_TOO_SMALL      NULL buffer or capacity below `trcart.romsize`.
 * @retval TRPAK_ERR_UNSUPPORTED_CARTRIDGE Mapper size combination that cannot be dumped safely.
 * @retval TRPAK_ERR_NO_CARTRIDGE          Cartridge removed mid-dump.
 * @retval TRPAK_ERR_TRANSFER_TIMEOUT       Cartridge never became ready, or reset repeatedly.
 * @retval TRPAK_ERR_IO                    Transfer failure.
 */
int trpak_read_rom(uint8_t *destination, size_t capacity, size_t *bytes_read);

/**
 * @brief Backs up cartridge save RAM into a buffer.
 *
 * Enables RAM, copies every RAM bank, normalizes MBC2's 4-bit cells to the low
 * nibble, and disables RAM again even when the copy fails. Status is checked
 * on both sides of each transaction. A reset mid-bank re-locks cartridge RAM
 * as well as clearing the bank register, so the bank is re-selected and the
 * affected block retried rather than read off the floating bus. As in
 * trpak_read_rom(), a header declaring more banks than the mapper can select
 * is refused up front instead of yielding a truncated backup.
 *
 * @param destination Buffer of at least `trcart.ramsize` bytes; must not be NULL.
 * @param capacity    Size of `destination` in bytes.
 * @param bytes_read  Optional; receives the number of bytes actually copied.
 *                    Always written, `0` when the operation is refused.
 * @retval TRPAK_OK                        Save copied.
 * @retval TRPAK_ERR_NO_RAM                Cartridge declares no save RAM.
 * @retval TRPAK_ERR_BUFFER_TOO_SMALL      NULL buffer or capacity below `trcart.ramsize`.
 * @retval TRPAK_ERR_UNSUPPORTED_CARTRIDGE More RAM banks than the mapper can select.
 * @retval TRPAK_ERR_NO_CARTRIDGE          Cartridge removed mid-operation.
 * @retval TRPAK_ERR_TRANSFER_TIMEOUT       Cartridge never became ready, or reset repeatedly.
 * @retval TRPAK_ERR_IO                    Transfer failure.
 */
int trpak_read_save(uint8_t *destination, size_t capacity, size_t *bytes_read);

/**
 * @brief Restores a save into cartridge RAM, optionally verifying each block.
 *
 * `size` must equal `trcart.ramsize` exactly — truncated or oversized saves are
 * rejected instead of being written partially. Status is checked on both sides
 * of each transaction. A reset mid-bank re-locks cartridge RAM, which would
 * make the cartridge drop every later write, so the bank is re-selected and
 * the affected block retried. With `verify_after_write` set, every 32-byte
 * block is read back and compared immediately, so the operation stops at the
 * first mismatch. RAM is disabled again on all paths.
 *
 * @warning This overwrites the cartridge save. Back it up with
 *          trpak_read_save() first; verification cannot recover lost data.
 *
 * @param source            Save image of exactly `trcart.ramsize` bytes.
 * @param size              Must equal `trcart.ramsize`.
 * @param verify_after_write Read back and compare each written block.
 * @retval TRPAK_OK                        Save written (and verified, if requested).
 * @retval TRPAK_ERR_NO_RAM                Cartridge declares no save RAM.
 * @retval TRPAK_ERR_INVALID_ARGUMENT      NULL source, or size differs from `trcart.ramsize`.
 * @retval TRPAK_ERR_UNSUPPORTED_CARTRIDGE More RAM banks than the mapper can select.
 * @retval TRPAK_ERR_VERIFY_FAILED         A block read back differently than written.
 * @retval TRPAK_ERR_NO_CARTRIDGE          Cartridge removed mid-operation.
 * @retval TRPAK_ERR_TRANSFER_TIMEOUT       Cartridge never became ready, or reset repeatedly.
 * @retval TRPAK_ERR_IO                    Transfer failure.
 */
int trpak_write_save(
    const uint8_t *source,
    size_t size,
    bool verify_after_write
);

/* ------------------------------------------------------------------------ */
/* Bulk operations through the DMA callbacks                                 */
/* ------------------------------------------------------------------------ */

/**
 * @brief Dumps the ROM straight to the configured DMA base address.
 *
 * Same traversal and validation as trpak_read_rom(), but each block goes
 * through trpak_io::dma_store instead of a memcpy, so no RDRAM buffer sized
 * for the whole ROM is needed.
 *
 * @param bytes_read Optional; receives the number of bytes transferred.
 * @return ::TRPAK_OK, ::TRPAK_ERR_IO when no DMA callback is installed or a
 *         transfer fails, or any error from trpak_read_rom().
 */
int trpak_read_rom_dma(size_t *bytes_read);

/**
 * @brief Backs up cartridge save RAM to the configured DMA base address.
 *
 * @param bytes_read Optional; receives the number of bytes transferred.
 * @return ::TRPAK_OK, ::TRPAK_ERR_IO when no DMA callback is installed or a
 *         transfer fails, or any error from trpak_read_save().
 */
int trpak_read_save_dma(size_t *bytes_read);

/**
 * @brief Restores a save read from the configured DMA base address.
 *
 * Exactly `trcart.ramsize` bytes are pulled through trpak_io::dma_load.
 *
 * @param verify_after_write Read back and compare each written block.
 * @return ::TRPAK_OK, ::TRPAK_ERR_IO when no DMA callback is installed or a
 *         transfer fails, or any error from trpak_write_save().
 */
int trpak_write_save_dma(bool verify_after_write);

/* ------------------------------------------------------------------------ */
/* Power, access state, and banking                                          */
/* ------------------------------------------------------------------------ */

/**
 * @brief Switches accessory power on or off.
 *
 * Writes the magic value to Transfer Pak address `0x8000` and, when a delay
 * callback exists, waits 200 ms for the cartridge to settle.
 *
 * @param enabled `true` powers up, `false` powers down.
 * @return ::TRPAK_OK or ::TRPAK_ERR_IO.
 */
int trpak_set_power(bool enabled);

/**
 * @brief Reads back the accessory power state.
 *
 * Only the enable magic (`0x84`) is treated as powered on; any other byte —
 * the value written to power down, `0x00`, or an undefined read-back — is
 * reported as off, so hardware that echoes its power-down write still works.
 *
 * @param enabled Receives `true` when powered, `false` when off.
 * @retval TRPAK_OK                   State read.
 * @retval TRPAK_ERR_INVALID_ARGUMENT `enabled` is NULL.
 * @retval TRPAK_ERR_IO               Transfer failure.
 */
int trpak_get_power(bool *enabled);

/**
 * @brief Enables or disables Transfer Pak access mode.
 *
 * Access mode must be on before the cartridge data window can be used, and is
 * turned off again by trpak_shutdown().
 *
 * @param enabled `true` enables access mode, `false` disables it.
 * @return ::TRPAK_OK or ::TRPAK_ERR_IO.
 */
int trpak_set_access_state(bool enabled);

/**
 * @brief Reads the raw status bitfield from Transfer Pak address `0xB000`.
 *
 * @param status Receives a combination of the `TRPAK_STATUS_*` bits.
 * @retval TRPAK_OK                   Status read.
 * @retval TRPAK_ERR_INVALID_ARGUMENT `status` is NULL.
 * @retval TRPAK_ERR_IO               Transfer failure.
 */
int trpak_get_status(uint8_t *status);

/**
 * @brief Condenses the status bitfield into a single access state.
 *
 * Checked in priority order, so a removed cartridge always wins over the
 * ready bit:
 *
 * | Value | Meaning                                   |
 * | ----- | ----------------------------------------- |
 * | `3`   | Cartridge removed or absent.              |
 * | `2`   | Reset in progress; retry shortly.         |
 * | `1`   | Ready for access.                         |
 * | `0`   | Powered but not ready (e.g. access mode off). |
 *
 * @param state Receives the state value described above.
 * @retval TRPAK_OK                   State computed.
 * @retval TRPAK_ERR_INVALID_ARGUMENT `state` is NULL.
 * @retval TRPAK_ERR_IO               Transfer failure.
 */
int trpak_get_access_state(int *state);

/**
 * @brief Selects the ROM bank exposed in the data window.
 *
 * Runs the write sequence required by `trcart.mapper` and, on success, records
 * the bank in `trcart.bank`. The requested bank is always reachable through
 * the same window addresses afterwards: bank `0` leaves the window on the
 * fixed region (Game Boy `0x0000`-`0x3FFF`) and other banks on the switchable
 * region (`0x4000`-`0x7FFF`), except for MBC1 banks `0x20`, `0x40` and `0x60`,
 * which only exist in the fixed region under advanced banking mode.
 *
 * @param bank Bank index; must be below `trcart.rombanks` and within the
 *             mapper's own limit (ROM-only `0x00`-`0x01`, MBC1 `0x00`-`0x7F`,
 *             MBC2 `0x00`-`0x0F`, MBC3 `0x00`-`0x7F`, MBC5 `0x000`-`0x1FF`,
 *             HuC1 and Camera `0x00`-`0x3F`). On a mapper-less cartridge the
 *             bank number is written straight to the window's slice register,
 *             so only the two slices that name ROM are accepted.
 * @retval TRPAK_OK                        Bank selected.
 * @retval TRPAK_ERR_INVALID_BANK          Bank out of range for the cartridge or mapper.
 * @retval TRPAK_ERR_UNSUPPORTED_CARTRIDGE Mapper has no implemented sequence.
 * @retval TRPAK_ERR_IO                    Transfer failure.
 */
int trpak_select_rom_bank(uint16_t bank);

/**
 * @brief Enables cartridge RAM and selects a RAM bank.
 *
 * Writes the RAM-enable magic (`0x0A`) and then the bank number using the
 * mapper's register layout. MBC2 has a single implicit bank, so only the
 * enable step runs, as does MBC1M — whose GB `0x6000`/`0x4000` registers
 * select a sub-game rather than a RAM bank. MBC3 is capped at 8 banks because
 * higher values address the RTC registers, and rumble MBC5 cartridges at 8
 * because the next bit of that register drives the motor.
 *
 * Every range check runs before the enable write, so a rejected bank never
 * leaves cartridge RAM unlocked.
 *
 * @param bank RAM bank index, below `trcart.rambanks` and within the mapper's
 *             own limit (ROM-only, MBC2 and MBC1M `0`, MBC1 and HuC1 `0`-`3`,
 *             MBC3 and rumble MBC5 `0`-`7`, MBC5 and Camera `0`-`15`).
 * @retval TRPAK_OK                        RAM enabled and bank selected.
 * @retval TRPAK_ERR_NO_RAM                Cartridge declares no RAM.
 * @retval TRPAK_ERR_INVALID_BANK          Bank out of range for the cartridge or mapper.
 * @retval TRPAK_ERR_UNSUPPORTED_CARTRIDGE Mapper has no implemented sequence.
 * @retval TRPAK_ERR_IO                    Transfer failure.
 */
int trpak_select_ram_bank(uint16_t bank);

/**
 * @brief Disables and write-protects cartridge RAM again.
 *
 * A no-op for cartridges without RAM or without an MBC. HuC1 cannot truly
 * disable RAM, so it is only kept in RAM mode rather than infrared mode. On a
 * conventional MBC1 this also restores simple banking mode, which
 * trpak_select_ram_bank() had to leave in advanced mode; MBC1M never changes
 * it.
 *
 * @return ::TRPAK_OK or ::TRPAK_ERR_IO.
 */
int trpak_disable_ram(void);

/* ------------------------------------------------------------------------ */
/* Low-level 32-byte window accessors                                        */
/* ------------------------------------------------------------------------ */

/**
 * @brief Reads 32 bytes from the ROM data window.
 *
 * Reads whichever bank was last selected with trpak_select_rom_bank(); this
 * function does no banking of its own.
 *
 * @param address 32-byte aligned Transfer Pak address in `0xC000`-`0xFFE0`.
 * @param data    Destination for ::TRPAK_TRANSFER_BLOCK_SIZE bytes.
 * @retval TRPAK_OK                   Block read.
 * @retval TRPAK_ERR_INVALID_ARGUMENT NULL buffer, or misaligned/out-of-range address.
 * @retval TRPAK_ERR_IO               Transfer failure.
 */
int trpak_read_rom_block(uint16_t address, uint8_t *data);

/**
 * @brief Reads 32 bytes from the cartridge RAM window.
 *
 * Switches the window to the Game Boy `0x8000`-`0xBFFF` slice first, so the
 * address range maps onto cartridge RAM at Game Boy `0xA000`-`0xBFFF`. RAM
 * must already be enabled through trpak_select_ram_bank().
 *
 * @param address 32-byte aligned Transfer Pak address in `0xE000`-`0xFFE0`.
 * @param data    Destination for ::TRPAK_TRANSFER_BLOCK_SIZE bytes.
 * @retval TRPAK_OK                   Block read.
 * @retval TRPAK_ERR_INVALID_ARGUMENT NULL buffer, or misaligned/out-of-range address.
 * @retval TRPAK_ERR_IO               Transfer failure.
 */
int trpak_read_ram_block(uint16_t address, uint8_t *data);

/**
 * @brief Writes 32 bytes into the cartridge RAM window.
 *
 * Same windowing and preconditions as trpak_read_ram_block(). No verification
 * is performed here; use trpak_write_save() for verified restores.
 *
 * @param address 32-byte aligned Transfer Pak address in `0xE000`-`0xFFE0`.
 * @param data    Source of ::TRPAK_TRANSFER_BLOCK_SIZE bytes.
 * @retval TRPAK_OK                   Block written.
 * @retval TRPAK_ERR_INVALID_ARGUMENT NULL buffer, or misaligned/out-of-range address.
 * @retval TRPAK_ERR_IO               Transfer failure.
 */
int trpak_write_ram_block(uint16_t address, const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
