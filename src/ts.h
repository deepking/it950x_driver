#ifndef TS_H

#define TS_H

#include <stdbool.h>
//#include <stdint.h>
#include <linux/string.h>

/** TS packet size */
#define TS_SZ	188

/**
 * Null packet
 */
static inline void ts_reset(unsigned char *header)
{
    memset(header, 0, TS_SZ);
    header[0] = 0x47;
    header[1] = 0x1F;
    header[2] = 0xFF;
    header[3] = 0x10;
}
/** 
 * Check TS error conditions: 
 * sync_byte != 0x47 or transport_error_indicator != 0 or scrambling_control != 0 . 
 */
static inline bool ts_isValidPacket(const unsigned char *header)
{
    return !((header[0] != 0x47) || (header[1] & 0x80) || ((header[3] & 0xC0) != 0));
}
/** 
 * true means start of PES data or PSI otherwise false only.
 */
static inline bool ts_getPUSI(const unsigned char *header)
{
    return header[1] & 0x40;
}
static inline void ts_setPUSI(unsigned char *header, bool b)
{
    if (b)
        header[1] |= 0x40;
    else
        header[1] &= 0xBF;
}
/**
 * Packet ID
 */
static inline uint16_t ts_getPID(const unsigned char *header)
{
    return ((header[1] & 0x1F) << 8) | header[2];
}
static inline void ts_setPID(unsigned char *header, const uint16_t pid)
{
    header[1] &= ((pid >> 8) & 0x1F);
    header[2] = (pid & 0xFF);
}
/** 
 * Incremented only when a payload is present
 */
static inline uint8_t ts_getContinuityCounter(const unsigned char *header)
{
    return header[3] & 0x0F;
}
static inline void ts_setContinuityCounter(unsigned char *header, const uint8_t count)
{
    //header[3] &= 0xF0;// reset
    header[3] |= (count & 0x0F);
}
static inline uint8_t ts_getPayloadPointer(unsigned char *header)
{
    return (uint8_t) header[4];
}
static inline void ts_setPayloadPointer(unsigned char *header, uint8_t pp)
{
    header[4] = pp;
}

#endif /* end of include guard: TS_H */
