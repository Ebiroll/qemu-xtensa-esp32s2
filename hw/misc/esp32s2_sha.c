/*
 * ESP32 SHA accelerator
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "crypto/hash.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/boards.h"
#include "hw/misc/esp32s2_sha.h"

#define ESP32S2_SHA_REGS_SIZE (A_SHA512_BUSY + 4)

/* QEMU hash API includes only the "qcrypto_hash_bytes" function which takes
 * bytes as input, and calculates the digest. It doesn't allow "updating"
 * the state multiple times with blocks of input. Therefore we collect all
 * the input in an array (s->full_text) and when SHA_X_LOAD_REG is set,
 * we call "qcrypto_hash_bytes" to get the digest.
 */
#if 1
static void ESP32S2_sha_text_reg_byteswap_to(Esp32S2ShaState* s, uint32_t* dst, size_t len_words)
{
    for (int i = 0; i < len_words; ++i) {
        dst[i] = __builtin_bswap32(s->text[i]);
    }
}
static void ESP32S2_sha_text_reg_byteswap_result(Esp32S2ShaState* s, uint32_t* dst, size_t len_words)
{
    for (int i = 0; i < len_words; ++i) {
        dst[i] = __builtin_bswap32(s->hash[i]);
    }
}


#endif
/*
static inline QCryptoHashAlgorithm algorithm_for_addr(hwaddr reg_addr)
{
    const QCryptoHashAlgorithm hash_alg_for_reg[] = {
        QCRYPTO_HASH_ALG_SHA1,
        QCRYPTO_HASH_ALG_SHA224,
        QCRYPTO_HASH_ALG_SHA256,
        QCRYPTO_HASH_ALG_SHA384,
        QCRYPTO_HASH_ALG_SHA512,
        QCRYPTO_HASH_ALG_SHA512,
        QCRYPTO_HASH_ALG_SHA512,
        QCRYPTO_HASH_ALG_SHA512
    };
    size_t index = (reg_addr - A_SHA1_START) / 0x10;
    return hash_alg_for_reg[index];
}
*/

static uint32_t hash_block_bytes[] = {
    [QCRYPTO_HASH_ALG_SHA1] = 64,
    [QCRYPTO_HASH_ALG_SHA224] = 64,
    [QCRYPTO_HASH_ALG_SHA256] = 64,
    [QCRYPTO_HASH_ALG_SHA384] = 128,
    [QCRYPTO_HASH_ALG_SHA512] = 128,
};

static QCryptoHashAlgorithm index_to_alg(uint32_t ix) {
    switch (ix) {
        case 0:
           return (QCRYPTO_HASH_ALG_SHA1);
         break;
        case 1:
           return (QCRYPTO_HASH_ALG_SHA224);
         break;
        case 2:
           return (QCRYPTO_HASH_ALG_SHA256);
         break;
        case 3:
           return (QCRYPTO_HASH_ALG_SHA384);
         break;
        case 4:
           return (QCRYPTO_HASH_ALG_SHA512);
         break;
        case 5:
           return (QCRYPTO_HASH_ALG_SHA512);
         break;
        case 6:
           return (QCRYPTO_HASH_ALG_SHA512);
         break;
        case 7:
           return (QCRYPTO_HASH_ALG_SHA512);
         break;

    } 
    return (QCRYPTO_HASH_ALG_SHA1);
}




static void ESP32S2_sha_update_text(Esp32S2ShaState* s, QCryptoHashAlgorithm hash_alg)
{
    uint32_t block_len_bytes = hash_block_bytes[hash_alg];
    //error_report("block_len:%d",block_len_bytes/4);
    if (s->full_text_len + block_len_bytes > s->full_text_reserved) {
        uint32_t full_text_reserved = MAX(s->full_text_reserved * 2, s->full_text_reserved + block_len_bytes);
        uint8_t *new_full_text = g_realloc(s->full_text, full_text_reserved);
        s->full_text_reserved = full_text_reserved;
        s->full_text = new_full_text;
    }
    ESP32S2_sha_text_reg_byteswap_to(s, (uint32_t*) (s->full_text + s->full_text_len), block_len_bytes/4);
    s->full_text_len += block_len_bytes;
}

static void ESP32S2_sha_finish(Esp32S2ShaState *s, QCryptoHashAlgorithm hash_alg)
{
    /* ESP32 SHA accelerator accepts padded blocks (doesn't do any extra padding), but
     * qcrypto_hash_bytes adds padding after the last block. Remove the padding by checking
     * the bit count in the last block. This may give results different from what the hardware
     * would give if the guest application pads the block incorrectly.
     */
    uint8_t *result = (uint8_t*) s->hash;
    size_t result_len = qcrypto_hash_digest_len(hash_alg);
    assert(result_len <= sizeof(s->hash));
    if (s->full_text_len % hash_block_bytes[hash_alg] != 0) {
        error_report("ESP32S2_sha_finish: invalid text length %" PRIx32 "\n", s->full_text_len);
    } else {
        uint32_t byte_count=s->full_text_len;
        //memcpy(&byte_count, s->full_text + s->full_text_len - sizeof(byte_count), sizeof(byte_count));
        //byte_count = __builtin_bswap32(byte_count) / 8;
        if (byte_count > s->full_text_len) {
            error_report("ESP32S2_sha_finish: invalid byte count %" PRIx32 ",%" PRIx32 "\n", byte_count, s->full_text_len);
            qcrypto_hash_bytes(hash_alg, (const char*) s->full_text, byte_count, &result, &result_len, &error_abort);
            ESP32S2_sha_text_reg_byteswap_result(s, (uint32_t*) s->hash, result_len/4);
        } else {
            qcrypto_hash_bytes(hash_alg, (const char*) s->full_text, byte_count, &result, &result_len, &error_abort);
            error_report("ESP32S2_sha_len:%ld",result_len);
            ESP32S2_sha_text_reg_byteswap_result(s, (uint32_t*) s->hash, result_len/4);

        }
    }
    g_free(s->full_text);
    s->full_text = NULL;
    s->full_text_len = 0;
    s->full_text_reserved = 0;
}

static uint64_t ESP32S2_sha_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32S2ShaState *s = ESP32S2_SHA(opaque);
    uint64_t r = 0;
    switch (addr) {
        case 0:
           r=s->mode;
           printf("SHA Mode %d",s->mode);
        break;

        case 0x0040 ... 0x007C:
            {
                //printf("Read hash \n"); 
                if (addr==0x0040)  {
                    ESP32S2_sha_finish(s, index_to_alg(s->mode));                             
                }  
                r=0; // Patch with 0 until calculations works... 
                //r=s->hash[(addr-0x40)/ sizeof(uint32_t)];
            }
        break;
        case 0x0080 ... 0x00FC:
            {
                printf("Read text\n");
                r=s->text[(addr-0x80)/ sizeof(uint32_t)];
            }
        break;


/*        
    case 0 ... (ESP32S2_SHA_TEXT_REG_CNT - 1) * sizeof(uint32_t):
        r = s->text[addr / sizeof(uint32_t)];
        break;
*/
    }

    return r;
}

static void ESP32S2_sha_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32S2ShaState *s = ESP32S2_SHA(opaque);
    switch (addr) {
        case 0:
            s->mode=(value &0xf);
            break;
        case 0x0004: // String content register
             printf("STRING CONTENT\n");
        break;
        case 0x0008: // String length register
             printf("STRING LENGTH\n");
        break;

        case 0x0010: // SHA_START_REG
            ESP32S2_sha_update_text(s, index_to_alg(s->mode));
        break;
        case 0x0014: // SHA_CONTINUE_REG
            //printf("----------\n");
            ESP32S2_sha_update_text(s, index_to_alg(s->mode));
        break;

        case 0x0040 ... 0x007C:
            {
                printf("Write hash\n");
                s->hash[(addr-0x40)/ sizeof(uint32_t)]= value;
            }
        break;
        case 0x0080 ... 0x00FC:
            {
                //printf("Write text %lx\n",(addr-0x80)/ sizeof(uint32_t));
                s->text[(addr-0x80)/ sizeof(uint32_t)]= value;               
            }
        break;
/*    
    switch (addr) {
    case 0 ... (ESP32S2_SHA_TEXT_REG_CNT - 1) * sizeof(uint32_t):
        s->text[addr / sizeof(uint32_t)] = value;
        break;
    case A_SHA1_START:
    case A_SHA1_CONTINUE:
    case A_SHA256_START:
    case A_SHA256_CONTINUE:
    case A_SHA384_START:
    case A_SHA384_CONTINUE:
    case A_SHA512_START:
    case A_SHA512_CONTINUE:
        ESP32S2_sha_update_text(s, algorithm_for_addr(addr));
        break;
    case A_SHA1_LOAD:
    case A_SHA256_LOAD:
    case A_SHA384_LOAD:
    case A_SHA512_LOAD:
        ESP32S2_sha_finish(s, algorithm_for_addr(addr));
        break;
*/
    }

}

static const MemoryRegionOps ESP32S2_sha_ops = {
    .read =  ESP32S2_sha_read,
    .write = ESP32S2_sha_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void ESP32S2_sha_reset(DeviceState *dev)
{
    Esp32S2ShaState *s = ESP32S2_SHA(dev);
    g_free(s->full_text);
    s->full_text = NULL;
    s->full_text_len = 0;
    s->full_text_reserved = 0;
    s->mode = 0;
}

static void ESP32S2_sha_init(Object *obj)
{
    Esp32S2ShaState *s = ESP32S2_SHA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ESP32S2_sha_ops, s,
                          TYPE_ESP32S2_SHA, ESP32S2_SHA_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void ESP32S2_sha_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = ESP32S2_sha_reset;
}

static const TypeInfo ESP32S2_sha_info = {
    .name = TYPE_ESP32S2_SHA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32S2ShaState),
    .instance_init = ESP32S2_sha_init,
    .class_init = ESP32S2_sha_class_init
};

static void ESP32S2_sha_register_types(void)
{
    type_register_static(&ESP32S2_sha_info);
}

type_init(ESP32S2_sha_register_types)
