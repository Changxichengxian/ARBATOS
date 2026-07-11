/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BSP_RESET_EVIDENCE_POLICY_H
#define BSP_RESET_EVIDENCE_POLICY_H

#include <stdint.h>

#include "BspResetEvidence.h"

#define BSP_RESET_EVIDENCE_MAGIC 0x52464556u
#define BSP_RESET_EVIDENCE_CHECKSUM_SEED 2166136261u

typedef struct
{
    uint32_t magic;
    uint32_t magicInv;
    BspResetEvidenceRecord record;
    uint32_t checksum;
    uint32_t checksumInv;
} BspResetEvidenceStorage;

static inline uint32_t BspResetEvidenceNextSequence(uint32_t current)
{
    current++;
    return (current == 0u) ? 1u : current;
}

static inline uint32_t BspResetEvidenceChecksumUpdate(uint32_t checksum,
                                                      uint32_t word)
{
    checksum ^= word;
    return checksum * 16777619u;
}

#if defined(BSP_RESET_EVIDENCE_POLICY_TEST)
static inline uint32_t BspResetEvidenceChecksum(const BspResetEvidenceRecord *record)
{
    const uint32_t *words;
    uint32_t checksum = BSP_RESET_EVIDENCE_CHECKSUM_SEED;

    if (record == 0)
    {
        return 0u;
    }

    words = (const uint32_t *)record;
    for (uint32_t i = 0u;
         i < (uint32_t)(sizeof(*record) / sizeof(uint32_t));
         i++)
    {
        checksum = BspResetEvidenceChecksumUpdate(checksum, words[i]);
    }
    return checksum;
}

static inline uint8_t BspResetEvidenceStorageValid(const BspResetEvidenceStorage *storage)
{
    uint32_t checksum;

    if (storage == 0 ||
        storage->magic != BSP_RESET_EVIDENCE_MAGIC ||
        storage->magicInv != (uint32_t)(~BSP_RESET_EVIDENCE_MAGIC) ||
        storage->record.formatVersion != BSP_RESET_EVIDENCE_FORMAT_VERSION ||
        storage->record.recordSize != (uint32_t)sizeof(storage->record))
    {
        return 0u;
    }

    checksum = BspResetEvidenceChecksum(&storage->record);
    return (uint8_t)(storage->checksum == checksum &&
                     storage->checksumInv == (uint32_t)(~checksum));
}

static inline void BspResetEvidenceStorageBuild(BspResetEvidenceStorage *storage,
                                                const BspResetEvidenceRecord *record)
{
    if (storage == 0 || record == 0)
    {
        return;
    }

    storage->magic = 0u;
    storage->magicInv = 0u;
    storage->record = *record;
    storage->checksum = BspResetEvidenceChecksum(&storage->record);
    storage->checksumInv = (uint32_t)(~storage->checksum);
    storage->magicInv = (uint32_t)(~BSP_RESET_EVIDENCE_MAGIC);
    storage->magic = BSP_RESET_EVIDENCE_MAGIC;
}
#endif

#endif
