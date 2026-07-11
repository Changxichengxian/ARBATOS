/*
 * SPDX-FileCopyrightText: 2026 Xie Yuhan <2811158416@qq.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "BspResetEvidence.h"

#include "BspResetEvidencePolicy.h"
#include "main.h"

/* 备份 SRAM 共 4 KiB；这里只保留开头一份固定记录，禁止其他模块私自复用。 */
#if defined(STM32H723xx)
#define BSP_RESET_EVIDENCE_STORAGE_ADDRESS D3_BKPSRAM_BASE
#elif defined(STM32F407xx) || defined(STM32F427xx)
#define BSP_RESET_EVIDENCE_STORAGE_ADDRESS BKPSRAM_BASE
#else
#error "BspResetEvidence requires a supported STM32 target"
#endif

typedef char BspResetEvidenceStorageMustFit[
    (sizeof(BspResetEvidenceStorage) <= 512u) ? 1 : -1];

#define BSP_RESET_EVIDENCE_FORMAT_WORD_INDEX 0u
#define BSP_RESET_EVIDENCE_SIZE_WORD_INDEX 1u
#define BSP_RESET_EVIDENCE_SEQUENCE_WORD_INDEX 2u

static BspResetEvidenceBoot s_boot;
static volatile uint32_t s_boot_ready;

static volatile BspResetEvidenceStorage *BspResetEvidenceRawStorage(void)
{
    return (volatile BspResetEvidenceStorage *)BSP_RESET_EVIDENCE_STORAGE_ADDRESS;
}

static void BspResetEvidenceEnableAccess(void)
{
#if defined(STM32H723xx)
    PWR->CR1 |= PWR_CR1_DBP;
    (void)PWR->CR1;
    RCC->AHB4ENR |= RCC_AHB4ENR_BKPRAMEN;
    (void)RCC->AHB4ENR;
#else
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR |= PWR_CR_DBP;
    (void)PWR->CR;
    RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
    (void)RCC->AHB1ENR;
#endif
    __DSB();
}

static void BspResetEvidenceCacheInvalidate(void)
{
#if defined(STM32H723xx)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u)
    {
        SCB_InvalidateDCache_by_Addr(
            (uint32_t *)BSP_RESET_EVIDENCE_STORAGE_ADDRESS,
            (int32_t)sizeof(BspResetEvidenceStorage));
    }
#endif
    __DSB();
}

static void BspResetEvidenceCacheClean(void)
{
#if defined(STM32H723xx)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u)
    {
        SCB_CleanDCache_by_Addr(
            (uint32_t *)BSP_RESET_EVIDENCE_STORAGE_ADDRESS,
            (int32_t)sizeof(BspResetEvidenceStorage));
    }
#endif
    __DSB();
}

static uint8_t BspResetEvidenceRawValid(uint32_t *sequence)
{
    volatile const BspResetEvidenceStorage *storage =
        BspResetEvidenceRawStorage();
    volatile const uint32_t *words =
        (volatile const uint32_t *)&storage->record;
    uint32_t checksum = BSP_RESET_EVIDENCE_CHECKSUM_SEED;
    uint32_t stored_checksum;

    if (storage->magic != BSP_RESET_EVIDENCE_MAGIC ||
        storage->magicInv != (uint32_t)(~BSP_RESET_EVIDENCE_MAGIC) ||
        storage->record.formatVersion != BSP_RESET_EVIDENCE_FORMAT_VERSION ||
        storage->record.recordSize != (uint32_t)sizeof(storage->record))
    {
        return 0u;
    }

    for (uint32_t i = 0u;
         i < (uint32_t)(sizeof(storage->record) / sizeof(uint32_t));
         i++)
    {
        checksum = BspResetEvidenceChecksumUpdate(checksum, words[i]);
    }
    stored_checksum = storage->checksum;
    __DMB();
    if (storage->magic != BSP_RESET_EVIDENCE_MAGIC ||
        storage->magicInv != (uint32_t)(~BSP_RESET_EVIDENCE_MAGIC) ||
        stored_checksum != checksum ||
        storage->checksumInv != (uint32_t)(~checksum))
    {
        return 0u;
    }
    if (sequence != 0)
    {
        *sequence = storage->record.sequence;
    }
    return 1u;
}

static void BspResetEvidenceRawRecordCopy(BspResetEvidenceRecord *out)
{
    volatile const uint32_t *src =
        (volatile const uint32_t *)&BspResetEvidenceRawStorage()->record;
    uint32_t *dst = (uint32_t *)out;

    for (uint32_t i = 0u;
         i < (uint32_t)(sizeof(*out) / sizeof(uint32_t));
         i++)
    {
        dst[i] = src[i];
    }
}

static uint32_t BspResetEvidenceResetFlags(void)
{
#if defined(STM32H723xx)
    return RCC->RSR;
#else
    return RCC->CSR;
#endif
}

static void BspResetEvidenceResetFlagsClear(void)
{
#if defined(STM32H723xx)
    RCC->RSR |= RCC_RSR_RMVF;
#else
    RCC->CSR |= RCC_CSR_RMVF;
#endif
}

void BspResetEvidenceCaptureBoot(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (s_boot_ready != 0u)
    {
        if (primask == 0u)
        {
            __enable_irq();
        }
        return;
    }

    BspResetEvidenceEnableAccess();
    s_boot.resetFlags = BspResetEvidenceResetFlags();
    BspResetEvidenceCacheInvalidate();
    s_boot.evidenceValid = BspResetEvidenceRawValid(0);
    if (s_boot.evidenceValid != 0u)
    {
        BspResetEvidenceRawRecordCopy(&s_boot.evidence);
    }

    BspResetEvidenceResetFlagsClear();
    __DMB();
    s_boot_ready = 1u;
    __DMB();

    if (primask == 0u)
    {
        __enable_irq();
    }
}

uint8_t BspResetEvidenceGetBoot(BspResetEvidenceBoot *out)
{
    if (out == 0)
    {
        return 0u;
    }

    BspResetEvidenceCaptureBoot();
    __DMB();
    *out = s_boot;
    return 1u;
}

void BspResetEvidenceWriteFatal(const BspResetEvidenceRecord *record)
{
    volatile BspResetEvidenceStorage *storage;
    volatile uint32_t *dst;
    const uint32_t *src;
    uint32_t sequence = 0u;
    uint32_t checksum = BSP_RESET_EVIDENCE_CHECKSUM_SEED;
    const uint32_t primask = __get_PRIMASK();

    if (record == 0)
    {
        return;
    }

    __disable_irq();
    BspResetEvidenceEnableAccess();
    BspResetEvidenceCacheInvalidate();
    (void)BspResetEvidenceRawValid(&sequence);
    sequence = BspResetEvidenceNextSequence(sequence);

    storage = BspResetEvidenceRawStorage();
    storage->magic = 0u;
    storage->magicInv = 0u;
    BspResetEvidenceCacheClean();

    dst = (volatile uint32_t *)&storage->record;
    src = (const uint32_t *)record;
    for (uint32_t i = 0u;
         i < (uint32_t)(sizeof(*record) / sizeof(uint32_t));
         i++)
    {
        uint32_t value = src[i];

        if (i == BSP_RESET_EVIDENCE_FORMAT_WORD_INDEX)
        {
            value = BSP_RESET_EVIDENCE_FORMAT_VERSION;
        }
        else if (i == BSP_RESET_EVIDENCE_SIZE_WORD_INDEX)
        {
            value = (uint32_t)sizeof(*record);
        }
        else if (i == BSP_RESET_EVIDENCE_SEQUENCE_WORD_INDEX)
        {
            value = sequence;
        }
        dst[i] = value;
        checksum = BspResetEvidenceChecksumUpdate(checksum, value);
    }
    storage->checksum = checksum;
    storage->checksumInv = (uint32_t)(~checksum);
    BspResetEvidenceCacheClean();

    storage->magicInv = (uint32_t)(~BSP_RESET_EVIDENCE_MAGIC);
    BspResetEvidenceCacheClean();
    storage->magic = BSP_RESET_EVIDENCE_MAGIC;
    BspResetEvidenceCacheClean();

    if (primask == 0u)
    {
        __enable_irq();
    }
}

void BspResetEvidenceAcknowledge(uint32_t sequence)
{
    volatile BspResetEvidenceStorage *storage;
    uint32_t current_sequence = 0u;
    const uint32_t primask = __get_PRIMASK();

    if (sequence == 0u)
    {
        return;
    }

    __disable_irq();
    BspResetEvidenceEnableAccess();
    BspResetEvidenceCacheInvalidate();
    if (BspResetEvidenceRawValid(&current_sequence) != 0u &&
        current_sequence == sequence)
    {
        storage = BspResetEvidenceRawStorage();
        storage->magic = 0u;
        storage->magicInv = 0u;
        BspResetEvidenceCacheClean();
    }

    if (primask == 0u)
    {
        __enable_irq();
    }
}
