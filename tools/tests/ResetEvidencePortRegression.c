/* Host regression for the real Zephyr backup-SRAM evidence backend.
 * It models a system reset by clearing only the backend's BSS state. */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define STM32H723xx 1
#define BSP_RESET_EVIDENCE_POLICY_TEST 1

#include "soc.h"
#include "BspResetEvidence.h"

ResetEvidenceTestPwr ResetEvidenceTestPwrRegs;
ResetEvidenceTestRcc ResetEvidenceTestRccRegs;
ResetEvidenceTestScb ResetEvidenceTestScbRegs;
uint8_t ResetEvidenceTestBackupSram[512];

#include "../../shared/zephyr/port/platform/BspResetEvidence.c"

static int Check(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static void SimulateSystemReset(uint32_t reset_flags)
{
    (void)memset(&s_boot, 0, sizeof(s_boot));
    s_boot_ready = 0u;
    ResetEvidenceTestRccRegs.RSR = reset_flags;
}

static BspResetEvidenceRecord Record(uint32_t reason)
{
    BspResetEvidenceRecord record;

    (void)memset(&record, 0, sizeof(record));
    record.reason = reason;
    record.arg0 = 0x11223344u;
    record.pc = 0x08001235u;
    record.cfsr = 0x01020304u;
    return record;
}

static int TestWriteResetRead(void)
{
    BspResetEvidenceBoot boot;
    const BspResetEvidenceRecord record = Record(4u);

    (void)memset(ResetEvidenceTestBackupSram, 0, sizeof(ResetEvidenceTestBackupSram));
    BspResetEvidenceWriteFatal(&record);
    SimulateSystemReset(0x12340000u);
    BspResetEvidenceCaptureBoot();
    if (!Check(BspResetEvidenceGetBoot(&boot) != 0u && boot.evidenceValid != 0u,
               "software reset must retain a complete record")) return 0;
    if (!Check(boot.resetFlags == 0x12340000u && boot.evidence.reason == 4u &&
                   boot.evidence.arg0 == record.arg0 && boot.evidence.pc == record.pc,
               "boot copy must retain reset flags and evidence fields")) return 0;
    BspResetEvidenceCaptureBoot();
    return Check(boot.evidence.sequence == 1u && s_boot.evidence.sequence == 1u,
                 "CaptureBoot must be idempotent");
}

static int TestDamagedAndHalfCommittedRejected(void)
{
    BspResetEvidenceBoot boot;
    BspResetEvidenceStorage *storage = (BspResetEvidenceStorage *)ResetEvidenceTestBackupSram;

    storage->record.pc ^= 4u;
    SimulateSystemReset(0x22u);
    BspResetEvidenceCaptureBoot();
    if (!Check(BspResetEvidenceGetBoot(&boot) != 0u && boot.evidenceValid == 0u,
               "checksum-damaged record must be rejected")) return 0;

    (void)memset(ResetEvidenceTestBackupSram, 0, sizeof(ResetEvidenceTestBackupSram));
    storage->magicInv = ~BSP_RESET_EVIDENCE_MAGIC;
    SimulateSystemReset(0x33u);
    BspResetEvidenceCaptureBoot();
    return Check(BspResetEvidenceGetBoot(&boot) != 0u && boot.evidenceValid == 0u,
                 "half-written magic must be rejected");
}

static int TestAcknowledgeAndSequence(void)
{
    BspResetEvidenceBoot boot;
    BspResetEvidenceRecord record = Record(7u);

    (void)memset(ResetEvidenceTestBackupSram, 0, sizeof(ResetEvidenceTestBackupSram));
    BspResetEvidenceWriteFatal(&record);
    SimulateSystemReset(0x44u);
    BspResetEvidenceCaptureBoot();
    (void)BspResetEvidenceGetBoot(&boot);
    BspResetEvidenceAcknowledge(boot.evidence.sequence + 1u);
    SimulateSystemReset(0x45u);
    BspResetEvidenceCaptureBoot();
    if (!Check(BspResetEvidenceGetBoot(&boot) != 0u && boot.evidenceValid != 0u,
               "mismatched sequence must not acknowledge evidence")) return 0;
    BspResetEvidenceAcknowledge(boot.evidence.sequence);
    SimulateSystemReset(0x46u);
    BspResetEvidenceCaptureBoot();
    if (!Check(BspResetEvidenceGetBoot(&boot) != 0u && boot.evidenceValid == 0u,
               "matching sequence must acknowledge evidence")) return 0;

    record = Record(8u);
    BspResetEvidenceWriteFatal(&record);
    SimulateSystemReset(0x47u);
    BspResetEvidenceCaptureBoot();
    (void)BspResetEvidenceGetBoot(&boot);
    return Check(boot.evidence.sequence == 1u,
                 "acknowledged storage has no valid sequence and must restart at one");
}

static int TestSequenceWrap(void)
{
    BspResetEvidenceStorage *storage = (BspResetEvidenceStorage *)ResetEvidenceTestBackupSram;
    BspResetEvidenceRecord old_record = Record(9u);
    BspResetEvidenceRecord new_record = Record(10u);
    BspResetEvidenceBoot boot;

    (void)memset(ResetEvidenceTestBackupSram, 0, sizeof(ResetEvidenceTestBackupSram));
    old_record.sequence = UINT32_MAX;
    BspResetEvidenceStorageBuild(storage, &old_record);
    BspResetEvidenceWriteFatal(&new_record);
    SimulateSystemReset(0x48u);
    BspResetEvidenceCaptureBoot();
    (void)BspResetEvidenceGetBoot(&boot);
    return Check(boot.evidenceValid != 0u && boot.evidence.sequence == 1u,
                 "sequence must wrap from UINT32_MAX to one");
}

int main(void)
{
    if (!TestWriteResetRead()) return 1;
    if (!TestDamagedAndHalfCommittedRejected()) return 1;
    if (!TestAcknowledgeAndSequence()) return 1;
    if (!TestSequenceWrap()) return 1;
    (void)puts("PASS: reset evidence Zephyr-port host regression");
    return 0;
}
