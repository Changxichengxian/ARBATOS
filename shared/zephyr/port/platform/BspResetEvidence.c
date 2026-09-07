/* SPDX-License-Identifier: Apache-2.0 */
#include "BspResetEvidence.h"
#include <string.h>
/* SRAM-only fallback: valid for the current boot diagnostics, deliberately
 * not advertised as persistent evidence across reset. */
static BspResetEvidenceBoot Boot;
void BspResetEvidenceCaptureBoot(void) { memset(&Boot, 0, sizeof(Boot)); }
uint8_t BspResetEvidenceGetBoot(BspResetEvidenceBoot *out) { if (!out) return 0; *out = Boot; return Boot.evidenceValid; }
void BspResetEvidenceWriteFatal(const BspResetEvidenceRecord *record) { if (!record) return; Boot.evidence = *record; Boot.evidenceValid = 1; }
void BspResetEvidenceAcknowledge(uint32_t sequence) { if (Boot.evidenceValid && Boot.evidence.sequence == sequence) Boot.evidenceValid = 0; }
