/* 复位证据格式回归：提交顺序、损坏拒绝和序号回绕。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BSP_RESET_EVIDENCE_POLICY_TEST 1
#include "BspResetEvidencePolicy.h"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static BspResetEvidenceRecord TestRecord(void)
{
    BspResetEvidenceRecord record;

    (void)memset(&record, 0, sizeof(record));
    record.formatVersion = BSP_RESET_EVIDENCE_FORMAT_VERSION;
    record.recordSize = (uint32_t)sizeof(record);
    record.sequence = 7u;
    record.reason = 4u;
    record.arg0 = 0x11223344u;
    record.cfsr = 0x01020304u;
    record.pc = 0x08001235u;
    record.lr = 0x08004567u;
    record.bootStage = 9u;
    return record;
}

static int TestCommitAndCorruption(void)
{
    BspResetEvidenceStorage storage;
    const BspResetEvidenceRecord record = TestRecord();

    (void)memset(&storage, 0, sizeof(storage));
    BspResetEvidenceStorageBuild(&storage, &record);
    if (!TestCheck(BspResetEvidenceStorageValid(&storage) != 0u,
                   "完整提交必须通过校验")) return 0;

    storage.magic = 0u;
    if (!TestCheck(BspResetEvidenceStorageValid(&storage) == 0u,
                   "正文写完但有效标记未提交时必须无效")) return 0;
    BspResetEvidenceStorageBuild(&storage, &record);
    storage.record.pc ^= 4u;
    if (!TestCheck(BspResetEvidenceStorageValid(&storage) == 0u,
                   "正文单字损坏必须被校验拒绝")) return 0;
    BspResetEvidenceStorageBuild(&storage, &record);
    storage.checksumInv ^= 1u;
    return TestCheck(BspResetEvidenceStorageValid(&storage) == 0u,
                     "校验反码损坏必须被拒绝");
}

static int TestFormatAndSequence(void)
{
    BspResetEvidenceStorage storage;
    BspResetEvidenceRecord record = TestRecord();

    record.formatVersion++;
    BspResetEvidenceStorageBuild(&storage, &record);
    if (!TestCheck(BspResetEvidenceStorageValid(&storage) == 0u,
                   "未知格式版本不能误读")) return 0;
    record = TestRecord();
    record.recordSize--;
    BspResetEvidenceStorageBuild(&storage, &record);
    if (!TestCheck(BspResetEvidenceStorageValid(&storage) == 0u,
                   "不完整记录长度不能误读")) return 0;
    if (!TestCheck(BspResetEvidenceNextSequence(0u) == 1u &&
                       BspResetEvidenceNextSequence(41u) == 42u,
                   "正常序号必须从一开始单调推进")) return 0;
    return TestCheck(BspResetEvidenceNextSequence(UINT32_MAX) == 1u,
                     "完整回绕不能生成全零保留序号");
}

int main(void)
{
    if (!TestCommitAndCorruption()) return 1;
    if (!TestFormatAndSequence()) return 1;
    (void)puts("PASS: reset evidence policy regression");
    return 0;
}
