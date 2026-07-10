#ifndef CONTROL_MGR_TEST_CRITICAL_H
#define CONTROL_MGR_TEST_CRITICAL_H

void ControlMgrTestEnterCritical(void);
void ControlMgrTestExitCritical(void);

#define CONTROL_MANAGER_ENTER_CRITICAL() ControlMgrTestEnterCritical()
#define CONTROL_MANAGER_EXIT_CRITICAL() ControlMgrTestExitCritical()

#endif
