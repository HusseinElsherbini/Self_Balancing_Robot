#ifndef TASK_SCHEDULER_H_
#define TASK_SCHEDULER_H_

#include <stdint.h>

// STACK memory calculations
#define TASK_STACK_SIZE       1024U
#define SCHED_STACK_SIZE      1024U
#define SRAM_START            0x20000000U
#define SRAM_SIZE             ((64) * (1024))
#define SRAM_END			  ((SRAM_START) + (SRAM_SIZE))

#define SCHED_STACK_START     ((SRAM_END) - (MAX_TASKS*TASK_STACK_SIZE))
#define TASK_STACK_START(task_num)       ((SRAM_END) - (task_num*TASK_STACK_SIZE))

#define TICK_HZ               1000U

#define MAX_TASKS             6

#define TASK_READY_STATE      0x00
#define TASK_BLOCKED_STATE    0xFF
#define TASK_IDLE_STATE       0x01
#define UNLOCKED              0x30
#define LOCKED                0x31

#define IMU_RETRIEVE_RAW_DATA   0U
#define IMU_PROCESS_RAW_DATA    1U

// GLOBAL VARS
extern uint8_t current_task;
extern uint32_t gTickCount;
extern void (*taskHandlers[MAX_TASKS])(void);

typedef struct
{
	uint32_t psp_value;
	uint32_t block_count;
	uint8_t lock;
	uint8_t current_state;
	uint32_t maxStackSpaceUsed;
	uint32_t taskStackStartAddress;
	void (*task_handler)(void);
}TCB_t;

extern TCB_t user_tasks[MAX_TASKS];

void task1_handler(void);
void task2_handler(void);
void task3_handler(void);
void task4_handler(void);
void task5_handler(void);
void task6_handler(void);
void init_tasks_stack(void);
void task_delay(uint32_t tick_no, uint8_t lock_state);
void schedule(void);
void update_global_tick_count(void);
uint32_t get_psp_value(void);
void save_psp_value(uint32_t current_psp_value);
void update_next_task(void);
void unblock_tasks(void);
void lock_task(uint8_t task_num);
void taskStackTraceDepth(TCB_t * userTask);
__attribute__((naked)) void switch_sp_to_psp(void);
__attribute__((naked)) void init_schedulaer_stack(uint32_t sched_stack_start);

#endif /* TASK_SCHEDULER_H_ */