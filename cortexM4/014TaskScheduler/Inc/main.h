/*
 * main.h
 *
 *  Created on: Nov 4, 2022
 *      Author: fahmad
 */

#ifndef MAIN_H_
#define MAIN_H_

void task1_handler();
void task2_handler();
void task3_handler();
void task4_handler();

void init_systick_timer(uint32_t tick_hz);
__attribute__((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack);
void init_tasks_stack();
__attribute__((naked)) void switch_sp_to_psp();
void enable_processor_faults();
uint32_t get_psp_value();
void save_psp_value(uint32_t current_psp_value);
void update_next_task();
void task_delay(uint32_t tick_count);
void idle_task();
void update_global_tick_count();
void unblock_tasks();
void schedule();

void HardFault_Handler();
void MemManage_Handler();
void BusFault_Handler();

void SysTick_Handler();

// stack memory calculations
#define SIZE_TASK_STACK 		1024U
#define SIZE_SCHEDULER_STACK 	1024U

// see STM32F446RETX_RAM.ld
#define SRAM_START 				0x20000000U
#define SIZE_RAM				(128 * 1024)
#define SRAM_END				(SRAM_START + SIZE_RAM)

#define T1_STACK_START			SRAM_END
#define T2_STACK_START			(SRAM_END - (1 * SIZE_TASK_STACK))
#define T3_STACK_START			(SRAM_END - (2 * SIZE_TASK_STACK))
#define T4_STACK_START			(SRAM_END - (3 * SIZE_TASK_STACK))
#define SCHED_STACK_START		(SRAM_END - (4 * SIZE_TASK_STACK))
#define IDLE_STACK_START		(SRAM_END - (5 * SIZE_TASK_STACK))

#define TICK_HZ 				1000U

#define HSI_CLOCK				16000000U
#define SYSTICK_TIMER_CLOCK		HSI_CLOCK

#define MAX_TASKS 				5
#define DUMMY_XPSR				0x01000000U

#define TASK_READY_STATE		0x00
#define TASK_BLOCKED_STATE		0xFF

#define INTERRUPT_DISABLE()		do{asm volatile ("MOV R0,#0x1"); asm volatile("MSR PRIMASK,R0;");}while(0)
#define INTERRUPT_ENABLE()		do{asm volatile ("MOV R0,#0x0"); asm volatile("MSR PRIMASK,R0;");}while(0)

#endif /* MAIN_H_ */
