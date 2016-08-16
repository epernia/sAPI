/* Copyright 2013, Michael J. Pont.
 * Copyright 2016, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Date: 2016-08-16 */

#ifndef _SEOS_H_
#define _SEOS_H_

/*==================[inclusions]=============================================*/

#include "sAPI.h"         /* <= sAPI header */

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/* The maximum number of tasks required at any one time during the execution
   of the program. MUST BE ADJUSTED FOR EACH NEW PROJECT */
#define SCH_MAX_TASKS (3)

/*==================[typedef]================================================*/

/* Store in DATA area, if possible, for rapid access.
   Total memory per task is 7 bytes. */
typedef struct
{
   /* Pointer to the task (must be a 'void (void)' function) */
   void (* pTask)(void);
   /* Delay (ticks) until the function will (next) be run
      - see SCH_Add_Task() for further details */
   int Delay;
   /* Interval (ticks) between subsequent runs.
      - see SCH_Add_Task() for further details */
   int Period;
   /* Incremented (by scheduler) when task is due to execute */
   int RunMe;
} sTask;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/* FUNCION que contiene el despachador de tareas. */
void SCH_Dispatch_Tasks(void);

/* FUNCION que añade una tarea al planificador. */
char SCH_Add_Task(void (*) (void), const int, const int);

/* FUNCION que remueve una tarea del planificador. */
char SCH_Delete_Task(const int);

/* FUNCION que reporta el estado del sistema. */
void SCH_Report_Status(void);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _SEOS_H_ */
