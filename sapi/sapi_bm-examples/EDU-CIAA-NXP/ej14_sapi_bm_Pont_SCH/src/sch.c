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

/*==================[inclusions]=============================================*/

#include "sch.h"          /* <= own header */

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/* Keeps track of time since last error was recorded (see below) */
static int Error_tick_count;

/* The code of the last error (reset after ~1 minute) */
static char Last_error_code;

/*==================[internal functions declaration]=========================*/

static void SCH_Go_To_Sleep(void);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/* The array of tasks */
sTask SCH_tasks[SCH_MAX_TASKS];

/* Used to display the error code */
char Error_code = 0;

/*==================[internal functions definition]==========================*/


/*==================[external functions definition]==========================*/

/*------------------------------------------------------------------*-
SCH_Dispatch_Tasks()
This is the 'dispatcher' function. When a task (function)
is due to run, SCH_Dispatch_Tasks() will run it.
This function must be called (repeatedly) from the main loop.
-*------------------------------------------------------------------*/
void SCH_Dispatch_Tasks(void){

   int Index;
   /* Dispatches (runs) the next task (if one is ready) */
   for (Index = 0; Index < SCH_MAX_TASKS; Index++){
      
      if (SCH_tasks[Index].RunMe > 0){
         
         (*SCH_tasks[Index].pTask)(); /* Run the task */
         SCH_tasks[Index].RunMe -= 1; /* Reset / reduce RunMe flag */
         
         /* Periodic tasks will automatically run again
            - if this is a 'one shot' task, remove it from the array */
         if (SCH_tasks[Index].Period == 0){
            SCH_Delete_Task(Index);
         }
         
      }
   }
   
   /* Report system status */
   SCH_Report_Status();
   /* The scheduler enters idle mode at this point */
   SCH_Go_To_Sleep();
}

/*------------------------------------------------------------------*-
SCH_Add_Task()
Causes a task (function) to be executed at regular intervals
or after a user-defined delay
Fn_P - The name of the function which is to be scheduled.

NOTE: All scheduled functions must be 'void, void' -
that is, they must take no parameters, and have
a void return type.

DELAY - The interval (TICKS) before the task is first executed

PERIOD - If 'PERIOD' is 0, the function is only called once,
at the time determined by 'DELAY'. If PERIOD is non-zero,
then the function is called repeatedly at an interval
determined by the value of PERIOD (see below for examples
which should help clarify this)

RETURN VALUE:
Returns the position in the task array at which the task has been
added. If the return value is SCH_MAX_TASKS then the task could
not be added to the array (there was insufficient space). If the
return value is < SCH_MAX_TASKS, then the task was added
successfully.
Note: this return value may be required, if a task is
to be subsequently deleted - see SCH_Delete_Task().

EXAMPLES:
Task_ID = SCH_Add_Task(Do_X,1000,0);
Causes the function Do_X() to be executed once after 1000 sch ticks.
Task_ID = SCH_Add_Task(Do_X,0,1000);
Causes the function Do_X() to be executed regularly, every 1000 sch ticks.
Task_ID = SCH_Add_Task(Do_X,300,1000);
Causes the function Do_X() to be executed regularly, every 1000 ticks.
Task will be first executed at T = 300 ticks, then 1300, 2300, etc.
-*------------------------------------------------------------------*/
char SCH_Add_Task( void (* pFunction)(void),
                            const int DELAY,
                            const int PERIOD
                 ){

   int Index = 0;

   /* First find a gap in the array (if there is one) */
   while ((SCH_tasks[Index].pTask != 0) && (Index < SCH_MAX_TASKS)){
      Index++;
   }

   /* Have we reached the end of the list? */
   if (Index == SCH_MAX_TASKS){
      /* Task list is full
         Set the global error variable */
      Error_code = 2; /* ERROR_SCH_TOO_MANY_TASKS; */
      /* Also return an error code */
      return SCH_MAX_TASKS;
   }

   /* If we're here, there is a space in the task array */
   SCH_tasks[Index].pTask = pFunction;
   SCH_tasks[Index].Delay = DELAY;
   SCH_tasks[Index].Period = PERIOD;
   SCH_tasks[Index].RunMe = 0;

   return Index; /* return position of task (to allow later deletion) */
}

/*------------------------------------------------------------------*-
SCH_Delete_Task()
Removes a task from the scheduler. Note that this does
*not* delete the associated function from memory:
it simply means that it is no longer called by the scheduler.

TASK_INDEX - The task index. Provided by SCH_Add_Task().

RETURN VALUE: RETURN_ERROR or RETURN_NORMAL
-*------------------------------------------------------------------*/
char SCH_Delete_Task(const int TASK_INDEX){

   char Return_code;

   if (SCH_tasks[TASK_INDEX].pTask == 0){
      /* No task at this location...
         Set the global error variable */
      Error_code = 2; /* ERROR_SCH_CANNOT_DELETE_TASK; */
      /* ...also return an error code */
      Return_code = -1; /* RETURN_ERROR; */
   }
   else
   {
      Return_code = 0; /* RETURN_NORMAL; */
   }
   SCH_tasks[TASK_INDEX].pTask = 0x0000;
   SCH_tasks[TASK_INDEX].Delay = (int)0;
   SCH_tasks[TASK_INDEX].Period = (int)0;
   SCH_tasks[TASK_INDEX].RunMe = (char)0;
   return Return_code; /* return status */
}

/*------------------------------------------------------------------*-
SCH_Report_Status()
Simple function to display error codes.
This version displays code on a port with attached LEDs:
adapt, if required, to report errors over serial link, etc.
Errors are only displayed for a limited period
(60000 ticks = 1 minute at 1ms tick interval).
After this the the error code is reset to 0.
This code may be easily adapted to display the last
error 'for ever': this may be appropriate in your
application.
-*------------------------------------------------------------------*/
void SCH_Report_Status(void){

#ifdef SCH_REPORT_ERRORS
   /* ONLY APPLIES IF WE ARE REPORTING ERRORS
      Check for a new error code */
   if (Error_code != Last_error_code){

      /* Negative logic on LEDs assumed */
      Error_port = 255 - Error_code;
      Last_error_code = Error_code;

      if (Error_code!= 0){
         Error_tick_count = 60000;
      }
      else{
         Error_tick_count = 0;
      }
   }
   else{
      if (Error_tick_count != 0){
         if (--Error_tick_count == 0){
            Error_code = 0; /* Reset error code */
         }
      }
   }
#endif
}

/*------------------------------------------------------------------*-
SCH_Go_To_Sleep()
This scheduler enters 'idle mode' between clock ticks
to save power. The next clock tick will return the processor
to the normal operating state.
*** May wish to disable this if using a watchdog ***
*** ADAPT AS REQUIRED FOR YOUR HARDWARE ***
-*------------------------------------------------------------------*/
static void SCH_Go_To_Sleep(){

   /* Se pone el sistema en bajo consumo hasta que ocurra la proxima 
      interrupcion, en este caso la de Tick. */
   sleepUntilNextInterrupt();
}

/*==================[end of file]============================================*/