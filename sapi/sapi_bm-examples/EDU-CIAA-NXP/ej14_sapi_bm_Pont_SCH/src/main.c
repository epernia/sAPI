/* Copyright 2016, Eric Pernia.
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
 */

/* Date: 2016-08-16
 *
 * Ejemplo de Sistema Operativo Cooperativo Simple de M. J. Pont. En este RTOS 
 * todas las tareas pueden ser Real-Time si se planifican utilizando offsets 
 * adecuados y además tienen un comportamiento cooperativo (las tareas deben 
 * ser NO bloqueantes, la duración de cada tarea menor a 1 Tick si se aplican 
 * offsets).
 * 
 * Este planificador cooperativo contiene:
 * 
 *  - Estructura de datos del planificador. Contiene información de 
 *    planificación y ejecución de cada tarea: sTask
 *  - Función de inicialización del planificador: SCH_Init();
 *  - Una única RTI que actualice el planificador periódicamente: SCH_Update();
 *  - Una función de despacho de tareas que arranque la tarea que corresponda
 *    ejecutar: SCH_Dispatch_Tasks();
 *  - Una función para agregar tareas al planificador 
 *    SCH_Add_Task(Task_Name, Delay, Period );
 *  - Una función para remover tareas del planificador (si fuese necesario): 
 *    SCH_Delete_Task(char TASK_INDEX);
 */

/*==================[inclusions]=============================================*/

#include "main.h"         /* <= own header */

#include "sAPI.h"         /* <= sAPI header */

#include "tasks.h"        /* <= tasks header */
#include "isr.h"          /* <= scheduler and systema initialization header */
#include "sch.h"          /* <= dispatcher and task management header */

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void){

   /* ------------- INICIALIZACIONES ------------- */

   /* Inicializacion del planificador. */
   SCH_Init();

   /* Inicializacion de las tareas. */
   tasksInit();

   /* Se agrega task1 al planificador */
   SCH_Add_Task( task1, /* tarea a añadir */
                     0, /* offset de ejecucion en ticks */
                    40  /* periodicidad de ejecucion en ticks */
               );

   /* Se agrega task2 al planificador */
   SCH_Add_Task( task2, 1, 500 );

   /* Se agrega task3 al planificador */
   SCH_Add_Task( task3, 2, 1000 );
   
   /* Se inicializa la interrupcion que ejecuta el planificador de tareas.
      Se puede poner de 1 a 50ms */
   SCH_Start(1);
   
   /* ----- REPETIR POR SIEMPRE (SUPER LOOP) ----- */
   
   while(1) {
      
      /* Se despachan (ejecutan) las tareas marcadas para su ejecucion.
         Luego se pone el sistema en bajo consumo hasta que ocurra la 
         proxima interrupcion, en este caso la de Tick.
         Al ocurrir la interrupcion de Tick se ejecutara el planificador 
         que revisa cuales son las tareas a marcar para su ejecucion. */
	   SCH_Dispatch_Tasks();

   }

   /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
      por ningun S.O. */
   return 0 ;
}

/*==================[end of file]============================================*/
