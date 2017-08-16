::/*****************************************************************************/
::/* Copyright 2017 Eric Pernia.                                               */
::/* All rights reserved.                                                      */
::/*                                                                           */
::/* This file is part of IDE4PLC Firmware. http://ide4plc.wordpress.com and   */
::/* part of CIAA Firmware. http://proyecto-ciaa.com.ar                        */
::/*                                                                           */
::/* Redistribution and use in source and binary forms, with or without        */
::/* modification, are permitted provided that the following conditions are    */
::/* met:                                                                      */
::/*                                                                           */
::/* 1. Redistributions of source code must retain the above copyright notice, */
::/*    this list of conditions and the following disclaimer.                  */
::/*                                                                           */
::/* 2. Redistributions in binary form must reproduce the above copyright      */
::/*    notice, this list of conditions and the following disclaimer in the    */
::/*    documentation and/or other materials provided with the distribution.   */
::/*                                                                           */
::/* 3. Neither the name of the copyright holders nor the names of its         */
::/*    contributors may be used to endorse or promote products derived from   */
::/*    this software without specific prior written permission.               */
::/*                                                                           */
::/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       */
::/* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED */
::/* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A           */
::/* PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER */
::/* OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  */
::/* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,       */
::/* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR        */
::/* PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
::/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING      */
::/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        */
::/* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              */
::/*                                                                           */
::/*****************************************************************************/
::
@echo OFF
set HOME=%HOMEDRIVE%%HOMEPATH%
set PROJECT_PATH=%~dp0..\
set TOOLS_PATH=C:\CIAA\EmbeddedIDE
set PATH=%TOOLS_PATH%\bin;%TOOLS_PATH%\gcc-arm-embedded\bin;%TOOLS_PATH%\oocd\bin

echo Borrar memoria flash de la placa...
make -C %PROJECT_PATH% Borrar_memoria_flash

:: Con pause te deja ver el resultado, con exit lo cierra al terminar
::pause
exit
