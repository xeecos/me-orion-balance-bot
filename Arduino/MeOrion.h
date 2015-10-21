/**
 * \par Copyright (C), 2012-2015, MakeBlock
 * \brief   Driver for MeOrion board.
 * @file    MeOrion.h
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/09/01
 * @brief   Driver for MeOrion board.
 *
 * \par Copyright
 * This software is Copyright (C), 2012-2015, MakeBlock. Use is subject to license \n
 * conditions. The main licensing options available are GPL V2 or Commercial: \n
 *
 * \par Open Source Licensing GPL V2
 * This is the appropriate option if you want to share the source code of your \n
 * application with everyone you distribute it to, and you also want to give them \n
 * the right to share who uses it. If you wish to use this software under Open \n
 * Source Licensing, you must contribute all your source code to the open source \n
 * community in accordance with the GPL Version 2 when your application is \n
 * distributed. See http://www.gnu.org/copyleft/gpl.html
 *
 * \par Description
 * This file is the driver for MeOrion hoard by MakeBlock.
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Mark Yan         2015/09/01     1.0.0            Rebuild the old lib.
 * Rafael Lee       2015/09/02     1.0.0            Added some comments and macros.
 * </pre>
 */
#ifndef MeOrion_H
#define MeOrion_H

#include <arduino.h>
#include "MeConfig.h"
#include "MePort.h"

// Supported Modules drive needs to be added here

/*********************  Orion Board GPIO Map *********************************/
// struct defined in MePort.h
MePort_Sig mePort[11] =
{
  { NC, NC }, { 11, 10 }, { 3,  9 }, { 12, 13 }, { 8, 2 },
  { NC, NC }, { A2, A3 }, { A6, A1 }, { A7, A0 }, { 6, 7 },
  { 5, 4 }
};

#define buzzerOn()  pinMode(SCL,OUTPUT),digitalWrite(SCL, HIGH)
#define buzzerOff() pinMode(SCL,OUTPUT),digitalWrite(SCL, LOW)

#endif // MeOrion_H
