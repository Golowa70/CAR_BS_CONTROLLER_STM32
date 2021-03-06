/*
 * file version V0.0.0
 * https://github.com/alambe94/Embedded_Utilities/tree/master/Button
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef BUTTON_H_
#define BUTTON_H_

/** std includes */
#include <stdint.h>

typedef enum Button_Event_t
{
    Button_Idle = 0,

    /** still pressed */
    Button_Pressed,

    /** at least one clicked and still pressed */
    Button_Repressed,
    Button_Long_Pressed

} Button_Event_t;

typedef struct Button_Struct_t
{
    Button_Event_t Button_Event;
    uint32_t Button_Pressed_Ticks;
    uint32_t Button_Released_Ticks;

    /** used in Callback if defined */
    uint8_t Button_Clicked_Count;

    /** used for manual read in Button_Get_Clicked_Count */
    uint8_t Button_Count_Captured;

    /** enable clock, configure gpio pin as input */
    void (*Button_Init)(void);

    /** return if button is pressed or not */
    uint8_t (*Button_Read)(void);

    /** optional callback function, if defined will be called after button event */
    void (*Callback)(uint8_t Button_Clicked_Count);

} Button_Struct_t;

void Button_Loop(void);
void Button_Reset_Count(Button_Struct_t *handle);
int32_t Button_Add(Button_Struct_t *handle);
uint8_t Button_Get_Clicked_Count(Button_Struct_t *handle);
Button_Event_t Button_Get_Status(Button_Struct_t *handle);

#endif /* BUTTON_H_ */
