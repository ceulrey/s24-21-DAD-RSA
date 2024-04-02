/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// The only changes from TI's ring buffer is the omission of the HwiP calls


#include <ti/drivers/utils/RingBuf.h>

/*
 *  ======== RingBuf_construct ========
 */
void modifiedRingBuf_construct(RingBuf_Handle object, unsigned char *bufPtr,
    size_t bufSize)
{
    object->buffer = bufPtr;
    object->length = bufSize;
    object->count = 0;
    object->head = bufSize - 1;
    object->tail = 0;
    object->maxCount = 0;
}

/*
 *  ======== RingBuf_get ========
 */
int modifiedRingBuf_get(RingBuf_Handle object, unsigned char *data)
{


    if (!object->count) {
        return -1;
    }

    *data = object->buffer[object->tail];
    object->tail = (object->tail + 1) % object->length;
    object->count--;


    return (object->count);
}

/*
 *  ======== RingBuf_getCount ========
 */
int modifiedRingBuf_getCount(RingBuf_Handle object)
{
    return (object->count);
}

/*
 *  ======== RingBuf_isFull ========
 */
bool modifiedRingBuf_isFull(RingBuf_Handle object)
{
    return (object->count == object->length);
}

/*
 *  ======== RingBuf_getMaxCount ========
 */
int modifiedRingBuf_getMaxCount(RingBuf_Handle object)
{
    return (object->maxCount);
}

/*
 *  ======== RingBuf_peek ========
 */
int modifiedRingBuf_peek(RingBuf_Handle object, unsigned char *data)
{
    unsigned int key;
    int          retCount;


    *data = object->buffer[object->tail];
    retCount = object->count;


    return (retCount);
}

/*
 *  ======== RingBuf_put ========
 */
int modifiedRingBuf_put(RingBuf_Handle object, unsigned char data)
{
    unsigned int key;
    unsigned int next;


    if (object->count != object->length) {
        next = (object->head + 1) % object->length;
        object->buffer[next] = data;
        object->head = next;
        object->count++;
        object->maxCount = (object->count > object->maxCount) ?
                            object->count :
                            object->maxCount;
    }
    else {

        return (-1);
    }


    return (object->count);
}
