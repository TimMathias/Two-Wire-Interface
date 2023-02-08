#pragma once
#ifndef QUEUE_H
#define QUEUE_H

//
// MIT License
//
// Copyright (c) 2021-2023 Timothy Mathias
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <Arduino.h>

template <class T, byte length>
class Queue
{
    static_assert(length > 3, "Buffer length must be greater than three.");
    
    private:
        T data[length];
        byte head;
        byte tail;

    public:
        Queue();
        bool Dequeue(T &datum);
        bool Enqueue(T datum);
        byte Dequeueable() const;
        byte Enqueueable() const;
};


template <class T, byte length>
Queue<T, length>::Queue()
{
    memset(data, 0, length * sizeof(T));
    head = 0;
    tail = 0;
}

template <class T, byte length>
bool Queue<T, length>::Dequeue(T &datum)
{
    bool result = false;
    if (tail != head)
    {
        datum = data[tail];
        memset(&(data[tail]), 0, sizeof(T));
        tail++;
        tail %= length;
        result = true;
    }
    return result;
}

template <class T, byte length>
bool Queue<T, length>::Enqueue(T datum)
{
    bool result = false;
    byte new_head = (head + 1) % length;
    if (new_head != tail)
    {
        data[head] = datum;
        head = new_head;
        result = true;
    }
    return result;
}

template <class T, byte length>
byte Queue<T, length>::Dequeueable() const
{
    return (length + head - tail) % length;
}

template <class T, byte length>
byte Queue<T, length>::Enqueueable() const
{
    return (length - (head - tail) - 1) % length;
}

#endif
