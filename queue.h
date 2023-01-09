#pragma once
#ifndef QUEUE_H
#define QUEUE_H

#include <Arduino.h>

template <class T, byte length>
class Queue
{
    static_assert(length > 0, "Buffer length must be greater than zero.");
    
    private:
        T _data[length];
        byte _head;
        byte _tail;
        byte _length;

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
    _length = length;
    memset(_data, 0, length * sizeof(T));
    _head = 0;
    _tail = 0;
}

template <class T, byte length>
bool Queue<T, length>::Dequeue(T &datum)
{
    bool result = false;
    if (_head != _tail)
    {
        _tail++;
        _tail %= _length;
        datum = _data[_tail];
        memset(&(_data[_tail]), 0, sizeof(T));
        result = true;
    }
    return result;
}

template <class T, byte length>
bool Queue<T, length>::Enqueue(T datum)
{
    bool result = false;
    byte new_head = (_head + 1) % _length;
    if (new_head != _tail)
    {
        _data[new_head] = datum;
        _head = new_head;
        result = true;
    }
    return result;
}

template <class T, byte length>
byte Queue<T, length>::Dequeueable() const
{
    int16_t amount = (int16_t(_length) + _head - _tail) % _length;
    return amount;
}

template <class T, byte length>
byte Queue<T, length>::Enqueueable() const
{
    return (_length - Dequeueable() - 1);
}

#endif
